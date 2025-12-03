/**
 * @file dyn_a_star.cpp
 * @brief Dynamic A* path searching algorithm for 3D grid maps
 * 
 * Implements the A* search algorithm for finding collision-free paths
 * in a 3D occupancy grid map. Uses a dynamic node pool to efficiently
 * reuse allocated memory across multiple searches.
 * 
 * @author FAST-Lab, Zhejiang University
 */

#include "path_searching/dyn_a_star.h"

using namespace std;
using namespace Eigen;

/**
 * @brief Destructor - clean up allocated grid nodes
 */
AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

/**
 * @brief Initialize the grid map for path searching
 * 
 * Allocates a 3D grid of search nodes centered around the search space.
 * This pre-allocation allows fast reuse during subsequent searches.
 * 
 * @param occ_map Pointer to the occupancy grid map
 * @param pool_size Size of the node pool in each dimension
 */
void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;  // Center index for coordinate transformation

    // Allocate 3D array of grid nodes
    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

/**
 * @brief Compute diagonal distance heuristic
 * 
 * Calculates an admissible heuristic using the diagonal distance formula.
 * This is more accurate than Manhattan distance for 3D grids with 26-connectivity.
 * 
 * @param node1 Start node
 * @param node2 Goal node
 * @return double Heuristic distance estimate
 */
double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);  // Number of diagonal steps
    dx -= diag;
    dy -= diag;
    dz -= diag;

    // Cost = sqrt(3)*diagonal_steps + sqrt(2)*planar_diagonal + 1*straight
    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

/**
 * @brief Compute Manhattan distance heuristic
 * 
 * Simple but fast heuristic, admissible for 6-connectivity.
 * 
 * @param node1 Start node
 * @param node2 Goal node
 * @return double Heuristic distance estimate
 */
double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

/**
 * @brief Compute Euclidean distance heuristic
 * 
 * Most accurate but computationally expensive due to sqrt.
 * 
 * @param node1 Start node
 * @param node2 Goal node
 * @return double Heuristic distance estimate
 */
double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

/**
 * @brief Retrieve path by backtracking from goal to start
 * 
 * Follows the cameFrom pointers from the goal node back to the start
 * to reconstruct the path found by A*.
 * 
 * @param current Goal node
 * @return vector<GridNodePtr> Path from goal to start (needs to be reversed)
 */
vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    // Follow parent pointers back to start
    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;  // Note: path is from goal to start
}

/**
 * @brief Adjust start and end points if they are inside obstacles
 * 
 * If start or end point is in collision, iteratively moves the point
 * along the start-end line until a free point is found.
 * 
 * @param start_pt Start position in world coordinates
 * @param end_pt End position in world coordinates
 * @param start_idx Output start grid index
 * @param end_idx Output end grid index
 * @return true if valid indices found
 * @return false if unable to find valid points
 */
bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    // Convert world coordinates to grid indices
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    // Adjust start point if inside obstacle
    if (checkOccupancy(Index2Coord(start_idx)))
    {
        do
        {
            // Move start point away from end point
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    // Adjust end point if inside obstacle
    if (checkOccupancy(Index2Coord(end_idx)))
    {
        do
        {
            // Move end point away from start point
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

/**
 * @brief Main A* search function
 * 
 * Finds a collision-free path from start to end position using A* algorithm.
 * Uses 26-connectivity (all 3D neighbors) for path expansion.
 * 
 * Algorithm:
 * 1. Initialize start node with g=0, f=h(start, goal)
 * 2. While open set not empty:
 *    a. Pop node with lowest f value
 *    b. If goal reached, reconstruct path
 *    c. Expand to 26 neighbors
 *    d. Update g and f values if better path found
 * 3. Return false if no path exists
 * 
 * @param step_size Grid step size for conversion
 * @param start_pt Start position in world coordinates
 * @param end_pt End position in world coordinates
 * @return true if path found
 * @return false if no valid path exists
 */
bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();
    ++rounds_;  // Increment search round for node state tracking

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;  // Center of search space

    // Convert and adjust start/end points
    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        ROS_ERROR("Unable to handle the initial or end point, force return!");
        return false;
    }

    // Get start and end nodes from pool
    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    // Clear the open set (priority queue)
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    // Initialize start node
    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;                          // Cost from start is 0
    startPtr->fScore = getHeu(startPtr, endPtr);   // f = g + h
    startPtr->state = GridNode::OPENSET;
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr);

    endPtr->index = end_idx;

    double tentative_gScore;
    int num_iter = 0;

    // Main A* loop
    while (!openSet_.empty())
    {
        num_iter++;
        
        // Get node with lowest f-score
        current = openSet_.top();
        openSet_.pop();

        // Check if goal reached
        if (current->index(0) == endPtr->index(0) && 
            current->index(1) == endPtr->index(1) && 
            current->index(2) == endPtr->index(2))
        {
            gridPath_ = retrievePath(current);
            return true;
        }
        
        // Move current node to closed set
        current->state = GridNode::CLOSEDSET;

        // Expand to all 26 neighbors (3x3x3 cube minus center)
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;  // Skip self

                    // Calculate neighbor index
                    Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    // Check bounds
                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || 
                        neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || 
                        neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    // Get neighbor node
                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    // Skip if already in closed set
                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue;
                    }

                    neighborPtr->rounds = rounds_;

                    // Skip if occupied
                    if (checkOccupancy(Index2Coord(neighborPtr->index)))
                    {
                        continue;
                    }

                    // Calculate edge cost (Euclidean distance to neighbor)
                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        // Discovered a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr);
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    {
                        // Found better path to existing node
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        
        // Timeout check (200ms limit)
        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    // Timing diagnostic
    ros::Time time_2 = ros::Time::now();
    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;  // No path found
}

/**
 * @brief Get the found path as world coordinates
 * 
 * Converts the grid path to world coordinates and reverses
 * the order (from goal-to-start to start-to-goal).
 * 
 * @return vector<Vector3d> Path from start to goal in world coordinates
 */
vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    // Convert each grid node to world coordinates
    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    // Reverse to get start-to-goal order
    reverse(path.begin(), path.end());
    return path;
}
