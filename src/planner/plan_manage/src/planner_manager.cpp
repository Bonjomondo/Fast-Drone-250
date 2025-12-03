/**
 * @file planner_manager.cpp
 * @brief Ego-Planner Manager - Core trajectory planning algorithm implementation
 * 
 * This file implements the EGOPlannerManager class which coordinates the entire
 * trajectory planning process including:
 * - Global trajectory generation using minimum-snap polynomial
 * - Local trajectory optimization using B-spline
 * - Trajectory refinement for dynamic feasibility
 * - Multi-drone collision checking
 * 
 * @author FAST-Lab, Zhejiang University
 */

// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  /**
   * @brief Initialize all planning modules
   * 
   * Sets up the grid map, B-spline optimizer, and A* search algorithm
   * with parameters loaded from the ROS parameter server.
   * 
   * @param nh ROS node handle for parameter access
   * @param vis Visualization helper for debug visualization
   */
  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* Read algorithm parameters from ROS parameter server */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);           // Maximum velocity (m/s)
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);           // Maximum acceleration (m/s^2)
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);         // Maximum jerk (m/s^3)
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);  // Distance between control points
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);     // Look-ahead distance
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);  // Enable multi-trajectory selection
    nh.param("manager/drone_id", pp_.drone_id, -1);            // Drone ID for swarm

    local_data_.traj_id_ = 0;
    
    // Initialize grid map for obstacle representation
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    // Initialize B-spline optimizer
    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    
    // Initialize A* path finder within the optimizer
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  /**
   * @brief Main replanning function using the "rebound" approach
   * 
   * This is the core planning algorithm that generates collision-free,
   * dynamically feasible trajectories. The process involves:
   * 
   * 1. INIT: Generate initial path using min-snap polynomial or previous trajectory
   * 2. OPTIMIZE: Optimize B-spline to avoid obstacles while maintaining smoothness
   * 3. REFINE: Adjust time allocation to satisfy velocity/acceleration constraints
   * 
   * @param start_pt Start position
   * @param start_vel Start velocity
   * @param start_acc Start acceleration
   * @param local_target_pt Local target point (within planning horizon)
   * @param local_target_vel Target velocity at local target
   * @param flag_polyInit If true, always use polynomial initialization
   * @param flag_randomPolyTraj If true, add random perturbation to polynomial
   * @return true if planning succeeded
   * @return false if planning failed
   */
  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    // Check if already close to target - no need to replan
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT - Generate initial path ***/
    // Calculate appropriate time interval between control points
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // Use polynomial initialization on first call or when forced
      if (flag_first_call || flag_polyInit || flag_force_polynomial)
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        // Estimate trajectory duration using trapezoidal velocity profile
        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          // Generate simple one-segment min-snap trajectory
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          // Generate trajectory with random midpoint for exploration
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          
          // Random offset decreases with more failures
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        // Sample the polynomial trajectory to get control points
        double t;
        bool flag_too_far;
        ts *= 1.5; // Will be divided by 1.5 in the loop
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // Ensure minimum 7 points for B-spline
        
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Use previous trajectory for initialization
      {
        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        // Sample points along the current trajectory
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        // Add polynomial connection to target if needed
        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        // Resample points at uniform arc length
        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5;
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7);

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        // Check if path is too long - force polynomial regeneration
        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3)
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // Convert point set to B-spline control points
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // Initialize control points and detect collision segments
    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE - B-spline optimization ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // Generate multiple distinctive trajectories and select the best
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m" << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {
          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // Collect points for visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      // Single trajectory optimization
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = ros::Time::now() - t_start;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = ros::Time::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE - Time reallocation for dynamic feasibility ***/
    // Only adjust time in single drone mode or for drone_0
    if (pp_.drone_id <= 0)
    {
      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
      }
    }

    t_refine = ros::Time::now() - t_start;

    // Save planned results
    updateTrajInfo(pos, ros::Time::now());

    // Print timing statistics
    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt + t_refine).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;

    // Success!
    continous_failures_count_ = 0;
    return true;
  }

  /**
   * @brief Generate emergency stop trajectory
   * 
   * Creates a B-spline that keeps the drone at the specified position.
   * Used when emergency stop is triggered.
   * 
   * @param stop_pos Position to stop at
   * @return true Always returns true
   */
  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    // Create 6 control points all at the same position
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  /**
   * @brief Check for collision between this drone and another drone in swarm
   * 
   * Compares predicted positions along both trajectories to detect
   * potential collisions within the swarm clearance distance.
   * 
   * @param drone_id ID of the other drone to check against
   * @return true if collision is predicted
   * @return false if no collision detected
   */
  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // Skip if planning hasn't started yet
    if (local_data_.start_time_.toSec() < 1e9)
      return false;

    double my_traj_start_time = local_data_.start_time_.toSec();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();

    // Check overlapping time window
    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    // Sample both trajectories at 30ms intervals
    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;  // Collision detected
      }
    }

    return false;
  }

  /**
   * @brief Generate global trajectory through multiple waypoints
   * 
   * Creates a minimum-snap polynomial trajectory that passes through
   * the specified waypoints with given boundary conditions.
   * 
   * @param start_pos Start position
   * @param start_vel Start velocity
   * @param start_acc Start acceleration
   * @param waypoints Intermediate waypoints
   * @param end_vel End velocity
   * @param end_acc End acceleration
   * @return true if trajectory generation succeeded
   */
  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // Build point sequence: start + waypoints
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    // Calculate total path length
    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // Insert intermediate points if segments are too long
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);  // Threshold based on total length

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // Build position matrix for min-snap optimization
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    // Allocate time for each segment based on max velocity
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    // Give more time at start and end for acceleration/deceleration
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // Generate minimum-snap trajectory
    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  /**
   * @brief Generate simple global trajectory from start to end
   * 
   * Creates a minimum-snap polynomial trajectory between two points.
   * Automatically inserts intermediate points if distance is large.
   * 
   * @param start_pos Start position
   * @param start_vel Start velocity
   * @param start_acc Start acceleration
   * @param end_pos End position
   * @param end_vel End velocity
   * @param end_acc End acceleration
   * @return true if trajectory generation succeeded
   */
  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // Insert intermediate points if distance > 4m
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // Build position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    // Allocate time based on max velocity
    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // Generate trajectory
    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  /**
   * @brief Refine trajectory to satisfy dynamic constraints
   * 
   * Stretches the trajectory in time to reduce velocity/acceleration
   * violations, then re-optimizes to maintain smoothness.
   * 
   * @param traj Input B-spline trajectory
   * @param start_end_derivative Boundary velocity/acceleration
   * @param ratio Time stretch ratio
   * @param ts Time interval between control points
   * @param optimal_control_points Output refined control points
   * @return true if refinement succeeded
   */
  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts;

    // Lengthen time to reduce velocity/acceleration
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    // Sample reference points for fitness cost
    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    // Re-optimize with fitness constraint
    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  /**
   * @brief Update internal trajectory data structures
   * 
   * Stores the optimized trajectory and computes its derivatives.
   * 
   * @param position_traj Position B-spline
   * @param time_now Current timestamp
   */
  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  /**
   * @brief Reparameterize B-spline with new time allocation
   * 
   * Stretches the trajectory in time by the given ratio and resamples
   * to create new control points.
   * 
   * @param bspline Input B-spline to reparameterize
   * @param start_end_derivative Boundary conditions
   * @param ratio Time stretch ratio (>1 means slower)
   * @param ctrl_pts Output control points
   * @param dt Output time interval
   * @param time_inc Output time increase
   */
  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    // Stretch trajectory time
    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    // Resample at new time intervals
    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    
    // Convert back to B-spline control points
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
