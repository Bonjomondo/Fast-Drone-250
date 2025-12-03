/**
 * @file ego_planner_node.cpp
 * @brief Main entry point for the Ego-Planner trajectory planning node
 * 
 * This node implements a real-time trajectory planning system for quadrotor UAVs.
 * It uses a finite state machine (FSM) to manage planning states and generates
 * collision-free, dynamically feasible trajectories using B-spline optimization.
 * 
 * @author FAST-Lab, Zhejiang University
 * @note Part of the Fast-Drone-250 autonomous aerial robot project
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

/**
 * @brief Main function for the ego_planner_node
 * 
 * Initializes the ROS node, creates an instance of the EGOReplanFSM
 * (finite state machine for replanning), and enters the ROS event loop.
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Exit status (0 for success)
 */
int main(int argc, char **argv)
{
  // Initialize the ROS node with the name "ego_planner_node"
  ros::init(argc, argv, "ego_planner_node");
  
  // Create a private node handle for parameter access
  // The "~" prefix means parameters will be in the node's private namespace
  ros::NodeHandle nh("~");

  // Create the replanning finite state machine instance
  // This manages all planning states and trajectory generation
  EGOReplanFSM rebo_replan;

  // Initialize the FSM with ROS parameters and set up all subscribers/publishers
  rebo_replan.init(nh);

  // Enter the ROS event loop
  // This will process callbacks until the node is shut down
  ros::spin();

  return 0;
}

// #include <ros/ros.h>
// #include <csignal>
// #include <visualization_msgs/Marker.h>

// #include <plan_manage/ego_replan_fsm.h>

// using namespace ego_planner;

// void SignalHandler(int signal) {
//   if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
//     ros::shutdown();
//   }
// }

// int main(int argc, char **argv) {

//   signal(SIGINT, SignalHandler);
//   signal(SIGTERM,SignalHandler);

//   ros::init(argc, argv, "ego_planner_node", ros::init_options::NoSigintHandler);
//   ros::NodeHandle nh("~");

//   EGOReplanFSM rebo_replan;

//   rebo_replan.init(nh);

//   // ros::Duration(1.0).sleep();
//   ros::AsyncSpinner async_spinner(4);
//   async_spinner.start();
//   ros::waitForShutdown();

//   return 0;
// }