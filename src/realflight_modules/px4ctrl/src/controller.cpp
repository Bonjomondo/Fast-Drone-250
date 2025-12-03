/**
 * @file controller.cpp
 * @brief Linear position controller for quadrotor UAV
 * 
 * Implements a PD position controller that converts position/velocity/acceleration
 * commands into desired attitude and thrust commands for the flight controller.
 * Includes online thrust model estimation using recursive least squares.
 * 
 * @author FAST-Lab, Zhejiang University
 */

#include "controller.h"

using namespace std;

/**
 * @brief Extract yaw angle from quaternion
 * 
 * @param q Input quaternion
 * @return double Yaw angle in radians
 */
double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

/**
 * @brief Constructor - initialize controller with parameters
 * 
 * @param param Controller parameters including gains and physical properties
 */
LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/**
 * @brief Main control calculation function
 * 
 * Implements a PD controller that:
 * 1. Computes desired acceleration from position and velocity errors
 * 2. Converts desired acceleration to thrust magnitude
 * 3. Computes desired attitude (roll, pitch) from horizontal acceleration
 * 
 * Control Law:
 *   a_des = a_ff + Kp*(p_des - p) + Kv*(v_des - v) + g
 *   thrust = |a_des| / thr2acc
 *   roll = (a_x * sin(yaw) - a_y * cos(yaw)) / g
 *   pitch = (a_x * cos(yaw) + a_y * sin(yaw)) / g
 * 
 * @param des Desired state (position, velocity, acceleration, yaw)
 * @param odom Current odometry (position, velocity, orientation)
 * @param imu IMU data (orientation, angular velocity)
 * @param u Output control commands (attitude quaternion, thrust)
 * @return quadrotor_msgs::Px4ctrlDebug Debug message with internal states
 */
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
    // Compute desired acceleration using PD control + feedforward
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp, Kv;
    
    // Load PD gains from parameters
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
    
    // PD control law with feedforward acceleration
    // des_acc = a_ff + Kv*(v_des - v) + Kp*(p_des - p)
    des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
    
    // Add gravity compensation (pointing upward in world frame)
    des_acc += Eigen::Vector3d(0, 0, param_.gra);

    // Convert desired acceleration to thrust command
    u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
    
    // Compute desired attitude from horizontal acceleration
    double roll, pitch, yaw, yaw_imu;
    double yaw_odom = fromQuaternion2yaw(odom.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);
    
    // Decompose horizontal acceleration into body frame roll/pitch
    // These formulas come from small angle approximation of rotation matrix
    roll = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
    pitch = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
    
    yaw_imu = fromQuaternion2yaw(imu.q);
    
    // Construct desired attitude quaternion (ZYX Euler convention)
    Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    
    // Transform from odometry frame to IMU frame
    u.q = imu.q * odom.q.inverse() * q;

  // Fill debug message
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Store thrust-time pair for model estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();  // Keep only recent 100 samples
  }
  return debug_msg_;
}

/**
 * @brief Convert desired acceleration to throttle percentage
 * 
 * Uses the estimated thrust-to-acceleration ratio (thr2acc) to convert
 * the desired vertical acceleration into throttle command.
 * 
 * @param des_acc Desired acceleration vector
 * @return double Throttle percentage (0-1)
 */
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  // Simple linear model: a_z = thr2acc * throttle
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

/**
 * @brief Online estimation of thrust-to-acceleration model
 * 
 * Uses Recursive Least Squares (RLS) with vanishing memory to estimate
 * the relationship between throttle command and vertical acceleration.
 * This allows the controller to adapt to different battery levels and
 * payload conditions.
 * 
 * Model: a_z = thr2acc * throttle
 * 
 * @param est_a Estimated acceleration from IMU
 * @param param Controller parameters
 * @return true if estimation was updated
 * @return false if no valid data available
 */
bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Find thrust command from 35-45ms ago (accounts for system delay)
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    
    if (time_passed > 0.045) // Too old, discard
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // Too recent, wait
    {
      return false;
    }

    /***********************************************************/
    /* Recursive Least Squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr2acc * thr */
    /***********************************/
    // RLS update equations
    double gamma = 1 / (rho2_ + thr * P_ * thr);  // Gain normalization
    double K = gamma * P_ * thr;                   // Kalman gain
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);  // Update estimate
    P_ = (1 - K * thr) * P_ / rho2_;              // Update covariance

    return true;
  }
  return false;
}

/**
 * @brief Reset thrust model to initial estimate
 * 
 * Called at controller startup or when switching control modes.
 * Initializes thr2acc based on hover throttle and gravity.
 */
void 
LinearControl::resetThrustMapping(void)
{
  // Initial estimate: at hover throttle, acceleration equals gravity
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;  // High initial uncertainty for fast adaptation
}







