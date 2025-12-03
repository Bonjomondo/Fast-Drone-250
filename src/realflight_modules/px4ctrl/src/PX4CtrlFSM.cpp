/**
 * @file PX4CtrlFSM.cpp
 * @brief Finite State Machine for PX4 flight control
 * 
 * This file implements the core state machine that manages the quadrotor
 * flight modes including manual control, hover, command control, auto
 * takeoff, and auto landing. It bridges the trajectory planner with
 * the PX4 flight controller.
 * 
 * @author FAST-Lab, Zhejiang University
 */

#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

/**
 * @brief Constructor - initialize FSM with parameters and controller
 * 
 * @param param_ Flight control parameters
 * @param controller_ Linear control algorithm instance
 */
PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_) : param(param_), controller(controller_)
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}

/**
 * @brief Main FSM process function - called at high frequency
 * 
 * This implements a multi-level state machine for flight control:
 * 
 *       system start
 *             |
 *             v
 *     ----- > MANUAL_CTRL <-----------------
 *     |         ^   |    \                 |
 *     |         |   |     \                |
 *     |         |   |      > AUTO_TAKEOFF  |
 *     |         |   |        /             |
 *     |         |   |       /              |
 *     |         |   v     /                |
 *     |       AUTO_HOVER <                 |
 *     |         ^   |  \  \                |
 *     |         |   |   \  \               |
 *     |         |   |    > AUTO_LAND -------
 *     |         |   |
 *     |         |   v
 *     -------- CMD_CTRL
 * 
 * The FSM handles state transitions based on:
 * - RC (remote control) commands
 * - Odometry availability
 * - Trajectory commands
 * - Safety conditions
 */
void PX4CtrlFSM::process()
{

	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	Desired_State_t des(odom_data);
	bool rotor_low_speed_during_land = false;

	// STEP1: State machine runs - handle state transitions and set desired state
	switch (state)
	{
	case MANUAL_CTRL:
	{
		// MANUAL_CTRL: RC has full control, waiting for hover mode activation
		if (rc_data.enter_hover_mode) // Try to jump to AUTO_HOVER
		{
			// Safety checks before entering autonomous mode
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). You are sending commands before toggling into AUTO_HOVER, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 3.0)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_HOVER(L2). Odom_Vel=%fm/s, which seems that the locolization module goes wrong!", odom_data.v.norm());
				break;
			}

			// Transition to AUTO_HOVER
			state = AUTO_HOVER;
			controller.resetThrustMapping();  // Reset thrust model estimation
			set_hov_with_odom();              // Set current position as hover target
			toggle_offboard_mode(true);       // Enable OFFBOARD mode in PX4

			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)\033[32m");
		}
		else if (param.takeoff_land.enable && takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF)
		{
			// AUTO_TAKEOFF trigger received
			// Safety checks for takeoff
			if (!odom_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
				break;
			}
			if (cmd_is_received(now_time))
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands before toggling into AUTO_TAKEOFF, which is not allowed. Stop sending commands now!");
				break;
			}
			if (odom_data.v.norm() > 0.1)
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
				break;
			}
			if (!get_landed())
			{
				ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now!");
				break;
			}
			
			// Check RC is in correct position for auto takeoff
			if (rc_is_received(now_time))
			{
				if (!rc_data.is_hover_mode || !rc_data.is_command_mode || !rc_data.check_centered())
				{
					ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
					while (ros::ok())
					{
						ros::Duration(0.01).sleep();
						ros::spinOnce();
						if (rc_data.is_hover_mode && rc_data.is_command_mode && rc_data.check_centered())
						{
							ROS_INFO("\033[32m[px4ctrl] OK, you can takeoff again.\033[32m");
							break;
						}
					}
					break;
				}
			}

			// Initialize takeoff sequence
			state = AUTO_TAKEOFF;
			controller.resetThrustMapping();
			set_start_pose_for_takeoff_land(odom_data);
			toggle_offboard_mode(true);
			
			// Wait for mode switch to take effect
			for (int i = 0; i < 10 && ros::ok(); ++i)
			{
				ros::Duration(0.01).sleep();
				ros::spinOnce();
			}
			
			// Auto arm if enabled
			if (param.takeoff_land.enable_auto_arm)
			{
				toggle_arm_disarm(true);
			}
			takeoff_land.toggle_takeoff_land_time = now_time;

			ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
		}

		// Handle FCU reboot request (useful when EKF fails)
		if (rc_data.toggle_reboot)
		{
			if (state_data.current_state.armed)
			{
				ROS_ERROR("[px4ctrl] Reject reboot! Disarm the drone first!");
				break;
			}
			reboot_FCU();
		}

		break;
	}

	case AUTO_HOVER:
	{
		// AUTO_HOVER: Drone holds position, can accept commands or land
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			// Safety fallback to MANUAL_CTRL
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] AUTO_HOVER(L2) --> MANUAL_CTRL(L1)");
		}
		else if (rc_data.is_command_mode && cmd_is_received(now_time))
		{
			// Transition to command control when commands are being received
			if (state_data.current_state.mode == "OFFBOARD")
			{
				state = CMD_CTRL;
				des = get_cmd_des();
				ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			}
		}
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			// Initiate landing sequence
			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);

			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");
		}
		else
		{
			// Normal hover: apply RC adjustments to hover position
			set_hov_with_rc();
			des = get_hover_des();
			
			// Handle trigger signal for starting mission
			if ((rc_data.enter_command_mode) ||
				(takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second))
			{
				takeoff_land.delay_trigger.first = false;
				publish_trigger(odom_data.msg);
				ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
			}
		}

		break;
	}

	case CMD_CTRL:
	{
		// CMD_CTRL: Following trajectory commands from planner
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			// Safety fallback to MANUAL_CTRL
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] From CMD_CTRL(L3) to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode || !cmd_is_received(now_time))
		{
			// Return to hover when commands stop
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		else
		{
			// Normal operation: follow trajectory commands
			des = get_cmd_des();
		}

		// Reject landing during command control
		if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER. \
					Stop sending control commands for longer than %fs to let px4ctrl return to AUTO_HOVER first.",
					  param.msg_timeout.cmd);
		}

		break;
	}

	case AUTO_TAKEOFF:
	{
		// AUTO_TAKEOFF: Automated takeoff sequence
		if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME)
		{
			// Phase 1: Spin up motors before liftoff
			des = get_rotor_speed_up_des(now_time);
		}
		else if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height))
		{
			// Phase 2: Reached target height - transition to hover
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

			// Schedule trigger signal after delay
			takeoff_land.delay_trigger.first = true;
			takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
		}
		else
		{
			// Phase 1.5: Ascending to target height
			des = get_takeoff_land_des(param.takeoff_land.speed);
		}

		break;
	}

	case AUTO_LAND:
	{
		// AUTO_LAND: Automated landing sequence
		if (!rc_data.is_hover_mode || !odom_is_received(now_time))
		{
			// Safety fallback
			state = MANUAL_CTRL;
			toggle_offboard_mode(false);

			ROS_WARN("[px4ctrl] From AUTO_LAND to MANUAL_CTRL(L1)!");
		}
		else if (!rc_data.is_command_mode)
		{
			// Abort landing, return to hover
			state = AUTO_HOVER;
			set_hov_with_odom();
			des = get_hover_des();
			ROS_INFO("[px4ctrl] From AUTO_LAND to AUTO_HOVER(L2)!");
		}
		else if (!get_landed())
		{
			// Still descending
			des = get_takeoff_land_des(-param.takeoff_land.speed);
		}
		else
		{
			// Landed - idle motors and wait for disarm
			rotor_low_speed_during_land = true;

			static bool print_once_flag = true;
			if (print_once_flag)
			{
				ROS_INFO("\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
				print_once_flag = false;
			}

			// Check if PX4 confirms landing
			if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
			{
				static double last_trial_time = 0;
				if (now_time.toSec() - last_trial_time > 1.0)
				{
					if (toggle_arm_disarm(false))  // Disarm
					{
						print_once_flag = true;
						state = MANUAL_CTRL;
						toggle_offboard_mode(false);
						ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
					}

					last_trial_time = now_time.toSec();
				}
			}
		}

		break;
	}

	default:
		break;
	}

	// STEP2: Estimate thrust model during stable flight
	if (state == AUTO_HOVER || state == CMD_CTRL)
	{
		controller.estimateThrustModel(imu_data.a, param);
	}

	// STEP3: Compute control output
	if (rotor_low_speed_during_land)
	{
		// Idle motors during landing touchdown
		motors_idling(imu_data, u);
	}
	else
	{
		// Normal control calculation
		debug_msg = controller.calculateControl(des, odom_data, imu_data, u);
		debug_msg.header.stamp = now_time;
		debug_pub.publish(debug_msg);
	}

	// STEP4: Send control commands to PX4
	if (param.use_bodyrate_ctrl)
	{
		publish_bodyrate_ctrl(u, now_time);
	}
	else
	{
		publish_attitude_ctrl(u, now_time);
	}

	// STEP5: Detect if the drone has landed
	land_detector(state, des, odom_data);

	// STEP6: Clear one-shot flags
	rc_data.enter_hover_mode = false;
	rc_data.enter_command_mode = false;
	rc_data.toggle_reboot = false;
	takeoff_land_data.triggered = false;
}

/**
 * @brief Idle motors at low throttle
 * 
 * Used during landing to keep motors spinning but produce minimal thrust.
 * 
 * @param imu IMU data for current orientation
 * @param u Output control commands
 */
void PX4CtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
	u.q = imu.q;                      // Maintain current orientation
	u.bodyrates = Eigen::Vector3d::Zero();
	u.thrust = 0.04;                   // Minimum throttle to keep motors spinning
}

/**
 * @brief Detect if the drone has landed
 * 
 * Uses a simple heuristic based on:
 * 1. Desired position below current position
 * 2. Low velocity
 * 3. Both conditions maintained for a period of time
 * 
 * @param state Current FSM state
 * @param des Desired state
 * @param odom Current odometry
 */
void PX4CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
	static State_t last_state = State_t::MANUAL_CTRL;
	
	// Reset landed flag when taking off
	if (last_state == State_t::MANUAL_CTRL && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
	{
		takeoff_land.landed = false;
	}
	last_state = state;

	// Definitely landed if disarmed
	if (state == State_t::MANUAL_CTRL && !state_data.current_state.armed)
	{
		takeoff_land.landed = true;
		return;
	}

	// Land detection thresholds
	constexpr double POSITION_DEVIATION_C = -0.5;  // Target below actual by 0.5m
	constexpr double VELOCITY_THR_C = 0.1;         // Velocity under 0.1 m/s
	constexpr double TIME_KEEP_C = 3.0;            // Must hold for 3 seconds

	static ros::Time time_C12_reached;
	static bool is_last_C12_satisfy;
	
	if (takeoff_land.landed)
	{
		time_C12_reached = ros::Time::now();
		is_last_C12_satisfy = false;
	}
	else
	{
		bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
		
		if (C12_satisfy && !is_last_C12_satisfy)
		{
			time_C12_reached = ros::Time::now();
		}
		else if (C12_satisfy && is_last_C12_satisfy)
		{
			if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C)
			{
				takeoff_land.landed = true;
			}
		}

		is_last_C12_satisfy = C12_satisfy;
	}
}

/**
 * @brief Get desired state for hovering
 * 
 * @return Desired_State_t Hover state with zero velocity/acceleration
 */
Desired_State_t PX4CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

/**
 * @brief Get desired state from trajectory command
 * 
 * @return Desired_State_t State from planner command
 */
Desired_State_t PX4CtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	des.yaw = cmd_data.yaw;
	des.yaw_rate = cmd_data.yaw_rate;

	return des;
}

/**
 * @brief Get desired state for motor spin-up phase
 * 
 * Generates a smooth acceleration curve that ramps up motors
 * before actual liftoff.
 * 
 * @param now Current time
 * @return Desired_State_t State with negative z acceleration
 */
Desired_State_t PX4CtrlFSM::get_rotor_speed_up_des(const ros::Time now)
{
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
	
	// Exponential curve for smooth motor ramp-up
	// Starts very negative, approaches 0 at MOTORS_SPEEDUP_TIME
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0;
	
	if (des_a_z > 0.1)
	{
		ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

/**
 * @brief Get desired state for takeoff/landing trajectory
 * 
 * @param speed Vertical speed (positive=up, negative=down)
 * @return Desired_State_t Vertical motion state
 */
Desired_State_t PX4CtrlFSM::get_takeoff_land_des(const double speed)
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - 
	                 (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0);

	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

/**
 * @brief Set hover pose from current odometry
 */
void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);

	last_set_hover_pose_time = ros::Time::now();
}

/**
 * @brief Update hover pose based on RC stick inputs
 * 
 * Allows pilot to adjust hover position using RC sticks
 * while in AUTO_HOVER mode.
 */
void PX4CtrlFSM::set_hov_with_rc()
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time).toSec();
	last_set_hover_pose_time = now;

	// Integrate RC inputs to move hover position
	hover_pose(0) += rc_data.ch[1] * param.max_manual_vel * delta_t * (param.rc_reverse.pitch ? 1 : -1);
	hover_pose(1) += rc_data.ch[0] * param.max_manual_vel * delta_t * (param.rc_reverse.roll ? 1 : -1);
	hover_pose(2) += rc_data.ch[2] * param.max_manual_vel * delta_t * (param.rc_reverse.throttle ? 1 : -1);
	hover_pose(3) += rc_data.ch[3] * param.max_manual_vel * delta_t * (param.rc_reverse.yaw ? 1 : -1);

	// Limit minimum height
	if (hover_pose(2) < -0.3)
		hover_pose(2) = -0.3;
}

/**
 * @brief Store start pose for takeoff/landing
 * 
 * @param odom Current odometry
 */
void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);

	takeoff_land.toggle_takeoff_land_time = ros::Time::now();
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void PX4CtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();

	msg.thrust = u.thrust;

	ctrl_FCU_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}

bool PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{
	mavros_msgs::SetMode offb_set_mode;

	if (on_off)
	{
		state_data.state_before_offboard = state_data.current_state;
		if (state_data.state_before_offboard.mode == "OFFBOARD") // Not allowed
			state_data.state_before_offboard.mode = "MANUAL";

		offb_set_mode.request.custom_mode = "OFFBOARD";
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else
	{
		offb_set_mode.request.custom_mode = state_data.state_before_offboard.mode;
		if (!(set_FCU_mode_srv.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Exit OFFBOARD rejected by PX4!");
			return false;
		}
	}

	return true;

	// if (param.print_dbg)
	// 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool PX4CtrlFSM::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

	return true;
}

void PX4CtrlFSM::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO("Reboot FCU");

	// if (param.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}
