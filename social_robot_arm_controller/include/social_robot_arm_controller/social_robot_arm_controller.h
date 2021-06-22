/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Hye-Jong KIM */

#ifndef SOCIAL_ROBOT_ARM_CONTROLLER_H
#define SOCIAL_ROBOT_ARM_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <std_msgs/Empty.h>
#include <boost/thread.hpp>
#include <queue>
#include <unistd.h>
#include <cmath>
#include <gazebo_msgs/LinkStates.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "social_robot_arm_msgs/SetJointPosition.h"
#include "social_robot_arm_msgs/SetKinematicsPose.h"
#include "social_robot_arm_msgs/SetDrawingTrajectory.h"
#include "social_robot_arm_msgs/SetActuatorState.h"
#include "social_robot_arm_msgs/GetJointPosition.h"
#include "social_robot_arm_msgs/GetKinematicsPose.h"
#include "social_robot_arm_msgs/ManipulatorState.h"
#include "social_robot_arm_msgs/SetMode.h"
#include "social_robot_arm_msgs/MotionControl.h"
#include "social_robot_arm_msgs/MotionSave.h"

#include "social_robot_arm_libs/social_robot_arm.h"

class SocialRobotArmController
{
 private:
  /*****************************************************************************
   ** ROS
  *****************************************************************************/
  // ROS Node Handle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  //ROS Parameters
  bool using_platform_;                                                     // unusing gazebo flag
  bool using_moveit_;                                                       // using moveit flag
  double control_loop_time_;                                                // used to robotis manipulator control process
  bool using_left_arm_;
  bool using_right_arm_;

  // direct Teaching
  pthread_t motion_play_thread_;
  bool motion_playing_;
  size_t motion_cnt_;
  size_t pose_cnt_;
  uint32_t repeat_cnt_;
  bool motion_break_flag_;
  bool motion_stop_flag_;
  bool motion_return_first_flag_;
  bool motion_return_motion_flag_;
  std::vector<social_robot_arm_msgs::MotionSave> motion_scenario_;

  // ROS Publishers
  std::vector<ros::Publisher> arm_states_pub_;                              // to GUI
  std::vector<ros::Publisher> arms_joint_states_pub_;                       // to GUI
  std::vector<ros::Publisher> gazebo_goal_joint_position_command_pub_;      // to Gazebo
  std::vector<ros::Publisher> hands_kinematics_pose_pub_;                   // to GUI
  ros::Publisher moveit_update_start_state_pub_;                            // to moveit
  ros::Publisher arm_mode_state_pub_;                                       // Gravity_compensation or not
  ros::Publisher waist_state_pub_;  // waist joint publisher

  // ROS Subscribers
  ros::Subscriber robotis_manipulator_option_sub_;                        // to robotis manipulator controller
  ros::Subscriber gazebo_joint_states_sub_;                               // From gazebo
  ros::Subscriber gazebo_link_states_sub_;                                // From gazebo
  ros::Subscriber display_planned_path_sub_;                              // to moveit controller
  ros::Subscriber move_group_goal_sub_;                                   // to moveit controller
  ros::Subscriber execute_traj_goal_sub_;                                 // to moveit controller
  ros::Subscriber change_arm_mode_sub_;                                   // chenge mode to Gravity_compensation or normal
  ros::Subscriber stop_moving_sub_;                                       // e-stop
  ros::Subscriber motion_control_sub_;                                    // for motion
  ros::Subscriber motion_save_sub_;                                       // for motion
  ros::Subscriber set_mode_sub_;                                          // for motion

  // ROS service Servers
  ros::ServiceServer goal_joint_space_path_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_pose_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_position_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_orientation_server_;
  ros::ServiceServer goal_joint_space_path_from_present_server_;
  ros::ServiceServer goal_task_space_path_server_;
  ros::ServiceServer goal_task_space_path_position_only_server_;
  ros::ServiceServer goal_task_space_path_orientation_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_position_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_orientation_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_server_;
  ros::ServiceServer goal_drawing_trajectory_server_;
  ros::ServiceServer goal_tool_control_server_;
  ros::ServiceServer set_actuator_state_server_;

  ros::ServiceServer get_moveit_joint_position_server_;
  ros::ServiceServer get_moveit_kinematics_pose_server_;
  ros::ServiceServer set_moveit_joint_position_server_;
  ros::ServiceServer set_moveit_kinematics_pose_server_;

  /*****************************************************************************
   ** Thread
  *****************************************************************************/
  // Thread parameter
  pthread_t timer_thread_;
  // flag
  bool timer_thread_state_;

  /*****************************************************************************
   ** Robotis Manipulator
  *****************************************************************************/
  // Arms
  SocialRobotLeftArm left_arm_;
  SocialRobotRightArm right_arm_;

  RobotisManipulator* current_controled_arm_;

  // Waist
  sensor_msgs::JointState waist_joints_;

  /*****************************************************************************
   ** MoveIt
  *****************************************************************************/
  // Move_group
  moveit::planning_interface::MoveGroupInterface* left_arm_move_group_;
  moveit::planning_interface::MoveGroupInterface* right_arm_move_group_;
  // planed trajectory
  trajectory_msgs::JointTrajectory moveit_joint_trajectory_;
  // Parameters
  std::string moveit_plan_arm_name_;
  double moveit_sampling_time_;
  bool moveit_plan_only_;
  bool moveit_plan_state_;

 public:
  SocialRobotArmController(std::string usb_port, std::string baud_rate);
  ~SocialRobotArmController();

  /*****************************************************************************
   ** get Private Parameter Function
  *****************************************************************************/
  double getControlLoopTime();

  /*****************************************************************************
   ** Init Functions
  *****************************************************************************/
  // init messages
  void initPublisher();
  void initSubscriber();
  void initServer();

  /*****************************************************************************
   ** Publish Functions
  *****************************************************************************/
  void publishCallback(const ros::TimerEvent&);

  // Publish every loop
  void publishArmStates();
  void publishArmsJointStates();                    // using platform
  void publishGazeboGoalJointPositionCommand();     // using gazebo
  void publishHandsKinematicsPose();
  void publishModeStates();

  // Publish when moving
  void publishMoveitUpdateStartState();             // using moveit

  /*****************************************************************************
   ** Subscriber Callback Functions
  *****************************************************************************/
  // robocare waist and neck
  void waistCallback(const sensor_msgs::JointState::ConstPtr &msg);

  // robotis manipulator
  void changeModeCallback(const std_msgs::Empty::ConstPtr &msg);
  void stopMoveCallback(const std_msgs::Empty::ConstPtr &msg);
  void robotisManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg);

  // moveit
  void displayPlannedPathCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);
  void moveGroupActionGoalCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg);
  void executeTrajectoryActionGoalCallback(const moveit_msgs::ExecuteTrajectoryActionGoal::ConstPtr &msg);
  void gazeboJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void gazeboLinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg);

  // Motion
  void MotionControlCallback(const social_robot_arm_msgs::MotionControl::ConstPtr &msg);
  void MotionSaveCallback(const social_robot_arm_msgs::MotionSave::ConstPtr &msg);
  void SetModeCallback(const social_robot_arm_msgs::SetMode::ConstPtr &msg);
  /*****************************************************************************
   ** Sevice Server Callback Functions
  *****************************************************************************/
  // robotis manipulator
  bool goalJointSpacePathCallback(social_robot_arm_msgs::SetJointPosition::Request  &req,
                                  social_robot_arm_msgs::SetJointPosition::Response &res);

  bool goalJointSpacePathToKinematicsPoseCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                  social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathToKinematicsPositionCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                  social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathToKinematicsOrientationCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                  social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathFromPresentCallback(social_robot_arm_msgs::SetJointPosition::Request  &req,
                                             social_robot_arm_msgs::SetJointPosition::Response &res);

  bool goalTaskSpacePathCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                 social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathPositionOnlyCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                             social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathOrientationOnlyCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentPositionOnlyCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                        social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentOrientationOnlyCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                           social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                            social_robot_arm_msgs::SetKinematicsPose::Response &res);

  bool goalDrawingTrajectoryCallback(social_robot_arm_msgs::SetDrawingTrajectory::Request  &req,
                                     social_robot_arm_msgs::SetDrawingTrajectory::Response &res);

  bool goalToolControlCallback(social_robot_arm_msgs::SetJointPosition::Request  &req,
                               social_robot_arm_msgs::SetJointPosition::Response &res);

  bool setActuatorStateCallback(social_robot_arm_msgs::SetActuatorState::Request  &req,
                                social_robot_arm_msgs::SetActuatorState::Response &res);

  // moveit
  bool getMoveitJointPositionCallback(social_robot_arm_msgs::GetJointPosition::Request &req,
                                   social_robot_arm_msgs::GetJointPosition::Response &res);

  bool getMoveitKinematicsPoseCallback(social_robot_arm_msgs::GetKinematicsPose::Request &req,
                                    social_robot_arm_msgs::GetKinematicsPose::Response &res);

  bool setMoveitJointPositionCallback(social_robot_arm_msgs::SetJointPosition::Request &req,
                                   social_robot_arm_msgs::SetJointPosition::Response &res);

  bool setMoveitKinematicsPoseCallback(social_robot_arm_msgs::SetKinematicsPose::Request &req,
                                    social_robot_arm_msgs::SetKinematicsPose::Response &res);

  /*****************************************************************************
   ** Moveit CalcPlannedPath Functions
  *****************************************************************************/
  bool calcPlannedPath(const std::string planning_group, social_robot_arm_msgs::JointPosition msg);
  bool calcPlannedPath(const std::string planning_group, social_robot_arm_msgs::KinematicsPose msg);

  /*****************************************************************************
   ** Control Process Functions
  *****************************************************************************/
  void moveitProcess(double present_time);
  void robotisProcess(double present_time);
  /*****************************************************************************
   ** Thread Functions
  *****************************************************************************/
  void startTimerThread();
  static void *timerThread(void *param);
  /*****************************************************************************
   ** Motion Play Functions
  *****************************************************************************/
  double motionPlay();
  /*****************************************************************************
   ** Motion Thread Functions
  *****************************************************************************/
  void startMotionPlayThread();
  void stopMotionPlayThread();
  static void *motionPlayThread(void *param);
};

#endif //SOCIAL_ROBOT_ARM_CONTROLLER_H
