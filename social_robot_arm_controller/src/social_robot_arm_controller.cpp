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

#include "social_robot_arm_controller/social_robot_arm_controller.h"

/*****************************************************************************
 ** Main Functions
*****************************************************************************/

int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "social_robot_arm_controller");
  ros::NodeHandle node_handle("");

  // Dynamixel init parameter setting
  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc < 3)
  {
    log::error("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    usb_port = argv[1];
    baud_rate = argv[2];
  }

  // Declare Contorller
  SocialRobotArmController social_robot_arm_controller(usb_port, baud_rate);

  // Massege init
  social_robot_arm_controller.initPublisher();
  social_robot_arm_controller.initSubscriber();
  social_robot_arm_controller.initServer();
  log::println("[Info] Masseges Init Done.");

  //P Thread timer start for Controller
  social_robot_arm_controller.startTimerThread();

  // ROS Timer Start for Publishing Messages
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(social_robot_arm_controller.getControlLoopTime()), &SocialRobotArmController::publishCallback, &social_robot_arm_controller);

  // Main loop for Subscribing Messages
  double subscribe_frequency = 1000 / social_robot_arm_controller.getControlLoopTime();       //Hz
  ros::Rate loop_rate(subscribe_frequency);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

/*****************************************************************************
 ** SocialRobotArmController
*****************************************************************************/

SocialRobotArmController::SocialRobotArmController(std::string usb_port, std::string baud_rate)
    :node_handle_(""),
     priv_node_handle_("~"),
     using_platform_(false),
     using_moveit_(false),
     control_loop_time_(0.010),
     timer_thread_state_(false),
     motion_playing_(false),
     moveit_plan_arm_name_(""),
     moveit_sampling_time_(0.050),
     moveit_plan_only_(true),
     moveit_plan_state_(false),
     motion_cnt_(0),
     repeat_cnt_(0),
     pose_cnt_(0),
     motion_stop_flag_(false),
     motion_break_flag_(false)
{

  // ROS parameter Init
  using_platform_       = priv_node_handle_.param<bool>("use_platform", false);
  using_moveit_         = priv_node_handle_.param<bool>("use_moveit", false);
  control_loop_time_    = priv_node_handle_.param<double>("control_loop_time", 0.010);
  moveit_sampling_time_ = priv_node_handle_.param<double>("moveit_sample_duration", 0.050);
  using_left_arm_       = priv_node_handle_.param<bool>("use_left_arm", true);
  using_right_arm_      = priv_node_handle_.param<bool>("use_right_arm", true);

  std::string left_arm_planning_group_name = priv_node_handle_.param<std::string>("left_arm_planning_group_name", "left_arm");
  std::string right_arm_planning_group_name = priv_node_handle_.param<std::string>("right_arm_planning_group_name", "right_arm");

  // set MoveIt move_group
  if (using_moveit_ == true)
  {
    log::info("Ready to control ");
    left_arm_move_group_ = new moveit::planning_interface::MoveGroupInterface(left_arm_planning_group_name);
    log::info(" " + left_arm_planning_group_name);
    right_arm_move_group_ = new moveit::planning_interface::MoveGroupInterface(right_arm_planning_group_name);
    log::info(" " + right_arm_planning_group_name);
    log::info(" groups");
  }

  // set Robotis Manipulator libs
  if(using_platform_)
  {
    left_arm_.init(using_left_arm_, usb_port, baud_rate, control_loop_time_);
    right_arm_.init(using_right_arm_, usb_port, baud_rate, control_loop_time_);
  }
  else
  {
    left_arm_.init(false, usb_port, baud_rate, control_loop_time_);
    right_arm_.init(false, usb_port, baud_rate, control_loop_time_);
  }

  // set gazebo flag
  if (using_platform_ == true)        log::info("Succeeded to init " + priv_node_handle_.getNamespace());
  else if (using_platform_ == false)  log::info("Ready to simulate " +  priv_node_handle_.getNamespace() + " on Gazebo");
}

SocialRobotArmController::~SocialRobotArmController()
{
  stopMotionPlayThread();
  left_arm_.stopMoving();
  right_arm_.stopMoving();
  timer_thread_state_ = false;
  pthread_join(timer_thread_, NULL);                      // Wait for the thread associated with thread_p to complete
  log::info("Shutdown the Social Robot ARM Controller");
  left_arm_.disableAllActuator();
  right_arm_.disableAllActuator();

  ros::shutdown();
}

/*****************************************************************************
 ** get Private Parameter Function
*****************************************************************************/
double SocialRobotArmController::getControlLoopTime()
{
  return control_loop_time_;
}

/*****************************************************************************
 ** Init Functions
*****************************************************************************/
void SocialRobotArmController::initPublisher()
{

  std::vector<std::string> joint_names;
  std::vector<std::string> end_effector_names;

  // State Publisher
  arm_states_pub_.push_back(priv_node_handle_.advertise<social_robot_arm_msgs::ManipulatorState>("left_arm/states", 10));
  // save all component names
  auto left_arm_joint_names = left_arm_.getManipulator()->getAllActiveJointComponentName();
  joint_names.reserve(joint_names.size() + left_arm_joint_names.size());
  joint_names.insert(joint_names.end(), left_arm_joint_names.begin(), left_arm_joint_names.end());
  auto left_arm_end_effector_names = left_arm_.getManipulator()->getAllToolComponentName();
  end_effector_names.reserve(end_effector_names.size() + left_arm_end_effector_names.size());
  end_effector_names.insert(end_effector_names.end(), left_arm_end_effector_names.begin(), left_arm_end_effector_names.end());

  // State Publisher
  arm_states_pub_.push_back(priv_node_handle_.advertise<social_robot_arm_msgs::ManipulatorState>("right_arm/states", 10));
  // save all component names
  auto right_arm_joint_names = right_arm_.getManipulator()->getAllActiveJointComponentName();
  joint_names.reserve(joint_names.size() + right_arm_joint_names.size());
  joint_names.insert(joint_names.end(), right_arm_joint_names.begin(), right_arm_joint_names.end());
  auto right_arm_end_effector_names = right_arm_.getManipulator()->getAllToolComponentName();
  end_effector_names.reserve(end_effector_names.size() + right_arm_end_effector_names.size());
  end_effector_names.insert(end_effector_names.end(), right_arm_end_effector_names.begin(), right_arm_end_effector_names.end());

  // Kinematic Pose Publisher
  for(auto const& name:end_effector_names)
  {
    ros::Publisher temp_publisher;
    temp_publisher = priv_node_handle_.advertise<social_robot_arm_msgs::KinematicsPose>(name + "/kinematics_pose", 10);
    hands_kinematics_pose_pub_.push_back(temp_publisher);
  }
  // Waist Joint Publisher
  waist_state_pub_ = priv_node_handle_.advertise<sensor_msgs::JointState>("/cmd_pos", 10);

  // To GUI
  arms_joint_states_pub_.push_back(priv_node_handle_.advertise<sensor_msgs::JointState>("left_arm/joint_states", 10));
  arms_joint_states_pub_.push_back(priv_node_handle_.advertise<sensor_msgs::JointState>("right_arm/joint_states", 10));

  // use gazebo
  if(!using_platform_)
  {
    std::vector<std::string> gazebo_joints_name;
    gazebo_joints_name.reserve(gazebo_joints_name.size() + joint_names.size() + end_effector_names.size());
    gazebo_joints_name.insert(gazebo_joints_name.end(), joint_names.begin(), joint_names.end());
    gazebo_joints_name.insert(gazebo_joints_name.end(), end_effector_names.begin(), end_effector_names.end());
    for (auto const& name:gazebo_joints_name)
    {
      ros::Publisher temp_publisher;
      log::println("[Info] Init gazebo pub : " + name);
      temp_publisher = priv_node_handle_.advertise<std_msgs::Float64>(name + "_position/command", 10);
      gazebo_goal_joint_position_command_pub_.push_back(temp_publisher);
    }
    log::println("[Info] Init gazebo pub DONE");
  }
  else
  {
    arms_joint_states_pub_.push_back(priv_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10));
  }

  //  MoveIt update start state Publisher
  if (using_moveit_ == true)
  {
    moveit_update_start_state_pub_ = node_handle_.advertise<std_msgs::Empty>("rviz/moveit/update_start_state", 10);
  }

  // mode pub
  arm_mode_state_pub_ = priv_node_handle_.advertise<std_msgs::String>("mode_state",10);
}

void SocialRobotArmController::initSubscriber()
{
  // change mode sub
  change_arm_mode_sub_ = priv_node_handle_.subscribe("change_mode", 1, &SocialRobotArmController::changeModeCallback, this);
  // stop move sub
  stop_moving_sub_ = priv_node_handle_.subscribe("e_stop", 1, &SocialRobotArmController::stopMoveCallback, this);
  // MoveIt subscribers
  if (using_moveit_ == true)
  {
    display_planned_path_sub_ = node_handle_.subscribe("/move_group/display_planned_path", 100,
                                                       &SocialRobotArmController::displayPlannedPathCallback, this);
    move_group_goal_sub_ = node_handle_.subscribe("/move_group/goal", 100,
                                                       &SocialRobotArmController::moveGroupActionGoalCallback, this);
    execute_traj_goal_sub_ = node_handle_.subscribe("/execute_trajectory/goal", 100,
                                                       &SocialRobotArmController::executeTrajectoryActionGoalCallback, this);
  }
  if(!using_platform_)
  {
    gazebo_joint_states_sub_ = priv_node_handle_.subscribe("joint_states", 10,
                                                           &SocialRobotArmController::gazeboJointStatesCallback, this);
    gazebo_link_states_sub_ = priv_node_handle_.subscribe("link_states", 10, &SocialRobotArmController::gazeboLinkStatesCallback, this);
  }

  // Motion
  motion_control_sub_ = priv_node_handle_.subscribe("motion_control", 10, &SocialRobotArmController::MotionControlCallback, this);
  motion_save_sub_ = priv_node_handle_.subscribe("motion_save", 10, &SocialRobotArmController::MotionSaveCallback, this);
  set_mode_sub_ = priv_node_handle_.subscribe("set_mode", 10, &SocialRobotArmController::SetModeCallback, this);
}

void SocialRobotArmController::initServer()
{
  // Subscriber for Robotis Manipulator
  // Join Space
  goal_joint_space_path_server_                            = priv_node_handle_.advertiseService("goal_joint_space_path", &SocialRobotArmController::goalJointSpacePathCallback, this);
  goal_joint_space_path_to_kinematics_pose_server_         = priv_node_handle_.advertiseService("goal_joint_space_path_to_kinematics_pose", &SocialRobotArmController::goalJointSpacePathToKinematicsPoseCallback, this);
  goal_joint_space_path_to_kinematics_position_server_     = priv_node_handle_.advertiseService("goal_joint_space_path_to_kinematics_position", &SocialRobotArmController::goalJointSpacePathToKinematicsPositionCallback, this);
  goal_joint_space_path_to_kinematics_orientation_server_  = priv_node_handle_.advertiseService("goal_joint_space_path_to_kinematics_orientation", &SocialRobotArmController::goalJointSpacePathToKinematicsOrientationCallback, this);
  goal_joint_space_path_from_present_server_               = priv_node_handle_.advertiseService("goal_joint_space_path_from_present", &SocialRobotArmController::goalJointSpacePathFromPresentCallback, this);
  // Task Space
  goal_task_space_path_server_                             = priv_node_handle_.advertiseService("goal_task_space_path", &SocialRobotArmController::goalTaskSpacePathCallback, this);
  goal_task_space_path_position_only_server_               = priv_node_handle_.advertiseService("goal_task_space_path_position_only", &SocialRobotArmController::goalTaskSpacePathPositionOnlyCallback, this);
  goal_task_space_path_orientation_only_server_            = priv_node_handle_.advertiseService("goal_task_space_path_orientation_only", &SocialRobotArmController::goalTaskSpacePathOrientationOnlyCallback, this);
  goal_task_space_path_from_present_position_only_server_     = priv_node_handle_.advertiseService("goal_task_space_path_from_present_position_only", &SocialRobotArmController::goalTaskSpacePathFromPresentPositionOnlyCallback, this);
  goal_task_space_path_from_present_orientation_only_server_  = priv_node_handle_.advertiseService("goal_task_space_path_from_present_orientation_only", &SocialRobotArmController::goalTaskSpacePathFromPresentOrientationOnlyCallback, this);
  goal_task_space_path_from_present_server_                   = priv_node_handle_.advertiseService("goal_task_space_path_from_present", &SocialRobotArmController::goalTaskSpacePathFromPresentCallback, this);
  // Drawing
  goal_drawing_trajectory_server_   = priv_node_handle_.advertiseService("goal_drawing_trajectory", &SocialRobotArmController::goalDrawingTrajectoryCallback, this);
  // End-effector
  goal_tool_control_server_         = priv_node_handle_.advertiseService("goal_tool_control", &SocialRobotArmController::goalToolControlCallback, this);
  // Actuator states
  set_actuator_state_server_        = priv_node_handle_.advertiseService("set_actuator_state", &SocialRobotArmController::setActuatorStateCallback, this);

  // Subscriber for Moveit
  if (using_moveit_ == true)
  {
    get_moveit_joint_position_server_  = priv_node_handle_.advertiseService("moveit/get_joint_position", &SocialRobotArmController::getMoveitJointPositionCallback, this);
    get_moveit_kinematics_pose_server_ = priv_node_handle_.advertiseService("moveit/get_kinematics_pose", &SocialRobotArmController::getMoveitKinematicsPoseCallback, this);
    set_moveit_joint_position_server_  = priv_node_handle_.advertiseService("moveit/set_joint_position", &SocialRobotArmController::setMoveitJointPositionCallback, this);
    set_moveit_kinematics_pose_server_ = priv_node_handle_.advertiseService("moveit/set_kinematics_pose", &SocialRobotArmController::setMoveitKinematicsPoseCallback, this);
  }
}

/*****************************************************************************
 ** Publish Functions
*****************************************************************************/
void SocialRobotArmController::publishCallback(const ros::TimerEvent&)
{
  publishArmStates();                 // manipulator state
  if (using_platform_ == true)  publishArmsJointStates();     //joint states to actual manipulator
  else  publishGazeboGoalJointPositionCommand();              //joint states to gazebo
  publishHandsKinematicsPose();       // kinematic pose
  publishModeStates();                // Mode PUB
}

void SocialRobotArmController::publishArmStates()
{
  social_robot_arm_msgs::ManipulatorState msg;

  // Left Arm
  if(left_arm_.getMovingState())
    msg.manipulator_moving_state = msg.IS_MOVING;
  else
    msg.manipulator_moving_state = msg.STOPPED;

  if(left_arm_.getActuatorEnabledState(JOINT_DYNAMIXEL))
    msg.manipulator_actuator_state = msg.ACTUATOR_ENABLED;
  else
    msg.manipulator_actuator_state = msg.ACTUATOR_DISABLED;
  arm_states_pub_.at(0).publish(msg);

  // Right Arm
  if(right_arm_.getMovingState())
    msg.manipulator_moving_state = msg.IS_MOVING;
  else
    msg.manipulator_moving_state = msg.STOPPED;

  if(right_arm_.getActuatorEnabledState(JOINT_DYNAMIXEL))
    msg.manipulator_actuator_state = msg.ACTUATOR_ENABLED;
  else
    msg.manipulator_actuator_state = msg.ACTUATOR_DISABLED;

  arm_states_pub_.at(1).publish(msg);
}

void SocialRobotArmController::publishArmsJointStates()
{
  sensor_msgs::JointState all_arm_msg;
  sensor_msgs::JointState left_arm_msg;

  // Left Arm Header
  left_arm_msg.header.stamp = ros::Time::now();
  // Left Arm Joint
  auto left_arm_joints_name = left_arm_.getManipulator()->getAllActiveJointComponentName();
  auto left_arm_joint_value = left_arm_.getAllActiveJointValue();
  for(uint8_t i = 0; i < left_arm_joints_name.size(); i ++)
  {
    left_arm_msg.name.push_back(left_arm_joints_name.at(i));
    left_arm_msg.position.push_back(left_arm_joint_value.at(i).position);
    left_arm_msg.velocity.push_back(left_arm_joint_value.at(i).velocity);
    left_arm_msg.effort.push_back(left_arm_joint_value.at(i).effort);

    all_arm_msg.name.push_back(left_arm_joints_name.at(i));
    all_arm_msg.position.push_back(left_arm_joint_value.at(i).position);
    all_arm_msg.velocity.push_back(left_arm_joint_value.at(i).velocity);
    all_arm_msg.effort.push_back(left_arm_joint_value.at(i).effort);
  }
  // Left Arm End_effector
  auto left_arm_end_effector_name = left_arm_.getManipulator()->getAllToolComponentName();
  auto left_arm_end_effector_value = left_arm_.getAllToolValue();
  for(uint8_t i = 0; i < left_arm_end_effector_name.size(); i ++)
  {
    left_arm_msg.name.push_back(left_arm_end_effector_name.at(i));
    left_arm_msg.position.push_back(left_arm_end_effector_value.at(i).position);
    left_arm_msg.velocity.push_back(left_arm_end_effector_value.at(i).velocity);
    left_arm_msg.effort.push_back(left_arm_end_effector_value.at(i).effort);

    all_arm_msg.name.push_back(left_arm_end_effector_name.at(i));
    all_arm_msg.position.push_back(left_arm_end_effector_value.at(i).position);
    all_arm_msg.velocity.push_back(left_arm_end_effector_value.at(i).velocity);
    all_arm_msg.effort.push_back(left_arm_end_effector_value.at(i).effort);
  }
  // Left Arm Publish
  arms_joint_states_pub_.at(0).publish(left_arm_msg);

  sensor_msgs::JointState right_arm_msg;
  // Right Arm Header
  right_arm_msg.header.stamp = ros::Time::now();
  // Right Arm Joint
  auto right_arm_joints_name = right_arm_.getManipulator()->getAllActiveJointComponentName();
  auto right_arm_joint_value = right_arm_.getAllActiveJointValue();
  for(uint8_t i = 0; i < right_arm_joints_name.size(); i ++)
  {
    right_arm_msg.name.push_back(right_arm_joints_name.at(i));
    right_arm_msg.position.push_back(right_arm_joint_value.at(i).position);
    right_arm_msg.velocity.push_back(right_arm_joint_value.at(i).velocity);
    right_arm_msg.effort.push_back(right_arm_joint_value.at(i).effort);

    all_arm_msg.name.push_back(right_arm_joints_name.at(i));
    all_arm_msg.position.push_back(right_arm_joint_value.at(i).position);
    all_arm_msg.velocity.push_back(right_arm_joint_value.at(i).velocity);
    all_arm_msg.effort.push_back(right_arm_joint_value.at(i).effort);
  }
  // Right Arm End_effector
  auto right_arm_end_effector_name = right_arm_.getManipulator()->getAllToolComponentName();
  auto right_arm_end_effector_value = right_arm_.getAllToolValue();
  for(uint8_t i = 0; i < right_arm_end_effector_name.size(); i ++)
  {
    right_arm_msg.name.push_back(right_arm_end_effector_name.at(i));
    right_arm_msg.position.push_back(right_arm_end_effector_value.at(i).position);
    right_arm_msg.velocity.push_back(right_arm_end_effector_value.at(i).velocity);
    right_arm_msg.effort.push_back(right_arm_end_effector_value.at(i).effort);

    all_arm_msg.name.push_back(right_arm_end_effector_name.at(i));
    all_arm_msg.position.push_back(right_arm_end_effector_value.at(i).position);
    all_arm_msg.velocity.push_back(right_arm_end_effector_value.at(i).velocity);
    all_arm_msg.effort.push_back(right_arm_end_effector_value.at(i).effort);
  }
  // Right Arm Publish
  arms_joint_states_pub_.at(1).publish(right_arm_msg);

  if(using_platform_)
    arms_joint_states_pub_.at(2).publish(all_arm_msg);
}

void SocialRobotArmController::publishGazeboGoalJointPositionCommand()
{
  size_t cnt = 0;
//  ROS_WARN_STREAM("------------------------------------");
  // Left Arm Joint
  JointWaypoint left_arm_joint_value = left_arm_.getAllActiveJointValue();
  std::vector<std::string> left_arm_joint_name = left_arm_.getManipulator()->getAllActiveJointComponentName();
  for(uint8_t i = 0; i < left_arm_joint_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = left_arm_joint_value.at(i).position;
//      ROS_INFO_STREAM("name :" << left_arm_joint_name.at(i) << "\t ang :" << left_arm_joint_value.at(i).position << "\t pub :" << gazebo_goal_joint_position_command_pub_.at(cnt).getTopic());
    gazebo_goal_joint_position_command_pub_.at(cnt).publish(msg);
    cnt++;
  }

  //Right Arm Joint
  JointWaypoint right_arm_joint_value = right_arm_.getAllActiveJointValue();
  std::vector<std::string> right_arm_joint_name = right_arm_.getManipulator()->getAllActiveJointComponentName();
  for(uint8_t i = 0; i < right_arm_joint_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = right_arm_joint_value.at(i).position;
//      ROS_INFO_STREAM("name :" << right_arm_joint_name.at(i) << "\t ang :" << right_arm_joint_value.at(i).position << "\t pub :" << gazebo_goal_joint_position_command_pub_.at(cnt).getTopic());
    gazebo_goal_joint_position_command_pub_.at(cnt).publish(msg);
    cnt++;
  }

  // Left Arm End-effector
  JointWaypoint left_arm_end_effector_value = left_arm_.getAllToolValue();
  for(uint8_t i = 0; i < left_arm_end_effector_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = left_arm_end_effector_value.at(i).position;

    gazebo_goal_joint_position_command_pub_.at(cnt).publish(msg);
    cnt++;
  }

  //Right Arm End-effector
  JointWaypoint right_arm_end_effector_value = right_arm_.getAllToolValue();
  for(uint8_t i = 0; i < right_arm_end_effector_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = right_arm_end_effector_value.at(i).position;

    gazebo_goal_joint_position_command_pub_.at(cnt).publish(msg);
    cnt++;
  }
}

void SocialRobotArmController::publishHandsKinematicsPose()
{
  uint8_t index = 0;
  social_robot_arm_msgs::KinematicsPose msg;

  // Left Arm
  auto left_arm_end_effector_names = left_arm_.getManipulator()->getAllToolComponentName();
  for (auto const& names:left_arm_end_effector_names)
  {
    KinematicPose pose = left_arm_.getKinematicPose(names);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    hands_kinematics_pose_pub_.at(index).publish(msg);
    index++;
  }

  // Right Arm
  auto right_arm_end_effector_names = right_arm_.getManipulator()->getAllToolComponentName();
  for (auto const& names:right_arm_end_effector_names)
  {
    KinematicPose pose = right_arm_.getKinematicPose(names);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    hands_kinematics_pose_pub_.at(index).publish(msg);
    index++;
  }
}

void SocialRobotArmController::publishModeStates()
{
  std_msgs::String msg;
  msg.data = left_arm_.getMode();
  msg.data = right_arm_.getMode();
  arm_mode_state_pub_.publish(msg);
}

void SocialRobotArmController::publishMoveitUpdateStartState()
{
  if(using_moveit_)
  {
    if (moveit_update_start_state_pub_.getNumSubscribers() == 0)
    {
      log::warn("Could not update the start state! Enable External Communications at the Moveit Plugin");
    }
    std_msgs::Empty msg;
    moveit_update_start_state_pub_.publish(msg);
    log::println("Updated MoveIt Start position", "GREEN");
  }
}

void SocialRobotArmController::waistCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  waist_joints_ = *msg;
}

void SocialRobotArmController::changeModeCallback(const std_msgs::Empty::ConstPtr &msg)
{
  bool moving = false;

  moving = left_arm_.getMovingState();
  if(moving)
    moving = true;
  else
    moving = right_arm_.getMovingState();

  if(!moving)
  {
    std::string mode = left_arm_.getMode();
    if(mode != right_arm_.getMode())
    {
      left_arm_.changeMode("normal_mode");
      right_arm_.changeMode("normal_mode");
      log::info("Change to Normal Mode");
      return;
    }
    else
    {
      if(mode == "normal_mode")
      {
        left_arm_.changeMode("gravity_compensation_mode");
        right_arm_.changeMode("gravity_compensation_mode");
        log::info("Change to Gravity compensation mode (Motion edit mode)");
      }
      else
      {
        left_arm_.changeMode("normal_mode");
        right_arm_.changeMode("normal_mode");
        log::info("Change to Normal Mode");
      }
    }
  }
  else
  {
    log::error("Fail to change the mode. Robot is moving.");
  }
}

void SocialRobotArmController::stopMoveCallback(const std_msgs::Empty::ConstPtr &msg)
{
  stopMotionPlayThread();
  left_arm_.stopMoving();
  right_arm_.stopMoving();
  log::println("E-Stop!!","RED");
}

/*****************************************************************************
 ** Subscriber Callback Functions
*****************************************************************************/
void SocialRobotArmController::robotisManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "print_left_arm_setting")
    left_arm_.printManipulatorSetting();
  else if(msg->data == "print_right_arm_setting")
    right_arm_.printManipulatorSetting();
}

void SocialRobotArmController::displayPlannedPathCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  // if joint names exists
  if(msg->trajectory[0].joint_trajectory.joint_names.size() > 0)
  {
    log::println("[INFO] [displayPlannedPathCallback] Get Moveit planned path", "GREEN");

    trajectory_msgs::JointTrajectory joint_trajectory_planned = msg->trajectory[0].joint_trajectory;
    moveit_joint_trajectory_ = joint_trajectory_planned;

    std::map<Name, Component> left_arm_component = left_arm_.getManipulator()->getAllComponent();
    std::map<Name, Component> right_arm_component = right_arm_.getManipulator()->getAllComponent();
        
    if(std::find(moveit_joint_trajectory_.joint_names.begin(), moveit_joint_trajectory_.joint_names.end(), left_arm_component.begin()->first) != moveit_joint_trajectory_.joint_names.end() &
      std::find(moveit_joint_trajectory_.joint_names.begin(), moveit_joint_trajectory_.joint_names.end(), right_arm_component.begin()->first) != moveit_joint_trajectory_.joint_names.end())
    {
      moveit_plan_arm_name_ = "dual_arm";
      log::println("[INFO] [displayPlannedPathCallback] Dual arm motion planned", "GREEN");
    }
    else if(left_arm_component.find(moveit_joint_trajectory_.joint_names.back()) != left_arm_component.end())
    {
      moveit_plan_arm_name_ = "left_arm";
      log::println("[INFO] [displayPlannedPathCallback] Left arm motion planned", "GREEN");
    }
    else if(right_arm_component.find(moveit_joint_trajectory_.joint_names.back()) != right_arm_component.end())
    {
      moveit_plan_arm_name_ = "right_arm";
      log::println("[INFO] [displayPlannedPathCallback] Right arm motion planned", "GREEN");
    }
    else
    {
      log::error("[displayPlannedPathCallback] Wrong Group name");
      return;
    }

    if(moveit_plan_only_ == false)
    {
      log::println("[INFO] [displayPlannedPathCallback] Execute Moveit planned path", "GREEN");
      moveit_plan_state_ = true;
      priv_node_handle_.setParam("/social_robot/moveit_plan_state", false);
    }
    //else
    //  log::println("[INFO] [displayPlannedPathCallback] Get Moveit planned path", "GREEN");
  }
}

void SocialRobotArmController::moveGroupActionGoalCallback(const moveit_msgs::MoveGroupActionGoal::ConstPtr &msg)
{
  log::println("[INFO] [Social Robot Arm Controller] Get Moveit plnning option", "GREEN");
  moveit_plan_only_ = msg->goal.planning_options.plan_only; // click "plan & execute" or "plan" button
}

void SocialRobotArmController::executeTrajectoryActionGoalCallback(const moveit_msgs::ExecuteTrajectoryActionGoal::ConstPtr &msg)
{
  log::println("[INFO] [Social Robot Arm Controller] Execute Moveit planned path", "GREEN");

  // save trajectory points
  trajectory_msgs::JointTrajectory joint_trajectory_planned = msg->goal.trajectory.joint_trajectory;
  moveit_joint_trajectory_ = joint_trajectory_planned;

  std::map<Name, Component> left_arm_component = left_arm_.getManipulator()->getAllComponent();
  std::map<Name, Component> right_arm_component = right_arm_.getManipulator()->getAllComponent();
  
  if(std::find(moveit_joint_trajectory_.joint_names.begin(), moveit_joint_trajectory_.joint_names.end(), left_arm_component.begin()->first) != moveit_joint_trajectory_.joint_names.end() &
      std::find(moveit_joint_trajectory_.joint_names.begin(), moveit_joint_trajectory_.joint_names.end(), right_arm_component.begin()->first) != moveit_joint_trajectory_.joint_names.end())
  {
    moveit_plan_arm_name_ = "dual_arm";
    log::println("[INFO] [executeTrajectoryActionGoalCallback] Dual arm motion executed", "GREEN");
  }
  else if(left_arm_component.find(moveit_joint_trajectory_.joint_names.back()) != left_arm_component.end())
  {
    moveit_plan_arm_name_ = "left_arm";
    log::println("[INFO] [executeTrajectoryActionGoalCallback] Left arm motion executed", "GREEN");
  }
  else if(right_arm_component.find(moveit_joint_trajectory_.joint_names.back()) != right_arm_component.end())
  {
    moveit_plan_arm_name_ = "right_arm";
    log::println("[INFO] [executeTrajectoryActionGoalCallback] Right arm motion executed", "GREEN");
  }
  else
  {
    log::error("[executeTrajectoryActionGoalCallback] Wrong Group name");
    return;
  }

  moveit_plan_state_ = true;
  priv_node_handle_.setParam("/social_robot/moveit_plan_state", true);
}

void SocialRobotArmController::gazeboJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Publish joint states
  bool temp = false;
  sensor_msgs::JointState left_arm_msg;
  sensor_msgs::JointState right_arm_msg;
  for(uint8_t i = 0; i < msg->name.size(); i ++)
  {
    temp = false;
    // Left Arm
    std::vector<std::string> left_arm_joint_names = left_arm_.getManipulator()->getAllActiveJointComponentName();
    for(uint8_t j = 0; j < left_arm_joint_names.size(); j++)
    {
      if(msg->name.at(i) == left_arm_joint_names.at(j))
      {
        left_arm_msg.name.push_back(msg->name.at(i));
        left_arm_msg.position.push_back(msg->position.at(i));
        left_arm_msg.velocity.push_back(msg->velocity.at(i));
        left_arm_msg.effort.push_back(msg->effort.at(i));
        temp = true;
        break;
      }
    }
    std::vector<std::string> left_arm_end_effector_name = left_arm_.getManipulator()->getAllToolComponentName();
    for(uint8_t j = 0; j < left_arm_end_effector_name.size(); j++)
    {
      if(temp)
      {
        temp = false;
        break;
      }
      if(msg->name.at(i) == left_arm_end_effector_name.at(j))
      {
        left_arm_msg.name.push_back(msg->name.at(i));
        left_arm_msg.position.push_back(msg->position.at(i));
        left_arm_msg.velocity.push_back(msg->velocity.at(i));
        left_arm_msg.effort.push_back(msg->effort.at(i));
        temp = true;
        break;
      }
    }
    // Right Arm
    std::vector<std::string> right_arm_joint_names = right_arm_.getManipulator()->getAllActiveJointComponentName();
    for(uint8_t j = 0; j < right_arm_joint_names.size(); j++)
    {
      if(temp)
      {
        temp = false;
        break;
      }
      if(msg->name.at(i) == right_arm_joint_names.at(j))
      {
        right_arm_msg.name.push_back(msg->name.at(i));
        right_arm_msg.position.push_back(msg->position.at(i));
        right_arm_msg.velocity.push_back(msg->velocity.at(i));
        right_arm_msg.effort.push_back(msg->effort.at(i));
        temp = true;
        break;
      }
    }
    std::vector<std::string> right_arm_end_effector_name = right_arm_.getManipulator()->getAllToolComponentName();
    for(uint8_t j = 0; j < right_arm_end_effector_name.size(); j++)
    {
      if(temp)
      {
        temp = false;
        break;
      }
      if(msg->name.at(i) == right_arm_end_effector_name.at(j))
      {
        right_arm_msg.name.push_back(msg->name.at(i));
        right_arm_msg.position.push_back(msg->position.at(i));
        right_arm_msg.velocity.push_back(msg->velocity.at(i));
        right_arm_msg.effort.push_back(msg->effort.at(i));
        break;
      }
    }
  }

  arms_joint_states_pub_.at(0).publish(left_arm_msg);
  arms_joint_states_pub_.at(1).publish(right_arm_msg);
}

void SocialRobotArmController::gazeboLinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
  //set base pose
  KinematicPose body_pose;
  for(uint8_t index = 0; index < msg->name.size(); index++)
  {
    if(msg->name.at(index) == "social_robot::Waist_Pitch")
    {
      body_pose.position[0] = msg->pose[index].position.x;
      body_pose.position[1] = msg->pose[index].position.y;
      body_pose.position[2] = msg->pose[index].position.z;

      Eigen::Quaterniond q(msg->pose[index].orientation.w,
                           msg->pose[index].orientation.x,
                           msg->pose[index].orientation.y,
                           msg->pose[index].orientation.z);

      body_pose.orientation = math::convertQuaternionToRotationMatrix(q);
    }
  }
//  left_arm_.setWaistPitch(body_pose);
//  right_arm_.setWaistPitch(body_pose);
}

void SocialRobotArmController::MotionControlCallback(const social_robot_arm_msgs::MotionControl::ConstPtr &msg)
{
  if(msg->request == msg->PLAY)
  {
    if(!motion_scenario_.empty())
    {
      startMotionPlayThread();
    }
    else
    {
      log::warn("Motion Storage Empty");
    }
  }
  else if(msg->request == msg->PAUSE)
  {
    log::info("Motion pause.");
    stopMotionPlayThread();
  }
  else if(msg->request == msg->CLEAR)
  {
    stopMotionPlayThread();
    motion_scenario_.clear();
    motion_return_first_flag_ = true;
    log::info("Saved Motion cleard. motion number is ", motion_scenario_.size(),0);
  }
  else if(msg->request == msg->STOP)
  {
    log::info("Accept Motion stop request. Motion will be stop after playing this motion");
    motion_stop_flag_ = true;
  }
  else if(msg->request == msg->CANCEL_STOP)
  {
    log::info("Accept Motion stop cancel request. Motion will be playing after playing this motion");
    motion_stop_flag_ = false;
  }
  else if(msg->request == msg->BREAK)
  {
    log::info("Accept Motion break request. play next motion after playing this motion");
    motion_break_flag_ = true;
  }
  else if(msg->request == msg->CANCEL_BREAK)
  {
    log::info("Accept Motion break cancel request.");
    motion_break_flag_ = false;
  }
  else if(msg->request == msg->RETURN_FIRST)
  {
    log::info("Accept return first request.");
    motion_return_first_flag_ = true;
  }
  else if(msg->request == msg->RETURN_MOTION)
  {
    log::info("Accept return motion request.");
    motion_return_motion_flag_ = true;
  }
  else if(msg->request == msg->E_STOP)
  {
    left_arm_.stopMoving();
    right_arm_.stopMoving();
    stopMotionPlayThread();
    log::println("E-Stop!!","RED");
  }
  else
  {
    log::warn("Wrong Motion Reuest.");
  }
}

void SocialRobotArmController::MotionSaveCallback(const social_robot_arm_msgs::MotionSave::ConstPtr &msg)
{
  motion_scenario_.push_back(*msg);
  log::info("Motion saved. saved motion number is ",motion_scenario_.size(),0);
}

void SocialRobotArmController::SetModeCallback(const social_robot_arm_msgs::SetMode::ConstPtr &msg)
{
  bool moving = true;
  log::info("Change mode to " + msg->mode);
  while(moving)     // break to not moving
  {
    moving = left_arm_.getMovingState();
    if(!moving)
      moving = right_arm_.getMovingState();
  }
//  // Thread stop
//  timer_thread_state_ = false;
//  pthread_join(timer_thread_, NULL);                      // Wait for the thread associated with thread_p to complete
  left_arm_.modeChanging(true);
  right_arm_.modeChanging(true);
  if(msg->mode == msg->PLAY_MODE)
  {
    if(left_arm_.changeMode("normal_mode"))
      log::info("Left Arm Change to "+msg->mode);
    if(right_arm_.changeMode("normal_mode"))
      log::info("Right Arm Change to "+msg->mode);
  }
  else if(msg->mode == msg->EDIT_MODE)
  {
    if(left_arm_.changeMode("gravity_compensation_mode"))
      log::info("Left Arm Change to "+msg->mode);
    if(right_arm_.changeMode("gravity_compensation_mode"))
      log::info("Right Arm Change to "+msg->mode);
  }
  else
  {
    log::warn("Fail to change the mode. Wrong mode name.");
  }
  left_arm_.modeChanging(false);
  right_arm_.modeChanging(false);
//  // thread start
//  startTimerThread();
}


/*****************************************************************************
 ** Sevice Server Callback Functions
*****************************************************************************/
bool SocialRobotArmController::goalJointSpacePathCallback(social_robot_arm_msgs::SetJointPosition::Request  &req,
                                social_robot_arm_msgs::SetJointPosition::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Joint Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  std::vector <double> target_angle;
  //Joint name matching
  auto storage_names = current_controled_arm_->getManipulator()->getAllActiveJointComponentName();
  auto msg_names = req.joint_position.joint_name;
  for(uint8_t i = 0; i < storage_names.size(); i++)
  {
    for(uint8_t index = 0; index < msg_names.size(); index++)
    {
      if(storage_names.at(i) == msg_names[index])
      {
        target_angle.push_back(req.joint_position.position.at(index));
        break;
      }
    }
  }

  current_controled_arm_->makeJointTrajectory(target_angle, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalJointSpacePathToKinematicsPoseCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Joint Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;

  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  current_controled_arm_->makeJointTrajectory(req.end_effector_name, target_pose, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalJointSpacePathToKinematicsPositionCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Joint Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  current_controled_arm_->makeJointTrajectory(req.end_effector_name, target_pose.position, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalJointSpacePathToKinematicsOrientationCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Joint Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  current_controled_arm_->makeJointTrajectory(req.end_effector_name, target_pose.orientation, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalJointSpacePathFromPresentCallback(social_robot_arm_msgs::SetJointPosition::Request  &req,
                                           social_robot_arm_msgs::SetJointPosition::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Joint Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  std::vector <double> target_angle;
  //Joint name matching
  auto storage_names = current_controled_arm_->getManipulator()->getAllActiveJointComponentName();
  auto msg_names = req.joint_position.joint_name;
  for(uint8_t i = 0; i < storage_names.size(); i++)
  {
    for(uint8_t index = 0; index < msg_names.size(); index++)
    {
      if(storage_names.at(i) == msg_names[index])
      {
        target_angle.push_back(req.joint_position.position.at(index));
        break;
      }
    }
  }

  current_controled_arm_->makeJointTrajectoryFromPresentPosition(target_angle, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalTaskSpacePathCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                               social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Task Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);
  current_controled_arm_->makeTaskTrajectory(req.end_effector_name, target_pose, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalTaskSpacePathPositionOnlyCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                           social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Task Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  current_controled_arm_->makeTaskTrajectory(req.end_effector_name, target_pose.position, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalTaskSpacePathOrientationOnlyCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                              social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Task Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  current_controled_arm_->makeTaskTrajectory(req.end_effector_name, target_pose.orientation, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalTaskSpacePathFromPresentCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                          social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Task Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);
  current_controled_arm_->makeTaskTrajectoryFromPresentPose(req.end_effector_name, target_pose, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalTaskSpacePathFromPresentPositionOnlyCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                      social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Task Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  current_controled_arm_->makeTaskTrajectoryFromPresentPose(req.end_effector_name, target_pose.position, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalTaskSpacePathFromPresentOrientationOnlyCallback(social_robot_arm_msgs::SetKinematicsPose::Request  &req,
                                                         social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Task Space] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  KinematicPose target_pose;
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  current_controled_arm_->makeTaskTrajectoryFromPresentPose(req.end_effector_name, target_pose.orientation, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::goalDrawingTrajectoryCallback(social_robot_arm_msgs::SetDrawingTrajectory::Request  &req,
                                   social_robot_arm_msgs::SetDrawingTrajectory::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[Goal Drawing] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  try
  {
    if(req.drawing_trajectory_name == "circle")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;

      current_controled_arm_->makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, req.end_effector_name, p_draw_circle_arg, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());
    }
    else if(req.drawing_trajectory_name == "line")
    {
      TaskWaypoint draw_line_arg;
      draw_line_arg.kinematic.position(0) = req.param[0]; // x axis (m)
      draw_line_arg.kinematic.position(1) = req.param[1]; // y axis (m)
      draw_line_arg.kinematic.position(2) = req.param[2]; // z axis (m)
      void *p_draw_line_arg = &draw_line_arg;

      current_controled_arm_->makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, req.end_effector_name, p_draw_line_arg, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());
    }
    else if(req.drawing_trajectory_name == "rhombus")
    {
      double draw_rhombus_arg[3];
      draw_rhombus_arg[0] = req.param[0];  // radius (m)
      draw_rhombus_arg[1] = req.param[1];  // revolution (rev)
      draw_rhombus_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_rhombus_arg = &draw_rhombus_arg;

      current_controled_arm_->makeCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, req.end_effector_name, p_draw_rhombus_arg, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());
    }
    else if(req.drawing_trajectory_name == "heart")
    {
      double draw_heart_arg[3];
      draw_heart_arg[0] = req.param[0];  // radius (m)
      draw_heart_arg[1] = req.param[1];  // revolution (rev)
      draw_heart_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_heart_arg = &draw_heart_arg;

      current_controled_arm_->makeCustomTrajectory(CUSTOM_TRAJECTORY_HEART, req.end_effector_name, p_draw_heart_arg, req.path_time, current_controled_arm_->getManipulator()->getAllActiveJointValue());
    }
    res.is_planned = true;
    return true;
  }
  catch ( ros::Exception &e )
  {
    log::error("Creation the custom trajectory is failed!");
  }
  return true;
}

bool SocialRobotArmController::goalToolControlCallback(social_robot_arm_msgs::SetJointPosition::Request  &req,
                             social_robot_arm_msgs::SetJointPosition::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[goalToolControlCallback] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    current_controled_arm_->makeToolTrajectory(req.joint_position.joint_name.at(i), req.joint_position.position.at(i));

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::setActuatorStateCallback(social_robot_arm_msgs::SetActuatorState::Request  &req,
                              social_robot_arm_msgs::SetActuatorState::Response &res)
{
  //Set Group Name
  if(req.planning_group == "left_arm")
  {
    current_controled_arm_ = &left_arm_;
  }
  else if(req.planning_group == "right_arm")
  {
    current_controled_arm_ = &right_arm_;
  }
  else
  {
    log::error("[setActuatorStateCallback] Wrong Group name");
    res.is_planned = false;
    return false;
  }

  //Set Target
  if(req.set_actuator_state == true) // enable actuators
  {
    log::println("Wait a second for actuator enable", "GREEN");

    stopMotionPlayThread();
    left_arm_.stopMoving();
    right_arm_.stopMoving();
    timer_thread_state_ = false;
    pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
    current_controled_arm_->enableAllActuator();
    startTimerThread();
  }
  else // disable actuators
  {
    log::println("Wait a second fodirect teaching Functionsr actuator disable", "GREEN");

    stopMotionPlayThread();
    left_arm_.stopMoving();
    right_arm_.stopMoving();
    timer_thread_state_ = false;
    pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
    current_controled_arm_->disableAllActuator();
    startTimerThread();
  }

  res.is_planned = true;
  return true;
}

bool SocialRobotArmController::getMoveitJointPositionCallback(social_robot_arm_msgs::GetJointPosition::Request &req,
                               social_robot_arm_msgs::GetJointPosition::Response &res)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Set Group Name
  moveit::planning_interface::MoveGroupInterface* control_move_group;
  if(req.planning_group == "left_arm")
  {
    control_move_group = left_arm_move_group_;
  }
  else if(req.planning_group == "right_arm")
  {
    control_move_group = right_arm_move_group_;
  }
  else
  {
    log::error("[MoveIT] Wrong Move Group name");
    return false;
  }

  //Get Values
  const std::vector<std::string> &joint_names = control_move_group->getJointNames();
  std::vector<double> joint_values = control_move_group->getCurrentJointValues();

  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    res.joint_position.joint_name.push_back(joint_names[i]);
    res.joint_position.position.push_back(joint_values[i]);
  }

  spinner.stop();
  return true;
}

bool SocialRobotArmController::getMoveitKinematicsPoseCallback(social_robot_arm_msgs::GetKinematicsPose::Request &req,
                                  social_robot_arm_msgs::GetKinematicsPose::Response &res)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Set Group Name
  //Set Group Name
  moveit::planning_interface::MoveGroupInterface* control_move_group;
  if(req.planning_group == "left_arm")
  {
    control_move_group = left_arm_move_group_;
  }
  else if(req.planning_group == "right_arm")
  {
    control_move_group = right_arm_move_group_;
  }
  else
  {
    log::error("[MoveIT] Wrong Move Group name");
    return false;
  }

  //Get Pose
  geometry_msgs::PoseStamped current_pose = control_move_group->getCurrentPose();

  res.header                     = current_pose.header;
  res.kinematics_pose.pose       = current_pose.pose;

  spinner.stop();
  return true;
}

bool SocialRobotArmController::setMoveitJointPositionCallback(social_robot_arm_msgs::SetJointPosition::Request &req,
                                 social_robot_arm_msgs::SetJointPosition::Response &res)
{
  social_robot_arm_msgs::JointPosition msg = req.joint_position;
  res.is_planned = calcPlannedPath(req.planning_group, msg);

  return true;
}

bool SocialRobotArmController::setMoveitKinematicsPoseCallback(social_robot_arm_msgs::SetKinematicsPose::Request &req,
                                  social_robot_arm_msgs::SetKinematicsPose::Response &res)
{
  social_robot_arm_msgs::KinematicsPose msg = req.kinematics_pose;
  res.is_planned = calcPlannedPath(req.planning_group, msg);

  return true;
}

/*****************************************************************************
 ** Moveit CalcPlannedPath Functions
*****************************************************************************/
bool SocialRobotArmController::calcPlannedPath(const std::string planning_group, social_robot_arm_msgs::JointPosition msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Set Group Name
  moveit::planning_interface::MoveGroupInterface* control_move_group;
  bool arms_moving_state;
  //Set Group Name
  if(planning_group == "left_arm")
  {
    control_move_group = left_arm_move_group_;
    arms_moving_state = left_arm_.getMovingState();
  }
  else if(planning_group == "right_arm")
  {
    control_move_group = right_arm_move_group_;
    arms_moving_state = right_arm_.getMovingState();
  }
  else
  {
    log::error("[calcPlannedPath] Wrong Group name");
    return false;
  }

  //Get Current States
  const robot_state::JointModelGroup *joint_model_group = control_move_group->getCurrentState()->getJointModelGroup(planning_group);
  moveit::core::RobotStatePtr current_state = control_move_group->getCurrentState();

  //Get Current Joint Position and Update
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  for (uint8_t index = 0; index < joint_group_positions.size(); index++)
  {
    joint_group_positions.at(index) = msg.position[index];
  }

  //Set Goal Joint Value
  control_move_group->setJointValueTarget(joint_group_positions);

  //Set Scaling Factors
  control_move_group->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  control_move_group->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  //Planning
  bool is_planned = false;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (arms_moving_state == false)
  {
    bool success = (control_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
      is_planned = true;
    }
    else
    {
      log::warn("[calcPlannedPath] Failed to Plan (joint space goal)");
      is_planned = false;
    }
  }
  else
  {
    log::warn("[calcPlannedPath] Robot is moving");
    is_planned = false;
  }

  spinner.stop();
  return is_planned;
}

bool SocialRobotArmController::calcPlannedPath(const std::string planning_group, social_robot_arm_msgs::KinematicsPose msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Set Group Name
  moveit::planning_interface::MoveGroupInterface* control_move_group;
  bool arms_moving_state;
  if(planning_group == "left_arm")
  {
    control_move_group = left_arm_move_group_;
    arms_moving_state = left_arm_.getMovingState();
  }
  else if(planning_group == "right_arm")
  {
    control_move_group = right_arm_move_group_;
    arms_moving_state = right_arm_.getMovingState();
  }
  else
  {
    log::error("[calcPlannedPath] Wrong Group name");
    return false;
  }

  //Set Goal Pose
  geometry_msgs::Pose target_pose = msg.pose;
  control_move_group->setPoseTarget(target_pose);

  //Set Scaling Factors
  control_move_group->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  control_move_group->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  //Set Goal Tolerance
  control_move_group->setGoalTolerance(msg.tolerance);

  //Planning
  bool is_planned = false;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (arms_moving_state == false)
  {
    bool success = (control_move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
      is_planned = true;
    }
    else
    {
      log::warn("[calcPlannedPath] Failed to Plan (task space goal)");
      is_planned = false;
    }
  }
  else
  {
    log::warn("[calcPlannedPath] Robot is Moving");
    is_planned = false;
  }

  spinner.stop();
  return is_planned;
}

/*****************************************************************************
 ** Control Process Functions
*****************************************************************************/
void SocialRobotArmController::moveitProcess(double present_time)
{
  static uint32_t step_cnt = 0;
  static double start_time = 0.0f;
  static double next_time = 0.0f;
  
  //Set Group Name
  if (moveit_plan_state_ == true)
  {
    if (moveit_plan_arm_name_ == "dual_arm")
    {
      
    }
    else if(moveit_plan_arm_name_ == "left_arm")
    {
      current_controled_arm_ = &left_arm_;
    }
    else if(moveit_plan_arm_name_ == "right_arm")
    {
      current_controled_arm_ = &right_arm_;
    }
    else
    {
      log::error("[Social Robot Arm Controller] [moveitProcess] Wrong Group name");
      step_cnt = 0;
      moveit_plan_state_ = false;
      priv_node_handle_.setParam("/social_robot/moveit_plan_state", false);
      publishMoveitUpdateStartState();
    }

    // Set Start Time
    if(step_cnt == 0)
    {
      start_time = present_time;
      step_cnt = 1;
    }

    // Get Trajectory Size
    uint32_t all_time_steps = moveit_joint_trajectory_.points.size();

    // Set Next Time
    next_time = moveit_joint_trajectory_.points[step_cnt].time_from_start.sec
              + (moveit_joint_trajectory_.points[step_cnt].time_from_start.nsec*0.000000001);

    if (present_time - start_time >= next_time)
    {
      //Set Target angle on this step
      auto moveit_names = moveit_joint_trajectory_.joint_names;

      if(moveit_plan_arm_name_ == "left_arm" || moveit_plan_arm_name_ == "right_arm")
      {
        JointWaypoint target;

        //Joint name matching
        auto storage_names = current_controled_arm_->getManipulator()->getAllActiveJointComponentName();

        // arm joints
        for(uint8_t i = 0; i < storage_names.size(); i++)
        {
          for(uint8_t index = 0; index < moveit_names.size(); index++)
          {
            if(storage_names.at(i) == moveit_names.at(index))
            {
              JointValue temp;
              temp.position = moveit_joint_trajectory_.points[step_cnt].positions.at(index);
              temp.velocity = moveit_joint_trajectory_.points[step_cnt].velocities.at(index);
              temp.acceleration = moveit_joint_trajectory_.points[step_cnt].accelerations.at(index);
              target.push_back(temp);
              break;
            }
          }
        }

        //Set Path Time on this step
        double priv_time = moveit_joint_trajectory_.points[step_cnt-1].time_from_start.sec
            + (moveit_joint_trajectory_.points[step_cnt-1].time_from_start.nsec*0.000000001);
        double path_time = next_time - priv_time;

        //make manipulator trajectory
        current_controled_arm_->makeJointTrajectory(target, path_time);
      }
      else if(moveit_plan_arm_name_ == "dual_arm")
      {
        JointWaypoint left_target, right_target;

        //Joint name matching
        auto left_storage_names = left_arm_.getManipulator()->getAllActiveJointComponentName();
        auto right_storage_names = right_arm_.getManipulator()->getAllActiveJointComponentName();
        
        // set arm target joints
        // left
        for(uint8_t i = 0; i < left_storage_names.size(); i++)
        {
          for(uint8_t index = 0; index < moveit_names.size(); index++)
          {
            if(left_storage_names.at(i) == moveit_names.at(index))
            {
              JointValue temp;
              temp.position = moveit_joint_trajectory_.points[step_cnt].positions.at(index);
              temp.velocity = moveit_joint_trajectory_.points[step_cnt].velocities.at(index);
              temp.acceleration = moveit_joint_trajectory_.points[step_cnt].accelerations.at(index);
              left_target.push_back(temp);
              break;
            }
          }
        }
        // right
        for(uint8_t i = 0; i < right_storage_names.size(); i++)
        {
          for(uint8_t index = 0; index < moveit_names.size(); index++)
          {
            if(right_storage_names.at(i) == moveit_names.at(index))
            {
              JointValue temp;
              temp.position = moveit_joint_trajectory_.points[step_cnt].positions.at(index);
              temp.velocity = moveit_joint_trajectory_.points[step_cnt].velocities.at(index);
              temp.acceleration = moveit_joint_trajectory_.points[step_cnt].accelerations.at(index);
              right_target.push_back(temp);
              break;
            }
          }
        }

        //Set Path Time on this step
        double priv_time = moveit_joint_trajectory_.points[step_cnt-1].time_from_start.sec
            + (moveit_joint_trajectory_.points[step_cnt-1].time_from_start.nsec*0.000000001);
        double path_time = next_time - priv_time;

        //make manipulator trajectory
        left_arm_.makeJointTrajectory(left_target, path_time);
        right_arm_.makeJointTrajectory(right_target, path_time);
      }
      
      // Waist controll
      if(std::find(moveit_joint_trajectory_.joint_names.begin(), moveit_joint_trajectory_.joint_names.end(), "Waist_Roll") != moveit_joint_trajectory_.joint_names.end())
      {
        // set waist and head target
        waist_joints_.name = {"Waist_Roll","Waist_Pitch","Head_Pitch","Head_Yaw"};
        waist_joints_.position = {0.0, 0.0, -0.3, 0.0};
        waist_joints_.velocity = {0.0, 0.0, 0.0, 0.0};
        waist_joints_.effort = {0.0, 0.0, 0.0, 0.0};
        for(uint8_t i = 0; i < waist_joints_.name.size(); i++)  
        {
          for(uint8_t index = 0; index < moveit_names.size(); index++)
          {
            if(waist_joints_.name.at(i) == moveit_names.at(index))
            {
              waist_joints_.position.at(i) = moveit_joint_trajectory_.points[step_cnt].positions.at(index);
              waist_joints_.velocity.at(i) = std::abs(moveit_joint_trajectory_.points[step_cnt].velocities.at(index));
              break;
            }
          }
        }    
        // publish waist head goal trajectory
        waist_state_pub_.publish(waist_joints_);
      }

      // TODO: add mobile control


      //update step count
      step_cnt++;

      //over all trajectory
      if (step_cnt >= all_time_steps)
      {
        step_cnt = 0;
        moveit_plan_state_ = false;
        priv_node_handle_.setParam("/social_robot/moveit_plan_state", false);
      }
    }
  }
  else
  {
    start_time = present_time;

    // Update Moveit Start Position
    if(left_arm_.TrajectoryEnd() || right_arm_.TrajectoryEnd())
    {
      if(!moveit_plan_state_)
        publishMoveitUpdateStartState();
    }
  }
}

void SocialRobotArmController::robotisProcess(double present_time)
{
  left_arm_.process(present_time);
  right_arm_.process(present_time);
}

/*****************************************************************************
 ** Thread Functions
*****************************************************************************/
void SocialRobotArmController::startTimerThread()
{
  int error;
  if ((error = pthread_create(&this->timer_thread_, NULL, this->timerThread, this)) != 0)
  {
    log::error("Creating timer thread failed!!", (double)error);
    exit(-1);
  }
  log::println("[Info] Start Thread.");
  timer_thread_state_ = true;
}

void *SocialRobotArmController::timerThread(void *param)
{
  SocialRobotArmController *controller = (SocialRobotArmController *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(controller->timer_thread_state_)
  {
    // Get Time
    next_time.tv_sec += (next_time.tv_nsec + ((int64_t)(controller->getControlLoopTime() * 1000)) * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + ((int64_t)(controller->getControlLoopTime() * 1000)) * 1000000) % 1000000000;
    double time = next_time.tv_sec + (next_time.tv_nsec*0.000000001);

    // Process
    controller->robotisProcess(time);
    controller->moveitProcess(time);

    // Time Count
    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    double delta_nsec = controller->getControlLoopTime() - ((next_time.tv_sec - curr_time.tv_sec) + ((double)(next_time.tv_nsec - curr_time.tv_nsec)*0.000000001));
    if(delta_nsec > controller->getControlLoopTime())
    {
      log::warn("Over the control time : ", delta_nsec);
      next_time = curr_time;
    }
    else
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
  }

  return 0;
}

/*****************************************************************************
 ** Motion Play Functions
*****************************************************************************/
double SocialRobotArmController::motionPlay()
{
  double time_radio = 1.0;
  if(motion_return_first_flag_)
  {
    pose_cnt_=0;
    repeat_cnt_=0;
    motion_cnt_=0;
    motion_return_first_flag_=false;
  }

  if(motion_return_motion_flag_)
  {
    pose_cnt_=0;
    repeat_cnt_=0;
    motion_return_motion_flag_=false;
  }

  // Print number
  log::print("Motion Num :",motion_cnt_+1,0,"GREEN");
  log::print("/",motion_scenario_.size(),0,"GREEN");
  log::print(", Repeat Num :",repeat_cnt_+1,0,"GREEN");
  log::print("/",motion_scenario_.at(motion_cnt_).repeat_count,0,"GREEN");
  log::print(", Pose Num :",pose_cnt_+1,0,"GREEN");
  log::print("/",motion_scenario_.at(motion_cnt_).left_arm_motion.size(),0,"GREEN");
  log::println("");

  // set time
  double time = motion_scenario_.at(motion_cnt_).path_time.at(pose_cnt_);

  // get data from motion_scenario_
  sensor_msgs::JointState left_motion_pose = motion_scenario_.at(motion_cnt_).left_arm_motion.at(pose_cnt_);
  // left arm
  JointWaypoint left_arm_target_point = left_arm_.getTrajectory()->getPresentJointWaypoint();
  std::vector<std::string> left_joint_name = left_arm_.getManipulator()->getAllActiveJointComponentName();
  for(int8_t i=0; i<left_joint_name.size();i++)
  {
    for(int8_t j=0; j<left_motion_pose.name.size(); j++)
    {
      if(left_joint_name.at(i) == left_motion_pose.name.at(j))
      {
        left_arm_target_point.at(i).position = left_motion_pose.position.at(j);
        left_arm_target_point.at(i).velocity = left_motion_pose.velocity.at(j);
        left_arm_target_point.at(i).effort = left_motion_pose.effort.at(j);
        break;
      }
    }
  }
  // Play
  left_arm_.makeJointTrajectory(left_arm_target_point, time);

  // left hand
  std::vector<std::string> left_finger_name = left_arm_.getManipulator()->getAllToolComponentName();
  for(int8_t i=0; i<left_finger_name.size();i++)
  {
    for(int8_t j=0; j<left_motion_pose.name.size(); j++)
    {
      if(left_finger_name.at(i) == left_motion_pose.name.at(j))
      {
        left_arm_.makeToolTrajectory(left_motion_pose.name.at(j), left_motion_pose.position.at(j));
        break;
      }
    }
  }

  // get data from motion_scenario_
  sensor_msgs::JointState right_motion_pose = motion_scenario_.at(motion_cnt_).right_arm_motion.at(pose_cnt_);
  // right arm
  // right arm
  JointWaypoint right_arm_target_point = right_arm_.getTrajectory()->getPresentJointWaypoint();
  std::vector<std::string> rihgt_joint_name = right_arm_.getManipulator()->getAllActiveJointComponentName();
  for(int8_t i=0; i<rihgt_joint_name.size();i++)
  {
    for(int8_t j=0; j<right_motion_pose.name.size(); j++)
    {
      if(rihgt_joint_name.at(i) == right_motion_pose.name.at(j))
      {
        right_arm_target_point.at(i).position = right_motion_pose.position.at(j);
        right_arm_target_point.at(i).velocity = right_motion_pose.velocity.at(j);
        right_arm_target_point.at(i).effort = right_motion_pose.effort.at(j);
      }
    }
  }
  // Play
  right_arm_.makeJointTrajectory(right_arm_target_point, time);

  // Right hand
  std::vector<std::string> right_finger_name = right_arm_.getManipulator()->getAllToolComponentName();
  for(int8_t i=0; i<right_finger_name.size();i++)
  {
    for(int8_t j=0; j<right_motion_pose.name.size(); j++)
    {
      if(right_finger_name.at(i) == right_motion_pose.name.at(j))
      {
        // Play
        right_arm_.makeToolTrajectory(right_motion_pose.name.at(j), right_motion_pose.position.at(j));
      }
    }
  }

  // update and check cnt
  pose_cnt_++;
  if(motion_scenario_.at(motion_cnt_).left_arm_motion.size() <= pose_cnt_)
  {
    pose_cnt_=0;
    repeat_cnt_++;

    if(motion_stop_flag_)
    {
      motion_playing_=false;
      log::info("Motion stop due to motion stop request.");
      motion_stop_flag_ = false;
    }

    if(motion_break_flag_)
    {
      repeat_cnt_=0;
      motion_cnt_++;
      log::info("Skip to next motion due to motion break request.");
      motion_break_flag_ = false;
      if(motion_scenario_.size() <= motion_cnt_)
      {
        motion_cnt_=0;
        motion_playing_=false;
        log::info("All motion scenarios are complete.");
      }
    }

    if(motion_scenario_.at(motion_cnt_).repeat_count <= 0)      // infinity repeat
      return time;

    if(motion_scenario_.at(motion_cnt_).repeat_count <= repeat_cnt_)
    {
      repeat_cnt_=0;
      motion_cnt_++;
      if(motion_scenario_.size() <= motion_cnt_)
      {
        motion_cnt_=0;
        motion_playing_=false;
        log::info("All motion scenarios are complete.");
      }
    }
  }
  return time;
}
/*****************************************************************************
 ** Motion Thread Functions
*****************************************************************************/
void SocialRobotArmController::startMotionPlayThread()
{
  if(left_arm_.getMode()=="normal_mode" && right_arm_.getMode()=="normal_mode")
  {
    int error;
    motion_playing_ = true;
    if ((error = pthread_create(&this->motion_play_thread_, NULL, this->motionPlayThread, this)) != 0)
    {
      log::error("Creating motion play thread failed!!", (double)error);
      motion_playing_ = false;
      exit(-1);
    }
    log::info("Start Motion Playing.");

  }
  else
  {
    log::error("Can not start motion play while gravity compensation mode.");
  }
}

void SocialRobotArmController::stopMotionPlayThread()
{
  if(motion_playing_)
  {
    motion_playing_ = false;
    pthread_join(motion_play_thread_, NULL); // Wait for the thread associated with thread_p to complete
    log::info("Stop Motion Playing.");
  }
}

void *SocialRobotArmController::motionPlayThread(void *param)
{
  SocialRobotArmController *controller = (SocialRobotArmController *) param;
  struct timespec end_time;
  struct timespec curr_time;

  while(controller->motion_playing_)
  {
    // Make Trajectory
    log::print("[INFO] Motion Playing. ","GREEN");
    double move_time = controller->motionPlay();

    // Time Count
    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    end_time.tv_sec  = curr_time.tv_sec + (int64_t)move_time + (curr_time.tv_nsec + ((int64_t)(move_time*1000.0)*(int64_t)1000000%(int64_t)1000000000))/(int64_t)1000000000;
    end_time.tv_nsec = (curr_time.tv_nsec + ((int64_t)(move_time*1000.0)*(int64_t)1000000%(int64_t)1000000000))%(int64_t)1000000000;

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &end_time, nullptr);
    log::print(" / time: ", curr_time.tv_sec,0,"GREEN");
    log::print(".", curr_time.tv_nsec,0,"GREEN");
    log::print("s~", end_time.tv_sec,0,"GREEN");
    log::print(".", end_time.tv_nsec,0,"GREEN");
    log::println("s","GREEN");
  }

  return nullptr;
}

