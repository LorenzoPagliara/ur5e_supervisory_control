/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   haptic_ff_controller.cpp
 * Author:  Michele Marsico, Salvatore Paolino
 * Org.:    UNISA
 * Date:    Oct 2, 2024
 *
 * This module implements the HapticFeedforwardController class, a controller that
 * reads the pose of the haptic device, and create the robot pose reference.
 *
 * -------------------------------------------------------------------
 */

#include "haptic_ff_controller/haptic_ff_controller.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>

namespace haptic_ff_controller
{
HapticFeedforwardController::HapticFeedforwardController() = default;

HapticFeedforwardController::~HapticFeedforwardController() = default;

controller_interface::CallbackReturn HapticFeedforwardController::on_init()
{
  // Parameter handler
  parameter_handler_ = std::make_shared<haptic_ff_controller::ParamListener>(get_node());

  // kinematic configuration
  // the kinematics_interface requires the node to have the `robot_description` parameter. When working with a real robot, this parameter can be set
  // directly from the launch file. however, in Ignition Gazebo simulation, this is not possible. Therefore, when using simulation, the
  // `robot_description` parameter must be declared as a parameter of this node.
  if (!get_node()->has_parameter("robot_description"))
  {
    // retrieve the robot description from the node robot_state_publisher
    auto parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(get_node(), "/robot_state_publisher");
    std::chrono::duration<int, std::milli> ms(1000);
    while (!parameters_client_->wait_for_service(ms))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("HapticFeedforwardController"), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(rclcpp::get_logger("HapticFeedforwardController"), "Service not available, waiting again...");
    }

    std::string urdf_string;
    auto parameters = parameters_client_->get_parameters({ "robot_description" });
    for (auto& parameter : parameters)
    {
      if (parameter.get_name() == "robot_description")
      {
        urdf_string = parameter.value_to_string();
        break;
      }
    }

    // declaring the robot description as parameter of this node
    get_node()->declare_parameter("robot_description", urdf_string);
  }

  // load the differential IK plugin
  if (!parameter_handler_->get_params().kinematics.plugin_name.empty())
  {
    try
    {
      kinematics_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
          parameter_handler_->get_params().kinematics.plugin_package, "kinematics_interface::KinematicsInterface");
      kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
          kinematics_loader_->createUnmanagedInstance(parameter_handler_->get_params().kinematics.plugin_name));
      if (!kinematics_->initialize(get_node().get()->get_node_parameters_interface(), parameter_handler_->get_params().kinematics.tip))
      {
        return controller_interface::CallbackReturn::ERROR;
      }
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("HapticFeedforwardController"), "Exception while loading the IK plugin '%s': '%s'",
                   parameter_handler_->get_params().kinematics.plugin_name.c_str(), ex.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("HapticFeedforwardController"), "A differential IK plugin name was not specified in the config file.");
    return controller_interface::CallbackReturn::ERROR;
  }
  // Terminate the initialization of kinematics_interface

  // Initialize the robot and haptic device poses
  haptic_device_init_transform_ = Eigen::Isometry3d::Identity();

  // Initialize engagement variables
  robot_engaged_ = false;

  // Initialize the flag for start reading the haptic device pose from topic into simulation scenario
  haptic_device_pose_topic_ready_ = false;
  haptic_device_twist_topic_ready_ = false;

  number_of_channels_ = 7;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HapticFeedforwardController::command_interface_configuration() const
{
  std::vector<std::string> command_interfaces_config_names;
  std::string downstream_controller_name = parameter_handler_->get_params().downstream_controller_name;

  // Create the configuration names for the command interfaces
  if (has_position_command_interface_)
  {
    command_interfaces_config_names.push_back(downstream_controller_name + "/position.x");
    command_interfaces_config_names.push_back(downstream_controller_name + "/position.y");
    command_interfaces_config_names.push_back(downstream_controller_name + "/position.z");
    command_interfaces_config_names.push_back(downstream_controller_name + "/orientation.x");
    command_interfaces_config_names.push_back(downstream_controller_name + "/orientation.y");
    command_interfaces_config_names.push_back(downstream_controller_name + "/orientation.z");
    command_interfaces_config_names.push_back(downstream_controller_name + "/orientation.w");
  }

  if (has_velocity_command_interface_)
  {
    command_interfaces_config_names.push_back(downstream_controller_name + "/linear_velocity.x");
    command_interfaces_config_names.push_back(downstream_controller_name + "/linear_velocity.y");
    command_interfaces_config_names.push_back(downstream_controller_name + "/linear_velocity.z");
    command_interfaces_config_names.push_back(downstream_controller_name + "/angular_velocity.x");
    command_interfaces_config_names.push_back(downstream_controller_name + "/angular_velocity.y");
    command_interfaces_config_names.push_back(downstream_controller_name + "/angular_velocity.z");
  }

  if (!has_position_command_interface_ && !has_velocity_command_interface_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HapticFeedforwardController"),
                 "No command interface specified. The command interface must be either position or velocity.");
  }

  return { controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces_config_names };
}

controller_interface::InterfaceConfiguration HapticFeedforwardController::state_interface_configuration() const
{
  std::vector<std::string> state_interfaces_config_names;

  // Add first the state interfaces to read the state of joints of the robot
  for (const auto& joint : parameter_handler_->get_params().joints)
  {
    auto full_name = joint + "/position";
    state_interfaces_config_names.push_back(full_name);
  }

  // When the slave and the master are in the same scenario the haptic state is read from the command interface
  if (!simulation_)
  {
    std::string haptic_device_name = parameter_handler_->get_params().haptic_device_name;

    if (has_position_state_interface_)
    {
      state_interfaces_config_names.push_back(haptic_device_name + "/position.x");
      state_interfaces_config_names.push_back(haptic_device_name + "/position.y");
      state_interfaces_config_names.push_back(haptic_device_name + "/position.z");
      state_interfaces_config_names.push_back(haptic_device_name + "/orientation.x");
      state_interfaces_config_names.push_back(haptic_device_name + "/orientation.y");
      state_interfaces_config_names.push_back(haptic_device_name + "/orientation.z");
      state_interfaces_config_names.push_back(haptic_device_name + "/orientation.w");
    }

    if (has_velocity_state_interface_)
    {
      state_interfaces_config_names.push_back(haptic_device_name + "/linear_velocity.x");
      state_interfaces_config_names.push_back(haptic_device_name + "/linear_velocity.y");
      state_interfaces_config_names.push_back(haptic_device_name + "/linear_velocity.z");
      state_interfaces_config_names.push_back(haptic_device_name + "/angular_velocity.x");
      state_interfaces_config_names.push_back(haptic_device_name + "/angular_velocity.y");
      state_interfaces_config_names.push_back(haptic_device_name + "/angular_velocity.z");
    }

    if (!has_position_state_interface_ && !has_velocity_state_interface_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("HapticFeedforwardController"),
                   "No state interface correctly specified. The state interface must be either position or velocity.");
    }
  }

  return { controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names };
}

controller_interface::CallbackReturn HapticFeedforwardController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Configure the engagement threshold
  engagement_orientation_threshold_ = parameter_handler_->get_params().engagement_orientation_threshold;

  // Configure the scaling factor
  scaling_factor_ = Eigen::Vector3d(parameter_handler_->get_params().scaling_factor[0], parameter_handler_->get_params().scaling_factor[1],
                                    parameter_handler_->get_params().scaling_factor[2]);

  // Configure the robot base frame and the robot end-effector frame
  robot_base_frame_ = parameter_handler_->get_params().robot_base_frame;
  robot_ee_frame_ = parameter_handler_->get_params().robot_ee_frame;

  auto command_interfaces = parameter_handler_->get_params().command_interfaces;
  auto state_interfaces = parameter_handler_->get_params().state_interfaces;

  // Check which command interfaces are available
  has_position_command_interface_ = std::find(command_interfaces.begin(), command_interfaces.end(), "position") != command_interfaces.end();
  has_velocity_command_interface_ = std::find(command_interfaces.begin(), command_interfaces.end(), "velocity") != command_interfaces.end();

  // Check which state interfaces are available
  has_position_state_interface_ = std::find(state_interfaces.begin(), state_interfaces.end(), "position") != state_interfaces.end();
  has_velocity_state_interface_ = std::find(state_interfaces.begin(), state_interfaces.end(), "velocity") != state_interfaces.end();

  // Configure the subscriber for haptic device pose read from topic into distant scenario
  simulation_ = parameter_handler_->get_params().simulation;

  if (has_position_state_interface_)
  {
    haptic_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        parameter_handler_->get_params().haptic_pose_subscriber.topic_name, parameter_handler_->get_params().haptic_pose_subscriber.queue_size,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          topic_haptic_pose_ = *msg;
          haptic_device_pose_topic_ready_ = true;
        });
  }

  if (has_velocity_state_interface_)
  {
    haptic_twist_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
        parameter_handler_->get_params().haptic_twist_subscriber.topic_name, parameter_handler_->get_params().haptic_twist_subscriber.queue_size,
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg)
        {
          topic_haptic_twist_ = *msg;
          haptic_device_twist_topic_ready_ = true;
        });
  }

  // Initialize the last haptic pose
  last_robot_pose_.position.x = 0.0;
  last_robot_pose_.position.y = 0.0;
  last_robot_pose_.position.z = 0.0;
  last_robot_pose_.orientation.x = 0.0;
  last_robot_pose_.orientation.y = 0.0;
  last_robot_pose_.orientation.z = 0.0;
  last_robot_pose_.orientation.w = 0.0;

  try
  {
    // Configure the publisher haptic robot poses for engage in rviz
    haptic_pose_rviz_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        parameter_handler_->get_params().haptic_pose_rviz_publisher.topic_name, rclcpp::SystemDefaultsQoS());
    realtime_haptic_pose_rviz_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(haptic_pose_rviz_publisher_);

    // Configure the publisher for task space references logging
    reference_pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
        parameter_handler_->get_params().reference_pose_publisher.topic_name, rclcpp::SystemDefaultsQoS());
    realtime_reference_pose_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(reference_pose_publisher_);

    reference_twist_publisher_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
        parameter_handler_->get_params().reference_twist_publisher.topic_name, rclcpp::SystemDefaultsQoS());
    realtime_reference_twist_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(reference_twist_publisher_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during publisher configuration at configure stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize topic messages
  realtime_haptic_pose_rviz_publisher_->lock();
  realtime_haptic_pose_rviz_publisher_->msg_.header.frame_id = parameter_handler_->get_params().haptic_pose_rviz_publisher.frame_id;
  realtime_haptic_pose_rviz_publisher_->unlock();

  realtime_reference_pose_publisher_->lock();
  realtime_reference_pose_publisher_->msg_.header.frame_id = parameter_handler_->get_params().reference_pose_publisher.frame_id;
  realtime_reference_pose_publisher_->unlock();

  realtime_reference_twist_publisher_->lock();
  realtime_reference_twist_publisher_->msg_.header.frame_id = parameter_handler_->get_params().reference_twist_publisher.frame_id;
  realtime_reference_twist_publisher_->unlock();

  // Configure the rotation matrix from RPY angles from robot base to haptic base
  initRotationFromRPY_angles_(robot_base_to_haptic_base_, parameter_handler_->get_params().RPY_robot_base_to_haptic_base);

  // Configure the rotation matrix from RPY angles from haptic ee to tool tip
  initRotationFromRPY_angles_(haptic_ee_to_tool_tip_, parameter_handler_->get_params().RPY_haptic_ee_to_tool_tip);

  // Configure the isometry from robot ee to TCP
  std::vector<double> robot_ee_to_TCP_param = parameter_handler_->get_params().robot_ee_to_TCP;
  Eigen::Matrix3d robot_ee_to_TCP_rotation_matrix;
  const std::vector<double> robot_ee_to_TCP_RPY_angles = { robot_ee_to_TCP_param[0], robot_ee_to_TCP_param[1], robot_ee_to_TCP_param[2] };
  initRotationFromRPY_angles_(robot_ee_to_TCP_rotation_matrix, robot_ee_to_TCP_RPY_angles);
  Eigen::Vector3d robot_ee_to_TCP_translation_vector(robot_ee_to_TCP_param[3], robot_ee_to_TCP_param[4], robot_ee_to_TCP_param[5]);
  robot_ee_to_TCP_ =
      Eigen::Isometry3d(Eigen::Translation3d(robot_ee_to_TCP_translation_vector) * Eigen::Quaterniond(robot_ee_to_TCP_rotation_matrix));

  // Configure the moving average pose filter
  bool configured = false;
  configured = compute_average_pose_filter_chain_.configure(number_of_channels_, "filter_chain", get_node()->get_node_logging_interface(),
                                                            get_node()->get_node_parameters_interface());
  if (!configured)
  {
    RCLCPP_ERROR(rclcpp::get_logger("HapticFeedforwardController"), "Failed to configure the filter chain");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Waiting for the first message by topic into simulation scenario
  if (simulation_)
  {
    wait_read_topic_haptic_pose_();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticFeedforwardController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Get the state interfaces size
  robot_state_interface_size_ = parameter_handler_->get_params().joints.size();

  // Save the initial robot joint state for init_command_robot_transform_
  robot_init_joint_state_.resize(robot_state_interface_size_);
  for (size_t i = 0; i < robot_state_interface_size_; ++i)
  {
    robot_init_joint_state_[i] = state_interfaces_[i].get_value();
  }

  // Initialize the init_command_robot_transform_
  kinematics_->calculate_link_transform(robot_init_joint_state_, robot_base_frame_, robot_world_to_base_transform_);
  kinematics_->calculate_link_transform(robot_init_joint_state_, robot_ee_frame_, robot_world_to_ee_transform_);
  init_command_robot_transform_ = robot_world_to_base_transform_.inverse() * robot_world_to_ee_transform_;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticFeedforwardController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HapticFeedforwardController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  geometry_msgs::msg::TwistStamped robot_twist_stamped;
  robot_twist_stamped.twist.linear.x = 0.0;
  robot_twist_stamped.twist.linear.y = 0.0;
  robot_twist_stamped.twist.linear.z = 0.0;
  robot_twist_stamped.twist.angular.x = 0.0;
  robot_twist_stamped.twist.angular.y = 0.0;
  robot_twist_stamped.twist.angular.z = 0.0;

  if (robot_engaged_ == false)
  {
    engageRobot();

    // If exists the command interface for the position, it is always before the velocity command interfaces
    if (has_position_command_interface_)
    {
      // Apply the initial haptic robot pose
      Eigen::Matrix3d rotation_matrix_init_command_robot_transform_ = init_command_robot_transform_.rotation();
      Eigen::Quaterniond quaternion_init_command_robot_transform_(rotation_matrix_init_command_robot_transform_);

      last_robot_pose_.position.x = init_command_robot_transform_.translation().x();
      last_robot_pose_.position.y = init_command_robot_transform_.translation().y();
      last_robot_pose_.position.z = init_command_robot_transform_.translation().z();
      last_robot_pose_.orientation.x = quaternion_init_command_robot_transform_.x();
      last_robot_pose_.orientation.y = quaternion_init_command_robot_transform_.y();
      last_robot_pose_.orientation.z = quaternion_init_command_robot_transform_.z();
      last_robot_pose_.orientation.w = quaternion_init_command_robot_transform_.w();

      command_interfaces_[0].set_value(last_robot_pose_.position.x);
      command_interfaces_[1].set_value(last_robot_pose_.position.y);
      command_interfaces_[2].set_value(last_robot_pose_.position.z);
      command_interfaces_[3].set_value(last_robot_pose_.orientation.x);  // x
      command_interfaces_[4].set_value(last_robot_pose_.orientation.y);  // y
      command_interfaces_[5].set_value(last_robot_pose_.orientation.z);  // z
      command_interfaces_[6].set_value(last_robot_pose_.orientation.w);  // w

      // This is the case when is also enabled the velocity control
      if (has_velocity_command_interface_)
      {
        command_interfaces_[7].set_value(robot_twist_stamped.twist.linear.x);
        command_interfaces_[8].set_value(robot_twist_stamped.twist.linear.y);
        command_interfaces_[9].set_value(robot_twist_stamped.twist.linear.z);
        command_interfaces_[10].set_value(robot_twist_stamped.twist.angular.x);
        command_interfaces_[11].set_value(robot_twist_stamped.twist.angular.y);
        command_interfaces_[12].set_value(robot_twist_stamped.twist.angular.z);
      }
    }
    else
    {
      // This is the case when is enabled only the velocity control
      command_interfaces_[0].set_value(robot_twist_stamped.twist.linear.x);
      command_interfaces_[1].set_value(robot_twist_stamped.twist.linear.y);
      command_interfaces_[2].set_value(robot_twist_stamped.twist.linear.z);
      command_interfaces_[3].set_value(robot_twist_stamped.twist.angular.x);
      command_interfaces_[4].set_value(robot_twist_stamped.twist.angular.y);
      command_interfaces_[5].set_value(robot_twist_stamped.twist.angular.z);
    }

    if (has_velocity_command_interface_)
    {
      // Publish the initial twist of the robot
      if (realtime_reference_twist_publisher_ && realtime_reference_twist_publisher_->trylock())
      {
        realtime_reference_twist_publisher_->msg_.header.stamp = get_node()->now();
        realtime_reference_twist_publisher_->msg_.twist = robot_twist_stamped.twist;
        realtime_reference_twist_publisher_->unlockAndPublish();
      }
    }

    return controller_interface::return_type::OK;
  }

  read_haptic_state_();

  // Apply the moving average filter to the haptic pose
  filtered_haptic_pose_ = computeAveragePose_(haptic_pose_);

  geometry_msgs::msg::Twist robot_twist;
  geometry_msgs::msg::Pose haptic_robot_pose;

  // Get the haptic_robot_pose_ applying the conversion from haptic device pose to robot end-effector pose
  if (parameter_handler_->get_params().velocity_control && has_velocity_command_interface_ && has_velocity_state_interface_)
  {
    convertHapticEEPoseToRobotEEPose2_(filtered_haptic_pose_, haptic_twist_, haptic_robot_pose_, robot_twist, period);
  }
  else
  {
    convertHapticEEPoseToRobotEEPose_(filtered_haptic_pose_, haptic_robot_pose_);

    if (has_velocity_command_interface_)
    {
      KDL::Frame last_robot_pose = KDL::Frame(KDL::Rotation::Quaternion(last_robot_pose_.orientation.x, last_robot_pose_.orientation.y,
                                                                        last_robot_pose_.orientation.z, last_robot_pose_.orientation.w),
                                              KDL::Vector(last_robot_pose_.position.x, last_robot_pose_.position.y, last_robot_pose_.position.z));

      KDL::Frame robot_pose = KDL::Frame(KDL::Rotation::Quaternion(haptic_robot_pose_.orientation.x, haptic_robot_pose_.orientation.y,
                                                                   haptic_robot_pose_.orientation.z, haptic_robot_pose_.orientation.w),
                                         KDL::Vector(haptic_robot_pose_.position.x, haptic_robot_pose_.position.y, haptic_robot_pose_.position.z));

      KDL::Twist robot_twist_kdl = KDL::diff(last_robot_pose, robot_pose, period.seconds());

      robot_twist.linear.x = robot_twist_kdl.vel[0];
      robot_twist.linear.y = robot_twist_kdl.vel[1];
      robot_twist.linear.z = robot_twist_kdl.vel[2];
      robot_twist.angular.x = robot_twist_kdl.rot[0];
      robot_twist.angular.y = robot_twist_kdl.rot[1];
      robot_twist.angular.z = robot_twist_kdl.rot[2];

      last_robot_pose_ = haptic_robot_pose_;
    }
  }

  // Publish the robot twist
  if (realtime_reference_twist_publisher_ && realtime_reference_twist_publisher_->trylock())
  {
    realtime_reference_twist_publisher_->msg_.header.stamp = get_node()->now();
    realtime_reference_twist_publisher_->msg_.twist = robot_twist;
    realtime_reference_twist_publisher_->unlockAndPublish();
  }

  // Publish the message for logging
  if (realtime_reference_pose_publisher_ && realtime_reference_pose_publisher_->trylock())
  {
    realtime_reference_pose_publisher_->msg_.header.stamp = get_node()->now();
    realtime_reference_pose_publisher_->msg_.pose = haptic_robot_pose_;
    realtime_reference_pose_publisher_->unlockAndPublish();
  }

  // Write the command haptic_robot_pose_ to the command interfaces
  if (has_position_command_interface_ && command_interfaces_.size() >= 7)
  {
    command_interfaces_[0].set_value(haptic_robot_pose_.position.x);
    command_interfaces_[1].set_value(haptic_robot_pose_.position.y);
    command_interfaces_[2].set_value(haptic_robot_pose_.position.z);
    command_interfaces_[3].set_value(haptic_robot_pose_.orientation.x);  // x
    command_interfaces_[4].set_value(haptic_robot_pose_.orientation.y);  // y
    command_interfaces_[5].set_value(haptic_robot_pose_.orientation.z);  // z
    command_interfaces_[6].set_value(haptic_robot_pose_.orientation.w);  // w

    if (has_velocity_command_interface_ && command_interfaces_.size() >= 13)
    {
      command_interfaces_[7].set_value(robot_twist.linear.x);
      command_interfaces_[8].set_value(robot_twist.linear.y);
      command_interfaces_[9].set_value(robot_twist.linear.z);
      command_interfaces_[10].set_value(robot_twist.angular.x);
      command_interfaces_[11].set_value(robot_twist.angular.y);
      command_interfaces_[12].set_value(robot_twist.angular.z);
    }
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("HapticFeedforwardController"), "Not enough command interfaces available");
  }

  // Visualize the commanded robot pose in RViz

  // Read the robot joint state for direct kinematics
  read_robot_state_(robot_joint_state_);
  kinematics_->calculate_link_transform(robot_joint_state_, robot_ee_frame_, robot_world_to_ee_transform_);
  base_to_end_effector_actual_transform_ = robot_world_to_base_transform_.inverse() * robot_world_to_ee_transform_;
  read_haptic_state_();
  placeHapticEEFrameAtToolTip_(haptic_pose_, base_to_end_effector_actual_transform_);

  return controller_interface::return_type::OK;
}

geometry_msgs::msg::Pose HapticFeedforwardController::computeAveragePose_(geometry_msgs::msg::Pose& haptic_device_pose)
{
  compute_average_pose_input_data_ = { haptic_device_pose.position.x,    haptic_device_pose.position.y,    haptic_device_pose.position.z,
                                       haptic_device_pose.orientation.x, haptic_device_pose.orientation.y, haptic_device_pose.orientation.z,
                                       haptic_device_pose.orientation.w };
  compute_average_pose_output_data_.assign(compute_average_pose_input_data_.size(), 0.0);
  compute_average_pose_filter_chain_.update(compute_average_pose_input_data_, compute_average_pose_output_data_);

  geometry_msgs::msg::Pose output_pose;
  output_pose.position.x = compute_average_pose_output_data_[0];
  output_pose.position.y = compute_average_pose_output_data_[1];
  output_pose.position.z = compute_average_pose_output_data_[2];
  output_pose.orientation.x = compute_average_pose_output_data_[3];
  output_pose.orientation.y = compute_average_pose_output_data_[4];
  output_pose.orientation.z = compute_average_pose_output_data_[5];
  output_pose.orientation.w = compute_average_pose_output_data_[6];

  return output_pose;
}

void HapticFeedforwardController::engageRobot()
{
  // Read the robot joint state for direct kinematics
  read_robot_state_(robot_joint_state_);

  // Calculate the transform from the robot base to the end-effector
  kinematics_->calculate_link_transform(robot_joint_state_, robot_ee_frame_, robot_world_to_ee_transform_);
  base_to_end_effector_actual_transform_ = robot_world_to_base_transform_.inverse() * robot_world_to_ee_transform_;

  // In Rviz place the haptic end-effector frame at the tool tip
  read_haptic_state_();
  placeHapticEEFrameAtToolTip_(haptic_pose_, base_to_end_effector_actual_transform_);

  // Convert the haptic device orientation to the robot end-effector orientation for engagement check
  Eigen::Matrix3d robot_orientation_pose;
  read_haptic_state_();
  convertHapticEEOrientationToRobotEEOrientation_(haptic_pose_, robot_orientation_pose);

  // Compute relative rotation between current robot orientation and current converted haptic device orientation
  Eigen::Matrix3d transform_relative_rotation_eigen_ = base_to_end_effector_actual_transform_.rotation().transpose() * robot_orientation_pose;
  tf2::Matrix3x3 transform_relative_rotation(
      transform_relative_rotation_eigen_(0, 0), transform_relative_rotation_eigen_(0, 1), transform_relative_rotation_eigen_(0, 2),
      transform_relative_rotation_eigen_(1, 0), transform_relative_rotation_eigen_(1, 1), transform_relative_rotation_eigen_(1, 2),
      transform_relative_rotation_eigen_(2, 0), transform_relative_rotation_eigen_(2, 1), transform_relative_rotation_eigen_(2, 2));

  // Convert the relative rotation to Euler angles, in this case we use tf2::Matrix3x3 because
  // the tf2::Matrix3x3::getRPY() function is more stable than the eigen equivalent
  double roll, pitch, yaw;
  transform_relative_rotation.getRPY(roll, pitch, yaw);

  // Check if the haptic orientation is within the engagement threshold
  if (abs(angles::shortest_angular_distance(roll, 0.0)) < engagement_orientation_threshold_ &&
      abs(angles::shortest_angular_distance(pitch, 0.0)) < engagement_orientation_threshold_ &&
      abs(angles::shortest_angular_distance(yaw, 0.0)) < engagement_orientation_threshold_)
  {
    RCLCPP_INFO(rclcpp::get_logger("HapticFeedforwardController"), "Engagement Orientation completed");

    // Set the initial pose of the haptic device
    read_haptic_state_();
    Eigen::Isometry3d haptic_device_start_pose = Eigen::Isometry3d(
        Eigen::Translation3d(haptic_pose_.position.x, haptic_pose_.position.y, haptic_pose_.position.z) *
        Eigen::Quaterniond(haptic_pose_.orientation.w, haptic_pose_.orientation.x, haptic_pose_.orientation.y, haptic_pose_.orientation.z));

    // Set the haptic device and the robot starting transforms
    setRobotAndHapticDeviceEEPose_(haptic_device_start_pose, base_to_end_effector_actual_transform_);

    // Convert the haptic device pose to the robot end-effector pose
    geometry_msgs::msg::Pose robot_pose;
    read_haptic_state_();
    convertHapticEEPoseToRobotEEPose_(haptic_pose_, robot_pose);

    // Check if the robot pose matches the calculated transform
    if (robot_pose.position.x == base_to_end_effector_actual_transform_.translation().x() &&
        robot_pose.position.y == base_to_end_effector_actual_transform_.translation().y() &&
        robot_pose.position.z == base_to_end_effector_actual_transform_.translation().z())
    {
      // Engagement completed
      RCLCPP_INFO(rclcpp::get_logger("HapticFeedforwardController"), "Engagement Position completed");
      RCLCPP_INFO(rclcpp::get_logger("HapticFeedforwardController"), "Engagement completed");
      robot_engaged_ = true;
    }
  }
}

void haptic_ff_controller::HapticFeedforwardController::placeHapticEEFrameAtToolTip_(
    const geometry_msgs::msg::Pose& haptic_device_pose, const Eigen::Isometry3d& base_to_end_effector_actual_transform) const
{
  // Convert the haptic device pose to an Eigen::Isometry3d
  Eigen::Isometry3d haptic_base_to_haptic_ee_transform =
      Eigen::Isometry3d(Eigen::Translation3d(haptic_device_pose.position.x, haptic_device_pose.position.y, haptic_device_pose.position.z) *
                        Eigen::Quaterniond(haptic_device_pose.orientation.w, haptic_device_pose.orientation.x, haptic_device_pose.orientation.y,
                                           haptic_device_pose.orientation.z));

  // Get the rotation matrix from the haptic base frame to the haptic end-effector frame
  Eigen::Matrix3d haptic_base_to_haptic_ee = haptic_base_to_haptic_ee_transform.rotation();

  // Create the rotation matrix from the robot base frame to the robot end-effector frame
  Eigen::Matrix3d robot_base_to_robot_ee =
      robot_base_to_haptic_base_ * haptic_base_to_haptic_ee * haptic_ee_to_tool_tip_ * robot_ee_to_TCP_.rotation().transpose();

  // Merge the haptic device orientation with tool tip position
  Eigen::Isometry3d reference_tool_transform =
      Eigen::Isometry3d(Eigen::Translation3d((base_to_end_effector_actual_transform.translation() +
                                              base_to_end_effector_actual_transform.rotation() * robot_ee_to_TCP_.translation())) *
                        Eigen::Quaterniond(robot_base_to_robot_ee * robot_ee_to_TCP_.rotation()));

  // Publish the reference haptic robot pose in RViz
  geometry_msgs::msg::Pose reference_tool_pose;
  reference_tool_pose.position.x = reference_tool_transform.translation().x();
  reference_tool_pose.position.y = reference_tool_transform.translation().y();
  reference_tool_pose.position.z = reference_tool_transform.translation().z();

  Eigen::Quaterniond reference_tool_quat(reference_tool_transform.rotation());
  reference_tool_pose.orientation.w = reference_tool_quat.w();
  reference_tool_pose.orientation.x = reference_tool_quat.x();
  reference_tool_pose.orientation.y = reference_tool_quat.y();
  reference_tool_pose.orientation.z = reference_tool_quat.z();

  if (realtime_haptic_pose_rviz_publisher_ && realtime_haptic_pose_rviz_publisher_->trylock())
  {
    realtime_haptic_pose_rviz_publisher_->msg_.header.stamp = get_node()->now();
    realtime_haptic_pose_rviz_publisher_->msg_.pose = reference_tool_pose;
    realtime_haptic_pose_rviz_publisher_->unlockAndPublish();
  }
}

void haptic_ff_controller::HapticFeedforwardController::setRobotAndHapticDeviceEEPose_(const Eigen::Isometry3d& haptic_device_transform,
                                                                                       const Eigen::Isometry3d& robot_ee_transform)
{
  haptic_device_init_transform_ = haptic_device_transform;
  robot_base_to_ee_init_transform_ = robot_ee_transform;
  robot_previous_transform_ = robot_base_to_ee_init_transform_;
  is_start_transform_set_ = true;
}

void HapticFeedforwardController::initRotationFromRPY_angles_(Eigen::Matrix3d& rotation_matrix, const std::vector<double>& RPY_angles) const
{
  if (RPY_angles.size() != 3)
  {
    throw std::runtime_error("RPY_angles must contain exactly 3 elements.");
  }

  // Compute the quaternion from RPY angles, corresponding to a rotation
  // around X (roll) then around Y (pitch) and then around Z (yaw) in fixed frame
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(RPY_angles[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(RPY_angles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(RPY_angles[0], Eigen::Vector3d::UnitX());
  rotation_matrix = q.toRotationMatrix();

  RCLCPP_DEBUG(rclcpp::get_logger("HapticFeedforwardController"),
               "\nRotation Matrix\n"
               "%.10f %.10f %.10f\n"
               "%.10f %.10f %.10f\n"
               "%.10f %.10f %.10f\n",
               rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2), rotation_matrix(1, 0), rotation_matrix(1, 1),
               rotation_matrix(1, 2), rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
}

void HapticFeedforwardController::read_robot_state_(Eigen::VectorXd& robot_joint_state) const
{
  // Read the robot joint state
  robot_joint_state.resize(robot_state_interface_size_);
  for (size_t i = 0; i < robot_state_interface_size_; ++i)
  {
    robot_joint_state[i] = state_interfaces_[i].get_value();
  }
}

void HapticFeedforwardController::convertHapticEEPoseToRobotEEPose_(const geometry_msgs::msg::Pose& haptic_device_pose,
                                                                    geometry_msgs::msg::Pose& robot_pose)
{
  if (!is_start_transform_set_)
    throw haptic_ff_controller::Exception("The initial position of the robot and haptic device must be set");

  // Get the rotation matrix from the haptic base frame to the haptic end-effector frame
  Eigen::Isometry3d haptic_base_to_haptic_ee_transform =
      Eigen::Isometry3d(Eigen::Translation3d(haptic_device_pose.position.x, haptic_device_pose.position.y, haptic_device_pose.position.z) *
                        Eigen::Quaterniond(haptic_device_pose.orientation.w, haptic_device_pose.orientation.x, haptic_device_pose.orientation.y,
                                           haptic_device_pose.orientation.z));

  Eigen::Matrix3d haptic_base_to_haptic_ee = haptic_base_to_haptic_ee_transform.rotation();

  // Reference transformation of the robot
  Eigen::Isometry3d robot_base_to_robot_ee_transform = Eigen::Isometry3d::Identity();

  // Calculate the desired displacement of the robot
  Eigen::Vector3d robot_ee_displacement =
      robot_base_to_haptic_base_ *
      (scaling_factor_.cwiseProduct(haptic_base_to_haptic_ee_transform.translation() - haptic_device_init_transform_.translation()));

  // Calculate the desired orientation of the robot
  Eigen::Matrix3d robot_base_to_robot_ee =
      robot_base_to_haptic_base_ * haptic_base_to_haptic_ee * haptic_ee_to_tool_tip_ * robot_ee_to_TCP_.rotation().transpose();

  // Calculate the reference transformation of the robot
  robot_base_to_robot_ee_transform =
      Eigen::Isometry3d(Eigen::Translation3d(robot_base_to_ee_init_transform_.translation() + robot_ee_displacement +
                                             robot_base_to_ee_init_transform_.rotation() * robot_ee_to_TCP_.translation() -
                                             robot_base_to_robot_ee_transform.rotation() * robot_ee_to_TCP_.translation()) *
                        Eigen::Quaterniond(robot_base_to_robot_ee));

  // Calculate the reference pose of the robot
  robot_pose.position.x = robot_base_to_robot_ee_transform.translation().x();
  robot_pose.position.y = robot_base_to_robot_ee_transform.translation().y();
  robot_pose.position.z = robot_base_to_robot_ee_transform.translation().z();
  Eigen::Quaterniond robot_orientation(robot_base_to_robot_ee_transform.rotation());
  robot_pose.orientation.w = robot_orientation.w();
  robot_pose.orientation.x = robot_orientation.x();
  robot_pose.orientation.y = robot_orientation.y();
  robot_pose.orientation.z = robot_orientation.z();
}

void HapticFeedforwardController::convertHapticEEPoseToRobotEEPose2_(const geometry_msgs::msg::Pose& haptic_device_pose,
                                                                     const geometry_msgs::msg::Twist& haptic_device_twist,
                                                                     geometry_msgs::msg::Pose& robot_pose, geometry_msgs::msg::Twist& robot_twist,
                                                                     const rclcpp::Duration& period)
{
  if (!is_start_transform_set_)
    throw haptic_ff_controller::Exception("The initial position of the robot and haptic device must be set");

  // Get the rotation matrix from the haptic base frame to the haptic end-effector frame
  Eigen::Isometry3d haptic_base_to_haptic_ee_transform =
      Eigen::Isometry3d(Eigen::Translation3d(haptic_device_pose.position.x, haptic_device_pose.position.y, haptic_device_pose.position.z) *
                        Eigen::Quaterniond(haptic_device_pose.orientation.w, haptic_device_pose.orientation.x, haptic_device_pose.orientation.y,
                                           haptic_device_pose.orientation.z));

  // Get the haptic device stylus twist in the haptic device space
  Eigen::VectorXd haptic_ee_twist;
  haptic_ee_twist.resize(6);
  haptic_ee_twist << haptic_device_twist.linear.x, haptic_device_twist.linear.y, haptic_device_twist.linear.z, haptic_device_twist.angular.x,
      haptic_device_twist.angular.y, haptic_device_twist.angular.z;

  Eigen::Matrix3d R_z = Eigen::Matrix3d::Identity();

  // Compute the robot end-effector twist in the robot space
  Eigen::Vector3d robot_linear_velocity =
      robot_previous_transform_.rotation() *
      (scaling_factor_.asDiagonal() * (robot_ee_to_TCP_.rotation() * haptic_ee_to_tool_tip_.transpose() * R_z *
                                       haptic_base_to_haptic_ee_transform.rotation().transpose() * haptic_ee_twist.head(3)));

  Eigen::Vector3d robot_angular_velocity =
      robot_previous_transform_.rotation() * ((robot_ee_to_TCP_.rotation() * haptic_ee_to_tool_tip_.transpose() * R_z *
                                               haptic_base_to_haptic_ee_transform.rotation().transpose() * haptic_ee_twist.tail(3)));

  // Output the desired robot twist
  robot_twist.linear.x = robot_linear_velocity[0];
  robot_twist.linear.y = robot_linear_velocity[1];
  robot_twist.linear.z = robot_linear_velocity[2];
  robot_twist.angular.x = robot_angular_velocity[0];
  robot_twist.angular.y = robot_angular_velocity[1];
  robot_twist.angular.z = robot_angular_velocity[2];

  // Compute the desired robot end-effector twist
  KDL::Twist desired_robot_ee_twist(KDL::Vector(robot_linear_velocity[0], robot_linear_velocity[1], robot_linear_velocity[2]),
                                    KDL::Vector(robot_angular_velocity[0], robot_angular_velocity[1], robot_angular_velocity[2]));

  // RCLCPP_INFO(rclcpp::get_logger("HapticFeedforwardController"), "Desired robot twist: %f %f %f %f %f %f",
  //             desired_robot_ee_twist.vel[0], desired_robot_ee_twist.vel[1], desired_robot_ee_twist.vel[2],
  //             desired_robot_ee_twist.rot[0], desired_robot_ee_twist.rot[1], desired_robot_ee_twist.rot[2]);

  // Compute the desired robot pose
  KDL::Frame pose_increment = KDL::addDelta(transformToKDLFrame_(robot_previous_transform_), desired_robot_ee_twist, period.seconds());
  geometry_msgs::msg::TransformStamped transform_msg = tf2::kdlToTransform(pose_increment);

  Eigen::Isometry3d robot_base_to_robot_ee_transform = tf2::transformToEigen(transform_msg);

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("HapticFeedforwardController"), "Current Robot position: " <<
  // robot_base_to_robot_ee_transform.translation());

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("HapticFeedforwardController"), "Current Robot rotation: " << robot_base_to_robot_ee_transform.rotation());

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("HapticFeedforwardController"), "Previous Robot position: " << robot_previous_transform_.translation());

  // RCLCPP_INFO_STREAM(rclcpp::get_logger("HapticFeedforwardController"), "Previous Robot rotation: " << robot_previous_transform_.rotation());

  robot_base_to_robot_ee_transform.translation() = robot_base_to_robot_ee_transform.translation() +
                                                   robot_previous_transform_.rotation() * robot_ee_to_TCP_.translation() -
                                                   robot_base_to_robot_ee_transform.rotation() * robot_ee_to_TCP_.translation();

  // Output the desired robot pose
  robot_pose.position.x = robot_base_to_robot_ee_transform.translation().x();
  robot_pose.position.y = robot_base_to_robot_ee_transform.translation().y();
  robot_pose.position.z = robot_base_to_robot_ee_transform.translation().z();
  Eigen::Quaterniond q(robot_base_to_robot_ee_transform.rotation());
  robot_pose.orientation.x = q.x();
  robot_pose.orientation.y = q.y();
  robot_pose.orientation.z = q.z();
  robot_pose.orientation.w = q.w();

  // Read the robot joint state for direct kinematics
  read_robot_state_(robot_joint_state_);
  kinematics_->calculate_link_transform(robot_joint_state_, robot_ee_frame_, robot_world_to_ee_transform_);
  base_to_end_effector_actual_transform_ = robot_world_to_base_transform_.inverse() * robot_world_to_ee_transform_;

  // Set the actual pose of the robot and of the haptic device
  robot_previous_transform_ = base_to_end_effector_actual_transform_;
}

void HapticFeedforwardController::convertHapticEEOrientationToRobotEEOrientation_(const geometry_msgs::msg::Pose& haptic_device_pose,
                                                                                  Eigen::Matrix3d& robot_orientation_pose)
{
  // Convert the haptic device pose to an Eigen::Isometry3d
  Eigen::Isometry3d haptic_base_to_haptic_ee_transform =
      Eigen::Isometry3d(Eigen::Translation3d(haptic_device_pose.position.x, haptic_device_pose.position.y, haptic_device_pose.position.z) *
                        Eigen::Quaterniond(haptic_device_pose.orientation.w, haptic_device_pose.orientation.x, haptic_device_pose.orientation.y,
                                           haptic_device_pose.orientation.z));

  // Extract the rotation matrix from the haptic base frame to the haptic end-effector frame
  Eigen::Matrix3d haptic_base_to_haptic_ee = haptic_base_to_haptic_ee_transform.rotation();

  // Calculate the robot orientation pose
  robot_orientation_pose = robot_base_to_haptic_base_ * haptic_base_to_haptic_ee * haptic_ee_to_tool_tip_ * robot_ee_to_TCP_.rotation().transpose();
}

void HapticFeedforwardController::read_haptic_state_()
{
  if (!simulation_)
  {
    if (has_position_state_interface_)
    {
      haptic_pose_.position.x = state_interfaces_[robot_state_interface_size_ + 0].get_value();
      haptic_pose_.position.y = state_interfaces_[robot_state_interface_size_ + 1].get_value();
      haptic_pose_.position.z = state_interfaces_[robot_state_interface_size_ + 2].get_value();
      haptic_pose_.orientation.x = state_interfaces_[robot_state_interface_size_ + 3].get_value();
      haptic_pose_.orientation.y = state_interfaces_[robot_state_interface_size_ + 4].get_value();
      haptic_pose_.orientation.z = state_interfaces_[robot_state_interface_size_ + 5].get_value();
      haptic_pose_.orientation.w = state_interfaces_[robot_state_interface_size_ + 6].get_value();

      if (has_velocity_state_interface_)
      {
        haptic_twist_.linear.x = state_interfaces_[robot_state_interface_size_ + 7].get_value();
        haptic_twist_.linear.y = state_interfaces_[robot_state_interface_size_ + 8].get_value();
        haptic_twist_.linear.z = state_interfaces_[robot_state_interface_size_ + 9].get_value();
        haptic_twist_.angular.x = state_interfaces_[robot_state_interface_size_ + 10].get_value();
        haptic_twist_.angular.y = state_interfaces_[robot_state_interface_size_ + 11].get_value();
        haptic_twist_.angular.z = state_interfaces_[robot_state_interface_size_ + 12].get_value();
      }
    }
    else
    {
      haptic_twist_.linear.x = state_interfaces_[robot_state_interface_size_ + 0].get_value();
      haptic_twist_.linear.y = state_interfaces_[robot_state_interface_size_ + 1].get_value();
      haptic_twist_.linear.z = state_interfaces_[robot_state_interface_size_ + 2].get_value();
      haptic_twist_.angular.x = state_interfaces_[robot_state_interface_size_ + 3].get_value();
      haptic_twist_.angular.y = state_interfaces_[robot_state_interface_size_ + 4].get_value();
      haptic_twist_.angular.z = state_interfaces_[robot_state_interface_size_ + 5].get_value();
    }
  }
  else
  {
    if (haptic_device_pose_topic_ready_ && has_position_state_interface_)
      haptic_pose_ = topic_haptic_pose_.pose;
    if (haptic_device_twist_topic_ready_ && has_velocity_state_interface_)
      haptic_twist_ = topic_haptic_twist_.twist;
  }
}

void HapticFeedforwardController::wait_read_topic_haptic_pose_()
{
  RCLCPP_INFO(rclcpp::get_logger("HapticFeedforwardController"), "Wait for haptic device pose topic");
  while ((!haptic_device_pose_topic_ready_ && has_position_state_interface_) or (!haptic_device_twist_topic_ready_ && has_velocity_state_interface_))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(rclcpp::get_logger("HapticFeedforwardController"), "Waiting for haptic device pose topic");
  }
}

KDL::Frame HapticFeedforwardController::transformToKDLFrame_(Eigen::Isometry3d transform)
{
  geometry_msgs::msg::TransformStamped transform_msg = tf2::eigenToTransform(transform);
  return tf2::transformToKDL(transform_msg);
}

}  // namespace haptic_ff_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(haptic_ff_controller::HapticFeedforwardController, controller_interface::ControllerInterface)
