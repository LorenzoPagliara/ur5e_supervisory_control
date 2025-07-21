/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   haptic_fb_controller.cpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    Oct 3, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "haptic_fb_controller/haptic_fb_controller.hpp"
#include <vector>

using std::placeholders::_1;

namespace haptic_fb_controller
{
HapticFeedbackController::HapticFeedbackController() {}

HapticFeedbackController::~HapticFeedbackController() {}

controller_interface::CallbackReturn HapticFeedbackController::on_init()
{
  try
  {
    parameter_handler_ = std::make_shared<haptic_fb_controller::ParamListener>(get_node());
    simulation_ = parameter_handler_->get_params().simulation;
    read_force_torque_from_topic_ = parameter_handler_->get_params().read_force_torque_from_topic;
    enable_motion_error_ = parameter_handler_->get_params().enable_motion_error;
    is_sensor_bias_computed_ = false;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // The kinematics interface requires the node to have the `robot_description` parameter. When working with a real robot, this parameter can be set
  // directly from the launch file. However, in Ignition Gazebo simulation, this is not possible. Therefore, when using simulation, the
  // `robot_description` parameter must be declared as a parameter of this node.
  std::string robot_description;
  if (!get_node()->has_parameter("robot_description"))
  {
    // Retrieve the robot description from the node robot_state_publisher
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client =
        std::make_shared<rclcpp::SyncParametersClient>(get_node(), "robot_state_publisher");
    std::chrono::duration<int, std::milli> ms(1000);
    while (!parameters_client->wait_for_service(ms))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_node()->get_logger(), "Service not available, waiting again...");
    }

    robot_description = parameters_client->get_parameter<std::string>("robot_description");

    // Declaring the robot description as parameter of this node
    get_node()->declare_parameter("robot_description", robot_description);
  }
  else
  {
    get_node()->get_parameter("robot_description", robot_description);
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
        RCLCPP_ERROR(get_node()->get_logger(), "The Kinematics plugin '%s' failed to initialize. Using the link '%s' as tip link.",
                     parameter_handler_->get_params().kinematics.plugin_name.c_str(), parameter_handler_->get_params().kinematics.tip.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(get_node()->get_logger(), "Kinematics plugin loaded successfully");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception while loading the IK plugin '%s': '%s'",
                   parameter_handler_->get_params().kinematics.plugin_name.c_str(), ex.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "A differential IK plugin name was not specified in the config file.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize the force filter chain
  force_filter_chain_ = std::make_shared<filters::MultiChannelFilterChain<double>>("double");

  // Initialize the motion error filter chain
  if (enable_motion_error_)
  {
    motion_error_filter_chain_ = std::make_shared<filters::MultiChannelFilterChain<double>>("double");
  }

  // allocate dynamic memory for the filter chain
  filter_chain_input_.assign(11, 0.0);
  filter_chain_output_.assign(11, 0.0);

  cartesian_motion_error_filter_chain_input_.assign(6, 0.0);
  cartesian_motion_error_filter_chain_output_.assign(6, 0.0);

  input_wrench_ = geometry_msgs::msg::Wrench();
  output_wrench_ = geometry_msgs::msg::WrenchStamped();
  position_error_haptic_base_ = geometry_msgs::msg::Vector3Stamped();
  velocity_error_haptic_base_ = geometry_msgs::msg::Vector3Stamped();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HapticFeedbackController::command_interface_configuration() const
{
  if (!simulation_ && !enable_motion_error_)
  {
    controller_interface::InterfaceConfiguration command_interface_config;
    command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& haptic_force_controller_interfaces : parameter_handler_->get_params().haptic_force_controller_interfaces)
    {
      auto full_name = parameter_handler_->get_params().haptic_force_controller + "/" + haptic_force_controller_interfaces;
      command_interface_config.names.push_back(full_name);
    }
    return command_interface_config;
  }
  else if (!simulation_ && enable_motion_error_)
  {
    controller_interface::InterfaceConfiguration command_interface_config;
    command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interface_config.names.push_back(parameter_handler_->get_params().haptic_force_controller + "/position_error/x");
    command_interface_config.names.push_back(parameter_handler_->get_params().haptic_force_controller + "/position_error/y");
    command_interface_config.names.push_back(parameter_handler_->get_params().haptic_force_controller + "/position_error/z");
    command_interface_config.names.push_back(parameter_handler_->get_params().haptic_force_controller + "/velocity_error/x");
    command_interface_config.names.push_back(parameter_handler_->get_params().haptic_force_controller + "/velocity_error/y");
    command_interface_config.names.push_back(parameter_handler_->get_params().haptic_force_controller + "/velocity_error/z");
    return command_interface_config;
  }
  else
    return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration HapticFeedbackController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // The first state interface is the position interface
  auto position_interface = parameter_handler_->get_params().state_interfaces[0];

  // RCLCPP_INFO(get_node()->get_logger(), "Name of the first state interface: %s", position_interface.c_str());

  // The other state interfaces are the joints
  for (const auto& joint : parameter_handler_->get_params().joints)
  {
    auto full_name = joint + "/" + position_interface;
    // RCLCPP_INFO(get_node()->get_logger(), "Full name of the state interface: %s", full_name.c_str());
    state_interface_config.names.push_back(full_name);
  }

  // If the force and torque aren't read from the topic, then the force and torque are the last 6 state interfaces
  if (!read_force_torque_from_topic_)
  {
    for (size_t i = 1; i < parameter_handler_->get_params().state_interfaces.size(); i++)
    {
      state_interface_config.names.push_back(parameter_handler_->get_params().state_interfaces[i]);
    }
    return state_interface_config;
  }
  return state_interface_config;
}

controller_interface::CallbackReturn HapticFeedbackController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Configure the robot base frame and the robot end-effector frame
  robot_base_frame_ = parameter_handler_->get_params().robot_base_frame;
  robot_ee_frame_ = parameter_handler_->get_params().robot_ee_frame;

  // Configure the rotation matrix from RPY angles from robot base to haptic base
  std::vector<double> RPY_angles = parameter_handler_->get_params().RPY_robot_base_to_haptic_base;

  if (RPY_angles.size() != 3)
    throw std::runtime_error("RPY_angles must contain exactly 3 elements.");

  // Compute the quaternion from RPY angles, corresponding to a rotation
  // around X (roll) then around Y (pitch) and then around Z (yaw) in fixed frame
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(RPY_angles[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(RPY_angles[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(RPY_angles[0], Eigen::Vector3d::UnitX());
  robot_base_to_haptic_base_ = q.toRotationMatrix();

  // Configure state and command interfaces
  if (parameter_handler_->get_params().state_interfaces.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces specified. You have to specify at least the position state interface.");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!simulation_)
  {
    if (parameter_handler_->get_params().command_interfaces.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces specified");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Configure the force filter chain
  if (!force_filter_chain_->configure(11, "force_filter_chain", get_node()->get_node_logging_interface(),
                                      get_node()->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The force filter chain hasn't been correctly configured.");
    return controller_interface::CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "The force filter chain has been correctly configured.");
  }

  // Configure the publishers
  try
  {
    wrench_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(parameter_handler_->get_params().wrench_publisher.topic_name,
                                                                                        rclcpp::SystemDefaultsQoS());
    realtime_wrench_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(wrench_publisher_);

    if (enable_motion_error_)
    {
      position_error_publisher_ = get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
          parameter_handler_->get_params().position_error_publisher.topic_name, rclcpp::SystemDefaultsQoS());
      realtime_position_error_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(position_error_publisher_);

      velocity_error_publisher_ = get_node()->create_publisher<geometry_msgs::msg::Vector3Stamped>(
          parameter_handler_->get_params().velocity_error_publisher.topic_name, rclcpp::SystemDefaultsQoS());
      realtime_velocity_error_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(velocity_error_publisher_);
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during publisher configuration at configure stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  realtime_wrench_publisher_->lock();
  realtime_wrench_publisher_->msg_.header.frame_id = parameter_handler_->get_params().wrench_publisher.frame_id;
  realtime_wrench_publisher_->unlock();

  if (enable_motion_error_)
  {
    // Configure the motion error filter chain
    if (!motion_error_filter_chain_->configure(6, "motion_error_filter_chain", get_node()->get_node_logging_interface(),
                                               get_node()->get_node_parameters_interface()))
    {
      RCLCPP_ERROR(get_node()->get_logger(), "The position error filter chain hasn't been correctly configured.");
      return controller_interface::CallbackReturn::ERROR;
    }
    else
    {
      RCLCPP_INFO(get_node()->get_logger(), "The position error filter chain has been correctly configured.");
    }

    realtime_position_error_publisher_->lock();
    realtime_position_error_publisher_->msg_.header.frame_id = parameter_handler_->get_params().position_error_publisher.frame_id;
    realtime_position_error_publisher_->unlock();

    realtime_velocity_error_publisher_->lock();
    realtime_velocity_error_publisher_->msg_.header.frame_id = parameter_handler_->get_params().velocity_error_publisher.frame_id;
    realtime_velocity_error_publisher_->unlock();

    // Configure the subscriber for the Admittance state
    admittance_state_subscriber_ = get_node()->create_subscription<acg_control_msgs::msg::AdmittanceControllerState>(
        parameter_handler_->get_params().admittance_state_subscriber.topic_name,
        parameter_handler_->get_params().admittance_state_subscriber.queue_size,
        [this](const acg_control_msgs::msg::AdmittanceControllerState::SharedPtr msg) { admittance_state_msg_ = *msg; });
  }

  // Create the subscriber for the force and torque sensor
  if (read_force_torque_from_topic_)
  {
    wrench_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        parameter_handler_->get_params().wrench_subscriber.topic_name, parameter_handler_->get_params().wrench_subscriber.queue_size,
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { wrench_msg_ = *msg; });
  }

  // TODO: Remove this lines when change the base class from controller_interface::ChainableControllerInterface to
  // controller_interface::ControllerInterface
  reference_interface_names_.push_back("");
  reference_interfaces_.resize(reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticFeedbackController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!read_force_torque_from_topic_)
    robot_state_interface_size_ = state_interfaces_.size() - 6;
  else
    robot_state_interface_size_ = state_interfaces_.size();

  // RCLCPP_INFO(get_node()->get_logger(), "Size of robot state interface: %d", robot_state_interface_size_);

  // Set the transform from the robot world to the robot base and end-effector
  robot_joint_state_.resize(robot_state_interface_size_);
  read_robot_state_(state_interfaces_, robot_state_interface_size_, robot_joint_state_);

  if (!kinematics_->calculate_link_transform(robot_joint_state_, robot_base_frame_, robot_world_to_base_transform_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error while calculating the link transform");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!kinematics_->calculate_link_transform(robot_joint_state_, robot_ee_frame_, robot_world_to_ee_transform_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error while calculating the link transform");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (is_sensor_bias_computed_ == false)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Waiting for the sensor bias filter to became reliable.");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticFeedbackController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> HapticFeedbackController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interface_names_.size());

  for (size_t i = 0; i < reference_interface_names_.size(); i++)
  {
    reference_interfaces.push_back(
        hardware_interface::CommandInterface(get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

controller_interface::return_type HapticFeedbackController::update_and_write_commands(const rclcpp::Time& /*time*/,
                                                                                      const rclcpp::Duration& /*period*/)
{
  read_robot_state_(state_interfaces_, robot_state_interface_size_, robot_joint_state_);

  if (!kinematics_->calculate_link_transform(robot_joint_state_, robot_ee_frame_, robot_world_to_ee_transform_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error while calculating the link transform");
    return controller_interface::return_type::ERROR;
  }
  base_to_end_effector_actual_transform_ = robot_world_to_base_transform_.inverse() * robot_world_to_ee_transform_;

  // Reading force and torque from the topic or from the state interfaces
  if (read_force_torque_from_topic_)
  {
    // RCLCPP_INFO(rclcpp::get_logger("HapticFeedbackController"), "Force raw: x: %f, y: %f, z: %f", wrench_msg_.force.x, wrench_msg_.force.y,
    // wrench_msg_.force.z);
    input_wrench_ = wrench_msg_.wrench;
  }
  else
  {
    // RCLCPP_INFO(rclcpp::get_logger("HapticFeedbackController"), "Forze lette: x: %f, y: %f, z: %f",
    // state_interfaces_[robot_state_interface_size_].get_value(), state_interfaces_[robot_state_interface_size_+1].get_value(),
    // state_interfaces_[robot_state_interface_size_+2].get_value());
    input_wrench_.force.x = state_interfaces_[robot_state_interface_size_].get_value();
    input_wrench_.force.y = state_interfaces_[robot_state_interface_size_ + 1].get_value();
    input_wrench_.force.z = state_interfaces_[robot_state_interface_size_ + 2].get_value();
    input_wrench_.torque.x = state_interfaces_[robot_state_interface_size_ + 3].get_value();
    input_wrench_.torque.y = state_interfaces_[robot_state_interface_size_ + 4].get_value();
    input_wrench_.torque.z = state_interfaces_[robot_state_interface_size_ + 5].get_value();
  }

  // Filter the force and torque
  filter_chain_input_[0] = input_wrench_.force.x;
  filter_chain_input_[1] = input_wrench_.force.y;
  filter_chain_input_[2] = input_wrench_.force.z;
  filter_chain_input_[3] = input_wrench_.torque.x;
  filter_chain_input_[4] = input_wrench_.torque.y;
  filter_chain_input_[5] = input_wrench_.torque.z;

  // Extract the orientation from the transform's rotation part and convert it to a quaternion
  Eigen::Quaterniond orientation(base_to_end_effector_actual_transform_.rotation());
  filter_chain_input_[6] = orientation.inverse().w();
  filter_chain_input_[7] = orientation.inverse().x();
  filter_chain_input_[8] = orientation.inverse().y();
  filter_chain_input_[9] = orientation.inverse().z();

  // Update the filter chain
  force_filter_chain_->update(filter_chain_input_, filter_chain_output_);

  // RCLCPP_INFO(rclcpp::get_logger("HapticFeedbackController"), "Force filtered: x: %f, y: %f, z: %f", filter_chain_output_[0],
  // filter_chain_output_[1], filter_chain_output_[2]);

  if (filter_chain_output_[10] != 0.0 && is_sensor_bias_computed_ == false)  // the 10th output is the reliability of the filter
  {
    is_sensor_bias_computed_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "The filters have become reliable, allowing for the use of the controller now.");
  }

  // Convert the force from the robot end-effector frame to the haptic base frame
  force_vector_ << filter_chain_output_[0], filter_chain_output_[1], filter_chain_output_[2];
  torque_vector_ << filter_chain_output_[3], filter_chain_output_[4], filter_chain_output_[5];

  // RCLCPP_INFO(get_node()->get_logger(), "FORZE PRIMA CONVERSIONE: x: %f, y: %f, z: %f", force_vector_.x(), force_vector_.y(), force_vector_.z());

  HapticFeedbackController::convertRobotEEForceToHapticBaseForce_(force_vector_, base_to_end_effector_actual_transform_, haptic_base_force_);
  HapticFeedbackController::convertRobotEEForceToHapticBaseForce_(torque_vector_, base_to_end_effector_actual_transform_, haptic_base_torque_);

  // RCLCPP_INFO(get_node()->get_logger(), "FORZE DOPO CONVERSIONE: x: %f, y: %f, z: %f", haptic_base_force_.x(), haptic_base_force_.y(),
  // haptic_base_force_.z());

  // If the sensor bias has been computed, then the filtered force and torque can be used, otherwise they are set to zero
  if (is_sensor_bias_computed_)
  {
    force_vector_[0] = haptic_base_force_.x();
    force_vector_[1] = haptic_base_force_.y();
    force_vector_[2] = haptic_base_force_.z();
    torque_vector_[0] = haptic_base_torque_.x();
    torque_vector_[1] = haptic_base_torque_.y();
    torque_vector_[2] = haptic_base_torque_.z();
  }
  else
  {
    force_vector_.setZero();
    torque_vector_.setZero();
  }

  // Log the wrench
  output_wrench_.wrench.force.x = force_vector_[0];
  output_wrench_.wrench.force.y = force_vector_[1];
  output_wrench_.wrench.force.z = force_vector_[2];
  output_wrench_.wrench.torque.x = torque_vector_[0];
  output_wrench_.wrench.torque.y = torque_vector_[1];
  output_wrench_.wrench.torque.z = torque_vector_[2];

  // Publish wrench using real time mechanisms
  if (realtime_wrench_publisher_ && realtime_wrench_publisher_->trylock())
  {
    realtime_wrench_publisher_->msg_.header.stamp = get_node()->now();
    realtime_wrench_publisher_->msg_ = output_wrench_;
    realtime_wrench_publisher_->unlockAndPublish();
  }

  if (!enable_motion_error_)
  {
    // Write the force and torque to the command interfaces
    if (!simulation_)
    {
      command_interfaces_[0].set_value(force_vector_[0]);
      command_interfaces_[1].set_value(force_vector_[1]);
      command_interfaces_[2].set_value(force_vector_[2]);
      command_interfaces_[3].set_value(torque_vector_[0]);
      command_interfaces_[4].set_value(torque_vector_[1]);
      command_interfaces_[5].set_value(torque_vector_[2]);
      // RCLCPP_INFO(rclcpp::get_logger("HapticFeedbackController"), "Force: x: %f, y: %f, z: %f", command_interfaces_[0].get_value(),
      // command_interfaces_[1].get_value(), command_interfaces_[2].get_value());
    }
  }
  else
  {
    // Get the current position and velocity error in end-effector frame
    position_error_ << admittance_state_msg_.admittance_pose_error.position.x, admittance_state_msg_.admittance_pose_error.position.y,
        admittance_state_msg_.admittance_pose_error.position.z;
    velocity_error_ << admittance_state_msg_.admittance_twist_error.linear.x, admittance_state_msg_.admittance_twist_error.linear.y,
        admittance_state_msg_.admittance_twist_error.linear.z;

    // Convert the error from the robot control frame to the haptic base frame
    haptic_base_position_error_ = robot_base_to_haptic_base_.inverse() * position_error_;
    haptic_base_velocity_error_ = robot_base_to_haptic_base_.inverse() * velocity_error_;

    // RCLCPP_INFO(get_node()->get_logger(), "Position error haptic base: x: %f, y: %f, z: %f", haptic_base_position_error_.x(),
    // haptic_base_position_error_.y(), haptic_base_position_error_.z()); RCLCPP_INFO(get_node()->get_logger(), "Linear velocity error haptic base: x:
    // %f, y: %f, z: %f", haptic_base_velocity_error_.x(), haptic_base_velocity_error_.y(), haptic_base_velocity_error_.z());

    // Update the filter only when the input isn't Nan
    if (!std::isnan(haptic_base_position_error_.x()) && !std::isnan(haptic_base_position_error_.y()) &&
        !std::isnan(haptic_base_position_error_.z()) && !std::isnan(haptic_base_velocity_error_.x()) &&
        !std::isnan(haptic_base_velocity_error_.y()) && !std::isnan(haptic_base_velocity_error_.z()))
    {
      // Filter the position and velocity error
      cartesian_motion_error_filter_chain_input_[0] = haptic_base_position_error_.x();
      cartesian_motion_error_filter_chain_input_[1] = haptic_base_position_error_.y();
      cartesian_motion_error_filter_chain_input_[2] = haptic_base_position_error_.z();
      cartesian_motion_error_filter_chain_input_[3] = haptic_base_velocity_error_.x();
      cartesian_motion_error_filter_chain_input_[4] = haptic_base_velocity_error_.y();
      cartesian_motion_error_filter_chain_input_[5] = haptic_base_velocity_error_.z();

      motion_error_filter_chain_->update(cartesian_motion_error_filter_chain_input_, cartesian_motion_error_filter_chain_output_);

      // RCLCPP_INFO(get_node()->get_logger(), "Position error haptic base filtered: x: %f, y: %f, z: %f",
      // cartesian_motion_error_filter_chain_output_[0], cartesian_motion_error_filter_chain_output_[1],
      // cartesian_motion_error_filter_chain_output_[2]);
    }

    if (force_vector_.norm() < parameter_handler_->get_params().force_deadband)
    {
      cartesian_motion_error_filter_chain_output_[0] = 0.0;
      cartesian_motion_error_filter_chain_output_[1] = 0.0;
      cartesian_motion_error_filter_chain_output_[2] = 0.0;
      cartesian_motion_error_filter_chain_output_[3] = 0.0;
      cartesian_motion_error_filter_chain_output_[4] = 0.0;
      cartesian_motion_error_filter_chain_output_[5] = 0.0;
    }

    // Log the position and velocity error
    position_error_haptic_base_.vector.x = cartesian_motion_error_filter_chain_output_[0];
    position_error_haptic_base_.vector.y = cartesian_motion_error_filter_chain_output_[1];
    position_error_haptic_base_.vector.z = cartesian_motion_error_filter_chain_output_[2];

    velocity_error_haptic_base_.vector.x = cartesian_motion_error_filter_chain_output_[3];
    velocity_error_haptic_base_.vector.y = cartesian_motion_error_filter_chain_output_[4];
    velocity_error_haptic_base_.vector.z = cartesian_motion_error_filter_chain_output_[5];

    // Publish position and velocity error using real time mechanisms
    if (realtime_position_error_publisher_ && realtime_position_error_publisher_->trylock())
    {
      realtime_position_error_publisher_->msg_.header.stamp = get_node()->now();
      realtime_position_error_publisher_->msg_ = position_error_haptic_base_;
      realtime_position_error_publisher_->unlockAndPublish();
    }
    if (realtime_velocity_error_publisher_ && realtime_velocity_error_publisher_->trylock())
    {
      realtime_velocity_error_publisher_->msg_.header.stamp = get_node()->now();
      realtime_velocity_error_publisher_->msg_ = velocity_error_haptic_base_;
      realtime_velocity_error_publisher_->unlockAndPublish();
    }

    // Write the filtered position and velocity error to the command interfaces
    if (!simulation_)
    {
      command_interfaces_[0].set_value(cartesian_motion_error_filter_chain_output_[0]);
      command_interfaces_[1].set_value(cartesian_motion_error_filter_chain_output_[1]);
      command_interfaces_[2].set_value(cartesian_motion_error_filter_chain_output_[2]);
      command_interfaces_[3].set_value(cartesian_motion_error_filter_chain_output_[3]);
      command_interfaces_[4].set_value(cartesian_motion_error_filter_chain_output_[4]);
      command_interfaces_[5].set_value(cartesian_motion_error_filter_chain_output_[5]);
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type HapticFeedbackController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

bool HapticFeedbackController::on_set_chained_mode(bool chained_mode)
{
  if (chained_mode)
    RCLCPP_INFO(get_node()->get_logger(), "Chained mode enabled");
  return true;
}

void HapticFeedbackController::convertRobotEEForceToHapticBaseForce_(const Eigen::Vector3d& robot_end_effector_force,
                                                                     const Eigen::Isometry3d& base_to_end_effector_transform,
                                                                     Eigen::Vector3d& haptic_base_force) const
{
  // Convert force in the robot end effector frame to robot base force
  Eigen::Vector3d robot_base_frame_force = base_to_end_effector_transform.rotation() * robot_end_effector_force;

  // Convert force in the robot base frame to haptic base force
  haptic_base_force = robot_base_to_haptic_base_.inverse() * robot_base_frame_force;
}

void HapticFeedbackController::read_robot_state_(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces,
                                                 const size_t robot_state_interface_size, Eigen::VectorXd& robot_joint_state) const
{
  // Read the robot joint state
  for (size_t i = 0; i < robot_state_interface_size; ++i)
  {
    robot_joint_state[i] = state_interfaces[i].get_value();
  }
}

}  // namespace haptic_fb_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(haptic_fb_controller::HapticFeedbackController, controller_interface::ChainableControllerInterface)
