/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   haptic_forward_force_controller.cpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    Oct 11, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "haptic_forward_force_controller/haptic_forward_force_controller.hpp"
#include <vector>

using std::placeholders::_1;

namespace haptic_forward_force_controller
{
HapticForwardForceController::HapticForwardForceController() {}

HapticForwardForceController::~HapticForwardForceController() {}

controller_interface::CallbackReturn HapticForwardForceController::on_init()
{
  try
  {
    parameter_handler_ = std::make_shared<haptic_forward_force_controller::ParamListener>(get_node());
    simulation_ = parameter_handler_->get_params().simulation;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HapticForwardForceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interface_config.names = parameter_handler_->get_params().command_interfaces;
  return command_interface_config;
}

controller_interface::InterfaceConfiguration HapticForwardForceController::state_interface_configuration() const
{
  if (!simulation_)
  {
    controller_interface::InterfaceConfiguration state_interface_config;
    state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interface_config.names = parameter_handler_->get_params().state_interfaces;
    return state_interface_config;
  }
  else
    return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::CallbackReturn HapticForwardForceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
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

  if (parameter_handler_->get_params().command_interfaces.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!simulation_)
  {
    reference_interface_names_.push_back("force/x");
    reference_interface_names_.push_back("force/y");
    reference_interface_names_.push_back("force/z");
    reference_interface_names_.push_back("torque/x");
    reference_interface_names_.push_back("torque/y");
    reference_interface_names_.push_back("torque/z");
    reference_interfaces_.resize(reference_interface_names_.size(), 0.0);
  }
  else
  {
    reference_interface_names_.push_back("");
    reference_interfaces_.resize(reference_interface_names_.size(), 0.0);
  }

  // Configure publisher and subscriber
  if (simulation_)
  {
    wrench_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
        parameter_handler_->get_params().topic_name, 1, std::bind(&HapticForwardForceController::wrench_callback_, this, _1));
  }
  actuated_wrench_publisher_ =
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("/hd/haptic_forward_force_controller/actuated_forces", 1);

  realtime_wrench_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(actuated_wrench_publisher_);

  // Initialize variables
  haptic_base_force_ = Eigen::Vector3d::Zero();
  mapped_haptic_base_force_ = Eigen::Vector3d::Zero();
  actuated_wrench_ = geometry_msgs::msg::WrenchStamped();
  wrench_msg_ = geometry_msgs::msg::Wrench();
  max_haptic_device_force_ = Eigen::Vector3d::Zero();
  max_robot_force_ = Eigen::Vector3d::Zero();

  // Configure the maximum force that can be exerted by the haptic device
  HapticForwardForceController::updateMaxHapticDeviceForce_();

  // Configure the maximum force that can be exerted by the robot
  HapticForwardForceController::updateMaxRobotForce_();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticForwardForceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticForwardForceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> HapticForwardForceController::on_export_reference_interfaces()
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

controller_interface::return_type HapticForwardForceController::update_and_write_commands(const rclcpp::Time& /*time*/,
                                                                                          const rclcpp::Duration& /*period*/)
{
  if (simulation_)
  {
    haptic_base_force_.x() = wrench_msg_.force.x;
    haptic_base_force_.y() = wrench_msg_.force.y;
    haptic_base_force_.z() = wrench_msg_.force.z;
  }
  else
  {
    haptic_base_force_.x() = reference_interfaces_[0];
    haptic_base_force_.y() = reference_interfaces_[1];
    haptic_base_force_.z() = reference_interfaces_[2];
  }

  // Update the maximum force that can be exerted by the haptic device and the robot, and map the haptic base force into the haptic range
  HapticForwardForceController::updateMaxHapticDeviceForce_();
  HapticForwardForceController::updateMaxRobotForce_();
  HapticForwardForceController::mapHapticBaseForceIntoHapticRange_(haptic_base_force_, mapped_haptic_base_force_);

  // Write the force to the command interfaces
  command_interfaces_[0].set_value(mapped_haptic_base_force_.x());
  command_interfaces_[1].set_value(mapped_haptic_base_force_.y());
  command_interfaces_[2].set_value(mapped_haptic_base_force_.z());

  // Publish the actuated force
  actuated_wrench_.header.stamp = get_node()->get_clock()->now();
  actuated_wrench_.wrench.force.x = mapped_haptic_base_force_.x();
  actuated_wrench_.wrench.force.y = mapped_haptic_base_force_.y();
  actuated_wrench_.wrench.force.z = mapped_haptic_base_force_.z();
  actuated_wrench_.wrench.torque.x = 0.0;
  actuated_wrench_.wrench.torque.y = 0.0;
  actuated_wrench_.wrench.torque.z = 0.0;

  // Publish wrench using real time mechanisms
  if (realtime_wrench_publisher_->trylock())
  {
    realtime_wrench_publisher_->msg_ = actuated_wrench_;
    realtime_wrench_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type HapticForwardForceController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

bool HapticForwardForceController::on_set_chained_mode(bool chained_mode)
{
  if (chained_mode)
    RCLCPP_INFO(get_node()->get_logger(), "Chained mode enabled");
  return true;
}

void HapticForwardForceController::wrench_callback_(const geometry_msgs::msg::WrenchStamped& msg)
{
  wrench_msg_ = msg.wrench;
}

void HapticForwardForceController::mapHapticBaseForceIntoHapticRange_(const Eigen::Vector3d& haptic_base_force,
                                                                      Eigen::Vector3d& mapped_haptic_base_force) const
{
  std::vector<double> haptic_base_frame_force_vec{ haptic_base_force.x(), haptic_base_force.y(), haptic_base_force.z() };

  // RCLCPP_INFO(get_node()->get_logger(), "FORZE PRIMA DI MAPPING: x: %f, y: %f, z: %f", haptic_base_frame_force_vec[0],
  // haptic_base_frame_force_vec[1], haptic_base_frame_force_vec[2]);

  // Map haptic base force into the haptic range
  for (std::size_t i = 0; i < haptic_base_frame_force_vec.size(); i++)
  {
    double alpha = 1 / max_robot_force_[i] * std::log(4 + std::sqrt(15));

    if (std::signbit(haptic_base_frame_force_vec[i]))
      haptic_base_frame_force_vec[i] = -(std::cosh(haptic_base_frame_force_vec[i] * alpha) - 1);
    else
      haptic_base_frame_force_vec[i] = std::cosh(haptic_base_frame_force_vec[i] * alpha) - 1;

    haptic_base_frame_force_vec[i] = std::max(-max_haptic_device_force_[i], std::min(haptic_base_frame_force_vec[i], max_haptic_device_force_[i]));
  }

  // RCLCPP_INFO(get_node()->get_logger(), "FORZE DOPO MAPPING: x: %f, y: %f, z: %f", haptic_base_frame_force_vec[0], haptic_base_frame_force_vec[1],
  // haptic_base_frame_force_vec[2]);

  // Set the haptic feedback
  mapped_haptic_base_force.x() = haptic_base_frame_force_vec[0];
  mapped_haptic_base_force.y() = haptic_base_frame_force_vec[1];
  mapped_haptic_base_force.z() = haptic_base_frame_force_vec[2];
}

void HapticForwardForceController::updateMaxHapticDeviceForce_()
{
  max_haptic_device_force_.x() = parameter_handler_->get_params().max_haptic_device_force.x;
  max_haptic_device_force_.y() = parameter_handler_->get_params().max_haptic_device_force.y;
  max_haptic_device_force_.z() = parameter_handler_->get_params().max_haptic_device_force.z;
}

void HapticForwardForceController::updateMaxRobotForce_()
{
  // Configure the maximum force that can be exerted by the robot

  max_robot_force_.x() = parameter_handler_->get_params().max_robot_force.x;
  max_robot_force_.y() = parameter_handler_->get_params().max_robot_force.y;
  max_robot_force_.z() = parameter_handler_->get_params().max_robot_force.z;

  // Convert max force in the robot base frame to haptic base frame
  max_robot_force_ = (robot_base_to_haptic_base_.inverse() * max_robot_force_).cwiseAbs();
}

}  // namespace haptic_forward_force_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(haptic_forward_force_controller::HapticForwardForceController, controller_interface::ChainableControllerInterface)
