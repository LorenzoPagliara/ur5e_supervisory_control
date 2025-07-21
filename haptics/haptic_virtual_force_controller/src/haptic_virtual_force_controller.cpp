/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   haptic_virtual_force_controller.cpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    Nov 14, 2024
 *
 * This module implements the HapticVirtualForceController class, which is a controller
 * that that provides to create a force feedback on the haptic device.
 *
 * -------------------------------------------------------------------
 */

#include "haptic_virtual_force_controller/haptic_virtual_force_controller.hpp"
#include <vector>

using std::placeholders::_1;

namespace haptic_virtual_force_controller
{
HapticVirtualForceController::HapticVirtualForceController() {}

HapticVirtualForceController::~HapticVirtualForceController() {}

controller_interface::CallbackReturn HapticVirtualForceController::on_init()
{
  try
  {
    parameter_handler_ = std::make_shared<haptic_virtual_force_controller::ParamListener>(get_node());
    simulation_ = parameter_handler_->get_params().simulation;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HapticVirtualForceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interface_config;
  command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interface_config.names = parameter_handler_->get_params().command_interfaces;
  return command_interface_config;
}

controller_interface::InterfaceConfiguration HapticVirtualForceController::state_interface_configuration() const
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

controller_interface::return_type HapticVirtualForceController::update_and_write_commands(const rclcpp::Time& /*time*/,
                                                                                          const rclcpp::Duration& period)
{
  if (simulation_)
  {
    position_error_ << position_error_msg_.x, position_error_msg_.y, position_error_msg_.z;
    velocity_error_ << velocity_error_msg_.x, velocity_error_msg_.y, velocity_error_msg_.z;
  }
  else
  {
    position_error_ << reference_interfaces_[0], reference_interfaces_[1], reference_interfaces_[2];
    velocity_error_ << reference_interfaces_[3], reference_interfaces_[4], reference_interfaces_[5];
  }

  // Update the PID controller with the new parameters
  pid_[0].setGains(parameter_handler_->get_params().pid.x.p, parameter_handler_->get_params().pid.x.i, parameter_handler_->get_params().pid.x.d,
                   parameter_handler_->get_params().pid.x.i_clamp_max, parameter_handler_->get_params().pid.x.i_clamp_min,
                   parameter_handler_->get_params().pid.x.antiwindup);
  pid_[1].setGains(parameter_handler_->get_params().pid.y.p, parameter_handler_->get_params().pid.y.i, parameter_handler_->get_params().pid.y.d,
                   parameter_handler_->get_params().pid.y.i_clamp_max, parameter_handler_->get_params().pid.y.i_clamp_min,
                   parameter_handler_->get_params().pid.y.antiwindup);
  pid_[2].setGains(parameter_handler_->get_params().pid.z.p, parameter_handler_->get_params().pid.z.i, parameter_handler_->get_params().pid.z.d,
                   parameter_handler_->get_params().pid.z.i_clamp_max, parameter_handler_->get_params().pid.z.i_clamp_min,
                   parameter_handler_->get_params().pid.z.antiwindup);

  // Compute the force to be exerted by the haptic device
  haptic_base_force_.x() = -pid_.at(0).computeCommand(position_error_.x(), velocity_error_.x(), period.nanoseconds());
  haptic_base_force_.y() = -pid_.at(1).computeCommand(position_error_.y(), velocity_error_.y(), period.nanoseconds());
  haptic_base_force_.z() = -pid_.at(2).computeCommand(position_error_.z(), velocity_error_.z(), period.nanoseconds());

  // Saturate the haptic force
  HapticVirtualForceController::saturate_haptic_force(haptic_base_force_, saturated_haptic_device_force_);

  // Write the force to the command interfaces
  command_interfaces_[0].set_value(saturated_haptic_device_force_.x());
  command_interfaces_[1].set_value(saturated_haptic_device_force_.y());
  command_interfaces_[2].set_value(saturated_haptic_device_force_.z());

  // Publish the actuated force
  actuated_wrench_.header.stamp = get_node()->get_clock()->now();
  actuated_wrench_.wrench.force.x = saturated_haptic_device_force_.x();
  actuated_wrench_.wrench.force.y = saturated_haptic_device_force_.y();
  actuated_wrench_.wrench.force.z = saturated_haptic_device_force_.z();
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

controller_interface::CallbackReturn HapticVirtualForceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (parameter_handler_->get_params().command_interfaces.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!simulation_)
  {
    reference_interface_names_.push_back("position_error/x");
    reference_interface_names_.push_back("position_error/y");
    reference_interface_names_.push_back("position_error/z");
    reference_interface_names_.push_back("velocity_error/x");
    reference_interface_names_.push_back("velocity_error/y");
    reference_interface_names_.push_back("velocity_error/z");
    reference_interfaces_.resize(reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());
  }
  else
  {
    reference_interface_names_.push_back("");
    reference_interfaces_.resize(reference_interface_names_.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Configure publisher and subscriber
  if (simulation_)
  {
    position_error_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        parameter_handler_->get_params().position_error_topic_name, 1, std::bind(&HapticVirtualForceController::position_error_callback_, this, _1));
    velocity_error_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        parameter_handler_->get_params().velocity_error_topic_name, 1, std::bind(&HapticVirtualForceController::velocity_error_callback_, this, _1));
  }
  actuated_wrench_publisher_ =
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("/hd/haptic_virtual_force_controller/actuated_forces", 1);

  realtime_wrench_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(actuated_wrench_publisher_);

  // Configure the PID controller
  pid_[0].initPid(parameter_handler_->get_params().pid.x.p, parameter_handler_->get_params().pid.x.i, parameter_handler_->get_params().pid.x.d,
                  parameter_handler_->get_params().pid.x.i_clamp_max, parameter_handler_->get_params().pid.x.i_clamp_min,
                  parameter_handler_->get_params().pid.x.antiwindup);

  pid_[1].initPid(parameter_handler_->get_params().pid.y.p, parameter_handler_->get_params().pid.y.i, parameter_handler_->get_params().pid.y.d,
                  parameter_handler_->get_params().pid.y.i_clamp_max, parameter_handler_->get_params().pid.y.i_clamp_min,
                  parameter_handler_->get_params().pid.y.antiwindup);

  pid_[2].initPid(parameter_handler_->get_params().pid.z.p, parameter_handler_->get_params().pid.z.i, parameter_handler_->get_params().pid.z.d,
                  parameter_handler_->get_params().pid.z.i_clamp_max, parameter_handler_->get_params().pid.z.i_clamp_min,
                  parameter_handler_->get_params().pid.z.antiwindup);

  // Initialize variables
  haptic_base_force_ = Eigen::Vector3d::Zero();
  position_error_ = Eigen::Vector3d::Zero();
  velocity_error_ = Eigen::Vector3d::Zero();
  actuated_wrench_ = geometry_msgs::msg::WrenchStamped();
  position_error_msg_ = geometry_msgs::msg::Vector3();
  velocity_error_msg_ = geometry_msgs::msg::Vector3();
  saturated_haptic_device_force_ = Eigen::Vector3d::Zero();
  max_haptic_device_force_ = parameter_handler_->get_params().max_haptic_device_force;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticVirtualForceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticVirtualForceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> HapticVirtualForceController::on_export_reference_interfaces()
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

controller_interface::return_type HapticVirtualForceController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

bool HapticVirtualForceController::on_set_chained_mode(bool chained_mode)
{
  if (chained_mode)
    RCLCPP_INFO(get_node()->get_logger(), "Chained mode enabled");
  return true;
}

void HapticVirtualForceController::position_error_callback_(const geometry_msgs::msg::Vector3Stamped& msg)
{
  position_error_msg_ = msg.vector;
}

void HapticVirtualForceController::velocity_error_callback_(const geometry_msgs::msg::Vector3Stamped& msg)
{
  velocity_error_msg_ = msg.vector;
}

void HapticVirtualForceController::saturate_haptic_force(const Eigen::Vector3d& haptic_device_base_force,
                                                         Eigen::Vector3d& saturated_haptic_device_base_force) const
{
  if (haptic_device_base_force.norm() < max_haptic_device_force_)
    saturated_haptic_device_base_force = haptic_device_base_force;
  else
    saturated_haptic_device_base_force = max_haptic_device_force_ * haptic_device_base_force / haptic_device_base_force.norm();
}

}  // namespace haptic_virtual_force_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(haptic_virtual_force_controller::HapticVirtualForceController, controller_interface::ChainableControllerInterface)
