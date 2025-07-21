/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   task_space_mock_controller.cpp
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Dec 17, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <pluginlib/class_list_macros.hpp>
#include <acg_common_libraries/message_utilities.hpp>
#include "task_space_mock_controller/task_space_mock_controller.hpp"

namespace task_space_mock_controller
{

controller_interface::CallbackReturn TaskSpaceMockController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "Initializing TaskSpaceMockController");
  parameter_handler_ = std::make_shared<task_space_mock_controller::ParamListener>(get_node());

  // Setting the command interface names override struct based on the configuration file for the task space reference
  acg_hardware_interface_facade::CommandInterfaceNamesOverrideConfig reference_interface_names_override;
  auto params = parameter_handler_->get_params();
  auto params_override = params.reference_interfaces_names_override;
  reference_interface_names_override.task_space_pose_interface_names = params_override.task_space_pose;
  reference_interface_names_override.task_space_twist_interface_names = params_override.task_space_velocity;
  reference_interface_names_override.task_space_acceleration_interface_names = params_override.task_space_acceleration;
  reference_interface_names_override.task_space_wrench_interface_names = params_override.task_space_wrench;
  reference_interface_names_override.task_space_wrench_derivative_interface_names = params_override.task_space_wrench_derivative;

  reference_reader_.configure_interfaces(std::vector<std::string>(), std::vector<std::string>(), params.task_space_reference_interfaces,
                                         std::string(), get_node()->get_name(), reference_interface_names_override);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TaskSpaceMockController::command_interface_configuration() const
{
  return { controller_interface::interface_configuration_type::NONE, {} };
}

controller_interface::InterfaceConfiguration TaskSpaceMockController::state_interface_configuration() const
{
  std::vector<std::string> state_interfaces_config_names;
  return { controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names };
}

controller_interface::CallbackReturn TaskSpaceMockController::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring TaskSpaceMockController");

  // Get the publish period from the parameters
  publish_frequency_ = parameter_handler_->get_params().publish_frequency;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceMockController::on_activate(const rclcpp_lifecycle::State&)
{
  periodic_publisher_ = std::make_shared<acg_diagnostics::PeriodicPublisher<geometry_msgs::msg::PoseStamped>>(
      get_node(), "~/actual", rclcpp::SystemDefaultsQoS(), publish_frequency_);

  task_space_reference_frame_ = parameter_handler_->get_params().task_space_reference_frame;
  task_space_reference_ = acg_control_msgs::msg::TaskSpacePoint();

  reference_reader_.assign_command_interfaces(on_export_reference_interfaces(), reference_interfaces_);

  RCLCPP_INFO(get_node()->get_logger(), "Activating TaskSpaceMockController");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceMockController::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating TaskSpaceMockController");
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> TaskSpaceMockController::on_export_reference_interfaces()
{
  // Resize the reference_interfaces_ vector to the size of the reference interface names vector with NaN values
  ChainableControllerInterface::reference_interfaces_.resize(reference_reader_.available_interfaces().size(),
                                                             std::numeric_limits<double>::quiet_NaN());

  return reference_reader_.build_reference_interfaces(reference_interfaces_);
}

controller_interface::return_type TaskSpaceMockController::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type TaskSpaceMockController::update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration&)
{
  reference_reader_.read_from_reference_interfaces(task_space_reference_);

  periodic_publisher_->publish(acg_message_utilities::build_pose_stamped_msg(task_space_reference_.pose, task_space_reference_frame_, time), time);
  return controller_interface::return_type::OK;
}

}  // namespace task_space_mock_controller

PLUGINLIB_EXPORT_CLASS(task_space_mock_controller::TaskSpaceMockController, controller_interface::ChainableControllerInterface)
