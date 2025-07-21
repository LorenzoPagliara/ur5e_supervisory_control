/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   force_torque_sensor_reader.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Apr 09, 2025
 *
 * This module contains the ForceTorqueSensorReader class, which is
 * used to read from the state interfaces.
 *
 * -------------------------------------------------------------------
 */

#include <rclcpp/rclcpp.hpp>
#include "acg_hardware_interface_facade/force_torque_sensor_reader.hpp"

namespace acg_hardware_interface_facade
{

bool ForceTorqueSensorReader::configure_state_interfaces(const std::string& sensor_name,
                                                         const std::vector<std::string>& state_interface_names_override)
{
  if (configured_)
  {
    RCLCPP_WARN(rclcpp::get_logger("force_torque_sensor_reader"), "Force/torque sensor reader already configured.");
    return true;
  }

  sensor_name_ = sensor_name;

  // Configuring the semantic components and saving the names of the state interfaces, using the override configuration if provided
  if (!state_interface_names_override.empty())
  {
    if (state_interface_names_override.size() != 6)
    {
      RCLCPP_ERROR(rclcpp::get_logger("force_torque_sensor_reader"),
                   "The number of names in the override configuration for the state interfaces does not match the 6 measurement axes.");
      return false;
    }

    interface_names_ = state_interface_names_override;

    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(interface_names_[0], interface_names_[1], interface_names_[2],
                                                                                    interface_names_[3], interface_names_[4], interface_names_[5]);
  }
  else
  {
    interface_names_.push_back(sensor_name_ + "/" + "force.x");
    interface_names_.push_back(sensor_name_ + "/" + "force.y");
    interface_names_.push_back(sensor_name_ + "/" + "force.z");
    interface_names_.push_back(sensor_name_ + "/" + "torque.x");
    interface_names_.push_back(sensor_name_ + "/" + "torque.y");
    interface_names_.push_back(sensor_name_ + "/" + "torque.z");
    force_torque_sensor_ = std::make_unique<semantic_components::ForceTorqueSensor>(sensor_name_);
  }

  configured_ = true;
  return true;
}

std::vector<std::string> ForceTorqueSensorReader::available_state_interfaces() const
{
  log_error_if_not_configured_();
  if (!configured_)
  {
    return {};
  }

  std::vector<std::string> state_interfaces_config_names;
  state_interfaces_config_names.insert(state_interfaces_config_names.end(), interface_names_.begin(), interface_names_.end());

  return state_interfaces_config_names;
}

void ForceTorqueSensorReader::read_state_interfaces(geometry_msgs::msg::Wrench& state) const
{
  log_error_if_not_configured_();
  if (!configured_)
  {
    return;
  }

  if (!force_torque_sensor_->get_values_as_message(state))
  {
    RCLCPP_ERROR(rclcpp::get_logger("force_torque_sensor_reader"), "Error reading values from force/torque sensor.");
  }
}

bool ForceTorqueSensorReader::assign_loaned_state_interfaces(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces)
{
  log_error_if_not_configured_();
  if (!configured_)
  {
    return false;
  }

  return force_torque_sensor_->assign_loaned_state_interfaces(state_interfaces);
}

void ForceTorqueSensorReader::release_interfaces()
{
  if (!configured_)
  {
    RCLCPP_WARN(rclcpp::get_logger("force_torque_sensor_reader"),
                "Force/torque sensor reader not configured. Please configure it before releasing the state interfaces.");
    return;
  }
  force_torque_sensor_->release_interfaces();
}

void ForceTorqueSensorReader::log_error_if_not_configured_() const
{
  if (!configured_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("force_torque_sensor_reader"),
                 "Force/torque sensor reader not configured. Please configure the ForceTorqueSensorReader object before using it.");
  }
}

}  // namespace acg_hardware_interface_facade
