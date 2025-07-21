/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   twist_broadcaster.hpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    May 12, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "twist_broadcaster/twist_broadcaster.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace twist_broadcaster
{

controller_interface::InterfaceConfiguration TwistBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration TwistBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interface_config;
  state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interface_config.names = twist_sensor_->get_state_interface_names();

  return state_interface_config;
}

controller_interface::CallbackReturn TwistBroadcaster::on_init()
{
  try
  {
    parameter_handler_ = std::make_shared<twist_broadcaster::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  twist_sensor_ = std::make_unique<acg_semantic_components::TwistSensor>(parameter_handler_->get_params().twist_name);

  try
  {
    twist_publisher_ =
        get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(parameter_handler_->get_params().twist_topic, rclcpp::SystemDefaultsQoS());
    realtime_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(twist_publisher_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during publisher configuration at configure stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize twist message
  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = parameter_handler_->get_params().frame_id;
  realtime_publisher_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  twist_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  twist_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TwistBroadcaster::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  geometry_msgs::msg::Twist twist;
  twist_sensor_->get_values_as_message(twist);

  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = time;
    realtime_publisher_->msg_.twist = twist;
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

bool TwistBroadcaster::is_twist_valid(const geometry_msgs::msg::Twist& twist) const
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) && std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

}  // namespace twist_broadcaster

PLUGINLIB_EXPORT_CLASS(twist_broadcaster::TwistBroadcaster, controller_interface::ControllerInterface)
