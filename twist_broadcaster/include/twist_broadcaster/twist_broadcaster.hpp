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
 * This module implements the TwistBroadcaster class, a controller
 * that reads the twist from the state interfaces and publishes it
 * on a topic.
 * -------------------------------------------------------------------
 */

#pragma once

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include "acg_semantic_components/twist_sensor.hpp"
#include "twist_broadcaster/twist_broadcaster_parameters.hpp"

namespace twist_broadcaster
{

class TwistBroadcaster : public controller_interface::ControllerInterface
{
public:
  TwistBroadcaster() = default;

  ~TwistBroadcaster() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
  bool is_twist_valid(const geometry_msgs::msg::Twist& twist) const;

  std::shared_ptr<twist_broadcaster::ParamListener> parameter_handler_;

  std::unique_ptr<acg_semantic_components::TwistSensor> twist_sensor_;

  /**
   * @brief Twist publisher
   */
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;

  /**
   * @brief Realtime twist publisher
   */
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> realtime_publisher_;
};
}  // namespace twist_broadcaster
