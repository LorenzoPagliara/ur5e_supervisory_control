/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   haptic_virtual_force_controller.hpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    Nov 14, 2024
 *
 * This module implements the ForceE2FController class, which is a controller
 * that that provides to create a force feedback on the haptic device.
 *
 * -------------------------------------------------------------------
 */

#ifndef HAPTIC_VIRTUAL_FORCE_CONTROLLER__HAPTIC_VIRTUAL_FORCE_CONTROLLER_HPP_
#define HAPTIC_VIRTUAL_FORCE_CONTROLLER__HAPTIC_VIRTUAL_FORCE_CONTROLLER_HPP_

#include <haptic_virtual_force_controller/haptic_virtual_force_controller_parameters.hpp>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <Eigen/Geometry>
#include "control_toolbox/pid.hpp"
#include <realtime_tools/realtime_publisher.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace haptic_virtual_force_controller
{
class HapticVirtualForceController : public controller_interface::ChainableControllerInterface
{
public:
  HapticVirtualForceController();

  ~HapticVirtualForceController();

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  controller_interface::return_type update_reference_from_subscribers() override;

  bool on_set_chained_mode(bool chained_mode) override;

private:
  // Vector containing the reference interfaces to be exported
  std::vector<std::string> reference_interface_names_;

  // Flag to set the controller in simulation mode
  bool simulation_;

  // Max haptic force that can be exerted by the haptic device
  double max_haptic_device_force_;

  // Subscribers for the position and velocity error
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_error_subscriber_, velocity_error_subscriber_;

  // Force publisher for the force actuated by the haptic device
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr actuated_wrench_publisher_;

  // Realtime force publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> realtime_wrench_publisher_;

  // Position and velocity error msg
  geometry_msgs::msg::Vector3 position_error_msg_, velocity_error_msg_;

  // Actuated force
  geometry_msgs::msg::WrenchStamped actuated_wrench_;

  // Maximum force that can be exerted by the haptic device.
  Eigen::Vector3d saturated_haptic_device_force_;

  // Position and velocity error, and haptic base force
  Eigen::Vector3d position_error_, velocity_error_, haptic_base_force_;

  // Parameter handler
  std::shared_ptr<haptic_virtual_force_controller::ParamListener> parameter_handler_;

  // Callback function for the position error subscriber
  void position_error_callback_(const geometry_msgs::msg::Vector3Stamped& msg);

  // Callback function for the velocity error subscriber
  void velocity_error_callback_(const geometry_msgs::msg::Vector3Stamped& msg);

  // Saturate the haptic device base force
  void saturate_haptic_force(const Eigen::Vector3d& haptic_device_base_force, Eigen::Vector3d& saturated_haptic_device_base_force) const;

  /**
   * @brief PID for each axis to render force feedback from a position and a velocity error.
   */
  std::map<std::size_t, control_toolbox::Pid> pid_;
};
}  // namespace haptic_virtual_force_controller

#endif  // HAPTIC_VIRTUAL_FORCE_CONTROLLER__HAPTIC_VIRTUAL_FORCE_CONTROLLER_HPP_
