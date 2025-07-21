/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   haptic_forward_force_controller.hpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    Oct 11, 2024
 *
 * This module implements the HapticForwardForceController class,
 * which is a controller that provides to create a force feedback on
 * the haptic device.
 *
 * -------------------------------------------------------------------
 */

#ifndef HAPTIC_FORWARD_FORCE_CONTROLLER__HAPTIC_FORWARD_FORCE_CONTROLLER_HPP_
#define HAPTIC_FORWARD_FORCE_CONTROLLER__HAPTIC_FORWARD_FORCE_CONTROLLER_HPP_

#include <haptic_forward_force_controller/haptic_forward_force_controller_parameters.hpp>

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <Eigen/Geometry>
#include <realtime_tools/realtime_publisher.hpp>

namespace haptic_forward_force_controller
{
class HapticForwardForceController : public controller_interface::ChainableControllerInterface
{
public:
  HapticForwardForceController();

  ~HapticForwardForceController();

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
  // Reference interfaces names
  std::vector<std::string> reference_interface_names_;

  // Flag to enable the simulation mode for the controller
  bool simulation_;

  // Wrench subscriber
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber_;

  // Force publisher for the force actuated by the haptic device
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr actuated_wrench_publisher_;

  // Realtime force publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> realtime_wrench_publisher_;

  // Wrench message
  geometry_msgs::msg::Wrench wrench_msg_;

  // Haptic force
  Eigen::Vector3d haptic_base_force_;

  // Mapped haptic force
  Eigen::Vector3d mapped_haptic_base_force_;

  // Actuated force
  geometry_msgs::msg::WrenchStamped actuated_wrench_;

  // Parameters handler
  std::shared_ptr<haptic_forward_force_controller::ParamListener> parameter_handler_;

  // Robot base to haptic base transform.
  Eigen::Matrix3d robot_base_to_haptic_base_;

  // Maximum force that can be exerted by the haptic device.
  Eigen::Vector3d max_haptic_device_force_;

  // Maximum force that can be exerted by the robot.
  Eigen::Vector3d max_robot_force_;

  // Callback function for the force subscriber
  void wrench_callback_(const geometry_msgs::msg::WrenchStamped& msg);

  // Map the haptic base force into the haptic range
  void mapHapticBaseForceIntoHapticRange_(const Eigen::Vector3d& haptic_base_force, Eigen::Vector3d& mapped_haptic_base_force) const;

  // Update the maximum force that can be exerted by the haptic device
  void updateMaxHapticDeviceForce_();

  // Update the maximum force that can be exerted by the robot
  void updateMaxRobotForce_();
};
}  // namespace haptic_forward_force_controller

#endif  // HAPTIC_FORWARD_FORCE_CONTROLLER__HAPTIC_FORWARD_FORCE_CONTROLLER_HPP_
