/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   haptic_fb_controller.hpp
 * Author:  Salvatore Paolino
 * Org.:    UNISA
 * Date:    Oct 3, 2024
 *
 * This module implements the HapticFeedbackController class, which is
 * a controller that that provides to create a force feedback on the
 * haptic device.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <haptic_fb_controller/haptic_fb_controller_parameters.hpp>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "filters/filter_chain.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include <Eigen/Geometry>
#include <realtime_tools/realtime_publisher.hpp>
#include "acg_control_msgs/msg/admittance_controller_state.hpp"

namespace haptic_fb_controller
{
class HapticFeedbackController : public controller_interface::ChainableControllerInterface
{
public:
  HapticFeedbackController();

  ~HapticFeedbackController();

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
  std::vector<std::string> reference_interface_names_;

  // Force/torque subscriber
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscriber_;

  // Position and velocity error publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_error_publisher_, velocity_error_publisher_;

  // Wrench publisher
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;

  // Realtime wrench publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> realtime_wrench_publisher_;

  // Realtime position error publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>> realtime_position_error_publisher_;

  // Realtime velocity error publisher
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>> realtime_velocity_error_publisher_;

  // Admittance state subsciber
  rclcpp::Subscription<acg_control_msgs::msg::AdmittanceControllerState>::SharedPtr admittance_state_subscriber_;

  // Admittance state message
  acg_control_msgs::msg::AdmittanceControllerState admittance_state_msg_;

  // Wrench message
  geometry_msgs::msg::WrenchStamped wrench_msg_;

  // Flag to enable the simulation mode
  bool simulation_;

  // Flag to enable the force/torque reading from the topic
  bool read_force_torque_from_topic_;

  // Convert the force from the robot end-effector frame to the haptic base frame
  void convertRobotEEForceToHapticBaseForce_(const Eigen::Vector3d& robot_end_effector_force, const Eigen::Isometry3d& base_to_end_effector_transform,
                                             Eigen::Vector3d& haptic_base_force) const;

  void read_robot_state_(const std::vector<hardware_interface::LoanedStateInterface>& state_interfaces, const size_t robot_state_interface_size,
                         Eigen::VectorXd& robot_joint_state) const;

  int robot_state_interface_size_;

  std::string robot_base_frame_;
  std::string robot_ee_frame_;

  // Transform from robot world to base.
  Eigen::Isometry3d robot_world_to_base_transform_;

  // Transform from robot world to end-effector.
  Eigen::Isometry3d robot_world_to_ee_transform_;

  // Robot base to haptic base transform.
  Eigen::Matrix3d robot_base_to_haptic_base_;

  // Transform from robot base to robot end-effector.
  Eigen::Isometry3d base_to_end_effector_actual_transform_;

  // Robot position joint state
  Eigen::VectorXd robot_joint_state_;

  // Input wrench
  geometry_msgs::msg::Wrench input_wrench_;

  // Output wrench
  geometry_msgs::msg::WrenchStamped output_wrench_;

  // Output position and velocity error
  geometry_msgs::msg::Vector3Stamped position_error_haptic_base_;
  geometry_msgs::msg::Vector3Stamped velocity_error_haptic_base_;

  // Force and torque vector
  Eigen::Vector3d force_vector_, haptic_base_force_, torque_vector_, haptic_base_torque_;

  Eigen::Vector3d position_error_, velocity_error_, haptic_base_position_error_, haptic_base_velocity_error_;

  std::shared_ptr<haptic_fb_controller::ParamListener> parameter_handler_;

  /**
   * @brief Filter chain for the force/torque measurements.
   */
  std::shared_ptr<filters::MultiChannelFilterChain<double>> force_filter_chain_;

  /**
   * @brief Filter chain for the Cartesian motion error.
   */
  std::shared_ptr<filters::MultiChannelFilterChain<double>> motion_error_filter_chain_;

  // Number of channels for the force filter chain
  int number_of_channels_;

  // Filter chain input and output
  std::vector<double> filter_chain_input_, filter_chain_output_, cartesian_motion_error_filter_chain_input_,
      cartesian_motion_error_filter_chain_output_;

  // Flag to check if the sensor bias is computed and reliable (at the beginning it is not)
  bool is_sensor_bias_computed_;

  // Flag to enable the reference interfaces for the motion error
  bool enable_motion_error_;

  // Kinematics interface plugin loader
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>> kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;
};
}  // namespace haptic_fb_controller
