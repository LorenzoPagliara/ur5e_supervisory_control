/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   touch_haptic_device.hpp
 * Author:  Antonio Langella, Michele Marsico, Salvatore Paolino
 * Org.:    UNISA
 * Date:    Oct 1, 2024
 *
 * This module implements the HapticDeviceTouch class, which is a
 * ROS2 hardware interface for the Touch Haptic Device.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <HD/hd.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <stdexcept>

namespace haptic_device
{

typedef std::runtime_error Exception;

/**
 * @class HapticDeviceTouch
 * @brief A class representing a Touch haptic device interface for ROS2.
 *
 * This class implements the hardware_interface::SystemInterface to provide
 * an interface for the Touch haptic device within the ROS2 framework.
 * All of the public methods override the corresponding methods of the
 * hardware_interface::SystemInterface class. Please refer to the
 * hardware_interface::SystemInterface documentation for more information
 * on those methods.
 * @note This class is designed to be used with the ROS2 lifecycle management.
 */
class HapticDeviceTouch : public hardware_interface::SystemInterface
{
public:
  HapticDeviceTouch();

  ~HapticDeviceTouch();

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  static const unsigned int MIN_NUMBER_OF_STABLE_READINGS = 5;

private:
  /**
   * @brief Get the pose of the haptic device.
   *
   * This function retrieves the current pose of the haptic device and stores it in the provided Pose message.
   *
   * @param[out] haptic_device_pose The haptic device pose.
   * @return true if the pose was successfully retrieved, false otherwise.
   */
  bool get_haptic_device_pose_(geometry_msgs::msg::Pose& haptic_device_pose) const;

  /**
   * @brief Get the orientation angles of the haptic device.
   *
   * This function retrieves the haptic device orientation and stores the relating angles in the provided Vector3 message.
   *
   * @param[out] haptic_device_cartesian_angles The haptic device Cartesian angles
   * @return true if the angles were successfully retrieved, false otherwise.
   */
  bool get_haptic_device_cartesian_angles_(geometry_msgs::msg::Vector3& haptic_device_cartesian_angles) const;

  /**
   * @brief Get the Cartesian velocity of the haptic device.
   *
   * This function retrieves the current velocity of the haptic device and stores it in the provided Vector3 message.
   *
   * @param[out] haptic_device_cartesian_velocity The haptic device Cartesian velocity.
   * @return true if the velocity was successfully retrieved, false otherwise.
   */
  bool get_haptic_device_cartesian_velocity_(geometry_msgs::msg::Vector3& haptic_device_cartesian_velocity) const;

  /**
   * @brief Set the Cartesian forces on the haptic device.
   *
   * This function applies the specified Cartesian forces to the haptic device.
   *
   * @param[in] forces Cartesian forces to apply.
   * @return true if the forces were successfully applied, false otherwise.
   */
  bool set_haptic_device_cartesian_forces_(const geometry_msgs::msg::Vector3& forces) const;

  /**
   * @brief Set the joint torques on the haptic device.
   *
   * This function applies the specified joint torques to the haptic device.
   *
   * @param[in] torques The joint torques to apply.
   * @return true if the torques were successfully applied, false otherwise.
   */
  bool set_haptic_device_joint_torques_(const geometry_msgs::msg::Vector3& torques) const;

  /**
   * @brief Set the Cartesian torques on the haptic device.
   *
   * This function applies the specified Cartesian torques to the haptic device.
   *
   * @param[in] torques The Cartesian torques to apply.
   * @return true if the torques were successfully applied, false otherwise.
   */
  bool set_haptic_device_cartesian_torques_(const geometry_msgs::msg::Vector3& torques) const;

  /**
   * @brief Get the inkwell switch state of the haptic device.
   *
   * This function retrieves the current state of the inkwell switch and stores it in the provided boolean reference.
   *
   * @param[out] inkwell_switch The inkwell switch state.
   * @return true if the state was successfully retrieved, false otherwise.
   */
  bool get_inkwell_switch_state_(HDboolean& inkwell_switch) const;

  /**
   * @brief Get the angular velocity of the haptic device.
   *
   * This function retrieves the current angular velocity of the haptic device and stores it in the provided Vector3 message.
   *
   * @param[out] haptic_device_angular_velocity The haptic device angular velocity.
   * @return true if the angular velocity was successfully retrieved, false otherwise.
   */
  bool get_haptic_device_angular_velocity_(geometry_msgs::msg::Vector3& haptic_device_angular_velocity) const;

  /**
   * @brief Get the haptic device state.
   *
   * This function retrieves the current state of the haptic device and stores it in the provided boolean reference.
   *
   * @param[out] haptic_device_state The haptic device state.
   * @return true if the state was successfully retrieved, false otherwise.
   */
  bool get_update_rate_(int& update_rate);

  /**
   * @brief Enable the haptic interface
   *
   */
  void enable_device_();

  /**
   * @brief Disable the haptic interface
   */
  void disable_device_() const;

  /**
   * @brief Wait for stable haptic device pose measures
   *
   */
  void wait_for_stable_pose_readings_() const;

  /**
   * @brief Calibrate the haptic interface
   */
  bool calibrate_() const;

  /**
   * @brief Callback required to set the specified cartesian forces
   *
   * @param user_data where the cartesian forces to apply will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode set_haptic_device_cartesian_forces_callback_(void* user_data);

  /**
   * @brief Callback required to set the specified joint torques
   *
   * @param user_data where the joint torques to apply will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode set_haptic_device_joint_torques_callback_(void* user_data);

  /**
   * @brief Callback required to get the calibration status
   *
   * @param user_data where calibration status will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode calibration_status_callback_(void* user_data);

  /**
   * @brief Callback required to get the inkwell switch status
   *
   * @param user_data where inkwell switch state will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode get_inkwell_switch_state_callback_(void* user_data);

  /**
   * @brief Callback required to get haptic device end-effector pose, expressed in base frame
   *
   * @param user_data where device pose will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode get_haptic_device_pose_callback_(void* user_data);

  /**
   * @brief Callback required to get haptic device stylus gimbal joint angles
   *
   * @param user_data where device angles will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode get_haptic_device_cartesian_angles_callback_(void* user_data);

  /**
   * @brief Callback required to get haptic device velocity, expressed in base frame
   *
   * @param user_data where device velocity will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode get_haptic_device_cartesian_velocity_callback_(void* user_data);

  /**
   * @brief Callback required to get haptic device angular velocity, expressed in base frame
   *
   * @param user_data where device angular velocity will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode get_haptic_device_angular_velocity_callback_(void* user_data);

  /**
   * @brief Callback required to get the update rate of the haptic device
   *
   * @param user_data where update rate will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode get_update_rate_callback_(void* user_data);

  /**
   * @brief Callback required to get the last error put in the error stack during the usage of the API
   *
   * @param user_data where error info will be written
   * @return HD_CALLBACK_DONE if the callback is completed, otherwise HD_CALLBACK_CONTINUE if the callback
   * must continue and it will be rescheduled and run again during the next scheduler tick.
   */
  static HDCallbackCode get_last_haptic_device_error_callback_(void* user_data);

  /**
   * @brief Get the calibration status of the touch haptic device.
   * It can get one of these two values HD_CALIBRATION_OK or HD_CALIBRATION_NEEDS_MANUAL_INPUT.
   * For more details refer to hdCheckCalibration function documentation in OpenHaptics_RefGuide.pdf
   *
   * @param calibration_status reference to the HDenum where the state will be stored
   * @return true if the procedure succeeds, false otherwise.
   */
  static bool get_calibration_status_(HDenum& calibration_status);

  /**
   * @brief Handle the last error in the haptic device error stack
   *
   * @param error_message string to show in case of an haptic device error.
   */
  static void check_haptic_device_error_(std::string error_message);

  /**
   * @brief check for the last error in the haptic device error stack
   *
   * @return error info structure
   */
  static HDErrorInfo get_last_haptic_device_error_();

  // Haptic device state interfaces
  std::vector<double> touch_haptic_device_states_;

  // Haptic device command interfaces
  std::vector<double> touch_haptic_device_commands_;

  // Haptic device handler
  static HHD haptic_device_handler_;

  // Logger
  std::unique_ptr<rclcpp::Logger> logger_;

  // Message flag for inkwell switch
  bool message_flag_;

  // Last haptic device pose
  geometry_msgs::msg::Pose last_haptic_pose_;

  // Rendered force
  geometry_msgs::msg::Vector3 rendered_force_;

  // Variable to store the inkwell switch state to verify if the stylus is in the inkwell
  HDboolean inkwell_switch_;
};
}  // namespace haptic_device
