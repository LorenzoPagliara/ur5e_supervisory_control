/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   ur_force_torque_sensor.hpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Apr 1, 2025
 *
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <hardware_interface/sensor_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ur_rtde/rtde_receive_interface.h>

namespace ur_force_torque_sensor
{
class URFTSensor : public hardware_interface::SensorInterface
{
public:
  /**
   * @brief Construct a URFTSensor object.
   */
  URFTSensor();

  /**
   * @brief Destroy a URFTSensor object.
   */
  ~URFTSensor();

  /**
   * @brief Refer to the superclass documentation.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /**
   * @brief Read the sensor data in a background thread.
   */
  void read_background_();

  /**
   * @brief Object that stores information about components defined for a specific hardware
   *  in robot's URDF.
   */
  hardware_interface::ComponentInfo sensor_;

  /**
   * @brief The force/torque measurements to be written on the state interfaces.
   */
  std::vector<double> hw_sensor_states_;

  /**
   * @brief A real-time buffer for storing sensor measurements.
   */
  realtime_tools::RealtimeBuffer<std::vector<double>> sensor_readings_;

  /**
   * @brief The update rate for the sensor reading.
   */
  double update_rate_;

  /**
   * @brief Timer for periodic sensor reading.
   * The timer is used to call the read_background_() method at a specified interval.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Shared pointer to the ROS2 node instance.
   */
  rclcpp::Node::SharedPtr async_node_;

  /**
   * @brief Pointer to a thread running the ROS2 node.
   */
  std::unique_ptr<std::thread> node_thread_;

  /**
   * @brief Single-threaded executor for handling ROS2 callbacks.
   */
  rclcpp::executors::SingleThreadedExecutor executor_;

  /**
   * @brief Logger instance for the URFTSensor class.
   */
  rclcpp::Logger logger_;

  /**
   * @brief RTDEReceiveInterface instance for receiving data from the UR robot.
   */
  std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_interface_;
};

}  // namespace ur_force_torque_sensor
