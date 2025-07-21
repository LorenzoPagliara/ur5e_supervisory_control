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
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include "ur_force_torque_sensor/ur_force_torque_sensor.hpp"

namespace ur_force_torque_sensor
{
URFTSensor::URFTSensor() : hw_sensor_states_(6, 0.0), logger_(rclcpp::get_logger("ur_ft_sensor")) {}

URFTSensor::~URFTSensor() {}

hardware_interface::CallbackReturn URFTSensor::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Store sensor informations
  sensor_ = info.sensors[0];

  // Check the number of declared state interfaces
  if (sensor_.state_interfaces.size() != 6)
  {
    RCLCPP_ERROR(logger_, "The UR FT driver expects %d state interfaces. Number of state interfaces specified: %zu", 6,
                 sensor_.state_interfaces.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get the update rate from the hardware parameters
  update_rate_ = std::stod(info_.hardware_parameters["update_rate"]);
  if (update_rate_ <= 0.0)
  {
    RCLCPP_ERROR(logger_, "The update rate must be greater than 0.0. Specified value: %f", update_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  rtde_receive_interface_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(info_.hardware_parameters["robot_ip"], update_rate_);

  if (!rtde_receive_interface_->isConnected())
  {
    RCLCPP_ERROR(logger_, "Failed to connect to UR RTDE server");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> URFTSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  RCLCPP_INFO(logger_, "Exporting State Interfaces");

  for (size_t i = 0; i < sensor_.state_interfaces.size(); i++)
  {
    RCLCPP_INFO(logger_, "Sensor %s state %s", sensor_.name.c_str(), sensor_.state_interfaces[i].name.c_str());
    state_interfaces.emplace_back(sensor_.name, sensor_.state_interfaces[i].name, &hw_sensor_states_[i]);
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn URFTSensor::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  sensor_readings_.writeFromNonRT(std::vector<double>(6, 0.0));

  rclcpp::NodeOptions options;
  options.arguments({ "--ros-args", "-r", "__node:=ur_ft_hardware_internal_" + info_.name });
  async_node_ = rclcpp::Node::make_shared("_", options);

  timer_ = async_node_->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / update_rate_)),
                                          std::bind(&URFTSensor::read_background_, this));

  node_thread_ = std::make_unique<std::thread>(
      [&]()
      {
        executor_.add_node(async_node_);
        executor_.spin();
        executor_.remove_node(async_node_);
      });

  RCLCPP_INFO(logger_, "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn URFTSensor::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  executor_.cancel();
  node_thread_->join();
  node_thread_.reset();
  async_node_.reset();

  RCLCPP_INFO(logger_, "Successfully deactivated UR force/torque sensor.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type URFTSensor::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  hw_sensor_states_ = *(sensor_readings_.readFromRT());

  return hardware_interface::return_type::OK;
}

void URFTSensor::read_background_()
{
  sensor_readings_.writeFromNonRT(rtde_receive_interface_->getFtRawWrench());
}

}  // namespace ur_force_torque_sensor

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ur_force_torque_sensor::URFTSensor, hardware_interface::SensorInterface)
