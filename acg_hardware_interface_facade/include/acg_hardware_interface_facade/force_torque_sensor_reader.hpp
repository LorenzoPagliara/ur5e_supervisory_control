/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   force_torque_sensor_reader.hpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Apr 09, 2025
 *
 * This module contains the ForceTorqueSensorReader class, which is
 * used to read from the state interfaces.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <vector>
#include <string>

#include <hardware_interface/loaned_state_interface.hpp>
#include <semantic_components/force_torque_sensor.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace acg_hardware_interface_facade
{
/**
 * @brief Class that handles the reading of the force/torque sensor state interfaces, based on the state interfaces specified in the configuration
 * step.
 */
class ForceTorqueSensorReader
{
public:
  /**
   * @brief Construct a new state reader object. This function is not real-time safe.
   */
  ForceTorqueSensorReader() = default;

  /**
   * @brief Destruct this object. This function is not real-time safe.
   */
  ~ForceTorqueSensorReader() = default;

  /**
   * @brief Configure the state interfaces. This function must be called before reading the state interfaces.
   *
   * This function configures the state reader based on the state interfaces specified in the configuration step.
   * The user can override the standard naming convention with custom names through the StateInterfaceNamesOverrideConfig struct.
   * This function is not real-time safe.
   *
   * @param[in] sensor_name The sensor name.
   * @param[in] state_interface_names_override The vector that contains the names of the state interfaces that the user wants to override.
   *
   * @return true if the object is correctly configured, false otherwise.
   */
  bool configure_state_interfaces(const std::string& sensor_name, const std::vector<std::string>& state_interface_names_override = {});

  /**
   * @brief Get the state interfaces names.
   *
   * This function returns the state interfaces names based on the information specified via the configure_state_interfaces method.
   * This function is real-time safe.
   *
   * @return a vector of strings containing the state interfaces names.
   */
  std::vector<std::string> available_state_interfaces() const;

  /**
   * @brief Read the state interfaces.
   *
   * This function reads the state interfaces specified via the configure_state_interfaces function and stores
   * the state in the state variable. This function is real-time safe.
   *
   * @param[out] state The state to read the state interfaces to. Be sure to have assigned the correct sizes to the fields
   * of the output parameter state, otherwise the function won't read the state.
   */
  void read_state_interfaces(geometry_msgs::msg::Wrench& state) const;

  /**
   * @brief Assign the state interfaces to the force/torque sensor reader. This function should be called once during the controller activation step.
   *
   * This function assigns the state interfaces to the force/torque sensor reader, based on the state interfaces specified in the configuration
   * step. This function is not real-time safe.
   *
   * @param[in] state_interfaces The vector of loaned state interfaces.
   * @return true if the state interfaces are correctly assigned, false otherwise.
   */
  bool assign_loaned_state_interfaces(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  /**
   * @brief Release the interfaces. This function is not real-time safe.
   */
  void release_interfaces();

protected:
  std::string sensor_name_;

  std::vector<std::string> interface_names_;

  std::unique_ptr<semantic_components::ForceTorqueSensor> force_torque_sensor_;

  bool configured_{ false };

  /**
   * @brief Logs an error message if the interface is not configured.
   *
   * This function checks whether the interface is configured. If it is not,
   * it logs an appropriate error message. This function also returns a boolean
   * value indicating whether the interface is configured.
   *
   * @return true if the interface is configured, false otherwise.
   */
  void log_error_if_not_configured_() const;
};
;

}  // namespace acg_hardware_interface_facade
