/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   acg_admittance_controller.hpp
 * Author:  Francesco D'Onofrio,Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Feb 4, 2025
 *
 * This module defines the AdmittanceController class, which
 * implements MotionBasedInteractionControl interface. The class is
 * responsible for computing the control law.
 * -------------------------------------------------------------------
 */

#pragma once

#include <acg_control_msgs/msg/admittance_controller_state.hpp>
#include <acg_control_msgs/msg/task_space_point.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <filters/filter_chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <motion_based_interaction_controller/motion_based_interaction_controller.hpp>
#include "acg_admittance_controller/acg_admittance_controller_parameters.hpp"
#include "acg_admittance_controller/visibility_control.h"

namespace acg_admittance_controller
{
static const int NUM_CARTESIAN_DOF_ = 6;
typedef Eigen::Matrix<double, NUM_CARTESIAN_DOF_, NUM_CARTESIAN_DOF_> Matrix6d;
typedef Eigen::Matrix<double, NUM_CARTESIAN_DOF_, 1> Vector6d;

class AdmittanceController : public motion_based_interaction_controller::MotionBasedInteractionController
{
public:
  /**
   * @brief Refer to the superclass documentation.
   */
  ACG_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  ACG_ADMITTANCE_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

protected:
  /**
   * @brief Refer to the superclass documentation.
   */
  bool compute_control_law(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /**
   * @brief Update parameters if any parameters have changed since last update.
   */
  void apply_parameters_update_();

  /**
   * @brief Apply scaling to the task space position reference.
   * @param[out] task_space_scaled_reference The scaled task space reference.
   */
  void apply_reference_scaling_(acg_control_msgs::msg::TaskSpacePoint& task_space_scaled_reference);

  /**
   * @brief Parameter handler to handle the controller parameters.
   */
  std::shared_ptr<acg_admittance_controller::ParamListener> admittance_controller_params_handler_;

  /**
   * @brief Controller parameters.
   */
  acg_admittance_controller::Params admittance_controller_parameters_;

  /**
   * @brief Filter chain for the force/torque measurements.
   */
  std::shared_ptr<filters::MultiChannelFilterChain<double>> filter_chain_;

  /**
   * @brief Flag indicating whether the force limitation is enabled or not.
   */
  bool force_limitation_enabled_;

  /**
   * @brief The gain for the reference scaling.
   */
  double reference_scaling_gain_;

  /**
   * @brief Flag indicating whether the logging is enabled or not.
   */
  bool logging_enabled_;

  /**
   * @brief The controller state message.
   */
  acg_control_msgs::msg::AdmittanceControllerState state_message_;

  /**
   * @brief The publisher for the controller state.
   */
  std::shared_ptr<realtime_tools::RealtimePublisher<acg_control_msgs::msg::AdmittanceControllerState>> state_publisher_;
};

}  // namespace acg_admittance_controller
