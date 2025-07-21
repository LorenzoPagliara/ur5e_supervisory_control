/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   admittance_filter.hpp
 * Author:  Francesco D'Onofrio, Salvatore Paolino
 * Org.:    UNISA
 * Date:    Aug 23, 2024
 *
 * This class implements all the methods to use an admittance filter.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <memory>
#include <Eigen/Dense>
#include <interaction_filter_base/interaction_filter_base.hpp>
#include <admittance_filter/admittance_filter_parameters.hpp>

namespace interaction_filters
{
static const int NUM_CARTESIAN_DOF_ = 6;
typedef Eigen::Matrix<double, NUM_CARTESIAN_DOF_, NUM_CARTESIAN_DOF_> Matrix6d;
typedef Eigen::Matrix<double, NUM_CARTESIAN_DOF_, 1> Vector6d;

class AdmittanceFilter : public interaction_filters::InteractionFilterBase
{
public:
  /**
   * @brief Refer to the superclass documentation.
   */
  ~AdmittanceFilter() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  bool update(const acg_control_msgs::msg::TaskSpacePoint& task_space_reference, const rclcpp::Duration& delta_t,
              acg_control_msgs::msg::TaskSpacePoint& task_space_command) override;

  /**
   * @brief Refer to the superclass documentation.
   */
  bool reset() override;

  /**
   * @brief Refer to the superclass documentation.
   */
  void apply_parameters_update() override;

protected:
  /**
   * @brief Refer to the superclass documentation.
   */
  bool initialize() override;

private:
  /**
   * @brief Pointer to the parameter handler.
   */
  std::shared_ptr<admittance_filter::ParamListener> parameter_handler_;

  /**
   * @brief Filter parameters.
   */
  admittance_filter::Params parameters_;

  /**
   * @brief Mass matrix.
   */
  Matrix6d M_d_inv_;

  /**
   * @brief Damping matrix.
   */
  Matrix6d K_D_;

  /**
   * @brief Stiffness matrix.
   */
  Matrix6d K_P_;

  /**
   * @brief Task space pose command.
   */
  Vector6d x_c_;

  /**
   * @brief Task space twist command.
   */
  Vector6d x_dot_c_;

  /**
   * @brief Task space acceleration command.
   */
  Vector6d x_dot_dot_c_;

  /**
   * @brief Order of the filter in the set {0, 1, 2}.
   */
  unsigned short int order_;

  /**
   * @brief Vector of compliant axes, where 1.0 means compliant and 0.0 means stiff.
   */
  Vector6d compliant_axis_;
};

}  // namespace interaction_filters
