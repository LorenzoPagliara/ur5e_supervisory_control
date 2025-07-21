/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   admittance_filter.cpp
 * Author:  Francesco D'Onofrio, Salvatore Paolino
 * Org.:    UNISA
 * Date:    Aug 23, 2024
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <acg_common_libraries/message_utilities.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "admittance_filter/admittance_filter.hpp"

namespace interaction_filters
{
AdmittanceFilter::~AdmittanceFilter()
{
  reset();
}

bool AdmittanceFilter::update(const acg_control_msgs::msg::TaskSpacePoint& task_space_reference, const rclcpp::Duration& delta_t,
                              acg_control_msgs::msg::TaskSpacePoint& task_space_command)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(logging_interface_->get_logger(), "%s not initialized", filter_name_.c_str());
    return false;
  }

  // Update the filter parameters
  apply_parameters_update();

  // Get the reference values
  Vector6d x_d, x_dot_d, x_dot_dot_d, delta_h;
  tf2::fromMsg(task_space_reference.pose, x_d);
  tf2::fromMsg(task_space_reference.twist, x_dot_d);
  tf2::fromMsg(task_space_reference.acceleration, x_dot_dot_d);
  tf2::fromMsg(task_space_reference.wrench, delta_h);

  // Compute the admittance law
  switch (order_)
  {
    case 0:
    {
      x_dot_dot_c_ = x_dot_dot_d;
      x_dot_c_ = x_dot_d;
      x_c_ = -K_P_.inverse() * (-delta_h - K_P_ * x_d);
      break;
    }
    case 1:
    {
      x_dot_dot_c_ = x_dot_dot_d;
      x_dot_c_ = -K_D_.inverse() * (-delta_h - K_D_ * x_dot_d - K_P_ * (x_d - x_c_));
      x_c_ = x_dot_c_ * delta_t.seconds() + x_c_;
      break;
    }
    case 2:
    {
      x_dot_dot_c_ = x_dot_dot_d - M_d_inv_ * (-delta_h - K_D_ * (x_dot_d - x_dot_c_) - K_P_ * (x_d - x_c_));
      x_dot_c_ = x_dot_dot_c_ * delta_t.seconds() + x_dot_c_;
      x_c_ = x_dot_c_ * delta_t.seconds() + x_c_;
      break;
    }
    default:
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "The order of the %s should be an integer in the set {0, 1, 2}", filter_name_.c_str());
      return false;
    }
  }

  // Zero out the non-compliant axes
  x_dot_dot_c_ = x_dot_dot_c_.cwiseProduct(compliant_axis_) + x_dot_dot_d.cwiseProduct(Vector6d::Ones() - compliant_axis_);
  x_dot_c_ = x_dot_c_.cwiseProduct(compliant_axis_) + x_dot_d.cwiseProduct(Vector6d::Ones() - compliant_axis_);
  x_c_ = x_c_.cwiseProduct(compliant_axis_) + x_d.cwiseProduct(Vector6d::Ones() - compliant_axis_);

  tf2::toMsg(x_c_, task_space_command.pose);
  tf2::toMsg(x_dot_c_, task_space_command.twist);
  tf2::toMsg(x_dot_dot_c_, task_space_command.acceleration);

  return true;
}

bool AdmittanceFilter::reset()
{
  M_d_inv_ = Vector6d::Zero().asDiagonal();
  K_D_ = Vector6d::Zero().asDiagonal();
  K_P_ = Vector6d::Zero().asDiagonal();
  x_c_ = Vector6d::Zero();
  x_dot_c_ = Vector6d::Zero();
  x_dot_dot_c_ = Vector6d::Zero();
  compliant_axis_ = Vector6d::Zero();
  return true;
}

void AdmittanceFilter::apply_parameters_update()
{
  // Get filter parameters
  if (parameter_handler_->is_old(parameters_))
  {
    parameters_ = parameter_handler_->get_params();
  }

  order_ = parameters_.order;

  K_P_ = Vector6d(parameters_.stiffness.data()).asDiagonal();
  K_D_ = Vector6d::Zero().asDiagonal();
  M_d_inv_ = Vector6d::Zero().asDiagonal();

  for (std::size_t i = 0; i < NUM_CARTESIAN_DOF_; ++i)
  {
    M_d_inv_.diagonal()(i) = 1.0 / parameters_.mass[i];
    K_D_.diagonal()(i) = parameters_.damping_ratio[i] * 2 * sqrt(parameters_.mass[i] * K_P_(i, i));
    compliant_axis_(i) = parameters_.compliant_axis[i] ? 1.0 : 0.0;
  }
}

bool AdmittanceFilter::initialize()
{
  // Initialize the filter variables
  reset();

  // Initialize the parameter handler
  parameter_handler_ = std::make_shared<admittance_filter::ParamListener>(params_interface_, logging_interface_->get_logger(), filter_name_);

  // Update filter parameters
  apply_parameters_update();
  return true;
}
}  // namespace interaction_filters

PLUGINLIB_EXPORT_CLASS(interaction_filters::AdmittanceFilter, interaction_filters::InteractionFilterBase)
