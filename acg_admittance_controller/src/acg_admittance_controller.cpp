/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   acg_admittance_controller.cpp
 * Author:  Francesco D'Onofrio, Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Feb 4, 2025
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

#include <acg_common_libraries/message_utilities.hpp>
#include <acg_common_libraries/kinematics.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "acg_admittance_controller/acg_admittance_controller.hpp"

namespace acg_admittance_controller
{
controller_interface::CallbackReturn AdmittanceController::on_init()
{
  // Create the parameter listener and get the parameters
  try
  {
    admittance_controller_params_handler_ = std::make_shared<acg_admittance_controller::ParamListener>(get_node());
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error creating the parameter handler: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Call the base class on_init method
  controller_interface::CallbackReturn ret = motion_based_interaction_controller::MotionBasedInteractionController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Initialize the force filter chain
  filter_chain_ = std::make_shared<filters::MultiChannelFilterChain<double>>("double");

  // Update the controller parameters
  apply_parameters_update_();

  state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<acg_control_msgs::msg::AdmittanceControllerState>>(
      get_node()->create_publisher<acg_control_msgs::msg::AdmittanceControllerState>("~/status", rclcpp::SystemDefaultsQoS()));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AdmittanceController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Configure the force filter chain
  if (!filter_chain_->configure(3, "filter_chain_scaling", get_node()->get_node_logging_interface(), get_node()->get_node_parameters_interface()))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "The filter chain for reference scaling hasn't been correctly configured.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Call the base class on_configure method
  controller_interface::CallbackReturn ret = motion_based_interaction_controller::MotionBasedInteractionController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

bool AdmittanceController::compute_control_law(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (!reference_reader_->has_task_space_pose_interface() || acg_message_utilities::is_nan(task_space_reference_.pose))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "The task space pose interface is not available or the reference pose is NaN. The controller cannot compute the control law.");
    return false;
  }

  if (acg_message_utilities::is_nan(task_space_reference_.wrench))
  {
    task_space_reference_.wrench = geometry_msgs::msg::Wrench();
  }

  if (acg_message_utilities::is_nan(task_space_reference_.twist))
  {
    task_space_reference_.twist = geometry_msgs::msg::Twist();
  }

  // Convert the filtered exerted wrench to the desired frame
  geometry_msgs::msg::Wrench exerted_wrench_desired_frame = task_space_state_.wrench;
  acg_kinematics::transform_wrench_frame(*kinematics_, joint_space_state_.positions, task_space_reference_.pose, motion_reference_frame_,
                                         force_torque_measure_frame_, exerted_wrench_desired_frame);

  // Convert the reference wrench to the desired frame
  geometry_msgs::msg::Wrench reference_wrench_desired_frame = task_space_reference_.wrench;
  acg_kinematics::transform_wrench_frame(*kinematics_, joint_space_state_.positions, task_space_reference_.pose, motion_reference_frame_,
                                         wrench_reference_frame_, reference_wrench_desired_frame);

  // Compute the error between the reference and exerted wrench in the desired frame
  Vector6d wrench_error_desired_frame;
  acg_kinematics::compute_wrench_error(reference_wrench_desired_frame, exerted_wrench_desired_frame, wrench_error_desired_frame);

  // Set the reference for the admittance filter
  acg_control_msgs::msg::TaskSpacePoint compliant_frame_reference;
  compliant_frame_reference.pose = geometry_msgs::msg::Pose();
  compliant_frame_reference.twist = geometry_msgs::msg::Twist();
  compliant_frame_reference.acceleration = geometry_msgs::msg::Accel();
  tf2::toMsg(wrench_error_desired_frame, compliant_frame_reference.wrench);

  // Compute the admittance law
  acg_control_msgs::msg::TaskSpacePoint compliant_frame_state;
  if (!interaction_filter_->update(compliant_frame_reference, period, compliant_frame_state))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Admittance filter update failed");
    return false;
  }

  // Get the compliant frame transform with respect to the desired frame
  Eigen::Isometry3d desired_to_compliant_transform;
  tf2::fromMsg(compliant_frame_state.pose, desired_to_compliant_transform);

  // Get the compliant frame twist with respect to the desired frame
  Vector6d desired_to_compliant_twist;
  tf2::fromMsg(compliant_frame_state.twist, desired_to_compliant_twist);

  // // Update the controller parameters
  apply_parameters_update_();

  acg_control_msgs::msg::TaskSpacePoint task_space_scaled_reference = task_space_reference_;
  if (force_limitation_enabled_)
  {
    apply_reference_scaling_(task_space_scaled_reference);
  }

  // Compute the pose command
  Eigen::Isometry3d motion_to_desired_transform;
  tf2::fromMsg(task_space_scaled_reference.pose, motion_to_desired_transform);

  Eigen::Isometry3d motion_to_compliant_transform = motion_to_desired_transform * desired_to_compliant_transform;
  task_space_command_.pose = tf2::toMsg(motion_to_compliant_transform);

  // Compute the twist command
  Vector6d motion_to_desired_twist;
  tf2::fromMsg(task_space_scaled_reference.twist, motion_to_desired_twist);

  Vector6d motion_to_compliant_twist;
  motion_to_compliant_twist.head(3) = motion_to_desired_transform.rotation() * desired_to_compliant_twist.head(3);
  motion_to_compliant_twist.tail(3) = motion_to_desired_transform.rotation() * desired_to_compliant_twist.tail(3);

  tf2::toMsg(Vector6d(motion_to_desired_twist + motion_to_compliant_twist), task_space_command_.twist);

  if (logging_enabled_)
  {
    state_message_.motion_frame.data = motion_reference_frame_;
    state_message_.wrench_frame.data = wrench_reference_frame_;

    state_message_.admittance_pose_state = task_space_state_.pose;
    state_message_.admittance_pose_reference = task_space_reference_.pose;
    state_message_.admittance_pose_scaled_reference = task_space_scaled_reference.pose;
    Vector6d pose_error;
    acg_kinematics::compute_pose_error(task_space_reference_.pose, task_space_command_.pose, pose_error);
    tf2::toMsg(pose_error, state_message_.admittance_pose_error);
    Vector6d scaled_pose_error;
    acg_kinematics::compute_pose_error(task_space_scaled_reference.pose, task_space_command_.pose, scaled_pose_error);
    tf2::toMsg(scaled_pose_error, state_message_.admittance_pose_scaled_reference);

    state_message_.admittance_twist_state = task_space_state_.twist;
    state_message_.admittance_twist_reference = task_space_reference_.twist;
    state_message_.admittance_twist_scaled_reference = task_space_scaled_reference.twist;
    Vector6d twist_error;
    acg_kinematics::compute_twist_error(task_space_reference_.twist, task_space_command_.twist, twist_error);
    tf2::toMsg(twist_error, state_message_.admittance_twist_error);
    Vector6d scaled_twist_error;
    acg_kinematics::compute_twist_error(task_space_scaled_reference.twist, task_space_command_.twist, scaled_twist_error);
    tf2::toMsg(scaled_twist_error, state_message_.admittance_twist_scaled_reference);

    state_message_.wrench_state = task_space_state_.wrench;
    acg_kinematics::transform_wrench_frame(*kinematics_, joint_space_state_.positions, wrench_reference_frame_, force_torque_measure_frame_,
                                           state_message_.wrench_state);

    state_message_.wrench_reference = task_space_reference_.wrench;
    Vector6d wrench_error;
    acg_kinematics::compute_wrench_error(task_space_reference_.wrench, state_message_.wrench_state, wrench_error);
    tf2::toMsg(wrench_error, state_message_.wrench_error);

    state_message_.time_from_start = rclcpp::Duration(time.seconds(), time.nanoseconds());

    if (state_publisher_->trylock())
    {
      state_publisher_->msg_ = state_message_;
      state_publisher_->unlockAndPublish();
    }
  }

  return true;
}

void AdmittanceController::apply_parameters_update_()
{
  if (admittance_controller_params_handler_->is_old(admittance_controller_parameters_))
  {
    admittance_controller_parameters_ = admittance_controller_params_handler_->get_params();
  }

  force_limitation_enabled_ = admittance_controller_parameters_.force_limitation_enabled;

  reference_scaling_gain_ = admittance_controller_parameters_.reference_scaling_gain;

  logging_enabled_ = admittance_controller_parameters_.logging_enabled;
}

void AdmittanceController::apply_reference_scaling_(acg_control_msgs::msg::TaskSpacePoint& task_space_scaled_reference)
{
  // Get the filtered exerted wrench
  Vector6d filtered_exerted_wrench;
  tf2::fromMsg(task_space_state_.wrench, filtered_exerted_wrench);

  // Apply the filter chain to the filtered exerted wrench
  std::vector<double> filter_chain_input(3, 0.0);
  std::vector<double> filter_chain_output(3, 0.0);

  for (size_t i = 0; i < 3; i++)
  {
    filter_chain_input[i] = filtered_exerted_wrench(i);
  }

  // Update the filter chain with the input vector
  filter_chain_->update(filter_chain_input, filter_chain_output);

  // Assign the filtered values to the force
  Eigen::Vector3d filtered_exerted_force = Eigen::Vector3d(filter_chain_output.data());

  // Convert the position and velocity references to the end-effector frame
  Eigen::Isometry3d motion_to_end_effector_transform;
  tf2::fromMsg(task_space_state_.pose, motion_to_end_effector_transform);

  Eigen::Vector3d motion_to_desired_position(task_space_reference_.pose.position.x, task_space_reference_.pose.position.y,
                                             task_space_reference_.pose.position.z);

  Eigen::Vector3d motion_to_desired_velocity(task_space_reference_.twist.linear.x, task_space_reference_.twist.linear.y,
                                             task_space_reference_.twist.linear.z);

  Eigen::Vector3d end_effector_to_desired_position = motion_to_end_effector_transform.inverse() * motion_to_desired_position;
  Eigen::Vector3d end_effector_to_desired_velocity = motion_to_end_effector_transform.rotation().inverse() * motion_to_desired_velocity;

  // Compute the reference scaling
  // Note: The force vector employed may be expressed in a reference frame different from that of the end-effector.
  // However, its norm remains invariant with respect to the chosen frame of reference.

  Eigen::Vector3d scaled_end_effector_to_desired_position =
      end_effector_to_desired_position / (1 + reference_scaling_gain_ * filtered_exerted_force.norm());

  Eigen::Vector3d scaled_end_effector_to_desired_velocity =
      end_effector_to_desired_velocity / (1 + reference_scaling_gain_ * filtered_exerted_force.norm());

  // Convert the scaled position and velocity references to the motion reference frame
  Eigen::Vector3d scaled_motion_to_desired_position = motion_to_end_effector_transform * scaled_end_effector_to_desired_position;
  Eigen::Vector3d scaled_motion_to_desired_velocity = motion_to_end_effector_transform.rotation() * scaled_end_effector_to_desired_velocity;

  // Update the task space reference
  task_space_scaled_reference.pose.position.x = scaled_motion_to_desired_position(0);
  task_space_scaled_reference.pose.position.y = scaled_motion_to_desired_position(1);
  task_space_scaled_reference.pose.position.z = scaled_motion_to_desired_position(2);
  task_space_scaled_reference.twist.linear.x = scaled_motion_to_desired_velocity(0);
  task_space_scaled_reference.twist.linear.y = scaled_motion_to_desired_velocity(1);
  task_space_scaled_reference.twist.linear.z = scaled_motion_to_desired_velocity(2);
}
}  // namespace acg_admittance_controller

PLUGINLIB_EXPORT_CLASS(acg_admittance_controller::AdmittanceController, controller_interface::ChainableControllerInterface)
