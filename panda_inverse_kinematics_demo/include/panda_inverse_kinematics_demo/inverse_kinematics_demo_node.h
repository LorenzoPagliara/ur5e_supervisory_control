/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   inverse_kinematics_demo_node.h
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Aug 11, 2023
 *
 * This node is a demo showing three different modalities of
 * performing inverse kinematics in ROS: move_group's compute_ik
 * service, RobotState API and KinematicsBase API. The pose to be
 * inverted is retrieved from /tf. Since the seed state is assigned,
 * at each iteration, a default value, the numerical kinematics solver
 * might not be able to find a solution. The result of the second IK
 * computation (with RobotState API) is used as seed state for the
 * third modality, therefore the kinematics solver exits immediately
 * returing the seed state.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"

namespace panda_inverse_kinematics_demo
{

class InverseKinematicsDemo : public rclcpp::Node
{
public:
  /**
   * @brief Demo constructor
   *
   * @param[in] options default node options
   */
  explicit InverseKinematicsDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /**
   * @brief Initialize node's attributes after object construction
   */
  void initialize_();

  /**
   * @brief Perform IK calculation periodically
   */
  void onTimer_();

  /**
   * @brief Compute inverse kinematics through move_group service
   *
   * @param[in] pose to be inverted
   * @param[in] robot_state seed to start IK search from
   */
  void computeIKWithMoveGroupService_(const geometry_msgs::msg::PoseStamped& pose, moveit::core::RobotState& robot_state) const;

  /**
   * @brief Compute inverse kinematics through RobotState API
   *
   * @param[in] pose to be inverted
   * @param[in,out] robot_state seed to start IK search from, modified with IK result
   */
  void computeIKWithRobotState_(const geometry_msgs::msg::PoseStamped& pose, moveit::core::RobotState& robot_state) const;

  /**
   * @brief Compute inverse kinematics through KinematicsBase API
   *
   * @param[in] pose to be inverted
   * @param[in,out] robot_state seed to start IK search from, modified with IK result
   */
  void computeIKWithKinematicsBase_(const geometry_msgs::msg::PoseStamped& pose, moveit::core::RobotState& robot_state) const;

  /**
   * @brief Wait for IK service server
   */
  void waitForServiceServer_() const;

  /**
   * @brief Print IK result; if success print IK solution
   *
   * @param[in] ik_success whether IK succeded or not
   * @param[in] robot_state robot state corresponding to IK solution
   */
  void printIKResult_(bool ik_success, const moveit::core::RobotState& robot_state) const;

  /**
   * @brief Print robot state
   *
   * @param[in] robot_state robot state message to be printed
   */
  void printRobotState_(const moveit_msgs::msg::RobotState& robot_state) const;

  rclcpp::TimerBase::SharedPtr timer_{ nullptr };
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_service_client_{ nullptr };
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  bool initialized_{ false };
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_{ nullptr };
  moveit::core::JointModelGroupPtr joint_model_group_{ nullptr };
};

}  // namespace panda_inverse_kinematics_demo
