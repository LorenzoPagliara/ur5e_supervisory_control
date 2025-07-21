/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   forward_kinematics_demo_node.h
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Aug 11, 2023
 *
 * This node is a demo showing three different modalities of
 * performing forward kinematics in ROS: TF2, move_group's compute_fk
 * service and RobotState API. The current joint positions read from
 * /joint_states are input to the forward kinematic process in all
 * three modalities.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"

namespace panda_forward_kinematics_demo
{

class ForwardKinematicsDemo : public rclcpp::Node
{
public:
  /**
   * @brief Demo constructor
   *
   * @param[in] options default node options
   */
  explicit ForwardKinematicsDemo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /**
   * @brief Initialize node's attributes after object construction
   */
  void initialize_();

  /**
   * @brief Perform FK calculation periodically
   */
  void onTimer_();

  /**
   * @brief Compute forward kinematics through TF2
   */
  void computeFKWithTF_() const;

  /**
   * @brief Compute forward kinematics through move_group service
   */
  void computeFKWithMoveGroupService_() const;

  /**
   * @brief Compute forward kinematics through RobotState API
   */
  void computeFKWithRobotState_() const;

  /**
   * @brief Parse joint states
   *
   * @param[in] msg message received on the topic
   */
  void jointStatesCallback_(const sensor_msgs::msg::JointState& msg);

  /**
   * @brief Wait for FK service server
   */
  void waitForServiceServer_() const;

  /**
   * @brief Print transform to stdout
   *
   * @param[in] transform to be printed
   */
  void printTransform_(const geometry_msgs::msg::TransformStamped& transform) const;

  rclcpp::TimerBase::SharedPtr timer_{ nullptr };
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_{ nullptr };
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_service_client_{ nullptr };
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  const std::string source_frame_;
  const std::string target_frame_;
  sensor_msgs::msg::JointState current_joint_states_;
  bool initialized_{ false };
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_{ nullptr };
  moveit::core::JointModelGroupPtr joint_model_group_{ nullptr };
};

}  // namespace panda_forward_kinematics_demo
