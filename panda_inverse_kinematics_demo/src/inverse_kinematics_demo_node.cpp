/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   inverse_kinematics_demo_node.cpp
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Aug 11, 2023
 *
 * Refer to the header file documentation.
 *
 * -------------------------------------------------------------------
 */

#include "panda_inverse_kinematics_demo/inverse_kinematics_demo_node.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/utils/moveit_error_code.h"

using namespace panda_inverse_kinematics_demo;
using namespace std::chrono_literals;

InverseKinematicsDemo::InverseKinematicsDemo(const rclcpp::NodeOptions& options) : Node("inverse_kinematics_demo_node", options)
{
  // Declare parameters that the RobotModelLoader will look for
  this->declare_parameter("robot_description", "");
  this->declare_parameter("robot_description_semantic", "");
  this->declare_parameter("robot_description_kinematics", "");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  ik_service_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");

  timer_ = this->create_wall_timer(1s, std::bind(&InverseKinematicsDemo::onTimer_, this));
}

void InverseKinematicsDemo::initialize_()
{
  // If the node is launched through the launch file, the parameters are found in the node configuration,
  // otherwise they are retrieved through the other nodes on the ROS2 network
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this());
  joint_model_group_ = std::shared_ptr<moveit::core::JointModelGroup>(robot_model_loader_->getModel()->getJointModelGroup("panda_arm"));
  initialized_ = true;
}

void InverseKinematicsDemo::onTimer_()
{
  // Some initialization operations must be done after the node object has been created
  if (!initialized_)
    initialize_();

  // Get the pose to be inverted from tf2
  geometry_msgs::msg::TransformStamped t_msg;
  tf2::Transform t;

  try
  {
    t_msg = tf_buffer_->lookupTransform("panda/link0", "panda/link8", tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_INFO(this->get_logger(), "Could not transform panda/link8 to panda/link0: %s", ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped p;
  tf2::fromMsg(t_msg.transform, t);
  tf2::toMsg(t, p.pose);
  p.header = t_msg.header;

  // Robot state containing the seed
  moveit::core::RobotState rs(robot_model_loader_->getModel());
  rs.setToDefaultValues();

  computeIKWithMoveGroupService_(p, rs);

  computeIKWithRobotState_(p, rs);

  computeIKWithKinematicsBase_(p, rs);
}

void InverseKinematicsDemo::computeIKWithMoveGroupService_(const geometry_msgs::msg::PoseStamped& pose, moveit::core::RobotState& robot_state) const
{
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

  request->ik_request.group_name = "panda_arm";
  moveit::core::robotStateToRobotStateMsg(robot_state, request->ik_request.robot_state);
  request->ik_request.pose_stamped = pose;

  waitForServiceServer_();

  auto compute_ik_callback = [this](rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFutureWithRequest future)
  {
    const moveit_msgs::msg::MoveItErrorCodes result = future.get().second->error_code;

    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Printing IK with compute_ik service");
      printRobotState_(future.get().second->solution);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "IK solver exited with error: %s", moveit::core::error_code_to_string(result.val).c_str());
    }
  };

  // We cannot wait for the future here because we are in a callback,
  // thus we have to register another callback that activates when the response is ready
  ik_service_client_->async_send_request(request, std::move(compute_ik_callback));
}

void InverseKinematicsDemo::computeIKWithRobotState_(const geometry_msgs::msg::PoseStamped& pose, moveit::core::RobotState& robot_state) const
{
  auto ik_success = robot_state.setFromIK(joint_model_group_.get(), pose.pose);

  RCLCPP_INFO(get_logger(), "Printing IK with RobotState API");
  printIKResult_(ik_success, robot_state);
}

void InverseKinematicsDemo::computeIKWithKinematicsBase_(const geometry_msgs::msg::PoseStamped& pose, moveit::core::RobotState& robot_state) const
{
  auto ik_solver = joint_model_group_->getSolverInstance();
  std::vector<double> seed_state;

  // Initialize seed state (it must be a std::vector)
  // We use the result of previous IK, so this solver return immediately
  for (auto joint_name : joint_model_group_->getVariableNames())
    seed_state.push_back(robot_state.getVariablePosition(joint_name));

  std::vector<double> ik_solution;
  moveit_msgs::msg::MoveItErrorCodes result;

  auto ik_success = ik_solver->getPositionIK(pose.pose, seed_state, ik_solution, result);
  robot_state.setJointGroupPositions(joint_model_group_.get(), ik_solution);

  RCLCPP_INFO(get_logger(), "Printing IK with KinematicsBase API");
  printIKResult_(ik_success, robot_state);
}

void InverseKinematicsDemo::waitForServiceServer_() const
{
  while (!ik_service_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(get_logger(), "compute_ik service not available, waiting again...");
  }
}

void InverseKinematicsDemo::printIKResult_(bool ik_success, const moveit::core::RobotState& robot_state) const
{
  if (ik_success)
  {
    moveit_msgs::msg::RobotState rs_msg;
    moveit::core::robotStateToRobotStateMsg(robot_state, rs_msg);
    printRobotState_(rs_msg);
  }
  else
    RCLCPP_ERROR(get_logger(), "Could not compute IK");
}

void InverseKinematicsDemo::printRobotState_(const moveit_msgs::msg::RobotState& robot_state) const
{
  auto joint_state = robot_state.joint_state;

  RCLCPP_INFO(get_logger(), "[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", joint_state.position[0], joint_state.position[1], joint_state.position[2],
              joint_state.position[3], joint_state.position[4], joint_state.position[5], joint_state.position[6]);
}

RCLCPP_COMPONENTS_REGISTER_NODE(panda_inverse_kinematics_demo::InverseKinematicsDemo)
