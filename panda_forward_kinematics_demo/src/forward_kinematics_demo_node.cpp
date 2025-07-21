/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   forward_kinematics_demo_node.cpp
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Aug 11, 2023
 *
 * Refer to the header file documentation.
 *
 * -------------------------------------------------------------------
 */

#include <chrono>

#include "panda_forward_kinematics_demo/forward_kinematics_demo_node.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_state/conversions.h"

using namespace panda_forward_kinematics_demo;
using namespace std::chrono_literals;
using std::placeholders::_1;

ForwardKinematicsDemo::ForwardKinematicsDemo(const rclcpp::NodeOptions& options)
  : Node("forward_kinematics_demo_node", options), source_frame_("panda/link0"), target_frame_("panda/link8")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Declare parameters that the RobotModelLoader will look for
  this->declare_parameter("robot_description", "");
  this->declare_parameter("robot_description_semantic", "");
  this->declare_parameter("robot_description_kinematics", "");

  joint_states_subscription_ =
      this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&ForwardKinematicsDemo::jointStatesCallback_, this, _1));
  fk_service_client_ = this->create_client<moveit_msgs::srv::GetPositionFK>("compute_fk");

  timer_ = this->create_wall_timer(1s, std::bind(&ForwardKinematicsDemo::onTimer_, this));
}

void ForwardKinematicsDemo::initialize_()
{
  // If the node is launched through the launch file, the parameters are found in the node configuration,
  // otherwise they are retrieved through the other nodes on the ROS2 network
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this());
  joint_model_group_ = std::shared_ptr<moveit::core::JointModelGroup>(robot_model_loader_->getModel()->getJointModelGroup("panda_arm"));
  initialized_ = true;
}

void ForwardKinematicsDemo::onTimer_()
{
  // Some initialization operations must be done after the node object has been created
  if (!initialized_)
    initialize_();

  computeFKWithTF_();

  computeFKWithMoveGroupService_();

  computeFKWithRobotState_();
}

void ForwardKinematicsDemo::computeFKWithTF_() const
{
  geometry_msgs::msg::TransformStamped t;

  try
  {
    t = tf_buffer_->lookupTransform(source_frame_, target_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", target_frame_.c_str(), source_frame_.c_str(), ex.what());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Printing FK with TF2");
  printTransform_(t);
}

void ForwardKinematicsDemo::computeFKWithMoveGroupService_() const
{
  auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();

  // Convert JointState to RobotState
  moveit::core::RobotState robot_state(robot_model_loader_->getModel());
  moveit::core::jointStateToRobotState(current_joint_states_, robot_state);

  // Initialize service request
  moveit::core::robotStateToRobotStateMsg(robot_state, request->robot_state);
  request->header.frame_id = source_frame_;
  request->header.stamp = this->now();
  request->fk_link_names.push_back(target_frame_);

  waitForServiceServer_();

  auto compute_fk_callback = [logger = this->get_logger(), print_transform = std::bind(&ForwardKinematicsDemo::printTransform_, this, _1)](
                                 rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedFutureWithRequest future)
  {
    // This code can be improved by checking the error code in the result
    tf2::Transform t;
    geometry_msgs::msg::TransformStamped t_msg;
    geometry_msgs::msg::PoseStamped p = future.get().second->pose_stamped[0];
    tf2::fromMsg(p.pose, t);
    t_msg.transform = tf2::toMsg(t);
    t_msg.header = p.header;
    t_msg.child_frame_id = future.get().second->fk_link_names[0];
    RCLCPP_INFO(logger, "Printing FK with compute_fk service");
    print_transform(t_msg);
  };

  // We cannot wait for the future here because we are in a callback,
  // thus we have to register another callback that activates when the response is ready
  fk_service_client_->async_send_request(request, std::move(compute_fk_callback));
}

void ForwardKinematicsDemo::computeFKWithRobotState_() const
{
  geometry_msgs::msg::TransformStamped t;

  moveit::core::RobotState robot_state(robot_model_loader_->getModel());

  robot_state.updateLinkTransforms();
  auto isometry = robot_state.getGlobalLinkTransform(target_frame_);

  t = tf2::eigenToTransform(isometry);
  t.header.stamp = this->now();
  t.header.frame_id = source_frame_;
  t.child_frame_id = target_frame_;

  RCLCPP_INFO(this->get_logger(), "Printing FK with RobotState API");
  printTransform_(t);
}

void ForwardKinematicsDemo::jointStatesCallback_(const sensor_msgs::msg::JointState& msg)
{
  current_joint_states_ = msg;
}

void ForwardKinematicsDemo::waitForServiceServer_() const
{
  while (!fk_service_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "compute_fk service not available, waiting again...");
  }
}

void ForwardKinematicsDemo::printTransform_(const geometry_msgs::msg::TransformStamped& transform) const
{
  std::ostringstream ss;

  ss << std::endl
     << std::endl
     << "************ Transformation from " << transform.header.frame_id << " to " << transform.child_frame_id << " ************" << std::endl;

  ss << std::endl << "------- Translation -------" << std::endl;
  ss << "[" << transform.transform.translation.x << ", " << transform.transform.translation.y << ", " << transform.transform.translation.z << "]"
     << std::endl
     << std::endl;

  tf2::Quaternion quaternion;
  tf2::fromMsg(transform.transform.rotation, quaternion);
  tf2::Vector3 rotation_axis = quaternion.getAxis();

  ss << "------- Axis/angle -------" << std::endl;
  ss << "Axis = [" << rotation_axis.getX() << ", " << rotation_axis.getY() << ", " << rotation_axis.getZ() << "]" << std::endl;
  ss << "Angle = " << quaternion.getAngle() << std::endl;

  tf2::Matrix3x3 matrix(quaternion);

  ss << std::endl << "------- Rotation matrix -------" << std::endl;
  ss << "[ " << matrix[0][0] << ", " << matrix[0][1] << ", " << matrix[0][2] << " ]" << std::endl;
  ss << "[ " << matrix[1][0] << ", " << matrix[1][1] << ", " << matrix[1][2] << " ]" << std::endl;
  ss << "[ " << matrix[2][0] << ", " << matrix[2][1] << ", " << matrix[2][2] << " ]" << std::endl;

  tf2Scalar roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);

  ss << std::endl << "------- Euler angles (RPY) -------" << std::endl;
  ss << "[ " << roll << ", " << pitch << ", " << yaw << " ]" << std::endl;

  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

RCLCPP_COMPONENTS_REGISTER_NODE(panda_forward_kinematics_demo::ForwardKinematicsDemo)
