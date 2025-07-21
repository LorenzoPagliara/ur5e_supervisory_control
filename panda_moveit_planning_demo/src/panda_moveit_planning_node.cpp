/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   panda_moveit_planning_demo.cpp
 * Author:  Enrico Ferrentino
 * Org.:    UNISA
 * Date:    Jul 27, 2023
 *
 * This node is a demo over the MoveIt!2 planning framework. It
 * explores several planning modalities using the MoveGroupInterface
 * class.
 *
 * -------------------------------------------------------------------
 */

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

int main(int argc, char* argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create the node and iterate through node's parameter overrides
  // to declare any that have not already been declared (needed by MoveIt!).
  auto const node =
      std::make_shared<rclcpp::Node>("panda_moveit_planning_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("panda_moveit_planning_demo");

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread(
      [&executor]()
      {
        executor.spin();
        rclcpp::shutdown();
      });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "panda/link0", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel() };
  moveit_visual_tools.loadRemoteControl();

  // Construct a planning scene interface to manage objects in the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create closures for visualization and execution
  auto const draw_title = [&moveit_visual_tools](auto text)
  {
    auto const text_pose = []
    {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  auto const prompt = [&moveit_visual_tools](auto text) { moveit_visual_tools.prompt(text); };

  auto const draw_trajectory_tool_path = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("panda_arm")](
                                             auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };

  auto const execute =
      [&logger, &move_group_interface, &moveit_visual_tools, &draw_title, &prompt, &draw_trajectory_tool_path](auto const success, auto const plan)
  {
    if (success)
    {
      draw_trajectory_tool_path(plan.trajectory_);
      moveit_visual_tools.trigger();
      prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
      draw_title("Executing");
      moveit_visual_tools.trigger();
      move_group_interface.execute(plan);
    }
    else
    {
      // Hex is used in these strings to control spacing in RViz
      draw_title("Planning\xa0\x66\x61iled!");
      moveit_visual_tools.trigger();
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  };

  auto const remove_collision_objects = [&planning_scene_interface]()
  {
    std::vector<std::string> keys;
    for (auto entry : planning_scene_interface.getObjects())
      keys.push_back(entry.first);
    planning_scene_interface.removeCollisionObjects(keys);
  };

  //*******************************************************************
  // PLAN TO A TARGET JOINT-SPACE CONFIGURATION
  //*******************************************************************

  // Wait for user input, then clean the scene from previous executions
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan to the target joint-space configuration");
  moveit_visual_tools.deleteAllMarkers();
  draw_title("Planning");
  moveit_visual_tools.trigger();
  remove_collision_objects();

  // Set a target joint-space configuration
  auto const target_joints_1 = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.571 };
  bool within_bounds = move_group_interface.setJointValueTarget(target_joints_1);
  if (!within_bounds)
  {
    RCLCPP_WARN(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  // Scale velocity and acceleration
  // (this only affects time parametrization after planning)
  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  // Create a plan to the target joint-space configuration
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto success = move_group_interface.plan(plan);

  // Execute the plan
  execute(success, plan);

  //*******************************************************************
  // PLAN TO A TARGET POSE
  //*******************************************************************

  // Clean the scene
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan to the target pose");
  moveit_visual_tools.deleteAllMarkers();
  draw_title("Planning");
  moveit_visual_tools.trigger();

  // Set a target Pose
  auto const target_pose_1 = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose_1);

  // Scale velocity and acceleration to move faster
  move_group_interface.setMaxVelocityScalingFactor(0.3);
  move_group_interface.setMaxAccelerationScalingFactor(0.3);

  // Create a plan to the target pose
  success = move_group_interface.plan(plan);

  // Execute the plan
  execute(success, plan);

  //*******************************************************************
  // PLAN TO A TARGET POSE WITH PATH CONSTRAINTS
  //*******************************************************************

  // Clean the scene
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan to the target pose with path constraints");
  moveit_visual_tools.deleteAllMarkers();
  draw_title("Planning");
  moveit_visual_tools.trigger();

  // Set a target Pose
  auto const target_pose_2 = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.2;
    msg.position.y = -0.4;
    msg.position.z = 0.7;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose_2);

  // Set a constraint to keep the gripper's fingers up
  auto const constraints = []
  {
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "panda/link7";
    ocm.header.frame_id = "panda/link0";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    constraints.orientation_constraints.push_back(ocm);
    return constraints;
  }();

  move_group_interface.setPathConstraints(constraints);

  // Create a plan to the target pose
  // Note 1: constraint only respected at path waypoints
  // Note 2: try this function with enforce_joint_model_state_space in ompl_planning.yaml
  success = move_group_interface.plan(plan);

  // Execute the plan
  execute(success, plan);

  // Remove path constraints
  move_group_interface.clearPathConstraints();

  //*******************************************************************
  // PLAN TO A TARGET POSE WITH OBSTACLES
  //*******************************************************************

  // Clean the scene
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan with obstacles");
  moveit_visual_tools.deleteAllMarkers();
  draw_title("Planning");
  moveit_visual_tools.trigger();

  // Set a target Pose
  auto const target_pose_3 = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.4;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose_3);

  // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to the target pose
  success = move_group_interface.plan(plan);

  // Execute the plan
  execute(success, plan);

  //*******************************************************************
  // PLAN A CARTESIAN PATH
  //*******************************************************************

  // Clean the scene
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan a Cartesian path");
  moveit_visual_tools.deleteAllMarkers();
  draw_title("Moving\xa0to\xa0home\xa0\x63onfiguration\xa0\x61nd\xa0planning");
  moveit_visual_tools.trigger();
  remove_collision_objects();

  // Move to home configuration
  move_group_interface.setNamedTarget("ready");
  move_group_interface.move();

  auto const waypoints = [&move_group_interface]
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Set first waypoint to current robot pose
    auto target_pose_4 = move_group_interface.getCurrentPose().pose;
    waypoints.push_back(target_pose_4);  // start

    target_pose_4.position.z -= 0.2;
    waypoints.push_back(target_pose_4);  // down

    target_pose_4.position.y -= 0.2;
    waypoints.push_back(target_pose_4);  // right

    target_pose_4.position.z += 0.2;
    target_pose_4.position.y += 0.2;
    target_pose_4.position.x -= 0.2;
    waypoints.push_back(target_pose_4);  // up and left

    return waypoints;
  }();

  auto const [fraction, cartesian_plan] = [&logger, &move_group_interface, &waypoints]
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(logger, "Achieved %.2f%% of Cartesian path", fraction * 100.0);
    robotStateToRobotStateMsg(*move_group_interface.getCurrentState(), plan.start_state_);
    plan.trajectory_ = trajectory;
    return std::make_pair(fraction, plan);
  }();

  // Execute the plan
  execute(fraction == 1.0, cartesian_plan);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
