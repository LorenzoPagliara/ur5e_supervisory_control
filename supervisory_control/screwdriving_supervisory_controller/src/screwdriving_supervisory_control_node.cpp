/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   screwdriving_supervisory_controller.cpp
 * Author:  Lorenzo Pagliara
 * Org.:    UNISA
 * Date:    Jun 20, 2025
 *
 *
 * -------------------------------------------------------------------
 */

#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <sys/wait.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_state/conversions.h>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <acg_control_msgs/action/follow_joint_trajectory.hpp>
#include <acg_control_msgs/msg/joint_trajectory.hpp>
#include <acg_control_msgs/msg/joint_trajectory_point.hpp>
#include <supervisory_control_rviz_msg/msg/supervisory_controller_r_viz_state.hpp>

/**
 * @brief Enum defining the steps of the screwdriving procedure.
 */
enum Step
{
  INITIALIZATION,
  APPROACH_PLANNING,
  APPROACHING,
  CENTER_IDENTIFICATION,
  COUPLING_PLANNING,
  COUPLING,
  TIGHTENING,
  RELEASING,
  DISTANCING_PLANNING,
  DISTANCING,
  COMPLETED,
  HUMAN_VALIDATION,
  MANUAL_CONTROL
};

/**
 * @brief Struct representing the state of the supervisory control node to be displayed in RViz.
 */
struct SupervisoryControlPanelState
{
  std::string state;
  std::string message;
  std::vector<std::string> enabled_buttons;
};

/**
 * @brief List of buttons in the RViz GUI.
 */
const std::vector<std::string> BUTTONS = { "next", "repeat", "manual" };

/**
 * @brief Publisher for RViz state updates.
 */
rclcpp::Publisher<supervisory_control_rviz_msg::msg::SupervisoryControllerRVizState>::SharedPtr rviz_state_publisher;

/**
 * @brief Mutex for validation received from the human operator.
 */
std::mutex validation_mutex;

/**
 * @brief Message containing the validation received from the human operator.
 */
std_msgs::msg::String validation_msg;

/**
 * @brief Flag indicating whether validation has been received from the human operator.
 */
bool validation_received = false;

/**
 * @brief Flag indicating whether the goal has been reached.
 */
bool goal_reached = false;

/**
 * @brief Flag indicating whether manual control has been requested.
 */
bool manual_control_requested = false;

/**
 * @brief Client for switching controllers.
 */
rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client;

/**
 * @brief Client for unloading controllers.
 */
rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client;

/**
 * @brief Client for loading controllers.
 */
rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client;

/**
 * @brief Client for configuring controllers.
 */
rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client;

/**
 * @brief Client for the FollowJointTrajectory action.
 */
rclcpp_action::Client<acg_control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_client;

/**
 * @brief Provides the next step in the procedure based on the provided step.
 * @param current_step A generic step in the procedure.
 * @return The next step in the procedure.
 */
Step next_step(Step current_step);

/**
 * @brief Displays the current state of the supervisory control node in RViz.
 */
void display_state_on_rviz(const SupervisoryControlPanelState& state);

/**
 * @brief Callback function for validation messages received from the human operator.
 * @param msg The message containing the validation.
 */
void validation_callback(const std_msgs::msg::String::SharedPtr msg);

/**
 * @brief Stop and unload the specified controller.
 * @param node The ROS2 node.
 * @param logger The ROS2 logger.
 * @param controller_name The name of the controller to be stopped and unloaded.
 */
void stop_and_unload_controller(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger, const std::string& controller_name);

/**
 * @brief Load and start the specified controller.
 * @param node The ROS2 node.
 * @param logger The ROS2 logger.
 * @param controller_name The name of the controller to be loaded and started.
 */
void load_and_start_controller(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger, const std::string& controller_name);

/**
 * @brief Start the teleoperation controllers.
 * @param node The ROS2 node.
 * @param logger The ROS2 logger.
 */
void start_teleoperation_controllers(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger);

/**
 * @brief Stop the teleoperation controllers.
 * @param node The ROS2 node.
 * @param logger The ROS2 logger.
 */
void stop_teleoperation_controllers(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger);

void convert_moveit_trajectory_to_acg_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                                 acg_control_msgs::action::FollowJointTrajectory::Goal& trajectory_goal,
                                                 const double scaling_factor = 5.0);

int main(int argc, char* argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create the node and iterate through node's parameter overrides
  // to declare any that have not already been declared (needed by MoveIt!).
  auto const node = std::make_shared<rclcpp::Node>("screwdriving_supervisory_controller",
                                                   rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS2 logger
  auto const logger = rclcpp::get_logger("screwdriving_supervisory_controller");

  // Create a node for the controller manager clients
  auto const controllers_manager_node =
      std::make_shared<rclcpp::Node>("controller_manager_client_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create clients for the controller manager services
  switch_controller_client =
      controllers_manager_node->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
  unload_controller_client =
      controllers_manager_node->create_client<controller_manager_msgs::srv::UnloadController>("/controller_manager/unload_controller");
  load_controller_client =
      controllers_manager_node->create_client<controller_manager_msgs::srv::LoadController>("/controller_manager/load_controller");
  configure_controller_client =
      controllers_manager_node->create_client<controller_manager_msgs::srv::ConfigureController>("/controller_manager/configure_controller");

  // Create a client for the FollowJointTrajectory action
  follow_joint_trajectory_client =
      rclcpp_action::create_client<acg_control_msgs::action::FollowJointTrajectory>(node, "/joint_space_reference_generator/follow_joint_trajectory");

  auto send_goal_option = rclcpp_action::Client<acg_control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
  using GoalHandle = rclcpp_action::ClientGoalHandle<acg_control_msgs::action::FollowJointTrajectory>;

  send_goal_option.result_callback = [&logger](const GoalHandle::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
      {
        goal_reached = true;
        return;
      }
      case rclcpp_action::ResultCode::ABORTED:
      {
        goal_reached = false;
        return;
      }
      case rclcpp_action::ResultCode::CANCELED:
      {
        goal_reached = false;
        return;
      }
      default:
      {
        goal_reached = false;
        return;
      }
    };
  };

  // Create publisher for the supervisory control node state updates in RViz
  rviz_state_publisher =
      node->create_publisher<supervisory_control_rviz_msg::msg::SupervisoryControllerRVizState>("/supervisory_control/rviz_state", 10);

  // Create a subscription to receive validation messages from the human operator
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr validation_subscriber =
      node->create_subscription<std_msgs::msg::String>("/supervisory_control/validation", 10, &validation_callback);

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS and for subscription handling
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
  const std::string PLANNING_GROUP_NAME = node->get_parameter("move_group_name").as_string();
  if (PLANNING_GROUP_NAME.empty())
  {
    RCLCPP_ERROR(logger, "Move group name is not set. Please set the 'move_group_name' parameter.");
    return 1;
  }
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP_NAME);

  // Get the base reference frame from parameters
  const std::string BASE_REFERENCE_FRAME = node->get_parameter("base_reference_frame").as_string();
  if (BASE_REFERENCE_FRAME.empty())
  {
    RCLCPP_ERROR(logger, "Base reference frame is not set. Please set the ' base_reference_frame' parameter.");
    return 1;
  }

  // Get the end effector frame from parameters
  const std::string END_EFFECTOR_FRAME = node->get_parameter("end_effector_frame").as_string();
  if (END_EFFECTOR_FRAME.empty())
  {
    RCLCPP_ERROR(logger, "End effector frame is not set. Please set the 'end_effector_frame' parameter.");
    return 1;
  }

  // Set the reference frame for the MoveGroupInterface
  move_group_interface.setPoseReferenceFrame(BASE_REFERENCE_FRAME);

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{ node, BASE_REFERENCE_FRAME, rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                                                     move_group_interface.getRobotModel() };
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

  auto const draw_trajectory_tool_path = [&moveit_visual_tools, &move_group_interface,
                                          jmg = move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP_NAME)](auto const trajectory)
  { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg->getLinkModel(move_group_interface.getEndEffectorLink()), jmg); };

  auto const waypoints = [&moveit_visual_tools, &move_group_interface, &logger](const std::vector<geometry_msgs::msg::Pose>& target_poses,
                                                                                const std::string& label = "GOAL",
                                                                                auto scale = rviz_visual_tools::LARGE)
  {
    std::vector<geometry_msgs::msg::Pose> trajectory_waypoints;

    move_group_interface.startStateMonitor();
    move_group_interface.setStartStateToCurrentState();

    // Set the first waypoint to the current pose
    geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
    trajectory_waypoints.push_back(current_pose);

    for (const auto& target_pose : target_poses)
    {
      trajectory_waypoints.push_back(target_pose);
    }

    moveit_visual_tools.publishAxisLabeled(target_poses[target_poses.size() - 1], label, scale);
    moveit_visual_tools.trigger();

    return trajectory_waypoints;
  };

  auto const execute = [&logger, &move_group_interface, &moveit_visual_tools, &draw_title, &prompt, &draw_trajectory_tool_path,
                        &send_goal_option](auto const trajectory)
  {
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    while (!follow_joint_trajectory_client->wait_for_action_server(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(logger, "Interrupted while waiting for the action server. Exiting.");
        return false;
      }
      RCLCPP_INFO(logger, "Waiting for the action server to be available...");
    }

    follow_joint_trajectory_client->async_send_goal(trajectory, send_goal_option);
    while (!goal_reached && rclcpp::ok())
    {
      if (manual_control_requested)
      {
        return false;
      }
    }
    goal_reached = false;
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.trigger();
    return true;
  };

  // Load the pipes mesh from the specified package and filename and create a collision object
  std::string mesh_path = "file://" + ament_index_cpp::get_package_share_directory(node->get_parameter("mesh_package").as_string()) +
                          node->get_parameter("mesh_filename").as_string();
  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
  if (!mesh)
  {
    RCLCPP_ERROR(logger, "Failed to load mesh: %s", mesh_path.c_str());
    return 1;
  }

  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);

  shape_msgs::msg::Mesh mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = BASE_REFERENCE_FRAME;
  collision_object.id = "pipes";

  collision_object.meshes.push_back(mesh_msg);

  geometry_msgs::msg::Pose mesh_pose;
  mesh_pose.position.x = -0.31;
  mesh_pose.position.y = 0.45;
  mesh_pose.position.z = 0.0;

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, 3.14);
  mesh_pose.orientation.w = orientation.w();
  mesh_pose.orientation.x = orientation.x();
  mesh_pose.orientation.y = orientation.y();
  mesh_pose.orientation.z = orientation.z();

  collision_object.mesh_poses.push_back(mesh_pose);
  collision_object.operation = collision_object.ADD;

  // Declare variables for trajectory planning and execution
  moveit::core::MoveItErrorCode success;
  acg_control_msgs::action::FollowJointTrajectory::Goal trajectory_goal;

  moveit_msgs::msg::RobotTrajectory trajectory;
  geometry_msgs::msg::Pose target_pose;

  bool is_procedure_completed = false;
  bool is_manual_control_active = false;

  // Initialize the current step of the procedure
  Step step = INITIALIZATION;
  Step previous_step = step;

  // Declare displayed state variable
  SupervisoryControlPanelState displayed_state;

  while (!is_procedure_completed)
  {
    switch (step)
    {
      case Step::INITIALIZATION:
      {
        // Apply the collision object to the planning scene
        planning_scene_interface.applyCollisionObjects({ collision_object });

        displayed_state.state = "Initialization";
        displayed_state.message = "Initialization complete. Press 'Next Step' to proceed.";
        displayed_state.enabled_buttons = { "next" };
        display_state_on_rviz(displayed_state);

        previous_step = step;
        step = Step::HUMAN_VALIDATION;
      }
      break;

      case Step::APPROACH_PLANNING:
      {
        displayed_state.state = "Approach: Planning";
        displayed_state.message = "Planning to the target pose for the bolt's center identification.";
        display_state_on_rviz(displayed_state);

        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.trigger();

        // Set the target pose for the approach
        tf2::Quaternion orientation;
        orientation.setRPY(3.14, 0.0, 1.5708);
        target_pose.position.x = -0.534;
        target_pose.position.y = 0.079;
        target_pose.position.z = 0.380;
        target_pose.orientation.w = orientation.w();
        target_pose.orientation.x = orientation.x();
        target_pose.orientation.y = orientation.y();
        target_pose.orientation.z = orientation.z();

        std::vector<geometry_msgs::msg::Pose> target_poses;
        target_poses.push_back(target_pose);

        // Scale velocity and acceleration to move faster
        move_group_interface.setMaxVelocityScalingFactor(1.0);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);

        // Plan the trajectory to the target pose
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_interface.computeCartesianPath(waypoints(target_poses, "GOAL", rviz_visual_tools::LARGE), eef_step,
                                                                    jump_threshold, trajectory);

        if (fraction != 1.0)
        {
          displayed_state.message = "Failed to plan the trajectory.<br>Press 'Repeat' to repeat the step or 'Manual Control' to "
                                    "switch to manual control.";
          displayed_state.enabled_buttons = { "repeat", "manual" };
        }
        else
        {
          // Draw the planned trajectory in RViz
          draw_trajectory_tool_path(trajectory);
          moveit_visual_tools.trigger();

          displayed_state.message =
              "Planned to the target pose.<br>Press 'Next Step' to continue, 'Repeat' to repeat the step, or 'Manual Control' to "
              "switch to manual control.";
          displayed_state.enabled_buttons = { "next", "repeat", "manual" };
        }

        convert_moveit_trajectory_to_acg_trajectory(trajectory, trajectory_goal);

        previous_step = step;
        step = Step::HUMAN_VALIDATION;
      }
      break;

      case Step::APPROACHING:
      {
        displayed_state.state = "Approach: Executing";
        displayed_state.message = "Approaching the bolt for the center identification.<br>Press 'Next' in the RvizVisualToolsGui window to execute.";
        displayed_state.enabled_buttons = { "manual" };
        display_state_on_rviz(displayed_state);

        previous_step = step;

        // Execute the plan
        if (execute(trajectory_goal))
        {
          displayed_state.message = "Approach completed. Ready to identify the bolt's center.<br>Press 'Next Step' to continue or 'Manual "
                                    "Control' to switch to manual control.";
          displayed_state.enabled_buttons = { "next", "manual" };
          step = Step::HUMAN_VALIDATION;
        }
        else
        {
          displayed_state.message = "Approaching interrupted.<br>Manual control requested.";
          displayed_state.enabled_buttons = { "manual" };
          step = Step::MANUAL_CONTROL;
        }
      }
      break;

      case Step::CENTER_IDENTIFICATION:
      {
        displayed_state.state = "Center Identification";
        displayed_state.message = "Identifying the center of the bolt.";
        display_state_on_rviz(displayed_state);

        // Read the bolt center pose from parameters
        std::vector<double> bolt_center_pose = node->get_parameter("bolt_center_pose").as_double_array();
        tf2::Quaternion orientation;
        orientation.setRPY(bolt_center_pose[3], bolt_center_pose[4], bolt_center_pose[5]);
        target_pose.position.x = bolt_center_pose[0];
        target_pose.position.y = bolt_center_pose[1];
        target_pose.position.z = bolt_center_pose[2];
        target_pose.orientation.w = orientation.w();
        target_pose.orientation.x = orientation.x();
        target_pose.orientation.y = orientation.y();
        target_pose.orientation.z = orientation.z();

        moveit_visual_tools.publishAxisLabeled(target_pose, "Bolt's\xa0\x63\x65nter", rviz_visual_tools::LARGE);
        moveit_visual_tools.trigger();

        displayed_state.message = "Bolt's center identified. Ready to coupling the tool with the bolt.<br>Press 'Next Step' to continue, 'Repeat' to "
                                  "repeat the step, or 'Manual "
                                  "Control' to switch to manual control.";
        displayed_state.enabled_buttons = { "next", "repeat", "manual" };

        previous_step = step;
        step = Step::HUMAN_VALIDATION;
      }
      break;

      case Step::COUPLING_PLANNING:
      {
        displayed_state.state = "Coupling: Planning";
        displayed_state.message = "Planning to the bolt's center for coupling the tool with the bolt.";
        display_state_on_rviz(displayed_state);

        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.trigger();

        std::vector<geometry_msgs::msg::Pose> target_poses;
        target_poses.push_back(target_pose);

        // Scale velocity and acceleration to move faster
        move_group_interface.setMaxVelocityScalingFactor(1.0);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);

        // Plan the trajectory to the target pose
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_interface.computeCartesianPath(waypoints(target_poses, "GOAL", rviz_visual_tools::LARGE), eef_step,
                                                                    jump_threshold, trajectory);

        if (fraction != 1.0)
        {
          displayed_state.message = "Failed to plan the trajectory.<br>Press 'Repeat' to repeat the step or 'Manual Control' to "
                                    "switch to manual control.";
          displayed_state.enabled_buttons = { "repeat", "manual" };
        }
        else
        {
          draw_trajectory_tool_path(trajectory);
          moveit_visual_tools.trigger();

          displayed_state.message =
              "Planned to the bolt's center.<br>Press 'Next Step' to continue, 'Repeat' to repeat the step, or 'Manual Control' "
              "to switch to manual control.";
          displayed_state.enabled_buttons = { "next", "repeat", "manual" };
        }

        convert_moveit_trajectory_to_acg_trajectory(trajectory, trajectory_goal);

        previous_step = step;
        step = Step::HUMAN_VALIDATION;
      }
      break;

      case Step::COUPLING:
      {
        displayed_state.state = "Coupling: Executing";
        displayed_state.message = "Coupling the tool with the bolt.<br>Press 'Next' in the RvizVisualToolsGui window to execute.";
        displayed_state.enabled_buttons = { "manual" };
        display_state_on_rviz(displayed_state);

        previous_step = step;

        // Execute the plan
        if (execute(trajectory_goal))
        {
          displayed_state.message = "Tool and bolt coupled. Ready to tighten the bolt.<br>Press 'Next Step' to continue or 'Manual "
                                    "Control' to switch to manual control.";
          displayed_state.enabled_buttons = { "next", "manual" };
          step = Step::HUMAN_VALIDATION;
        }
        else
        {
          displayed_state.message = "Coupling interrupted.<br>Manual Control requested.";
          displayed_state.enabled_buttons = { "manual" };
          step = Step::MANUAL_CONTROL;
        }
      }
      break;

      case Step::TIGHTENING:
      {
        displayed_state.state = "Tightening";
        displayed_state.message = "Tightening the bolt.<br>Press 'Manual Control' to switch to manual control.";
        displayed_state.enabled_buttons = { "manual" };
        display_state_on_rviz(displayed_state);

        previous_step = step;

        std::atomic<pid_t> tightening_pid = -1;
        bool tightening_completed = false;

        std::thread tightening_thread(
            [&]()
            {
              pid_t pid = fork();
              if (pid == 0)
              {
                setpgid(0, 0);
                execlp("ros2", "ros2", "run", "erob70i_control", "erob70i_tightening", (char*)NULL);
                perror("execlp failed");
                displayed_state.message = "Actuation failure.<br>Press 'Manual Control' to switch to manual control.";
                displayed_state.enabled_buttons = { "manual" };
                step = Step::MANUAL_CONTROL;
              }
              else if (pid > 0)
              {
                tightening_pid = pid;
                int status;
                waitpid(pid, &status, 0);
                tightening_completed = (status == 0);
              }
            });

        while (tightening_thread.joinable())
        {
          if (manual_control_requested && tightening_pid > 0)
          {
            killpg(tightening_pid, SIGTERM);
            tightening_thread.join();
            displayed_state.message = "Tightening interrupted. Manual control requested.";
            displayed_state.enabled_buttons = { "manual" };
            step = Step::MANUAL_CONTROL;
            break;
          }

          if (tightening_completed)
          {
            break;
          }
        }

        if (step != Step::MANUAL_CONTROL)
        {
          tightening_thread.join();
          displayed_state.message =
              "Tightening completed. Ready to release the bolt.<br>Press 'Next Step' to continue, 'Repeat' to repeat the step, or 'Manual Control' "
              "to switch to manual control.";
          displayed_state.enabled_buttons = { "next", "repeat", "manual" };
          step = Step::HUMAN_VALIDATION;
        }
      }
      break;
      case Step::RELEASING:
      {
        displayed_state.state = "Releasing";
        displayed_state.message = "Releasing the bolt.";
        displayed_state.enabled_buttons = { "manual" };
        display_state_on_rviz(displayed_state);

        previous_step = step;

        std::atomic<pid_t> releasing_pid = -1;
        bool releasing_completed = false;

        std::thread releasing_thread(
            [&]()
            {
              pid_t pid = fork();
              if (pid == 0)
              {
                setpgid(0, 0);
                execlp("ros2", "ros2", "run", "erob70i_control", "erob70i_releasing", (char*)NULL);
                perror("execlp failed");
                displayed_state.message = "Actuation failure.<br>Press 'Manual Control' to switch to manual control.";
                displayed_state.enabled_buttons = { "manual" };
                step = Step::MANUAL_CONTROL;
              }
              else if (pid > 0)
              {
                releasing_pid = pid;
                int status;
                waitpid(pid, &status, 0);
                releasing_completed = true;
              }
            });

        while (releasing_thread.joinable())
        {
          if (manual_control_requested && releasing_pid > 0)
          {
            killpg(releasing_pid, SIGTERM);
            releasing_thread.join();
            displayed_state.message = "Releasing interrupted. Manual control requested.";
            displayed_state.enabled_buttons = { "manual" };
            step = Step::MANUAL_CONTROL;
            break;
          }

          if (releasing_completed)
          {
            break;
          }
        }

        releasing_thread.join();

        if (step != Step::MANUAL_CONTROL)
        {
          displayed_state.message = "Releasing completed. Ready to distance from the bolt.<br>Press 'Next Step' to continue, 'Repeat' to repeat the "
                                    "step, or 'Manual Control' "
                                    "to switch to manual control.";
          displayed_state.enabled_buttons = { "next", "repeat", "manual" };
          step = Step::HUMAN_VALIDATION;
        }
      }
      break;
      case Step::DISTANCING_PLANNING:
      {
        displayed_state.state = "Distancing: Planning";
        displayed_state.message = "Planning to distance from the bolt.";
        display_state_on_rviz(displayed_state);

        moveit_visual_tools.deleteAllMarkers();
        moveit_visual_tools.trigger();

        std::vector<geometry_msgs::msg::Pose> target_poses;
        tf2::Quaternion orientation;
        orientation.setRPY(3.14, 0.0, 1.5708);
        target_pose.position.x = -0.534;
        target_pose.position.y = 0.079;
        target_pose.position.z = 0.380;
        target_pose.orientation.w = orientation.w();
        target_pose.orientation.x = orientation.x();
        target_pose.orientation.y = orientation.y();
        target_pose.orientation.z = orientation.z();

        target_poses.push_back(target_pose);

        orientation.setRPY(3.14, 0.0, 1.5708);
        target_pose.position.x = -0.395;
        target_pose.position.y = 0.079;
        target_pose.position.z = 0.380;
        target_pose.orientation.w = orientation.w();
        target_pose.orientation.x = orientation.x();
        target_pose.orientation.y = orientation.y();
        target_pose.orientation.z = orientation.z();

        target_poses.push_back(target_pose);

        // Scale velocity and acceleration to move faster
        move_group_interface.setMaxVelocityScalingFactor(1.0);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);

        move_group_interface.startStateMonitor();
        move_group_interface.setStartStateToCurrentState();

        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = move_group_interface.computeCartesianPath(waypoints(target_poses, "GOAL", rviz_visual_tools::LARGE), eef_step,
                                                                    jump_threshold, trajectory);

        if (fraction != 1.0)
        {
          displayed_state.message = "Failed to plan the trajectory.<br>Press 'Repeat' to repeat the step or 'Manual Control' to "
                                    "switch to manual control.";
          displayed_state.enabled_buttons = { "repeat", "manual" };
        }
        else
        {
          draw_trajectory_tool_path(trajectory);
          moveit_visual_tools.trigger();

          displayed_state.message =
              "Planned to distance from the bolt.<br>Press 'Next Step' to continue, 'Repeat' to repeat the step, or 'Manual Control' "
              "to switch to manual control.";
          displayed_state.enabled_buttons = { "next", "repeat", "manual" };
        }

        convert_moveit_trajectory_to_acg_trajectory(trajectory, trajectory_goal);

        previous_step = step;
        step = Step::HUMAN_VALIDATION;
      }
      break;

      case Step::DISTANCING:
      {
        displayed_state.state = "Distancing: Executing";
        displayed_state.message = "Distancing from the bolt.<br>Press 'Next' in the RvizVisualToolsGui window to execute.";
        displayed_state.enabled_buttons = { "manual" };
        display_state_on_rviz(displayed_state);

        previous_step = step;

        // Execute the plan
        if (execute(trajectory_goal))
        {
          displayed_state.message = "Distanced from the bolt.<br>Press 'Next Step' to continue or 'Manual "
                                    "Control' to switch to manual control.";
          displayed_state.enabled_buttons = { "next", "manual" };
          step = Step::HUMAN_VALIDATION;
        }
        else
        {
          displayed_state.message = "Distancing interrupted.<br>Manual Control requested.";
          displayed_state.enabled_buttons = { "manual" };
          step = Step::MANUAL_CONTROL;
        }
      }
      break;

      case Step::HUMAN_VALIDATION:
      {
        display_state_on_rviz(displayed_state);

        std::unique_lock<std::mutex> lock(validation_mutex);
        if (validation_received)
        {
          if (validation_msg.data == "next")
          {
            displayed_state.enabled_buttons.clear();
            display_state_on_rviz(displayed_state);
            step = next_step(previous_step);
            previous_step = step;
          }
          else if (validation_msg.data == "repeat")
          {
            step = previous_step;
          }
          else if (validation_msg.data == "manual")
          {
            step = Step::MANUAL_CONTROL;
          }
          validation_received = false;
          lock.unlock();
        }
      }
      break;

      case Step::MANUAL_CONTROL:
      {
        if (!is_manual_control_active)
        {
          moveit_visual_tools.deleteAllMarkers();
          moveit_visual_tools.trigger();

          planning_scene_interface.removeCollisionObjects({ collision_object.id });

          manual_control_requested = false;
          is_manual_control_active = true;

          displayed_state.state = "Manual Control";
          displayed_state.message = "Manual control mode activated. Press 'Automatic Control' to return to automatic control mode.";
          displayed_state.enabled_buttons = { "manual" };
          display_state_on_rviz(displayed_state);

          start_teleoperation_controllers(controllers_manager_node, logger);
        }
        std::unique_lock<std::mutex> lock(validation_mutex);
        if (validation_received)
        {
          if (validation_msg.data == "automatic")
          {
            moveit_visual_tools.deleteAllMarkers();
            moveit_visual_tools.trigger();

            planning_scene_interface.applyCollisionObjects({ collision_object });

            displayed_state.enabled_buttons.clear();
            display_state_on_rviz(displayed_state);
            stop_teleoperation_controllers(controllers_manager_node, logger);

            if (previous_step == Step::APPROACH_PLANNING || previous_step == Step::COUPLING_PLANNING || previous_step == Step::DISTANCING_PLANNING)
            {
              step = next_step(next_step(previous_step));
            }
            else
            {
              // If we were in an execution step, we can continue to the next step
              step = next_step(previous_step);
            }

            moveit_visual_tools.deleteAllMarkers();
            moveit_visual_tools.trigger();

            previous_step = step;
            is_manual_control_active = false;
          }

          validation_received = false;
          lock.unlock();
        }
      }
      break;

      case Step::COMPLETED:
      {
        displayed_state.state = "Completed";
        displayed_state.message = "Procedure completed successfully.";
        displayed_state.enabled_buttons.clear();
        display_state_on_rviz(displayed_state);
        is_procedure_completed = true;
      }
      break;
    }
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}

Step next_step(Step current_step)
{
  switch (current_step)
  {
    case Step::INITIALIZATION:
      return Step::APPROACH_PLANNING;
    case Step::APPROACH_PLANNING:
      return Step::APPROACHING;
    case Step::APPROACHING:
      return Step::CENTER_IDENTIFICATION;
    case Step::CENTER_IDENTIFICATION:
      return Step::COUPLING_PLANNING;
    case Step::COUPLING_PLANNING:
      return Step::COUPLING;
    case Step::COUPLING:
      return Step::TIGHTENING;
    case Step::TIGHTENING:
      return Step::RELEASING;
    case Step::RELEASING:
      return Step::DISTANCING_PLANNING;
    case Step::DISTANCING_PLANNING:
      return Step::DISTANCING;
    case Step::DISTANCING:
      return Step::COMPLETED;
    case Step::COMPLETED:
      return Step::COMPLETED;
    case Step::HUMAN_VALIDATION:
      return current_step;
    case Step::MANUAL_CONTROL:
      return current_step;
    default:
      return Step::INITIALIZATION;
  }
}

void display_state_on_rviz(const SupervisoryControlPanelState& state)
{
  supervisory_control_rviz_msg::msg::SupervisoryControllerRVizState rviz_state_msg;
  rviz_state_msg.state.data = state.state;
  rviz_state_msg.message.data = state.message;

  for (const auto& btn : state.enabled_buttons)
  {
    std_msgs::msg::String enabled_button;
    enabled_button.data = btn;
    rviz_state_msg.enabled_buttons.push_back(enabled_button);
  }

  rviz_state_publisher->publish(rviz_state_msg);
}

void validation_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(validation_mutex);
  validation_msg.data = msg->data;
  validation_received = true;

  if (msg->data == "manual")
  {
    RCLCPP_INFO(rclcpp::get_logger("screwdriving_supervisory_controller"), "Manual control requested by the operator.");
    manual_control_requested = true;
  }
}

void load_and_start_controller(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger, const std::string& controller_name)
{
  auto load_req = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  load_req->name = controller_name;

  while (!load_controller_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the load_controller service. Exiting.");
      return;
    }
    RCLCPP_INFO(logger, "Waiting for the load_controller service to be available...");
  }

  auto result = load_controller_client->async_send_request(load_req);

  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Failed to load %s", controller_name.c_str());
    return;
  }
  RCLCPP_INFO(logger, "Loaded %s", controller_name.c_str());

  auto conf_req = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
  conf_req->name = controller_name;

  while (!configure_controller_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the configure_controller service. Exiting.");
      return;
    }
    RCLCPP_INFO(logger, "Waiting for the configure_controller service to be available...");
  }

  auto result_conf = configure_controller_client->async_send_request(conf_req);
  if (rclcpp::spin_until_future_complete(node, result_conf) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Failed to configure %s", controller_name.c_str());
    return;
  }
  RCLCPP_INFO(logger, "Configured %s", controller_name.c_str());

  auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  switch_req->activate_controllers.push_back(controller_name);
  switch_req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  while (!switch_controller_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the switch_controller service. Exiting.");
      return;
    }
    RCLCPP_INFO(logger, "Waiting for the switch_controller service to be available...");
  }

  auto result_switch = switch_controller_client->async_send_request(switch_req);
  if (rclcpp::spin_until_future_complete(node, result_switch) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Failed to activate %s", controller_name.c_str());
    return;
  }
  RCLCPP_INFO(logger, "Activated %s", controller_name.c_str());
}

void stop_and_unload_controller(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger, const std::string& controller_name)
{
  auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  switch_req->deactivate_controllers.push_back(controller_name);
  switch_req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

  while (!switch_controller_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the switch_controller service. Exiting.");
      return;
    }
    RCLCPP_INFO(logger, "Waiting for the switch_controller service to be available...");
  }

  auto result_switch = switch_controller_client->async_send_request(switch_req);

  if (rclcpp::spin_until_future_complete(node, result_switch) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(logger, "Failed to deactivate %s", controller_name.c_str());
    return;
  }
  RCLCPP_INFO(logger, "Deactivated %s", controller_name.c_str());

  auto unload_req = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
  unload_req->name = controller_name;

  while (!unload_controller_client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the unload_controller service. Exiting.");
      return;
    }
    RCLCPP_INFO(logger, "Waiting for the unload_controller service to be available...");
  }

  auto result_unload = unload_controller_client->async_send_request(unload_req);
  if (rclcpp::spin_until_future_complete(node, result_unload) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Failed to unload %s", controller_name.c_str());
    return;
  }
  RCLCPP_INFO(logger, "Unloaded %s", controller_name.c_str());
}

void start_teleoperation_controllers(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger)
{
  std::string reference_generator_controller = node->get_parameter("reference_generator_controller").as_string();
  std::vector<std::string> teleoperation_controllers = node->get_parameter("teleoperation_controllers").as_string_array();

  stop_and_unload_controller(node, logger, reference_generator_controller);

  for (std::size_t i = 0; i < teleoperation_controllers.size(); ++i)
  {
    load_and_start_controller(node, logger, teleoperation_controllers[i]);
  }
}

void stop_teleoperation_controllers(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger)
{
  std::string reference_generator_controller = node->get_parameter("reference_generator_controller").as_string();
  std::vector<std::string> teleoperation_controllers = node->get_parameter("teleoperation_controllers").as_string_array();

  for (std::size_t i = 0; i < teleoperation_controllers.size(); ++i)
  {
    stop_and_unload_controller(node, logger, teleoperation_controllers[teleoperation_controllers.size() - 1 - i]);
  }

  load_and_start_controller(node, logger, reference_generator_controller);
}

void convert_moveit_trajectory_to_acg_trajectory(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                                 acg_control_msgs::action::FollowJointTrajectory::Goal& trajectory_goal, const double scaling_factor)
{
  trajectory_goal = acg_control_msgs::action::FollowJointTrajectory::Goal();
  trajectory_goal.trajectory.header = trajectory.joint_trajectory.header;
  trajectory_goal.trajectory.joint_names = trajectory.joint_trajectory.joint_names;

  for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
  {
    acg_control_msgs::msg::JointTrajectoryPoint trajectory_point;
    trajectory_point.point.positions = trajectory.joint_trajectory.points[i].positions;
    trajectory_point.time_from_start = rclcpp::Duration(trajectory.joint_trajectory.points[i].time_from_start) * scaling_factor;
    trajectory_goal.trajectory.points.push_back(trajectory_point);
  }
}
