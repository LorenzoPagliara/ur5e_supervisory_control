# Based on ur_moveit_config/launch/ur_moveit.launch.py

import os
from pathlib import Path

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):
    # UR5e Specific Arguments
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    enable_ft_sensing = LaunchConfiguration("enable_ft_sensing")
    sim_ignition = LaunchConfiguration("sim_ignition")
    # End-effector arguments
    ee_config_package = LaunchConfiguration("ee_config_package")
    ee_config_file = LaunchConfiguration("ee_config_file")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            "ur5e",
            "default_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            FindPackageShare("ur_description"),
            "config",
            "ur5e",
            "physical_parameters.yaml",
        ]
    )

    # If the ee_config_file is a relative path, it is converted to an absolute path
    if len(Path(ee_config_file.perform(context)).parents) == 1:
        # If the ee_config_file is a relative path, the search is defaulted to the `config` folder of the runtime_config_package
        ee_config_file = PathJoinSubstitution(["config", ee_config_file])

    ee_config_file_path = PathJoinSubstitution(
        [FindPackageShare(ee_config_package), ee_config_file]
    ).perform(context)
    if not ee_config_file.perform(context):
        ee_config_file_path = ""

    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur5e", "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "enable_ft_sensing:=",
            enable_ft_sensing,
            " ",
            "ee_config_file:=",
            ee_config_file_path,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            "ur",
            " ",
            "prefix:=",
            tf_prefix,
            " ",
            "ee_config_file:=",
            ee_config_file_path,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "ur_moveit_config",
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "ur_moveit_config", os.path.join("config", "ompl_planning.yaml")
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml(
        "ur_moveit_config", os.path.join("config", "controllers.yaml")
    )
    controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    publish_robot_description_semantic = {"publish_robot_description_semantic": True}

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
        ],
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(rviz_config_package), rviz_config_file]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_moveit_config", os.path.join("config", "ur_servo.yaml"))
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    nodes_to_start = [move_group_node, rviz_node, servo_node]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # UR5e specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="acg_resources_ur5e_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur5e_with_end_effector.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="acg_resources_ur5e_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur5e_with_end_effector.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits that augment or override the values from the URDF robot_description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planning in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="ur_moveit_config",
            description="Package with the RViz configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="rviz/view_robot.rviz",
            description="RViz configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo", default_value="true", description="Launch Servo?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Specifies if the robot is simulated in Ignition Gazebo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_ft_sensing",
            default_value="false",
            description="Enable force/torque sensing",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_config_package",
            default_value="acg_resources_ur5e_moveit_config",
            description="Package with the end-effector configuration file under the `config` folder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_config_file",
            default_value="",
            description="End-effector configuration file. \
            The file is searched under the `config` folder of the specified `ee_config_package` or in the provided relative path.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
