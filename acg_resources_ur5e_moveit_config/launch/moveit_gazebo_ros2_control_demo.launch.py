# Based on Universal_Robots_ROS2_GZ_Simulation/ur_simulation_gz/launch/ur_sim_moveit.launch.py

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

DEFAULT_CONTROLLERS_FILE = "ur5e_simulation_controllers.yaml"
DEFAULT_EFFORT_CONTROLLERS_FILE = "ur5e_simulation_effort_controllers.yaml"
DEFAULT_FT_CONTROLLERS_FILE = "ur5e_simulation_ft_controllers.yaml"
DEFAULT_EFFORT_FT_CONTROLLERS_FILE = "ur5e_simulation_effort_ft_controllers.yaml"


def launch_setup(context, *args, **kwargs):
    # UR5e specific arguments
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    initial_positions_package = LaunchConfiguration("initial_positions_package")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    enable_effort_interfaces = LaunchConfiguration("enable_effort_interfaces")
    enable_ft_sensing = LaunchConfiguration("enable_ft_sensing")
    ft_plot = LaunchConfiguration("ft_plot")
    # End-effector arguments
    ee_config_package = LaunchConfiguration("ee_config_package")
    ee_config_file = LaunchConfiguration("ee_config_file")
    # Spawn position in Gazebo
    spawn_z = LaunchConfiguration("spawn_z")
    # Gazebo world
    gazebo_world_package = LaunchConfiguration("gazebo_world_package")
    gazebo_world_file_path = LaunchConfiguration("gazebo_world_file_path")

    # If a controller file is not specified by the user, choose which one to use among default ones
    controllers_file = controllers_file.perform(context)
    if controllers_file == DEFAULT_CONTROLLERS_FILE:
        if (
            enable_effort_interfaces.perform(context) == "true"
            and enable_ft_sensing.perform(context) == "true"
        ):
            controllers_file = DEFAULT_EFFORT_FT_CONTROLLERS_FILE
        else:
            if enable_effort_interfaces.perform(context) == "true":
                controllers_file = DEFAULT_EFFORT_CONTROLLERS_FILE

            if enable_ft_sensing.perform(context) == "true":
                controllers_file = DEFAULT_FT_CONTROLLERS_FILE

    # If the controllers_file is a relative path, it is converted to an absolute path
    if len(Path(controllers_file).parents) == 1:
        # If the controllers_file is a relative path, the search is defaulted to the `config` folder of the runtime_config_package
        controllers_file = PathJoinSubstitution(["config", controllers_file])

    # If the ee_config_file is a relative path, it is converted to an absolute path
    if len(Path(ee_config_file.perform(context)).parents) == 1:
        # If the ee_config_file is a relative path, the search is defaulted to the `config` folder of the runtime_config_package
        ee_config_file = PathJoinSubstitution(["config", ee_config_file])

    # We create our own `ur_control_launch` call because, in the original
    # `ur_simulation_gz/launch/ur_sim_moveit.launch.py` file, the launch file name to be included is hardcoded.
    gazebo_ros2_control_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("acg_resources_ur5e_moveit_config"),
                    "launch",
                    "gazebo_ros2_control_demo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "initial_positions_package": initial_positions_package,
            "initial_positions_file": initial_positions_file,
            "initial_joint_controller": initial_joint_controller,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "tf_prefix": tf_prefix,
            "rviz_config_package": rviz_config_package,
            "rviz_config_file": rviz_config_file,
            "launch_rviz": launch_rviz,
            "enable_ft_sensing": enable_ft_sensing,
            "enable_effort_interfaces": enable_effort_interfaces,
            "ft_plot": ft_plot,
            "ee_config_package": ee_config_package,
            "ee_config_file": ee_config_file.perform(context),
            "spawn_z": spawn_z,
            "gazebo_world_package": gazebo_world_package,
            "gazebo_world_file_path": gazebo_world_file_path,
        }.items(),
    )

    ur5e_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("acg_resources_ur5e_moveit_config"),
                    "launch",
                    "ur5e_moveit.launch.py",
                ]
            )
        ),
        launch_arguments={
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": tf_prefix,
            "use_sim_time": "true",
            "rviz_config_package": rviz_config_package,
            "rviz_config_file": rviz_config_file,
            "launch_rviz": "true",
            "sim_ignition": "true",
            "ee_config_package": ee_config_package,
            "ee_config_file": ee_config_file.perform(context),
        }.items(),
    )

    nodes_to_launch = [
        gazebo_ros2_control_demo_launch,
        ur5e_moveit_launch,
    ]

    return nodes_to_launch


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
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_package",
            default_value="ur_description",
            description="Package with the initial positions of the robot joints.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="File with the initial positions of the robot joints.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="acg_resources_ur5e_moveit_config",
            description="Package with the controller's configuration. \
        Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=DEFAULT_CONTROLLERS_FILE,
            description="YAML file with the controllers configuration. \
            The file is searched under the `config` folder of the specified `ee_config_package` or in the provided relative path.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="acg_resources_ur5e_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
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
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
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
            "launch_rviz", default_value="false", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_ft_sensing",
            default_value="false",
            description="This parameter, if true, enable the simulation with force torque sensor in gazebo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ft_plot",
            default_value="false",
            description="Enables the launch of the PlotJuggler tool for real-time visualization of F/T plots.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_effort_interfaces",
            default_value="false",
            description="If true enables the effort command interfaces.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Z coordinate of the robot spawn position in Gazebo.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_package",
            default_value="acg_resources_ft_sensor_gazebo_description",
            description="Package with the Gazebo world file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_file_path",
            default_value="",
            description="The path to the Gazebo world file inside the gazebo_world_package. \
            If not specified, the default world file is used. If the force/torque sensing is enabled, \
            the world with the force/torque sensor is used, otherwise the empty world is used.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
