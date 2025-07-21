import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    FindExecutable,
    Command,
)
from launch_ros.substitutions import (
    FindPackageShare,
)
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

ROBOT_DEFAULT_FT_CONTROLLERS_FILE = "force_based_teleop_simulation_controllers.yaml"


def launch_setup(context, *args, **kwargs):
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    robot_controllers_file = LaunchConfiguration("robot_controllers_file")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    haptic_device_controllers_file = LaunchConfiguration(
        "haptic_device_controllers_file"
    )
    haptic_device_description_package = LaunchConfiguration(
        "haptic_device_description_package"
    )
    haptic_device_description_file = LaunchConfiguration(
        "haptic_device_description_file"
    )
    tf_prefix = LaunchConfiguration("tf_prefix")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    initial_positions_package = LaunchConfiguration("initial_positions_package")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    gazebo_world_package = LaunchConfiguration("gazebo_world_package")
    gazebo_world_file_path = LaunchConfiguration("gazebo_world_file_path")
    plot_teleoperation_state = LaunchConfiguration("plot_teleoperation_state")
    spawn_z = LaunchConfiguration("spawn_z")
    ee_config_package = LaunchConfiguration("ee_config_package")
    ee_config_file = LaunchConfiguration("ee_config_file")
    spawn_z = LaunchConfiguration("spawn_z")

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
            "safety_limits": "true",
            "runtime_config_package": runtime_config_package,
            "controllers_file": robot_controllers_file,
            "initial_joint_controller": initial_joint_controller,
            "description_package": robot_description_package,
            "description_file": robot_description_file,
            "prefix": tf_prefix,
            "rviz_config_package": rviz_config_package,
            "rviz_config_file": rviz_config_file,
            "launch_rviz": "true",
            "enable_ft_sensing": "true",
            "enable_effort_interfaces": "false",
            "ft_plot": "false",
            "ee_config_package": ee_config_package,
            "ee_config_file": ee_config_file,
            "initial_positions_package": initial_positions_package,
            "initial_positions_file": initial_positions_file,
            "gazebo_world_package": gazebo_world_package,
            "gazebo_world_file_path": gazebo_world_file_path,
            "spawn_z": spawn_z,
        }.items(),
    )

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare(haptic_device_description_package),
            "urdf",
            haptic_device_description_file,
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )

    haptic_device_description = {"robot_description": robot_description_content}

    haptic_device_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            haptic_device_controllers_file,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[haptic_device_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/hd/robot_description"),
        ],
        namespace="hd",
    )

    haptic_device_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[haptic_device_description],
        namespace="hd",
    )

    haptic_pose_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="hd",
        arguments=["haptic_pose_broadcaster", "-c", "/hd/controller_manager"],
    )

    haptic_twist_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="hd",
        arguments=["haptic_twist_broadcaster", "-c", "/hd/controller_manager"],
    )

    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "admittance_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    haptic_ff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["haptic_ff_controller", "-c", "/controller_manager"],
    )

    haptic_fb_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["haptic_fb_controller", "-c", "/controller_manager"],
    )

    delay_admittance_controller_spawner = TimerAction(
        period=5.0,
        actions=[admittance_controller_spawner],
    )

    delay_ff_after_admittance_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=admittance_controller_spawner,
            on_exit=[haptic_ff_controller_spawner],
        )
    )

    delay_fb_after_ff_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=haptic_ff_controller_spawner,
            on_exit=[haptic_fb_controller_spawner],
        )
    )

    haptic_forward_force_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["haptic_forward_force_controller", "-c", "/hd/controller_manager"],
        namespace="hd",
    )

    layout_path_plotjuggler = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            "force_based_teleop_state_plots.xml",
        ]
    )

    plotjuggler_starter = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", layout_path_plotjuggler],
        condition=IfCondition(plot_teleoperation_state),
    )

    nodes_to_start = [
        control_node,
        gazebo_ros2_control_demo_launch,
        delay_admittance_controller_spawner,
        delay_ff_after_admittance_controller_spawner,
        delay_fb_after_ff_controller_spawner,
        haptic_device_state_publisher_node,
        haptic_pose_broadcaster_controller_spawner,
        haptic_twist_broadcaster_controller_spawner,
        haptic_forward_force_controller_spawner,
        plotjuggler_starter,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur5e_app_screwdriving_teleop",
            description='Package with the controller\'s configuration in "config" folder. \
            Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controllers_file",
            default_value=ROBOT_DEFAULT_FT_CONTROLLERS_FILE,
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="acg_resources_ur5e_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_file",
            default_value="ur5e_with_end_effector.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "haptic_device_controllers_file",
            default_value="touch_controllers.yaml",
            description="YAML file with the haptic device controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "haptic_device_description_package",
            default_value="touch_haptic_device",
            description="Description package with haptic device URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "haptic_device_description_file",
            default_value="touch_haptic_device.xacro",
            description="URDF/XACRO description file with the haptic device.",
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
            default_value="ur5e_app_screwdriving_teleop",
            description="Package with the RViz configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="config/ur5e_moveit.rviz",
            description="RViz configuration file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="cartesian_pose_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_package",
            default_value="ur5e_app_screwdriving_teleop",
            description="Package with the initial joint positions file under the `config` folder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_robot_positions.yaml",
            description="Initial joint positions of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_package",
            default_value="acg_resources_ft_sensor_gazebo_description",
            description="Package with the world file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_world_file_path",
            default_value="world/world_with_ft_sensor.sdf",
            description="World file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "plot_teleoperation_state",
            default_value="true",
            description="Enables the launch of the PlotJuggler tool for real-time visualization of F/T plots.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Z coordinate of the robot spawn position.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_config_package",
            default_value="ur5e_app_screwdriving_teleop",
            description="Package with the end-effector configuration file under the `config` folder.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_config_file",
            default_value="config/gazebo_ft_sensor_and_screwdriver_ee_config.yaml",
            description="End-effector configuration file. \
            The file is searched under the `config` folder of the specified `ee_config_package` or in the provided relative path.",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
