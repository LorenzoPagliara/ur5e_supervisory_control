from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    robot_state_publisher_frequency_arg = DeclareLaunchArgument(
        "robot_state_publisher_frequency",
        default_value="1000.0",
        description="The frequency (in Hz) at which the robot_state_publisher publishes.",
    )
    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("touch_haptic_device"),
            "urdf",
            "touch_haptic_device.xacro",
        ]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_frequency = {
        "publish_frequency": LaunchConfiguration("robot_state_publisher_frequency")
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("touch_haptic_device"),
            "config",
            "haptic_pose_broadcaster.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, robot_state_publisher_frequency],
    )

    haptic_pose_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["haptic_pose_broadcaster", "-c", "/controller_manager"],
    )

    # Path to the layout file for the PlotJuggler configuration
    layout_path = PathJoinSubstitution(
        [
            FindPackageShare("touch_haptic_device"),
            "config",
            "haptic_pose_plot.xml",
        ]
    )

    # Launch PlotJuggler with the layout file
    node_plot = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", layout_path],
    )

    nodes = [
        control_node,
        robot_state_publisher_frequency_arg,
        robot_state_pub_node,
        haptic_pose_broadcaster_controller_spawner,
        node_plot,
    ]

    return LaunchDescription(nodes)
