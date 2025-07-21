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

    # Path to the xacro file of the haptic device
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

    # Robot state publisher node to publish the robot description, aka the touch haptic device
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, robot_state_publisher_frequency],
        namespace="hd",
    )

    return LaunchDescription(
        [robot_state_publisher_frequency_arg, robot_state_publisher_node]
    )
