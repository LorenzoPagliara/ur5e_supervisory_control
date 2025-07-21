from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Path to the xacro file of the haptic device
    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("touch_haptic_device"),
            "urdf",
            "touch_haptic_device_and_link.ros2_control.xacro",
        ]
    )

    # Get URDF via xacro command
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_path,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Load the configuration of the controllers for the haptic feedback system
    robot_controllers_hd_cm = PathJoinSubstitution(
        [
            FindPackageShare("haptic_ff_controller"),
            "config",
            "master_controllers.yaml",
        ]
    )

    # Controller manager node for the haptic device
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_hd_cm],
        output="both",
        remappings=[
            ("~/robot_description", "/hd/robot_description"),
        ],
        namespace="hd",
    )

    # Robot state publisher node for publishing the robot description of the haptic device
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        namespace="hd",
    )

    # Haptic pose broadcaster controller spawner
    haptic_pose_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="hd",
        arguments=["haptic_pose_broadcaster", "-c", "/hd/controller_manager"],
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        haptic_pose_broadcaster_controller_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # Return the launch description with the declared arguments and launch setup
    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
