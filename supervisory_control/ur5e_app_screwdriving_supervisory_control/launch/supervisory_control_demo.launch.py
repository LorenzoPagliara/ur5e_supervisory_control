import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from datetime import datetime


def launch_setup(context, *args, **kwargs):
    wrong_bolt_identification = LaunchConfiguration("wrong_bolt_identification")

    if wrong_bolt_identification == "true":
        configuration_file = "supervisory_control_manual_config.yaml"
    else:
        configuration_file = "supervisory_control_automatic_config.yaml"

    supervisory_control_configuration = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_app_screwdriving_supervisory_control"),
            "config",
            "supervisory_control_manual_config.yaml",
        ]
    )

    supervisory_control_node = Node(
        package="screwdriving_supervisory_controller",
        executable="screwdriving_supervisory_controller",
        name="screwdriving_supervisory_controller",
        output="screen",
        parameters=[supervisory_control_configuration],
    )

    camera_streaming_node = Node(
        package="endoscopic_camera_streaming_rviz",
        executable="camera_publisher",
        name="camera_publisher",
        output="screen",
    )

    return [supervisory_control_node, camera_streaming_node]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "wrong_bolt_identification",
            default_value="false",
            description="",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
