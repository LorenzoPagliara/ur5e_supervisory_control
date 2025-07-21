from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def launch_setup(context, *args, **kwargs):

    supervisory_control_configuration = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_app_screwdriving_supervisory_control"),
            "config",
            "supervisory_control_gazebo_config.yaml",
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
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
