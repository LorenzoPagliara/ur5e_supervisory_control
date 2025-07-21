from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix", default_value="", description="Prefix for the tf names."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_state_publisher_frequency",
            default_value="1000.0",
            description="The frequency (in Hz) at which the robot_state_publisher publishes.",
        )
    )

    description_package = FindPackageShare("acg_resources_ur10_description")
    description_file = PathJoinSubstitution(
        [description_package, "urdf", "ur10.urdf.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution(
        [description_package, "config", "view_robot.rviz"]
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                description_file,
                " tf_prefix:=",
                LaunchConfiguration("tf_prefix"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {
                "publish_frequency": LaunchConfiguration(
                    "robot_state_publisher_frequency"
                )
            },
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    return LaunchDescription(
        declared_arguments
        + [joint_state_publisher_gui_node, robot_state_publisher_node, rviz_node]
    )
