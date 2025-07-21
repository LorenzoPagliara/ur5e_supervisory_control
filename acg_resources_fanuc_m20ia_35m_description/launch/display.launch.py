from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_path = FindPackageShare("acg_resources_fanuc_m20ia_35m_description")
    default_model_path = PathJoinSubstitution(
        [package_path, "urdf", "fanuc_m20ia_35m.xacro"]
    )
    default_rviz_config_path = PathJoinSubstitution(
        [package_path, "config", "urdf.rviz"]
    )

    tf_prefix_arg = DeclareLaunchArgument(
        name="tf_prefix",
        default_value="fanuc_m20ia_35m/",
        description="Prefix for all TF frames",
    )
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Flag to enable joint_state_publisher_gui",
    )
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot urdf file",
    )
    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=default_rviz_config_path,
        description="Absolute path to rviz config file",
    )
    robot_state_publisher_frequency_arg = DeclareLaunchArgument(
        name="robot_state_publisher_frequency",
        default_value="1000.0",
        description="The frequency (in Hz) at which the robot_state_publisher publishes.",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
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

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            tf_prefix_arg,
            gui_arg,
            model_arg,
            rviz_arg,
            robot_state_publisher_frequency_arg,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
