import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Value argument from launch configuration
    gui = LaunchConfiguration("gui")
    controller = LaunchConfiguration("controller")

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller",
            default_value="panda_forward_position_controller",
            description="Load the forward position controller by default.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix", default_value="panda/", description="Prefix for the tf names."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_state_publisher_frequency",
            default_value="1000.0",
            description="The frequency (in Hz) at which the robot_state_publisher publishes.",
        )
    )

    # Get URDF via xacro
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_panda_moveit_config"),
            "config",
            "panda.urdf.xacro",
        ]
    )

    # Load robot description without mock hardware
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " ",
            "use_mock_hardware:=false",
            " ",
            "tf_prefix:=",
            LaunchConfiguration("tf_prefix"),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_frequency = {
        "publish_frequency": LaunchConfiguration("robot_state_publisher_frequency")
    }

    # Load RViz configuration without planning plugins
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_panda_moveit_config"),
            "config",
            "ros2_control_demo.rviz",
        ]
    )

    # Prepare Gazebo resources and launch path:
    # this is needed by Gazebo to find the meshes referenced by the URDF
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (
        get_package_share_directory("acg_resources_panda_description") + "/.."
    )

    gazebo_launch_file = PathJoinSubstitution(
        [FindPackageShare("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py"]
    )

    # Include Gazebo launch file to launch its environment
    launches = []

    launches.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments=[("gz_args", [" -r -v 4 empty.sdf"])],
        )
    )

    # Define all nodes that have to be launched

    # Spawn the model in Gazebo. This also loads the plugin and starts the control node.
    # At the time of writing this script, the 'create' command does not allow spawning
    # the robot at a specific joint configuration
    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "panda",
            "-allow_renaming",
            "true",
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, robot_state_publisher_frequency],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    motion_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller, "--controller-manager", "/controller_manager"],
    )

    # Delay `joint_state_broadcaster` start after spawning the model in Gazebo
    delay_controllers_after_ignition_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, motion_controller_spawner],
        )
    )

    # Delay RViz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        ignition_spawn_entity,
        robot_state_pub_node,
        delay_controllers_after_ignition_spawn,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + launches + nodes)
