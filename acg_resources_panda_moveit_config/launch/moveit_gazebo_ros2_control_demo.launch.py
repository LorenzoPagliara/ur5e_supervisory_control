import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Value argument from launch configuration
    gui = LaunchConfiguration("gui")

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

    # Prepare Gazebo resources and launch path
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (
        get_package_share_directory("acg_resources_panda_description") + "/.."
    )

    gazebo_launch_file = PathJoinSubstitution(
        [FindPackageShare("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py"]
    )

    # Prepare other launch paths
    move_group_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_panda_moveit_config"),
            "launch",
            "move_group.launch.py",
        ]
    )

    rviz_launch_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_panda_moveit_config"),
            "launch",
            "moveit_rviz.launch.py",
        ]
    )

    # Include launch files in this one
    launches = []

    launches.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments=[("gz_args", ["-r -v 4 empty.sdf"])],
        )
    )

    move_group_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_file),
        launch_arguments=[("moveit_manage_controllers", "false")],
    )

    rviz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file), condition=IfCondition(gui)
    )

    # Define Node actions

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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # This is spawned only temporarily to recover from the fall
    # under the effect of gravity. Therefore we define the spawner...
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # ... and the unspawner
    forward_position_controller_unspawner = Node(
        package="controller_manager",
        executable="unspawner",
        arguments=[
            "panda_forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # This is the controller that we shall use when
    # the MoveIt! planning pipeline is active, like in this case
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_arm_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
    )

    # This is the temporary reference generator communicating with the
    # forward position controller to recover from the fall
    topic_echo_process = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "-t 3",
            "/panda_forward_position_controller/commands",
            "std_msgs/msg/Float64MultiArray",
            "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.785, 0.0]}",
        ],
        name="bring_panda_to_home",
        output="both",
    )

    # Define GroupAction actions (needed to enforce use_sim_time)
    group_actions = []

    group_actions.append(
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=True),
                move_group_launch_description,
                robot_state_pub_node,
            ]
        )
    )

    # Load controllers after spawning the model in Gazebo
    delay_controllers_after_ignition_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                forward_position_controller_spawner,
            ],
        )
    )

    # Send command to recover from the fall
    # after the forward position controller has been activated
    delay_topic_echo_after_forward_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=topic_echo_process,
        )
    )

    # Unspawn forward position controller after placing the robot at the desired configuration
    delay_unspawner_after_topic_echo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=topic_echo_process,
            on_exit=[forward_position_controller_unspawner],
        )
    )

    # After unloading the forward position controller, the
    # trajectory controller can be activated
    delay_trajectory_controller_spawner_after_unspawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_position_controller_unspawner,
            on_exit=[trajectory_controller_spawner, hand_controller_spawner],
        )
    )

    # Delay RViz start after spawning joint trajectory controller
    delay_rviz_after_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=trajectory_controller_spawner,
            on_exit=[rviz_launch_description],
        )
    )

    nodes = [
        ignition_spawn_entity,
        delay_controllers_after_ignition_spawn,
        delay_topic_echo_after_forward_position_controller_spawner,
        delay_unspawner_after_topic_echo,
        delay_trajectory_controller_spawner_after_unspawner,
        delay_rviz_after_trajectory_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + launches + group_actions + nodes)
