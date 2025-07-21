from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    OpaqueFunction,
    TimerAction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    enable_ft_sensing = LaunchConfiguration("enable_ft_sensing")
    # description_file = LaunchConfiguration("description_file")
    initial_joint_state = LaunchConfiguration("initial_joint_state")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    dynamic_simulation = LaunchConfiguration("dynamic_simulation")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    controllers_file = LaunchConfiguration("controllers_file")
    first_controller_spawner_delay = LaunchConfiguration(
        "first_controller_spawner_delay"
    )
    cpc = LaunchConfiguration("cpc")

    if dynamic_simulation.perform(context) == "true":
        controllers_file = "slave_controllers_dynamic.yaml"
        initial_joint_controller = "pid_controller"
    else:
        controllers_file = "slave_controllers.yaml"
        initial_joint_controller = "cartesian_pose_controller"

    # Include the UR10 MoveIt configuration launch file, with runtime configuration options
    launch_ur10_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("acg_resources_ur10_moveit_config"),
                "/launch/gazebo_ros2_control_demo.launch.py",
            ]
        ),
        launch_arguments={
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "enable_ft_sensing": enable_ft_sensing,
            # "description_file": description_file,
            "initial_joint_state": initial_joint_state,
            "rviz_config_package": rviz_config_package,
            "rviz_config_file": rviz_config_file,
            "initial_joint_controller": initial_joint_controller,
            "enable_effort_interfaces": dynamic_simulation,
            "ft_plot": "false",
        }.items(),
    )

    # Path to the xacro file of the haptic device
    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("touch_haptic_device"),
            "urdf",
            "touch_haptic_device.xacro",
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

    # Load the configuration of the controllers for the master side
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

    # Haptic twist broadcaster controller spawner
    haptic_twist_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="hd",
        arguments=["haptic_twist_broadcaster", "-c", "/hd/controller_manager"],
    )

    # Admittance controller spawner
    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["admittance_controller", "-c", "/controller_manager"],
    )

    # Cartesian pose controller spawner
    cartesian_pose_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_pose_controller", "-c", "/controller_manager"],
    )

    # Haptic feedback controller spawner
    haptic_ff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["haptic_ff_controller", "-c", "/controller_manager"],
    )

    # Path to the layout file for the PlotJuggler configuration
    layout_path = PathJoinSubstitution(
        [
            FindPackageShare("haptic_ff_controller"),
            "config",
            "robot_twist_plot.xml",
        ]
    )

    # Launch PlotJuggler with the layout file
    node_plot = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", layout_path],
    )

    # Publish a command to the /pid_controller/reference topic
    publish_initial_position = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/pid_controller/reference",
            "control_msgs/msg/MultiDOFCommand",
            "{dof_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], values: [0.0, -1.67, 1.87, 2.94, -1.57, 0.0]}",
            "-t",
            "2",  # Duration of the publication
        ],
        output="screen",
    )

    if cpc.perform(context) == "true":
        # Conditional logic for controller launch based on whether dynamic simulation is enabled
        if dynamic_simulation.perform(context) == "true":
            # If dynamic simulation is true, launch the cartesian pose controller with a delay
            first_controller_spawner_with_delay = TimerAction(
                period=first_controller_spawner_delay,
                actions=[cartesian_pose_controller_spawner],
            )

            # After cartesian pose controller exits, launch the haptic feedforward controller
            start_haptic_ff_after_first_controller = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=cartesian_pose_controller_spawner,
                    on_exit=[haptic_ff_controller_spawner],
                )
            )

            # List of actions for dynamic simulation scenario
            nodes_to_start = [
                control_node,
                robot_state_publisher_node,
                haptic_pose_broadcaster_controller_spawner,
                haptic_twist_broadcaster_controller_spawner,
                publish_initial_position,
                first_controller_spawner_with_delay,
                start_haptic_ff_after_first_controller,
                launch_ur10_demo,
                node_plot,
            ]

        else:
            # If dynamic simulation is false, launch the haptic feedforward controller with a delay
            first_controller_spawner_with_delay = TimerAction(
                period=first_controller_spawner_delay,
                actions=[haptic_ff_controller_spawner],
            )

            # List of actions for non-dynamic simulation scenario
            nodes_to_start = [
                control_node,
                robot_state_publisher_node,
                haptic_pose_broadcaster_controller_spawner,
                haptic_twist_broadcaster_controller_spawner,
                first_controller_spawner_with_delay,
                launch_ur10_demo,
                node_plot,
            ]
    else:
        # Conditional logic for controller launch based on whether dynamic simulation is enabled
        if dynamic_simulation.perform(context) == "true":
            # If dynamic simulation is true, launch the cartesian pose controller with a delay
            first_controller_spawner_with_delay = TimerAction(
                period=first_controller_spawner_delay,
                actions=[cartesian_pose_controller_spawner],
            )

            start_admittance_controller_after_first_controller = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=cartesian_pose_controller_spawner,
                    on_exit=[admittance_controller_spawner],
                )
            )

            # After admittance controller exits, launch the haptic feedforward controller
            start_haptic_ff_after_first_controller = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=admittance_controller_spawner,
                    on_exit=[haptic_ff_controller_spawner],
                )
            )

            # List of actions for dynamic simulation scenario
            nodes_to_start = [
                control_node,
                robot_state_publisher_node,
                haptic_pose_broadcaster_controller_spawner,
                haptic_twist_broadcaster_controller_spawner,
                publish_initial_position,
                first_controller_spawner_with_delay,
                start_admittance_controller_after_first_controller,
                start_haptic_ff_after_first_controller,
                launch_ur10_demo,
                node_plot,
            ]

        else:
            # If dynamic simulation is false, launch the haptic feedforward controller with a delay
            first_controller_spawner_with_delay = TimerAction(
                period=first_controller_spawner_delay,
                actions=[admittance_controller_spawner],
            )

            # After admittance controller exits, launch the haptic feedforward controller
            start_haptic_ff_after_first_controller = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=admittance_controller_spawner,
                    on_exit=[haptic_ff_controller_spawner],
                )
            )

            # List of actions for non-dynamic simulation scenario
            nodes_to_start = [
                control_node,
                robot_state_publisher_node,
                haptic_pose_broadcaster_controller_spawner,
                haptic_twist_broadcaster_controller_spawner,
                first_controller_spawner_with_delay,
                start_haptic_ff_after_first_controller,
                launch_ur10_demo,
                node_plot,
            ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # Declare launch arguments for various configurations
    declared_arguments.append(
        DeclareLaunchArgument(
            "first_controller_spawner_delay",
            default_value="15.0",
            description="Delay before the first controller spawner",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="haptic_ff_controller",
            description="Package with runtime configuration",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="slave_controllers_dynamic.yaml",
            description="YAML file with controller configuration",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="config/haptic_control_rviz_config.rviz",
            description="RViz configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_package",
            default_value="haptic_ff_controller",
            description="RViz configuration package",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "dynamic_simulation",
            default_value="true",
            description="Enable dynamic simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="admittance_controller",
            description="Initial joint controller for the robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_state",
            default_value="0.0, -1.67, 1.87, 2.94, -1.57, 0.0",
            description="Initial joint state for the robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_ft_sensing",
            default_value="true",
            description="Simulation with FT sensor",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "cpc",
            default_value="false",
            description="Enable only Cartesian Pose Controller",
        )
    )

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_file",
    #         default_value="ur10_with_robotiq_fts150_and_chalk_holder.urdf.xacro",
    #         description="URDF file with FT sensor",
    #     )
    # )

    # Return the launch description with the declared arguments and launch setup
    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
