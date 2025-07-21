from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    OpaqueFunction,
    TimerAction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def set_initial_joint_controller(context, *args, **kwargs):
    dynamic_simulation = LaunchConfiguration("dynamic_simulation").perform(context)
    if dynamic_simulation == "true":
        return [SetLaunchConfiguration("initial_joint_controller", "pid_controller")]
    else:
        return [
            SetLaunchConfiguration("initial_joint_controller", "admittance_controller")
        ]


def set_controllers_file(context, *args, **kwargs):
    dynamic_simulation = LaunchConfiguration("dynamic_simulation").perform(context)
    if dynamic_simulation == "true":
        return [
            SetLaunchConfiguration("controllers_file", "slave_controllers_dynamic.yaml")
        ]
    else:
        return [SetLaunchConfiguration("controllers_file", "slave_controllers.yaml")]


def launch_setup(context, *args, **kwargs):
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    enable_ft_sensing = LaunchConfiguration("enable_ft_sensing")
    description_file = LaunchConfiguration("description_file")
    initial_joint_state = LaunchConfiguration("initial_joint_state")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    dynamic_simulation = LaunchConfiguration("dynamic_simulation")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    controllers_file = LaunchConfiguration("controllers_file")
    first_controller_spawner_delay = LaunchConfiguration(
        "first_controller_spawner_delay"
    )

    # Include the UR10 MoveIt configuration launch file, with runtime configuration options
    launch_admittance_control_demo = IncludeLaunchDescription(
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
            "description_file": description_file,
            "initial_joint_state": initial_joint_state,
            "rviz_config_package": rviz_config_package,
            "rviz_config_file": rviz_config_file,
            "initial_joint_controller": initial_joint_controller,
            "enable_effort_interfaces": dynamic_simulation,
        }.items(),
    )

    # Admittance controller spawner
    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["admittance_controller", "-c", "/controller_manager"],
    )

    # Haptic feedback controller spawner
    haptic_ff_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["haptic_ff_controller", "-c", "/controller_manager"],
    )

    # Publish a command to the /pid_controller/reference topic
    publish_initial_position = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/pid_controller/reference",
            "control_msgs/msg/MultiDOFCommand",
            "{dof_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], values: [0.0, -1.57, 1.57, 3.14, -1.57, 0.0]}",
            "-t",
            "2",  # Duration of the publication
        ],
        output="screen",
    )

    # Conditional logic for controller launch based on whether dynamic simulation is enabled
    if dynamic_simulation.perform(context) == "true":
        # If dynamic simulation is true, launch the admittance controller with a delay
        first_controller_spawner_with_delay = TimerAction(
            period=first_controller_spawner_delay,
            actions=[admittance_controller_spawner],
        )

        # After admittance controller exits, launch the cartesian pose controller
        start_next_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=admittance_controller_spawner,
                on_exit=[haptic_ff_controller_spawner],
            )
        )

        # List of actions for dynamic simulation scenario
        nodes_to_start = [
            publish_initial_position,
            first_controller_spawner_with_delay,
            start_next_controller,
            launch_admittance_control_demo,
        ]

    else:
        # If dynamic simulation is false, launch the cartesian pose controller with a delay
        first_controller_spawner_with_delay = TimerAction(
            period=first_controller_spawner_delay,
            actions=[haptic_ff_controller_spawner],
        )

        # List of actions for non-dynamic simulation scenario
        nodes_to_start = [
            first_controller_spawner_with_delay,
            launch_admittance_control_demo,
        ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # Declare launch arguments for various configurations
    declared_arguments.append(
        DeclareLaunchArgument(
            "first_controller_spawner_delay",
            default_value="20.0",
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
            "enable_ft_sensing",
            default_value="true",
            description="Simulation with FT sensor",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur10_with_ft_sensor_and_long_handle.urdf.xacro",
            description="URDF file with FT sensor",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="haptic_control_rviz_config.rviz",
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
            "dynamic_simulation", default_value="true", description="Dynamic simulation"
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
            default_value="0.0, -1.57, 1.57, 3.14, -1.57, 0.0",
            description="Initial joint state for the robot",
        )
    )

    # Return the launch description with the declared arguments and launch setup
    return LaunchDescription(
        declared_arguments
        + [
            OpaqueFunction(function=set_initial_joint_controller),
            OpaqueFunction(function=set_controllers_file),
            OpaqueFunction(function=launch_setup),
        ]
    )
