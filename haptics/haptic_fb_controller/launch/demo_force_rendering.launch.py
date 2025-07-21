from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Launch the simulation of the UR10 robot with the ignition force torque sensor
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("acg_resources_ur10_moveit_config"),
                "/launch/gazebo_ros2_control_demo.launch.py",
            ]
        ),
        launch_arguments={
            "enable_ft_sensing": "true",
            "ft_plot": "false",
            "runtime_config_package": "haptic_fb_controller",
            "controllers_file": "slave_controllers_force_rendering.yaml",
            "spawn_z": "1.03",
            "sensed_world": "my_sensed_world_with_ramp.sdf",
            "description_file": "ur10_with_ft_sensor_and_long_handle.urdf.xacro",
            "initial_joint_controller": "pid_controller",
            "enable_effort_interfaces": "true",
        }.items(),
    )

    # Path to the xacro file of the haptic device
    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("touch_haptic_device"),
            "urdf",
            "touch_haptic_device_and_link.ros2_control.xacro",
        ]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_path,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Load the configuration of the controllers
    robot_controllers_hd_cm = PathJoinSubstitution(
        [
            FindPackageShare("haptic_fb_controller"),
            "config",
            "master_controllers.yaml",
        ]
    )

    # Robot state publisher node to publish the robot description, aka the touch haptic device
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        namespace="hd",
    )

    # Controller manager node
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

    # Admittance controller spawner
    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["admittance_controller", "-c", "/controller_manager"],
    )

    # Haptic feedback controller spawner
    haptic_fb_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["haptic_fb_controller", "-c", "/controller_manager"],
    )

    # Haptic forward force controller spawner
    haptic_forward_force_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["haptic_forward_force_controller", "-c", "/hd/controller_manager"],
        namespace="hd",
    )

    # Publish the reference points fo the PID
    publish_initial_position1 = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/pid_controller/reference",
            "control_msgs/msg/MultiDOFCommand",
            "{dof_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], values: [0.3644, -0.950874, 1.944186, 3.71903, -1.570797, -1.618085]}",
            "-t",
            "2",
        ],
        output="screen",
    )

    publish_initial_position2 = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/pid_controller/reference",
            "control_msgs/msg/MultiDOFCommand",
            "{dof_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], values: [0.3644, -0.9407659195236712, 1.9508985864308315, -2.5715380163097286, -1.570796326589745, 4.50600062264331]}",
            "-t",
            "2",
        ],
        output="screen",
    )

    publish_initial_position3 = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/pid_controller/reference",
            "control_msgs/msg/MultiDOFCommand",
            "{dof_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], values: [1.364407969258157, -0.9407659195236712, 1.9508985864308315, -2.5715380163097286, -1.570796326589745, 4.50600062264331]}",
            "-t",
            "2",
        ],
        output="screen",
    )

    publish2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=publish_initial_position1,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[publish_initial_position2],
                )
            ],
        )
    )

    publish3 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=publish_initial_position2,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[publish_initial_position3],
                )
            ],
        )
    )

    first_controller_spawner_with_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=publish_initial_position3,
            on_exit=[admittance_controller_spawner],
        )
    )

    delay_feedback_controller_spawner_after_force_torque_sensor_broadcaster = (
        TimerAction(
            period=15.0,
            actions=[haptic_fb_controller_spawner],
        )
    )

    layout_path = PathJoinSubstitution(
        [
            FindPackageShare("haptic_fb_controller"),
            "config",
            "plot_raw_forces_and_actuated_forces_config.xml",
        ]
    )

    node_plot = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", layout_path],
    )

    # List of nodes to start
    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        delay_feedback_controller_spawner_after_force_torque_sensor_broadcaster,
        haptic_forward_force_controller_spawner,
        publish_initial_position1,
        publish2,
        publish3,
        first_controller_spawner_with_delay,
        launch_simulation,
        node_plot,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
