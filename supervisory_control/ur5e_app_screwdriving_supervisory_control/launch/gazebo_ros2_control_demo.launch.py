from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch.event_handlers import OnProcessExit


def launch_setup(context, *args, **kwargs):
    moveit_gazebo_ros2_control_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("acg_resources_ur5e_moveit_config"),
                    "launch",
                    "moveit_gazebo_ros2_control_demo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "safety_limits": "true",
            "initial_positions_package": "ur5e_app_screwdriving_supervisory_control",
            "initial_positions_file": "initial_robot_positions.yaml",
            "initial_joint_controller": "cartesian_pose_controller",
            "runtime_config_package": "ur5e_app_screwdriving_supervisory_control",
            "controllers_file": "ur5e_simulation_ft_controllers.yaml",
            "description_package": "acg_resources_ur5e_description",
            "description_file": "ur5e_with_end_effector.urdf.xacro",
            "tf_prefix": "",
            "launch_rviz": "false",
            "rviz_config_package": "ur5e_app_screwdriving_supervisory_control",
            "rviz_config_file": "config/ur5e_supervisory_control.rviz",
            "enable_ft_sensing": "true",
            "enable_effort_interfaces": "false",
            "ft_plot": "false",
            "ee_config_package": "ur5e_app_screwdriving_supervisory_control",
            "ee_config_file": "gazebo_ft_sensor_and_screwdriver_ee_config.yaml",
            "spawn_z": "0.0",
        }.items(),
    )

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("touch_haptic_device"),
            "urdf",
            "touch_haptic_device.xacro",
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )

    haptic_device_description = {"robot_description": robot_description_content}

    haptic_device_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_app_screwdriving_supervisory_control"),
            "config",
            "touch_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[haptic_device_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/hd/robot_description"),
        ],
        namespace="hd",
    )

    haptic_device_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[haptic_device_description],
        namespace="hd",
    )

    haptic_pose_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="hd",
        arguments=["haptic_pose_broadcaster", "-c", "/hd/controller_manager"],
    )

    haptic_twist_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="hd",
        arguments=["haptic_twist_broadcaster", "-c", "/hd/controller_manager"],
    )

    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "admittance_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    reference_generator_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_space_reference_generator",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delay_admittance_controller_spawner = TimerAction(
        period=5.0, actions=[admittance_controller_spawner]
    )

    delay_reference_after_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=admittance_controller_spawner,
            on_exit=[reference_generator_spawner],
        )
    )

    layout_path_plotjuggler = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_app_screwdriving_supervisory_control"),
            "config",
            "robot_state_plots.xml",
        ]
    )

    plotjuggler_starter = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=["--layout", layout_path_plotjuggler],
    )

    return [
        moveit_gazebo_ros2_control_demo_launch,
        control_node,
        delay_admittance_controller_spawner,
        delay_reference_after_controller_spawner,
        haptic_device_state_publisher_node,
        haptic_pose_broadcaster_controller_spawner,
        haptic_twist_broadcaster_controller_spawner,
        plotjuggler_starter,
    ]


def generate_launch_description():
    declared_arguments = []
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
