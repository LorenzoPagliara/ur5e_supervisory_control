from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit


def launch_setup(context, *args, **kwargs):
    joint_trajectory_controller_unspawner = Node(
        package="controller_manager",
        executable="unspawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    cartesian_pose_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_pose_controller",
            "--controller-manager",
            "/controller_manager",
        ],
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

    delay_cartesian_after_joint_trajectory_controller_unspawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_unspawner,
            on_exit=[cartesian_pose_controller_spawner],
        )
    )

    delay_admittance_after_cartesian_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cartesian_pose_controller_spawner,
            on_exit=[admittance_controller_spawner],
        )
    )

    delay_reference_after_admittance_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=admittance_controller_spawner,
            on_exit=[reference_generator_spawner],
        )
    )

    return [
        joint_trajectory_controller_unspawner,
        delay_cartesian_after_joint_trajectory_controller_unspawner,
        delay_admittance_after_cartesian_controller_spawner,
        delay_reference_after_admittance_controller_spawner,
    ]


def generate_launch_description():
    declared_arguments = []
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
