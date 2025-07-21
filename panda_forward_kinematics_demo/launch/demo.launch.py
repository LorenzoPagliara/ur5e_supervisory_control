from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "panda", package_name="acg_resources_panda_moveit_config"
    ).to_moveit_configs()

    node_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
    ]

    return LaunchDescription(
        [
            Node(
                package="panda_forward_kinematics_demo",
                executable="forward_kinematics_demo_node",
                parameters=node_parameters,
            )
        ]
    )
