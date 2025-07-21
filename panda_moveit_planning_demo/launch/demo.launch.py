from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="panda_moveit_planning_demo",
                executable="panda_moveit_planning_node",
            )
        ]
    )
