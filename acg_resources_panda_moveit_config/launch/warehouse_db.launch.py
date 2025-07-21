from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def setup_launch(context, *args, **kwargs):
    # Get the tf_prefix value from the LaunchConfiguration and context
    xacro_args = {"tf_prefix": LaunchConfiguration("tf_prefix").perform(context)}

    # Build the MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="acg_resources_panda_moveit_config")
        .robot_description(mappings=xacro_args)
        .robot_description_semantic(mappings=xacro_args)
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_action(generate_warehouse_db_launch(moveit_config))

    return ld.entities


def generate_launch_description():
    declared_arguments = []

    # Declare the tf_prefix argument
    tf_prefix_arg = DeclareLaunchArgument(
        "tf_prefix", default_value="panda/", description="Prefix for the tf names."
    )
    declared_arguments.append(tf_prefix_arg)

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=setup_launch)]
    )
