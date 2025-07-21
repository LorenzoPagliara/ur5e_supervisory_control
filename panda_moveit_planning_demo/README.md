# panda_moveit_planning_demo

This package provides a demo for MoveIt!2 planning capabilities by using the `MoveGroupInterface` class.
The source code is quite self-explanatory, so step through its main functions to grasp what the demo does.
At execution, the `acg_resources_panda_moveit_config` package must be installed (see usage instructions below).

## Usage

In a first shell launch the `move_group` node and utility nodes, like RViz, from the `acg_resources_panda_moveit_config` package.
Make sure to select the appropriate RViz configuration:

```bash
ros2 launch acg_resources_panda_moveit_config demo.launch.py rviz_config:=planning_demo.rviz
```

In another shell run this package's planning demo:

```bash
ros2 run panda_moveit_planning_demo panda_moveit_planning_node
```

Use the **RVizVisualToolsGUI** to step through the demo.

## Debug the node

The launch file `launch/demo.launch.py` executes the same demo from a launch file. This is only needed to debug the node through the VSCode ROS extension.
