# panda_forward_kinematics_demo

This package implements a node that performs forward kinematics with the Panda robot in three different modalities:

* Using transforms broadcasted onto the `tf2` topic
* Querying the `move_group` node's `compute_fk` service; in this case the input joint positions are read from the `joint_states` topic
* Using the `RobotState` class API; as before, the input joint positions are read from the `joint_states` topic

## Dependencies

Together with other ROS2 and MoveIt! standard dependencies declared in `package.xml`, this package requires the `acg_resources_panda_description` and `acg_resources_panda_moveit_config` in order to access the robot description and launch the `move_group` and `robot_state_publisher` nodes.

## Usage

First, make sure to launch the `move_group` node through

```bash
ros2 launch acg_resources_panda_moveit_config demo.launch.py
```

Then, launch the demo through either

```bash
ros2 run panda_forward_kinematics_demo forward_kinematics_demo_node
```

or

```bash
ros2 launch panda_forward_kinematics_demo demo.launch.py
```

In the former case, the parameters needed to load the robot model, namely `robot_description`, `robot_description_semantic` and `robot_description_kinematics` are retrieved from the nodes on the ROS2 network that are publishing them.
In the latter case, the same parameters are loaded through the launch file.

Once the demo is up and running, try planning trajectories with the `MotionPlanning` plugin and executing them.
As the robot joint state changes, you should see forward kinematics transforms get updated by the `forward_kinematics_demo_node`.
