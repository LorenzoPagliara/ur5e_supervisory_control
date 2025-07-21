# panda_inverse_kinematics_demo

This package implements a node that performs inverse kinematics with the Panda robot in three different modalities:

* Querying the `move_group` node's `compute_ik` service
* Using the `RobotState` class API
* Using the `KinematicsBase` class API

The pose to be inverted is retrieved from `tf2`.

## Dependencies

Together with other ROS2 and MoveIt! standard dependencies declared in `package.xml`, this package requires the `acg_resources_panda_description` and `acg_resources_panda_moveit_config` in order to access the robot description and launch the `move_group` and `robot_state_publisher` nodes.

## Usage

First, make sure to launch the `move_group` node through

```bash
ros2 launch acg_resources_panda_moveit_config demo.launch.py
```

Then, launch the demo through

```bash
ros2 launch panda_inverse_kinematics_demo demo.launch.py
```

The latter `demo.launch.py` launch file makes sure that the `robot_description_kinematics` is loaded in the node configuration so that a robot model with a kinematics solver can be instantiated.

## Expected behavior

If a numerical kinematics solver is configured in `acg_resources_panda_moveit_config`, some inverse kinematics computations are likely not to find a solution in the allotted time.
In this case, error messages are printed.

In order to compute IK with the `KinematicsBase` API, the seed state from previous IK (with `RobotState` API) is used.
In this case, the solver immediately returns the seed state, as it is a valid IK solution.
