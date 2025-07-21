# ur10_admittance_controller_demo

This package contains the configuration files and launch files to run the admittance controller with the UR10 robot in Gazebo.

## Demos

In this package, two demos are provided to show the admittance controller in action. The first demo consists of a vertical wall sliding task, while the second one consists of an inclined ramp sliding task.
In both demos, the admittance controller is configured to work in chain with the `cartesian_pose_controller` controller, responsible for computing the IK of the compliant pose, and the `task_space_reference_generator` controller, responsible for providing task space references.

### Vertical wall sliding

In order to start the demo with the simulated UR10 and a vertical wall, run:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py
```

To execute the same demo with the dynamic simulation, run the following command instead:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py enable_effort_interfaces:=true
```

To command the robot to follow a trajectory along the wall, run the following command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory acg_control_msgs/action/FollowTaskSpaceTrajectory "$(cat ./src/unisa_acg_ros2/ur10_admittance_controller_demo/config/wall_sliding_trajectory.yaml)" --feedback
```

### Inclined ramp sliding

In order to start the demo with the simulated UR10 and an inclined ramp, run:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py gazebo_world_file_path:=world/world_with_ft_sensor_and_ramp.sdf spawn_z:=1.0 initial_positions_file:="initial_positions_ramp.yaml" controllers_file:="ur10_simulation_admittance_controller_ramp.yaml"
```

To execute the same demo with the dynamic simulation, run the following command instead:

```bash
ros2 launch ur10_admittance_controller_demo admittance_controller_gazebo_demo.launch.py enable_effort_interfaces:=true gazebo_world_file_path:=world/world_with_ft_sensor_ramp.sdf spawn_z:=1.0 initial_positions_file:="initial_positions_ramp.yaml" controllers_file:="ur10_simulation_admittance_controller_ramp_effort.yaml" initial_joint_controller:="pid_controller"
```

To command the robot to follow a trajectory along the ramp, run the following command:

```bash
ros2 action send_goal /task_space_reference_generator/follow_task_space_trajectory acg_control_msgs/action/FollowTaskSpaceTrajectory "$(cat ./src/unisa_acg_ros2/ur10_admittance_controller_demo/config/ramp_sliding_trajectory.yaml)" --feedback
```
