# ur5e_app_haptic_teleop

This package contains the configuration files and launch files to teleoperate the UR5e robot in Gazebo and in real world.

## Demos

In this package, four demos are provided to show teleoperate the UR5e both in simulation and real world, using both force-based and error-based force feedback.

### Simulation with a vertical wall

In order to start the teleoperation demo with the simulated UR5e, a vertical wall, and force-based force feedback, run:

```bash
ros2 launch ur5e_app_haptic_teleop demo_gazebo_force_based_teleop.launch.py
```

To execute the same demo with the error-based force feedback, run the following command instead:

```bash
ros2 launch ur5e_app_haptic_teleop demo_gazebo_error_based_teleop.launch.py
```

### UR10 real robot demo

In order to start the teleoperation demo with the UR10 real robot and force-based force feedback, run:

```bash
ros2 launch ur5e_app_haptic_teleop demo_force_based_teleop.launch.py
```

After you click "play" on the teach pendant, open another shell and run the following command to load the controllers:

```bash
ros2 launch ur5e_app_haptic_teleop force_based_teleop_setup.launch.py
```

To execute the same demo with the error-based force feedback, run the following command instead:

```bash
ros2 launch ur5e_app_haptic_teleop demo_error_based_teleop.launch.py
```

After you click "play" on the teach pendant, open another shell and run the following command to load the controllers:

```bash
ros2 launch ur5e_app_haptic_teleop error_based_teleop_setup.launch.py
```
