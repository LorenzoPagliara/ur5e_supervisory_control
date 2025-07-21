# acg_resources_ur5e_description

This package contains description files for the UR5e robot. In particular:

* the [`config`](config/) folder contains:
    * [`ur5e_kinematics.yaml`](config/ur5e_kinematics.yaml): the calibration file of the UNISA Robotics Lab's UR5e manipulator;
    * [`view_robot.rviz`](config/view_robot.rviz): a custom RViz configuration file, optimized for a convenient visualization of the UR5e robot.
    * Other custom RViz configuration files suitable for [joint-space](./config/joint_space_reference_generator_view_ee_pose.rviz) and [task-space](./config/task_space_reference_generator_view_ee_pose.rviz) [reference generators](../reference_generator/README.md)
* the [`launch`](launch/) folder contains:
    * [`display.launch.py`](launch/display.launch.py): a launch file that starts RViz and displays the UR5e robot;
* the [`urdf`](urdf/) folder contains:
    * [`ur.urdf.xacro`](urdf/ur.urdf.xacro): the general description of a UR-manipulator.
    This file is a modified version of the official `ur_description/urdf/ur.urdf.xacro`, updated to disable the
    inclusion of the default `ros2_control` configuration;
    * [`ur5e.ros2_control_effort.xacro`](urdf/ur5e.ros2_control_effort.xacro): this file extends the official `ur_description/urdf/ur.ros2_control.xacro` by adding the definition of effort command interfaces used in the simulated environment;
    * [`ur5e.urdf.xacro`](urdf/ur5e.urdf.xacro): the description of the UR5e robot.
    The flag `enable_effort_interfaces` can be set to `true` to load an extended UR5e description activating effort command interfaces;
    * [`ur5e_with_end_effector.urdf.xacro`](urdf/ur5e_with_end_effector.urdf.xacro): augments the UR5e description with an end-effector:
        * The argument `enable_ft_sensing` can be set to `true` to load the `ros2_control` configuration for the F/T sensor;
        * The argument `sim_ignition` can be set to `true` to load the `ros2_control` interface and the Ignition Gazebo plugin for the F/T sensor;
        * The argument `ee_config_file` can be set to load a custom end-effector configuration file.
        For more information, please refer to the [`end_effector_builder`](../end_effectors/end_effector_builder/README.md) package.

## Usage

In order to visualize the UR5e robot, after building, use the launch file provided in this package:

```bash
ros2 launch acg_resources_ur5e_description display.launch.py
```

RViz will be launched and the UR5e robot visualized (with its reference frames).
Use the Joint State Publisher's GUI to move single joints.

To specify a robot's prefix name, use the `tf_prefix` launch argument. In this case, [`view_robot.rviz`](config/view_robot.rviz) must be manually updated.
