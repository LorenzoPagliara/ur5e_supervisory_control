# Haptic Feedforward Controller
The Haptic Feedforward Controller is designed to integrate haptic devices with robotic systems, allowing for precise control. This controller processes input data from haptic devices and computes the corresponding control commands for the robot. It uses pose filter chains to smooth and process the input data, ensuring accurate and stable control.

## Calibration and alignment
Calibration of the haptic device stylus is performed if the calibration file is missing and a new one needs to be generated. You can delete the calibration file (see [Touch Haptic Device Readme](https://bitbucket.org/eferre89/unisa_acg_ros2/src/3cd5a6d3c933d1886ef5a1140b958b7c1ee38a85/touch_haptic_device/Readme.md?at=feature%2Ftouch_driver_installation) for more info) to force the calibration procedure. Calibration requires the stylus to be placed in the inkwell.

Alignment is the procedure by which the robot slightly moves to match the haptic device stylus orientation. The alignment procedure only accounts for small misalignments. Therefore, in order for the alignment procedure to be successful, it is necessary that the stylus' and the end-effector's orientations are already close enough. If alignment is performed together with calibration, because the stylus must be in the inkwell, the alignment procedure can only be successful if the robot end-effector's orientation already matches the stylus-in-the-inkwell orientation.

To avoid using old calibration files, calibration is forced at any startup (if present, the old one will be overwritten). This requires to start the Haptic Feedforward Controller with the stylus in the inkwell. However, the stylus can be moved after calibration and before the alignment procedure. In this case, it suffices that the end-effector orientation and haptic device stylus orientation are close to each other to complete the alignment and engage the two devices.

**WARNING**: when alignment is performed, because the robot moves without haptic control, make sure to clear the robot workspace and hold the stylus steady!

## Configuration parameters
The Haptic Feedforward Controller module present different configuration parameters which are described below.

- **joints**: Specifies the joint names for the haptic feedforward controller.
- **state_interfaces**: Specifies which state interfaces the controller will claim.
- **command_interface**: Specifies which command interfaces the controller will claim.
- **haptic_pose_subscriber**: Specifies  the haptic pose subscriber for simulated scenarios, it is used because in simulated sceanario the haptic device and the haptic feedforward controller are on two different controller manager. For this reason the Haptic Feedforward Controller in simulated scenario receive the haptic pose from topic.
  - **topic_name**: Specifies the topic name for the haptic pose subscriber.
  - **queue_size**: Specifies the queue size for the haptic pose subscriber.
- **haptic_pose_rviz_publisher**: Specifies the haptic pose rviz publisher for publishing the commanded task space reference in rviz.
  - **topic_name**: Specifies the topic name for the haptic robot poses publisher.
  - **queue_size**: Specifies the queue size for the haptic robot poses publisher.
- **task_space_reference_publisher**: Specifies the task space reference publisher for publishing the commanded task space reference.
  - **topic_name**: Specifies the topic name for the task space reference publisher.
  - **queue_size**: Specifies the queue size for the task space reference publisher.
- **kinematics**: Specifies the kinematics parameters.
  - **plugin_name**: Specifies the name of the kinematics plugin to load.
  - **plugin_package**: Specifies the package name that contains the kinematics plugin.
  - **base**: Specifies the base link of the robot description used by the kinematics plugin.
  - **tip**: Specifies the end effector link of the robot description used by the kinematics plugin.
  - **alpha**: Specifies the damping coefficient for the Jacobian pseudo inverse.
- **engagement_orientation_threshold**: Specifies the maximum allowed difference in Euler angles between the current robot end-effector orientation and the reference one, before trying to compute the IK and engage the robot.
- **RPY_robot_base_to_haptic_base**: Specifies the roll, pitch, and yaw angles for the transform from the robot base to the haptic base.
- **RPY_haptic_ee_to_tool_tip**: Specifies the roll, pitch, and yaw angles for the transform from the haptic end-effector to the tool tip.
- **robot_ee_to_TCP**: Specifies the TCP transform as a 6-element array [x, y, z, roll, pitch, yaw]. It is used for controlling the tip of any tool. This parameter expresses the transform from the robot end-effector to the tool tip.
- **scaling_factor**: Specifies the scaling factor for position control for each axis from haptic device to robot.
- **robot_base_frame**: Specifies the robot base frame.
- **robot_ee_frame**: Specifies the robot end-effector frame.
- **simulation**: Specifies if the simulation mode is enabled.
- **haptic_device_interfaces**: Specifies the haptic device state interface.
- **filter_chain**: Specifies the filter chain parameters for haptic pose filtering.
  - **filter1**:
    - **name**: Specifies the name of the filter.
    - **type**: Specifies the type of the filter.
    - **params**:
      - **number_of_observations**: Specifies the number of observations for the filter.

## Configuration file
Several example configuration files are provided to help you set up the Haptic Feedforward Controller:

- [**slave_controller_dynamic.yaml**](config/slave_controllers_dynamic.yaml): Contains a configuration for the controller manager on the slave side for dynamic simulation.
- [**slave_controller.yaml**](config/slave_controllers.yaml): Contains a configuration for the controller manager on the slave side for non-dynamic simulation.
- [**master_controller.yaml**](config/master_controllers.yaml): Contains a configuration for the controller manager of the master side. It can be used for a remote teleoperation.
- [**local_teleoperation_controllers.yaml**](config/local_teleoperation_controllers.yaml): Contains a configuration for a local teleoperation.

## How to Run Demos
You need to express the relative orientations between the haptic device's base frames and the robot's base frames. You can do this through the configuration file named into the **Configuration file** section.

Once the frames are set up, you can run the demos. To do this, first build the package of the real Touch Haptic Device inside your workspace:

```sh
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Simulation
To run the simulation demo, you have two options: non-dynamic simulation and dynamic simulation. It can be chosen by using the `dynamic_simulation` parameter. The default value is `true`.

To run the dynamic simulation demo, use the following command:

```sh
ros2 launch haptic_ff_controller demo_haptic_ff_controller.launch.py
```

To run the non-dynamic simulation, use the following command:

```sh
ros2 launch haptic_ff_controller demo_haptic_ff_controller.launch.py dynamic_simulation:=false
```

### Real UR10
To run the demo with the real UR10 robot:

#### Real UR10 with Robotiq fts150 and Touch Haptic Device
To run the demo with the UR10 robot equipped with the real Robotiq FTS150 and the touch haptic device, use the following command:
```sh
ros2 launch acg_resources_ur10_moveit_config ur10_control.launch.py robot_ip:=192.168.1.6 runtime_config_package:=haptic_ff_controller controllers_file:=local_teleoperation_controllers.yaml description_package:=acg_resources_ur10_description description_file:=ur10_with_real_robotiq_fts150_and_touch_haptic_device.urdf.xacro
```

#### Real UR10 with Robotiq fts150, Chalk Holder and Touch Haptic Device
To run the demo with the UR10 robot equipped with the real Robotiq FTS150, the touch haptic device, and the chalk holder, use the following command:
```sh
ros2 launch acg_resources_ur10_moveit_config ur10_control.launch.py robot_ip:=192.168.1.6 runtime_config_package:=haptic_ff_controller controllers_file:=local_teleoperation_controllers.yaml description_package:=acg_resources_ur10_description description_file:=ur10_with_real_robotiq_fts150_chalk_holder_and_touch_haptic_device.urdf.xacro
```

### Remote Teleoperation with UR10 and Touch Haptic Device
In this scenario, there is a computer to which the real robot is connected (**slave_pc**) and a computer to which the haptic device is connected (**master_pc**).

#### Set up the teleoperation between the slave_pc and the master_pc
Check into the `~/.bashrc` of the **slave_pc** if the following lines are present, otherwise add them:
```sh
  export ROS_DISCOVERY_SERVER=127.0.0.1:11811
  export ROS_LOCALHOST_ONLY=0
```
Check into the `~/.bashrc` of the **master_pc** check if the following lines are present, otherwise add them:
```sh
  export ROS_DISCOVERY_SERVER=robot_pc_public_ip:11811
  export ROS_LOCALHOST_ONLY=0
```

For example, for the remote teleoperation with PCROB-02 as **slave_pc** from the robotics lab, the `~/.bashrc` of the **master_pc** will be as follows:

```sh
  export ROS_DISCOVERY_SERVER=192.205.164.163:11811
  export ROS_LOCALHOST_ONLY=0
```
#### Test the set up configuration
From the **slave_pc** run the following command:

- First shell:
 ```sh
  fastdds discovery --server-id 0
```
- Second shell:
```sh
  ros2 run demo_nodes_cpp listener
```

From the **master_pc** run the following command:
```sh
  ros2 run demo_nodes_cpp talker
```
#### Start remote teleoperation
From the **master_pc** run the following command:

- First shell:
```sh
ros2 launch haptic_ff_controller master_side.launch.py
```
- Second shell:
```sh
rviz2 -d src/unisa_acg_ros2/haptic_ff_controller/config/distance_haptic_device_config.rviz
```

From the **slave_pc** run the following command:
- First shell:
```sh
  fastdds discovery --server-id 0
```
- On the second shell run the following command if you want to teleoperate the robot in Gazebo:
```sh
ros2 launch haptic_ff_controller slave_side.launch.py
```

- Instead, run the following command if you want to teleoperate the real robot:
```sh
ros2 launch acg_resources_ur10_moveit_config ur10_control.launch.py robot_ip:=192.168.1.6 runtime_config_package:=haptic_ff_controller controllers_file:=local_teleoperation_controllers.yaml description_package:=acg_resources_ur10_description description_file:=ur10_with_robotiq_fts150_chalk_holder_and_distance_touch_haptic_device.urdf.xacro
```

- After the start of the communication, run on a third shell:
```sh
chmod +x src/unisa_acg_ros2/haptic_ff_controller/config/controller_teleoperation_setup.sh
./src/unisa_acg_ros2/haptic_ff_controller/config/controller_teleoperation_setup.sh
```
