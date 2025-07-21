# Haptic Feedback Controller
Haptic feedback controller is a controller designed to read forces from a force/torque sensor (either reading from topic or from the sensor's state interfaces), filters them by applying a gravity compensation filter and then a moving average filter, extracts the linear forces, performs the conversion from the tern in which they are expressed to the base tern of the haptic device, and finally sends them to the control interfaces, or publishes them on topic in the case of a simulation.

## Configuration parameters
- **joints**: Specifies the joint names for the haptic feedback controller.
- **state_interfaces**: Specifies which state interfaces the controller will claim.
- **command_interface**: Specifies which command interfaces the controller will claim.
- **RPY_robot_base_to_haptic_base**: Specifies the roll, pitch, and yaw angles for the transform from the robot base to the haptic base.
- **robot_base_frame**: Specifies the robot base frame.
- **robot_ee_frame**: Specifies the robot end-effector frame.
- **simulation**: Specifies if the simulation mode is enabled.
- **topic_name_raw_forces_torques**: Specifies the name of the topic from which to read raw forces and torques.
- **read_force_torque_from_topic**: Specifies a flag enable the reading of forces and torques from a topic.
- **number_of_channels**: Specifies the number of channels of the filter chain.
- **haptic_device_interfaces**: Specifies the haptic device state interface.
- **haptic_force_controller**: Specifies the name of the haptic force controller.
- **kinematics**: Specifies the kinematics parameters.
  - **plugin_name**: Specifies the name of the kinematics plugin to load.
  - **plugin_package**: Specifies the package name that contains the kinematics plugin.
  - **base**: Specifies the base link of the robot description used by the kinematics plugin.
  - **tip**: Specifies the end effector link of the robot description used by the kinematics plugin.
  - **alpha**: Specifies the damping coefficient for the Jacobian pseudo inverse.
- **filter_chain**: Specifies the filter chain parameters for haptic pose filtering.
  - **filter1**:
    - **name**: Specifies the name of the first filter. (gravity_and_sensor_bias_filter)
    - **type**: Specifies the type of the first filter. (gravity_and_sensor_bias_filter/GravityAndSensorBiasFilterDouble)
    - **params**:
      - **robot_description**: Specifies the URDF robot description.
      - **ft_sensor_link_name**: Specifies the name of the link where the force-torque sensor is mounted.
      - **number_of_observations**: Specifies the number of observations used by the MultiChannelMovingAverageFilter.
      - **tolerance**: Specifies the tolerance for the force-torque sensor.
      - **number_of_stable_samples_for_bias**: Specifies the number of stable samples required to compute a reliable sensor bias.
  - **filter2**:
    - **name**: Specifies the name of the second filter. (moving_average_filter)
    - **type**: Specifies the type of the second filter. (moving_average_filter/MultiChannelMovingAverageFilterDouble)
    - **params**:
      - **number_of_observations**: Specifies the window size parameter for the second filter.

### Example of configuration file
The example configuration files are as follows:
- [slave_controllers.yaml](config/slave_controllers.yaml): Contains a configuration for the controller manager on the slave side.
- [master_controllers.yaml](config/master_controllers.yaml): Contains a configuration for the controller manager on the master side.

## How to Run Demos
First you need to follow the [guide](../touch_haptic_device/README.md) and verify that the haptic device is installed correctly and is working properly.

Then, you need to express the relative orientations between the haptic device's base frames and the robot's base frames. You can do this through the configuration file named into the [Example of configuration file](#example-of-configuration-file) section, by going to modify the **RPY_robot_base_to_haptic_base** parameter.

Once the frames are set up, you can run the demos. To do this, first build the packages of the workspace:

```sh
cd ~/ros2_ws
colcon build --packages-up-to haptic_fb_controller; source install/setup.bash
```

The following command must be used to start the demo:

```sh
ros2 launch haptic_fb_controller demo_haptic_fb_controller.launch.py
```

Once the admittance controller finished the configuration, and it's ready to use, run this command to start a trajectory

```sh
ros2 launch acg_follow_task_space_trajectory_action_client follow_task_space_trajectory_action_client.launch.py input_trajectory_filename:=ur10_gazebo_dynamic_180s_0N_const action_name:=admittance_controller fraction_feedback_messages_to_save:=25
```

Through this launch the demo is launched with the UR10 mounting the robotiq_fts150 with long_handle. The world loaded is [my_sensed_world_with_ramp.sdf](../acg_resources_force_torque_sensor_description/world/my_sensed_world_with_ramp.sdf) in which there is an inclined plane where the robot must perform a trajectory in contact with the plane. Through Plotjuggler it is also possible to see in real time the forces read by the force sensor and the forces implemented on the haptic device.
