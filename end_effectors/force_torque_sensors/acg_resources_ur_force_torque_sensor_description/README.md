# acg_resources_generic_force_torque_sensor_description

This package contains description files for a integrated Universal Robots force-torque sensor.

## Configuration Parameters

The following configuration parameters can be passed via the `config` parameter (a dictionary) to the `build_force_torque_sensor` macro when including the sensor in an end-effector:

* `tf_prefix`: the prefix to be added to the TF frame of the sensor.
  This parameter is optional and defaults to an empty string;
* `robot_ip`: the IP address of the robot to which the sensor is connected.
  This parameter is mandatory;
* `update_rate`: the update rate of the sensor in Hz.
  This parameter is mandatory;
