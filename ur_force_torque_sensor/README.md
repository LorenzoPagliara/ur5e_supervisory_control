# ur_force_torque_sensor

This package provides a wrapper for the Universal Robots e-series robots integrated force/torque sensor.

- The [`include`](include/) and [`src`](src/) folders contain header and source files developing the [`URFTSensor`](./include/ur_force_torque_sensor/ur_force_torque_sensor.hpp) plugin.
- The [`config`](./config/) folder contain :
    - [`ur_fts.ros2_control.xacro`](./config/ur_fts.ros2_control.xacro): defines the configuration for the `URFTSensor`;

The main parameters expected by the sensor, are:

- `tf_prefix`: the prefix of the tf frame name;
- `robot_ip`: the IP address of the robot;
- `update_rate`: the update rate of the sensor;

The sensor is expected to be used with the `ros2_control` framework, and the configuration file is expected to be used in the `ros2_control` configuration file.

## Dependencies

Since the sensor make access to the force/torque measurements of the robot trhough the RTDE protocol, this package depends on the `ur_rtde` library. To install it, you can use the following commands:

```bash
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-dev
```
