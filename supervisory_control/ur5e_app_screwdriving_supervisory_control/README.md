# ur5e_app_screwdriving_supervisory_control

This package contains the configuration files and launch files to execute a supervisory control application for screwdriving tasks with the UR5e robot.

## Build

In order to build the package, run the following command:

```bash
colcon build --packages-up-to ur5e_app_screwdriving_supervisory_control
source install/setup.bash
```

## Demos

In this package, two demos are provided to show the supervisory control application for screwdriving tasks with the UR5e robot, both in simulation and real world.

### Simulation demo

In order to start the supervisory control demo with the simulated UR5e robot, run:

```bash
ros2 launch ur5e_app_screwdriving_supervisory_control gazebo_ros2_control_demo.launch.py
```

To start the supervisory control application, run the following command in another terminal:

```bash
ros2 launch ur5e_app_screwdriving_supervisory_control supervisory_control_gazebo_demo.launch.py
```

### UR5e real robot demo

Before running the demo with the UR5e real robot, ensure that the EtherCAT master is running. You can start it with the following command:

```bash
sudo /etc/init.d/ethercat start
```

In order to start the supervisory control demo with the UR5e real robot and error-based force feedback, run:

```bash
ros2 launch ur5e_app_screwdriving_supervisory_control ur5e_control.launch.py
```

After you click "play" on the teach pendant, open another shell and run the following command to load the controllers:

```bash
ros2 launch ur5e_app_screwdriving_supervisory_control controllers_setup.launch.py
```

To start the supervisory control application, run the following command in another terminal:

```bash
ros2 launch ur5e_app_screwdriving_supervisory_control supervisory_control_demo.launch.py
```

By default the mock bolt's center identification is perfect, so the robot can complete the screwdriving task without any errors.
If you want to test the center identification failure and switch to the manual control mode, execute the following command instead  :

```bash
ros2 launch ur5e_app_screwdriving_supervisory_control supervisory_control_demo.launch.py wrong_bolt_identification:=true
```
