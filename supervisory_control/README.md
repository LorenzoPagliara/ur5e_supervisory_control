# supervisory_control

This container provides a set of packages for supervisory control of a robotic system.

## Packages

- [`ur5e_app_screwdriving_supervisory_control`](./ur5e_app_screwdriving_supervisory_control/README.md): This package contains the configuration files and launch files to execute a supervisory control application for screwdriving tasks with the UR5e robot. It includes demos for both simulation and real-world applications.
- [`supervisory_control_rviz_panel`](./supervisory_control_rviz_panel/README.md): This package provides an RViz panel for supervisory control applications, allowing users to interact with the supervisory control system and visualize its state.
- [`supervisory_control_rviz_msg`](./supervisory_control_rviz_msg/README.md): This package contains the message definitions used in the supervisory control system, facilitating communication between the supervisory controller and RViz for visualization purposes.
- [`screwdriving_supervisory_controller`](./screwdriving_supervisory_controller/README.md): This package implements a supervisory controller for screwdriving operations, coordinating the actions of the robot and ensuring safe and efficient task execution.
- [`endoscopic_camera_streaming_rviz`](./endoscopic_camera_streaming_rviz/README.md): This package provides a simple ROS2 node to stream images from an endoscopic camera using OpenCV and publish them as sensor_msgs/Image messages to be visualized in RViz.
