# Description

This repository contains packages to support screwdrving operations in ROS 2 under supervisory control using the UR5e robot.

## Requirements

In order to run the demos in this repository, you need to:

- Install ROS 2 Humble. You can follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) for Ubuntu 22.04;
- Install the ROS 2 packages for the UR5e robot.
To install the UR5e packages, run the following command:

    ```bash
    sudo apt install ros-humble-ur-*
    ```

- Install the RQT packages. To install the RQT packages, run the following command:

    ```bash
    sudo apt install ros-humble-rqt*
    ```

- Install the Touch haptic device driver and API. To install them, refer to the [Touch drivers installation guide](https://support.3dsystems.com/s/article/OpenHaptics-for-Linux-Developer-Edition-v34?language=en_US);
- Install EtherLab. To install it refer to the [EtherLab installation guide](https://github.com/alesof/ethercat_ROS2?tab=readme-ov-file#manual-install-etherlab);
- Install other dependencies by running the following command from the root of your `colcon` workspace:

    ```bash
    rosdep update && rosdep install --from-paths src --ignore-src --rosdistro humble
    ```

- Create a `colcon` workspace and clone this repository into the `src` folder of your workspace:

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

- Clone this repository:

    ```bash
    git clone git@github.com:LorenzoPagliara/ur5e_supervisory_control.git
    ```

## Demo

To execute the demo of the screwdriving task under supervisory control with the UR5e, refer to the package [`ur5e_app_screwdriving_supervisory_control`](./supervisory_control/ur5e_app_screwdriving_supervisory_control/README.md).
