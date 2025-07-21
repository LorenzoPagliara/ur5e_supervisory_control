# touch_haptic_device

This package contains a `ros2_control` implementation of the Touch haptic device from 3DSystems.

- The [`include`](include/) and [`src`](src/) folders contain header and source files for the `TouchHapticDevice` plugin.
- The [`urdf`](./urdf/) folder contains:
    - [`touch_haptic_device.ros2_control.xacro`](./urdf/touch_haptic_device.ros2_control.xacro): the hardware description of the Touch haptic device;
    - [`touch_haptic_device.xacro`](./urdf/touch_haptic_device.xacro): the description file that adds a base link to the `ros2_control` hardware description above.
    This is needed to load the robot description to the `robot_state_publisher`, which disseminates it to the whole ROS network.
    Loading the robot description to the `controller_manager` directly is a deprecated behavior.
- The [`config`](config/) folder contains:
    - [`haptic_pose_broadcaster.yaml`](config/haptic_pose_broadcaster.yaml): the controllers configuration file used in the haptic pose broadcaster demo.
    - [`haptic_pose_plot.xml`](config/haptic_pose_plot.xml): a configuration file needed for the `Plotjuggler` layout.
- The [`launch`](launch/) folder contains:
    - [`demo.launch.py`](launch/demo.launch.py): a demo to load and display information about the haptic device description file, and publish the description on the `/hd/robot_description` topic.
    - [`demo_haptic_pose_broadcaster.launch.py`](launch/demo_haptic_pose_broadcaster.launch.py): a demo where a broadcaster publishes the pose of the haptic device on a topic, and starts a `Plotjuggler` node to graphically display the pose.


## Driver and API installation for Touch haptic device

In order to check if installation was already performed, execute

```bash
ls /usr/bin/ | grep 'ListUSBHapticDevices\|Touch_Diagnostic\|Touch_Setup'
ls /usr/lib | grep libPhantomIOLib42.so
ls /usr/include/HD | grep hd.h
ls /usr/include/HDU | grep hdu.h
```

If you have a result for each command, you don't need to follow the installation procedure in the corresponding section.

Moreover, you may need to change the Linux **Locale** in order to work with the API. In our case, the solution is to create and edit a file *`$HOME/.pam_environment`* with the following content:

```bash
LC_NUMERIC=en_US.UTF-8
LANG=en_US.UTF-8
```

In order to apply the changes, you have to log out. After that, you can see the changes by running:

```bash
locale
```

### Installation procedure

Download the [API](https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz) and install it with the following commands

```bash
wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/support/downloads/KB+Files/Open+Haptics/openhaptics_3.4-0-developer-edition-amd64.tar.gz
tar -xvzf openhaptics_3.4-0-developer-edition-amd64.tar.gz
cd openhaptics_3.4-0-developer-edition-amd64/
sudo ./install
rm -r openhaptics_3.4-0-developer-edition-amd64*
```

Download the [Driver](https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver2022_04_04.tgz) provided by 3D Systems with the following command

```bash
wget https://s3.amazonaws.com/dl.3dsystems.com/binaries/Sensable/Linux/TouchDriver2022_04_04.tgz
tar -xvzf TouchDriver2022_04_04.tgz
```

Next you need to install the driver.
To do so, execute the following commands from the same directory as before

```bash
sudo cp ./TouchDriver2022_04_04/usr/lib/libPhantomIOLib42.so /usr/lib/
sudo cp ./TouchDriver2022_04_04/bin/Touch_Setup /usr/bin/
sudo cp ./TouchDriver2022_04_04/bin/Touch_Diagnostic /usr/bin/
sudo cp ./TouchDriver2022_04_04/ListUSBHapticDevices /usr/bin/
rm -r TouchDriver2022_04_04*
```

After that, you have to permanently set `GTDD_HOME` local environment variable.

Note that you do not need to run this command if someone else has followed this guide on the same machine before you.
You can check that running the following line:

```bash
cat /etc/profile.d/openhaptics.sh
```

If the string `export GTDD_HOME=$HOME/.3dsystems/` is already present, you can skip the next step:

```bash
sudo chmod 777 /etc/profile.d/openhaptics.sh

echo '# set GTDD_HOME path needed to use 3D Systems Haptic Device Touch
export GTDD_HOME=$HOME/.3dsystems/' >> /etc/profile.d/openhaptics.sh
```

In order to apply the changes, you have to log out.
This last step isn't compulsory for having a working device, but it is useful to avoid annoying messages due to the forementioned local variable which can be unset.

Lastly, you have to install some additional packages through the following commands:

``` bash
sudo apt install qtbase5-dev qt5-qmake
sudo apt install libncurses5 libncurses5-dev freeglut3 build-essential
```

### Usage

In order to use the device, you must have access to the `ttyACM` port; to do so, add your username to the `dialout` group and reboot your machine:

```bash
sudo usermod -a -G dialout $USER
sudo shutdown -r now
```

You can check that the device is available by running the command `ListUSBHapticDevices`.
By default, the device name is set to `ttyACM0`.

The first time you use the haptic device, you need to run the following command and save the configuration (if no port appears in the drop-down menu 'Device List', the driver is not working properly and has to be re-installed):

```bash
Touch_Setup
```

Now you should be able to see the file `Default Device.config` by running:

```bash
ls ~/.3dsystems/config/
```

Before running the diagnostic, the user shall make sure that the device's serial number appears under the field `Hardware > Device Serial Num` in the `Touch_Setup`.
In order to check whether or not the device works correctly run:

```bash
Touch_Diagnostic
```

The user shall run all the tests included in the `Touch_Diagnostic` software in order to ensure that all the device's functionalities are working properly.

If the setup is successful, by running the command:

```bash
cat ~/.3dsystems/config/Default\ Device.config
```

the user must see the following values of the reset angles:

```text
ResetAngle0=0.000000
ResetAngle1=0.000000
ResetAngle2=0.000000
ResetAngle3=-2.617994
ResetAngle4=4.188790
ResetAngle5=-2.617994
```

In case of anomalies, a new calibration can be forced by cleaning the `~/.3dsystems/config` folder, and repeating the steps in this section of the guide.

## Launch demos

After installing dependencies, from the colcon main folder, build the package with:

```bash
colcon build --packages-up-to touch_haptic_device
source install/setup.bash
```

To launch the [demo](launch/demo.launch.py) to load and display information about the haptic device description file, and publish the description to the `/hd/robot_description` topic, execute the following command on one shell:

```bash
ros2 launch touch_haptic_device demo.launch.py
```

And check the content of the `/hd/robot_description` topic by running this command on another shell:

```bash
ros2 topic echo -f /hd/robot_description
```

Instead, to launch the [haptic broadcaster demo](launch/demo_haptic_pose_broadcaster.launch.py) where a broadcaster publishes the pose of the haptic device on a topic, and starts a `Plotjuggler` node to graphically display the pose, execute the following command:

```bash
ros2 launch touch_haptic_device demo_haptic_pose_broadcaster.launch.py
```
