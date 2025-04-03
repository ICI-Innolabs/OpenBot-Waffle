# Software Installation

- Ideally you will be using an external dev machine running Ubuntu 24.04 and ROS2 Jazzy along with the OpenBot Waffle robot that uses a Raspberry Pi 5 running Ubuntu 24.04 and ROS2 Jazzy.
- To install ROS2 Jazzy on Ubuntu 24.04 follow the instructions from the official ROS2 website: [ROS2 Jazzy Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Make sure to install the Desktop version of ROS2 Jazzy.
- To install Ubuntu 24.04 on the Raspberry Pi 5, get a new SD card (at least 32 GB), and flash it with Ubuntu using the [Raspberry Pi imager](https://www.raspberrypi.com/software/). After flashing the SD card, insert it into the Raspberry Pi and boot it up. Follow the instructions to set up the Raspberry Pi and proceed with the [installation of ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
- After installing ROS2 Jazzy, clone the OpenBot Waffle repository to your workspace with the following command:
```
git clone https://github.com/ICI-Innolabs/OpenBot-Waffle.git -b main
```
<!-- - After cloning the repository, also clone the repositories for IMU unit and Dynamixel motors (make sure you clone the jazzy branch) in the `src` folder of your workspace:
```
git clone https://github.com/the-hive-lab/bno055_driver
git clone https://github.com/dynamixel-community/dynamixel_hardware.git -b jazzy
``` -->
- Everything you will run on the Raspberry Pi must be done from the root user. To switch to the root user run the following command (if you will try to launch the node for the imu, it won't work without root permissions):
```
sudo su
```
- Then install the ROS2 packages needed for the OpenBot Waffle robot:
```
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-ros2-control
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-dynamixel-sdk
sudo apt install ros-jazzy-rplidar-ros
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo apt install ros-jazzy-cartographer-ros
sudo apt install ros-jazzy-nav2-amcl
sudo apt install ros-jazzy-nav2-lifecycle-manager
sudo apt install ros-jazzy-nav2-controller
sudo apt install ros-jazzy-nav2-planner
sudo apt install ros-jazzy-nav2-behaviors
sudo apt install ros-jazzy-nav2-bt-navigator
sudo apt install ros-jazzy-nav2-dwb-controller
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-librealsense2*
sudo apt install ros-jazzy-realsense2*
sudo apt install ros-jazzy-dynamixel-workbench-toolbox
sudo apt install ros-jazzy-joint-state-broadcaster
sudo apt install ros-jazzy-diff-drive-controller
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-nav2-map-server
```

- To install the python packages needed for the OpenBot Waffle, run the following command:
```
pip install -r requirements.txt --break-system-packages
```

## Dynamixel Motors configuration

- To configure the Dynamixel motors firstly connect the motors individually to the U2D2 module and connect the U2D2 to a machine with Dynamixel Wizard 2.0 installed.
- After connecting the first motor change its ID to 1 and the baud rate to 57600.
- Disconnect the first motor then connect the second motor, change its ID to 2 and the baud rate to 57600.
- Since the motors will be mirrored, it is possible to have one motor spinning in the opposite direction. To fix this, change the motor's spinning direction by ticking the box for Bit 0 - Reverse mode under the Drive mode tab.
- On the Raspberry Pi, configure the latency_timer to 1 instead of 16 by running the following commands or following the instructions form the [official documentation](https://emanual.robotis.com/docs/en/parts/interface/u2d2/#linux).
```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```
- After configuring the motors, connect them to the U2D2 module and connect the U2D2 module to the Raspberry Pi and test the motors. If the motors aren't operating as expected use the python scripts from the `extra/test_motors` folder to identify the issue.

## Configuration for D435I Depth camera

- Since we are using a custom power supply for the Raspberry Pi, by default the USB ports are limited to 0.6A. To increase the current limit to 1.6A run the following command:
```
sudo nano /boot/firmware/config.txt
```
- And add the following line at the end of the file:
```
usb_max_current_enable=1
```
- After adding the line, save the file by pressing `Ctrl + X`, then `Y` and then `Enter` and reboot the Raspberry Pi by running the following command:
```
sudo reboot
```


## Extra - ROS2 Jazzy Installation with Docker

- If you don't have a machine with Ubuntu 24.04, you can use Docker to create a container with ROS2 Jazzy.
- After installing docker you can follow this [guide](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html)

<!-- - to get both lidar and depth camera working. firstly launch robot launch (withouth the depth camera plugged in) then plug in depth camera and launch d435i node with: -->


<!-- - To publish the depth image from the D435I camera run the following command:
```
ros2 run realsense2_camera realsense2_camera_node
```

- In RVIZ2 you can visualize the depth image by adding a new image topic and selecting the `/camera/camera/depth/image_rect_raw` topic.

- useful links: https://github.com/IntelRealSense/realsense-ros and https://dev.intelrealsense.com/docs/ros2-pointcloud-examples -->