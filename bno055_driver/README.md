# bno055_driver

ROS2 Foxy Driver for Adafruit BNO055 IMU

To connect and get started follow Adafruit guide:

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython

This package requires you to install the following Python library:
```shell
sudo pip3 install adafruit-circuitpython-bno055
```

Run ROS2 node:
```shell
ros2 run bno055_driver bno055_driver
```

Launch files:
```shell
ros2 launch bno055_driver imu.launch.py
```

Example launch file:
```python
# Standard library imports
from os.path import join

# Third-party imports
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    """Launch bno055_driver node using the params.yaml config file."""

    config_file = join(get_package_share_directory('bno055_driver'),
                       'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='bno055_driver',
            executable='bno055_driver',
            name='bno055_driver',
            parameters=[config_file]
        )
    ])

```
Thanks [Patrick Marshall](https://github.com/marshallpt) :)
