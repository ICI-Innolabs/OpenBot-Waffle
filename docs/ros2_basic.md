# ROS2 Basic commands

Always before running any ROS2 commands, make sure you have sourced both your ROS2 installation and your workspace setup files. In our case, we are using ROS2 Jazzy distribution and the workspace is called `openbot_waffle`. 

``` 
    source /opt/ros/jazzy/setup.bash
    source ~/openbot_waffle/install/setup.bash
```

## Topics

 - to view all active topics in the current ROS2 network, use:
``` 
    ros2 topic list
```

- to inspect the messages being published on a particular topic, use the ros2 topic echo command. For example, to see the output of your lidar data (assuming the topic is named /scan):

``` 
    ros2 topic echo /scan
```

- to get detailed information about a specific topic (including its message type and publisher/subscriber details), use:
``` 
    ros2 topic info /scan
```

- to publish a message on a topic, use the ros2 topic pub command. For example, to publish a message on the /cmd_vel topic (assuming the message type is geometry_msgs/Twist), use:
``` 
    ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

## Nodes

- to view all active nodes in the current ROS2 network, use:
``` 
    ros2 node list
```

- to get detailed information about a specific node (including its name, namespace, and associated topics), use:
``` 
    ros2 node info /node_name
```

- to kill a node, use:
``` 
    ros2 node kill /node_name
```

## Packages

- to list all installed packages in the current ROS2 workspace, use:
``` 
    ros2 pkg list
```

- to create a new python package, use:
``` 
    ros2 pkg create --build-type ament_python package_name
```

- to create a new C++ package, use:
``` 
    ros2 pkg create --build-type ament_cmake package_name
```

- to build a specific package, use:
``` 
    colcon build --packages-select package_name
```

- or alternatively, to build all packages in the workspace, use:
``` 
    colcon build
```

- always remember to source the workspace setup file after building a new package:
``` 
    source ~/openbot_waffle/install/setup.bash
```

## Launch files

- to run a launch file, use:
``` 
    ros2 launch package_name launch_file_name.launch.py
```




