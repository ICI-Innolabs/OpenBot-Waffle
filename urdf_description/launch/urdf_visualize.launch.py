import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file = 'urdf_robot.urdf'
    #xacro_file = "urdfbot.xacro"
    package_description = "urdf_description"

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': False, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    return LaunchDescription(
        [            
            robot_state_publisher_node
        ]
    )