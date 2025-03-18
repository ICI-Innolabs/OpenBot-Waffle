from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='nav2room',
            executable='nav_to_room',
            name='nav_to_room',
            output='screen'),
    ])