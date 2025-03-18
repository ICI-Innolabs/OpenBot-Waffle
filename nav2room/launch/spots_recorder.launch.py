from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='nav2room',
            executable='spots_recorder',
            name='spots_recorder',
            output='screen'),
    ])