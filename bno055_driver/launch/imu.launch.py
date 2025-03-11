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
