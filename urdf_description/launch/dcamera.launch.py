from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import sys
import pathlib
from launch.actions import TimerAction

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [
    {'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
    {'name': 'camera_namespace', 'default': 'camera', 'description': 'camera namespace'},
    {'name': 'config_file', 'default': os.path.join(get_package_share_directory('urdf_description'), 'config', 'dcamera_params.yaml'), 'description': 'yaml config file'},
]

def set_configurable_parameters(local_params):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in local_params])

tf_publisher_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'dcamera_link', 'camera_depth_optical_frame'],
    output='screen'
)

delayed_tf_node = TimerAction(
    period=5.0,
    actions=[
        tf_publisher_node
    ]
)

def generate_launch_description():
    params = rs_launch.configurable_parameters
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params) +
        [
            OpaqueFunction(function=rs_launch.launch_setup, kwargs={'params': set_configurable_parameters(params)}),
            delayed_tf_node
        ]
    )