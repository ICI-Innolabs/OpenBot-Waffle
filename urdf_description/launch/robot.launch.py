from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import launch_ros.actions
from launch.actions import TimerAction


import os

def generate_launch_description():
    urdf_file = 'urdf_robot.urdf'
    package_description = "urdf_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)
    diff_drive_controller = os.path.join(get_package_share_directory(package_description), 'config', 'diff_drive_controller.yaml')

    imu_config_file = os.path.join(get_package_share_directory('bno055_driver'), 'config', 'params.yaml')
    ekf_config_path = os.path.join(get_package_share_directory('imu_odom_fusion'),'config','ekf_params.yaml')

    controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[diff_drive_controller],
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
            remappings=[
                ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
                ("/diff_drive_controller/odom", "/odom"),
            ]
        )
    
    rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            parameters=[{'use_sim_time': False, 'robot_description': Command(['xacro ', robot_desc_path])}],
            output="screen"
        )
    
    diff_drive_spawner = Node(
            package='controller_manager',
            executable='spawner', 
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    
    joint_broad_spawner = Node(
            package='controller_manager',
            executable='spawner', 
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        )
    
    # jsp_gui_node = Node(
    #         package='joint_state_publisher_gui',
    #         executable='joint_state_publisher_gui',
    #         output='screen'
    #     )

    # ros2 launch bno055_driver imu.launch.py
    imu_node = Node(
            package='bno055_driver',
            executable='bno055_driver',
            name='bno055_driver',
            parameters=[imu_config_file]
        )
        

    lidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB1', 'frame_id': 'laser_frame', 'angle_compensate': True, 'scan_mode': 'Standard'}]
        )

    delayed_lidar = TimerAction(
        period = 5.0,
        actions = [lidar_node]
    )

    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    delayed_ekf = TimerAction(
        period = 5.0,
        actions = [ekf_node]
    )

    
    return LaunchDescription([
        controller_manager_node,
        # rsp_node,
        diff_drive_spawner,
        joint_broad_spawner,
        imu_node,
        delayed_lidar,
        delayed_ekf
    ])