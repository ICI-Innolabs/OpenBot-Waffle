import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    urdf_file = 'urdf_robot.urdf'
    package_description = "urdf_description"

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    # Launch the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        # emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="both"
    )

    # Launch Gazebo Simulation
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r /home/gazebo/Desktop/ros2_ws/openbot_waffle/openbot_waffle_gazebo/sdf/world.sdf'}.items(),
        # launch_arguments={'world': '/home/bogdan/ros2_ws/src/diff_robot_gazebo/sdf/world.sdf'}.items(),
        # launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Spawn
    spawn = Node(package='ros_gz_sim', executable='create',
                 parameters=[{
                    # 'name': 'my_custom_model',
                    'x': 1.2,
                    'z': 2.3,
                    'Y': 3.4,
                    'topic': '/robot_description'}],
                 output='screen')
    
    # Bridge for /cmd_vel
    bridge = Node(
        package = "ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist", 
            "/world/empty/model/urdfbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        ],
        remappings=[
        ("/world/empty/model/urdfbot/joint_state", "/joint_states"),
        ("/model/urdfbot/pose", "/tf"),
        ("/model/urdfbot/pose_static", "/tf_static"),
        # ("/lidar", "/scans"),
    ]
    )

    return LaunchDescription(
        [            
            robot_state_publisher_node,
            gazebo,
            spawn,
            bridge
        ]
    )