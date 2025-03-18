import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseWithCovarianceStamped
from custom_interfaces.srv import MyServiceMessage
import os
from rclpy.executors import MultiThreadedExecutor

class SpotRecorder(Node):

    def __init__(self):
        super().__init__('spots_recorder')
        self.group1 = ReentrantCallbackGroup()

        self.srv = self.create_service(MyServiceMessage, 'spot_record', self.service_callback, callback_group=self.group1)
        self.subscriber_amcl_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),callback_group=self.group1)

        # self.txt_path = os.path.join(get_package_share_directory('project_localization'), 'txt/spots.txt')
        # self.txt_path = "/home/user/ros2_ws/src/project_path_planning/config/spots_to_nav.yaml"
        self.txt_path = "/home/bogdan/Desktop/ros_ws_github/OpenBot-Waffle/nav2room/config/spots_to_nav.yaml"
        self.spots = [] # list of dictionaries
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.orient_z = 0.0
        self.orient_w = 0.0
        self.end = False

    def service_callback(self, request, response):

        if request.label == 'start':
            f = open(self.txt_path, 'w')
            txt = 'nav_to_point:\n'
            f.write(txt)
            txt = '  ros__parameters:\n'
            f.write(txt)

        elif request.label == 'end':
            response.navigation_successfull = True
            self.end = True
            response.message = "All spots saved to txt file"

        else:
            f = open(self.txt_path, 'a')
            spot = {
                'label': request.label,
                'position_x': self.pose_x,
                'position_y': self.pose_y,
                'orientation_z': self.orient_z,
                'orientation_w': self.orient_w,
            }
            self.spots.append(spot)

            txt = '    ' + str(request.label) + ':\n'
            f.write(txt)

            txt = '      position:\n'
            f.write(txt)
            txt = '        position_x: ' + str(self.pose_x) + '\n'
            f.write(txt)
            txt = '        position_y: ' + str(self.pose_y) + '\n'
            f.write(txt)

            txt = '      orientation:\n'
            f.write(txt)
            txt = '        orientation_z: ' + str(self.orient_z) + '\n'
            f.write(txt)
            txt = '        orientation_w: ' + str(self.orient_w) + '\n'
            f.write(txt)

            response.navigation_successfull = True
            response.message = "Spot " + request.label + " registered."

        return response

    def amcl_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.orient_z = msg.pose.pose.orientation.z
        self.orient_w = msg.pose.pose.orientation.w
        self.get_logger().info('Data received : "%s"' % str(self.pose_x))

def main(args=None):
    # rclpy.init(args=args)
    # spot_recorder = SpotRecorder()
    # executor = MultiThreadedExecutor(num_threads=3)
    # executor.add_node(spot_recorder)
    # try:
    #     executor.spin()
    # finally:
    #     executor.shutdown()
    #     spot_recorder.destroy_node()
    # rclpy.shutdown()

    rclpy.init(args=args)
    service = SpotRecorder() 
    while not service.end:
        rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        