import rclpy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node

class NavToRoomActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_room')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.declare_parameter('room_name', rclpy.Parameter.Type.STRING)

        self.room_name = self.get_parameter('room_name').value # get room name
        self.get_logger().info('Received room_name: {}'.format(self.room_name))

        if self.room_name in ['iotlab', 'iot_annex', 'hall']:
            self.room_name = str(self.room_name)

            self.declare_parameter(self.room_name + '.position.position_x', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(self.room_name + '.position.position_y', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(self.room_name + '.orientation.orientation_z', rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(self.room_name + '.orientation.orientation_w', rclpy.Parameter.Type.DOUBLE)

            self.x = self.get_parameter(self.room_name + '.position.position_x').value
            self.y = self.get_parameter(self.room_name + '.position.position_y').value
            self.z = self.get_parameter(self.room_name + '.orientation.orientation_z').value
            self.w = self.get_parameter(self.room_name + '.orientation.orientation_w').value

            self.get_logger().info('Received x: {}'.format(self.x))
            self.get_logger().info('Received y: {}'.format(self.y))
            self.get_logger().info('Received z: {}'.format(self.z))
            self.get_logger().info('Received w: {}'.format(self.w))

        self.send_goal()
        
    def send_goal(self):
        self.get_logger().info('sending goal to action server')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.pose.position.x = self.x
        goal_msg.pose.pose.position.y = self.y
        goal_msg.pose.pose.orientation.z = self.z
        goal_msg.pose.pose.orientation.w = self.w

        self.get_logger().info('Waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server detected')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self.get_logger().info('Goal request sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback) )

        
def main(args=None):
    rclpy.init(args=args)
    action_client = NavToRoomActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
