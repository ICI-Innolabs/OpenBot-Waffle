import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import adafruit_bno055


def verify_values(value_tuple):
    """
    Iterate through a tuple and return False if any values are None,
    return True otherwise.

    :param value_tuple: tuple to check for None values
    :return: boolean
    """
    valid_data = True

    for value in value_tuple:
        if value is None:
            valid_data = False

    return valid_data


class ReadIMU(Node):
    def __init__(self):
        super().__init__('imu_driver')

        self.i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

        # Declaring parameters
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('imu_rate', 50.0)

        # Reading in parameters for node, by default the parameters can be found
        # under config/params.yaml
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.rate = self.get_parameter('imu_rate').get_parameter_value().double_value

        # Create topic publishers for IMU        
        self.imu_output = self.create_publisher(Imu, self.imu_topic, 1)

        # # Create timer for IMU publisher.
        # self.rate = 100  # 100 Hz
        self.timer_period = 1.0 / self.rate
        self.timer = self.create_timer(self.timer_period, self._publish_imu_msg)

    def _publish_imu_msg(self):
        """
        Read values from IMU on I2C bus. Verify these values are valid (not None).
        Once this is confirmed, create and publish an Imu message onto imu_topic param.

        :return: nothing
        """
        imuMsg = Imu()
        imuMsg.header.stamp = self.get_clock().now().to_msg()
        imuMsg.header.frame_id = 'imu_link'

        valid_values = False

        quaternion_values = (self.sensor.quaternion[0], self.sensor.quaternion[1], self.sensor.quaternion[2],
                             self.sensor.quaternion[3])
        gyro_values = (self.sensor.gyro[0], self.sensor.gyro[1], self.sensor.gyro[2])
        linear_acceleration_values = (self.sensor.linear_acceleration[0], self.sensor.linear_acceleration[1],
                                      self.sensor.linear_acceleration[2])

        quaternion_integrity = verify_values(quaternion_values)
        gyro_integrity = verify_values(gyro_values)
        linear_acceleration_integrity = verify_values(linear_acceleration_values)

        if quaternion_integrity and gyro_integrity and linear_acceleration_integrity:
            valid_values = True

        if valid_values:
            imuMsg.orientation.w = float(quaternion_values[0])
            imuMsg.orientation.x = float(quaternion_values[1])
            imuMsg.orientation.y = float(quaternion_values[2])
            imuMsg.orientation.z = float(quaternion_values[3])

            imuMsg.angular_velocity.x = float(gyro_values[0])
            imuMsg.angular_velocity.y = float(gyro_values[1])
            imuMsg.angular_velocity.z = float(gyro_values[2])

            imuMsg.linear_acceleration.x = float(linear_acceleration_values[0])
            imuMsg.linear_acceleration.y = float(linear_acceleration_values[1])
            imuMsg.linear_acceleration.z = float(linear_acceleration_values[2])

            self.imu_output.publish(imuMsg)

        else:
            self.get_logger().debug(f"ERROR: One or more values was None.")


def main(args=None):
    rclpy.init(args=args)
    imu_driver = ReadIMU()
    rclpy.spin(imu_driver)
    imu_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
