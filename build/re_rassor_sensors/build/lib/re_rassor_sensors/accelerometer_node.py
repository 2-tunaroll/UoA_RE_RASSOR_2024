import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import sys
import os

# Add the lib directory to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
module_path = os.path.join(current_dir, 'lib')
if module_path not in sys.path:
    sys.path.insert(0, module_path)

from DFRobot_LIS2DW12 import DFRobot_LIS2DW12_I2C

class AccelerometerNode(Node):
    def __init__(self):
        super().__init__('accelerometer_node')
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 10)
        self.timer = self.create_timer(0.5, self.publish_acceleration)

        # Initialize the hardware
        i2c_bus = 1
        address = 0x19
        self.acce = DFRobot_LIS2DW12_I2C(i2c_bus, address)
        self.acce.begin()
        self.acce.set_range(self.acce.RANGE_2G)  # You can set other ranges if needed
        self.acce.set_data_rate(self.acce.RATE_200HZ)

        self.get_logger().info('AccelerometerNode initialized')

    def publish_acceleration(self):
        # Read acceleration in mg
        x_mg = self.acce.read_acc_x()
        y_mg = self.acce.read_acc_y()
        z_mg = self.acce.read_acc_z()

        # Convert to m/s²
        x_ms2 = x_mg * 9.81 / 1000
        y_ms2 = y_mg * 9.81 / 1000
        z_ms2 = z_mg * 9.81 / 1000

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = x_ms2
        imu_msg.linear_acceleration.y = y_ms2
        imu_msg.linear_acceleration.z = z_ms2

        imu_msg.orientation_covariance[0] = -1.0  # Indicates no orientation estimate
        imu_msg.angular_velocity_covariance[0] = -1.0  # Indicates no angular velocity estimate
        imu_msg.linear_acceleration_covariance[0] = -1.0  # Indicates no acceleration covariance

        self.publisher_.publish(imu_msg)
        self.get_logger().info(f'Published IMU data: x={x_ms2}m/s², y={y_ms2}m/s², z={z_ms2}m/s²')

def main(args=None):
    rclpy.init(args=args)
    node = AccelerometerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


