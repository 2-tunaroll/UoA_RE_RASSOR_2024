#!/usr/bin/env python3
import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time

# Add the lib directory to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
module_path = os.path.join(current_dir, 'lib')
if module_path not in sys.path:
    sys.path.insert(0, module_path)

from DFRobot_LIS2DW12 import DFRobot_LIS2DW12_I2C

class AccelerometerNode(Node):
    def __init__(self):
        super().__init__('accelerometer_node')
        self.publisher_ = self.create_publisher(Imu, 'accel_data', 10)

        # Initialize LIS2DW12 accelerometer over I2C
        self.acce = DFRobot_LIS2DW12_I2C(bus=1, address=0x19)
        self.acce.begin()  # Initialize the accelerometer
        self.acce.set_power_mode(self.acce.CONT_LOWPWRLOWNOISE2_14BIT)  # Set power mode
        self.acce.set_range(self.acce.RANGE_2G)  # Set measurement range to ±2g
        self.acce.set_data_rate(self.acce.RATE_200HZ)  # Set data rate to 200Hz

        # Start publishing data
        self.timer = self.create_timer(0.1, self.publish_data)  # Adjust frequency as needed

    def publish_data(self):
        # Read accelerometer data
        x = self.acce.read_acc_x()
        y = self.acce.read_acc_y()
        z = self.acce.read_acc_z()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'accel_frame'
        imu_msg.linear_acceleration.x = x
        imu_msg.linear_acceleration.y = y
        imu_msg.linear_acceleration.z = z

        self.publisher_.publish(imu_msg)
        self.get_logger().info('Publishing accelerometer data')

def main(args=None):
    rclpy.init(args=args)
    accelerometer_node = AccelerometerNode()
    rclpy.spin(accelerometer_node)
    accelerometer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
