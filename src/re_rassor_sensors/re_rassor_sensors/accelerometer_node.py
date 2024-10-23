import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
from re_rassor_sensors.lib.DFRobot_LIS2DW12 import DFRobot_LIS2DW12_I2C

class AccelerometerNode(Node):
    def __init__(self):
        super().__init__('accelerometer_node')
        self.publisher_ = self.create_publisher(Imu, 'imu_data', 10)
        self.timer = self.create_timer(0.5, self.publish_acceleration)

        # Initialize the hardware
        i2c_bus = 1
        address = 0x19
        self.acce = DFRobot_LIS2DW12_I2C(i2c_bus, address)
        if not self.acce.begin():
            self.get_logger().error('Failed to initialize the accelerometer')
            return
        self.acce.set_range(self.acce.RANGE_2G)  # Set range to ±2g
        self.acce.set_data_rate(self.acce.RATE_200HZ)

        # Low-pass filter parameters
        self.alpha = 0.2  # Smoothing factor (0 < alpha < 1)
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_z = 0.0

        # Bias correction (assuming the sensor is stationary at startup)
        self.bias_x, self.bias_y, self.bias_z = self.calibrate_accelerometer()

    def calibrate_accelerometer(self):
        """Calibrate the accelerometer by taking multiple readings and averaging to find bias."""
        num_samples = 100
        sum_x, sum_y, sum_z = 0.0, 0.0, 0.0

        for _ in range(num_samples):
            sum_x += self.acce.read_acc_x()
            sum_y += self.acce.read_acc_y()
            sum_z += self.acce.read_acc_z()
            time.sleep(0.01)

        # Compute the average (in mg) and convert to m/s²
        bias_x = (sum_x / num_samples) * 9.81 / 1000
        bias_y = (sum_y / num_samples) * 9.81 / 1000
        bias_z = (sum_z / num_samples) * 9.81 / 1000

        self.get_logger().info(f"Calibration complete: Bias X={bias_x}, Y={bias_y}, Z={bias_z}")
        return bias_x, bias_y, bias_z

    def publish_acceleration(self):
        # Read acceleration in mg
        x_mg = self.acce.read_acc_x()
        y_mg = self.acce.read_acc_y()
        z_mg = self.acce.read_acc_z()

        # Convert to m/s² and apply bias correction
        x_ms2 = (x_mg * 9.81 / 1000) - self.bias_x
        y_ms2 = (y_mg * 9.81 / 1000) - self.bias_y
        z_ms2 = (z_mg * 9.81 / 1000) - self.bias_z

        # Apply low-pass filter (Exponential Moving Average)
        self.filtered_x = self.alpha * x_ms2 + (1 - self.alpha) * self.filtered_x
        self.filtered_y = self.alpha * y_ms2 + (1 - self.alpha) * self.filtered_y
        self.filtered_z = self.alpha * z_ms2 + (1 - self.alpha) * self.filtered_z

        # Create and populate the IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.linear_acceleration.x = self.filtered_x
        imu_msg.linear_acceleration.y = self.filtered_y
        imu_msg.linear_acceleration.z = self.filtered_z

        imu_msg.orientation_covariance[0] = -1.0  # Indicates no orientation estimate
        imu_msg.angular_velocity_covariance[0] = -1.0  # Indicates no angular velocity estimate
        imu_msg.linear_acceleration_covariance[0] = -1.0  # Indicates no acceleration covariance

        self.publisher_.publish(imu_msg)

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
