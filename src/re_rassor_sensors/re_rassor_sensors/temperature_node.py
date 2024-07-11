import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from .PiicoDev_TMP117 import PiicoDev_TMP117
from .PiicoDev_Unified import sleep_ms 

class TemperatureNode(Node):
    def __init__(self):
        super().__init__('temperature_node')
        self.temp_sensor = PiicoDev_TMP117()  # initialize the sensor
        self.publisher_c = self.create_publisher(Float64, 'temperature_celsius', 10)
        self.publisher_f = self.create_publisher(Float64, 'temperature_fahrenheit', 10)
        self.publisher_k = self.create_publisher(Float64, 'temperature_kelvin', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)  # 1 second timer

    def publish_temperature(self):
        temp_c = self.temp_sensor.readTempC()  # Celsius
        temp_f = self.temp_sensor.readTempF()  # Fahrenheit
        temp_k = self.temp_sensor.readTempK()  # Kelvin

        self.get_logger().info(f'Temperature: {temp_c} °C, {temp_f} °F, {temp_k} K')

        # Publish temperature readings
        self.publisher_c.publish(Float64(data=temp_c))
        self.publisher_f.publish(Float64(data=temp_f))
        self.publisher_k.publish(Float64(data=temp_k))

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()