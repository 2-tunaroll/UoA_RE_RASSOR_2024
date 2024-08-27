import os
import glob
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 

class ExternalTemperatureNode(Node):
    def __init__(self):
         
        super().__init__('temperature_node')
        self.publisher_c = self.create_publisher(Float64, 'external_temperature_c', 10)
        self.publisher_f = self.create_publisher(Float64, 'external_temperature_f', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
            
        
    def read_temp_raw(self):

        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')
        
        base_dir = '/sys/bus/w1/devices/'
        device_folder = glob.glob(base_dir + '28*')[0]
        device_file = device_folder + '/w1_slave'

        f = open(device_file, 'r')
        lines = f.readlines()
        f.close()
        return lines

    def publish_temperature(self):

        lines = self.read_temp_raw
        while lines[0].strip()[-3:] != 'YES':
            time.sleep(0.2)
            lines = self.read_temp_raw()
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            temp_c = Float64(temp_string) / 1000.0
            temp_f = temp_c * 9.0 / 5.0 + 32.0

            self.get_logger().info(f'Temperature: {temp_c} °C, {temp_f} °F')

            self.publisher_c.publish(data=temp_c)
            self.publisher_f.publish(data=temp_f)

def main(args=None):
    rclpy.init(args=args)
    node = ExternalTemperatureNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()