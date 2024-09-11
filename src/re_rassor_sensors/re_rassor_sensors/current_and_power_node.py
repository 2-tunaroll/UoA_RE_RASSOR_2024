import rclpy
from rclpy.node import Node
from re_rassor_sensors.lib.adafruit_ina260 import INA260
from std_msgs.msg import Bool, Int16
import board
import os

class CurrentandPowerNode(Node):
    def __init__(self):

        super().__init__('current_and_power_node')

        # initialise chip
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.ina260 = INA260(i2c) # default address 0x40

        self.shutdown_publisher_ = self.create_publisher(Bool, 'shutdown_cmd', 10)
        self.voltage_publisher_ = self.create_publisher(Int16, 'battery_voltage', 10)

        self.get_current_and_voltage()

    def get_current_and_voltage(self):

        # current
        current_msg = Bool()
        if self.ina260.current > 7500:
            current_msg.data = True
        else:
            current_msg.data = False
        self.shutdown_publisher_.publish(current_msg)
        
        # voltage
        voltage_msg = Int16()
        voltage_msg.data = self.ina260.voltage

        # shutdown the Raspberry Pi if the voltage drops below 14.8V
        if voltage_msg.data <= 14.8:
            os.system("sudo shutdown -h now")
            return

        self.voltage_publisher_.publish(voltage_msg)

def main(args=None):

    rclpy.init(args=args)
    
    node = CurrentandPowerNode()
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
