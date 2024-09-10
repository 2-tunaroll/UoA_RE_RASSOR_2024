import rclpy
from rclpy.node import Node
from time import sleep
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import board
from custom_msgs.msg import TJoint

class TJointNode(Node):
    def __init__(self):

        super().__init__('t_joint')

        self.kit = MotorKit(i2c=board.I2C(), address=0x61)

        self.subscription = self.create_subscription(TJoint, 't_joint_cmd', self.listener_callback, 10)

    def listener_callback(self, msg):

        


        return None

def main(args=None):
    rclpy.init(args=args)

    node = TJointNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


