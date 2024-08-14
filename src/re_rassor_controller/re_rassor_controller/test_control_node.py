import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json

class TestControlNode(Node):
    def __init__(self):
        super().__init__('test_control_node')

        # set up constants
        X_BUTTON = 0


        self.subscription = self.create_subscription(String, 'controller_state', self.listener_callback, 10)

    def listener_callback(self, msg):

        array = json.loads(msg.data)

        if array['buttons'][0] == 1:
            self.get_logger().info('X button pressed')

def main(args=None):

    rclpy.init(args=args)
    node = TestControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
