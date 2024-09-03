import rclpy
from rclpy.node import Node
from time import sleep
from adafruit_motor import motor
from custom_msgs.msg import BucketDrum
from motor_kit_singleton import MotorKitSingleton

class BucketDrumNode(Node):
    def __init__(self):

        super().__init__('bucket_drum_node')

        # get motor kit instance
        self.kit = MotorKitSingleton.get_instance()

        self.subscription = self.create_subscription(BucketDrum, 'bucket_drum_cmd', self.listener_callback, 10)

    def listener_callback(self, msg):
        
        if msg.forward == 1:

            for speed in range(0, 50, 10):
                self.kit.motor1.throttle = speed

        elif msg.backward == 1:

            for speed in range(0, -50, -10):
                self.kit.motor1.throttle = speed

        else:
            self.kit.motor1.throttle = 0

def main(args=None):
    rclpy.init(args=args)
    node = BucketDrumNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()