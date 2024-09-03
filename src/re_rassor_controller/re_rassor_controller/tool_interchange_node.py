import rclpy
from rclpy.node import Node
from time import sleep
from adafruit_motor import stepper
from std_msgs.msg import Int16
from motor_kit_singleton import MotorKitSingleton

class ToolInterchange(Node):
    def __init__(self):
        super().__init__('tool_interchange')

        self.kit = MotorKitSingleton.get_instance()

        # initialise direction as forward
        self.direction = stepper.FORWARD
        
        # number of steps
        self.steps = 13

        self.subscription = self.create_subscription(Int16, 'tool_interchange_cmd', self.listener_callback, 10)

    def listener_callback(self, msg):
        
        if msg.data == 1:
            for _ in range(self.steps):
                    self.kit.stepper1.onestep(direction=self.direction)
                    sleep(0.01)  # 10 milliseconds delay

            if self.direction == stepper.FORWARD:
                self.direction = stepper.BACKWARD

            else:
                self.direction = stepper.FORWARD

def main(args=None):
    rclpy.init(args=args)
    node = ToolInterchange()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()