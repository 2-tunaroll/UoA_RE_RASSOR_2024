import rclpy
from rclpy.node import Node
from time import sleep
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import board
from std_msgs.msg import Int16, Bool

class ToolInterchange(Node):
    def __init__(self):
        super().__init__('tool_interchange')

        self.kit = MotorKit(i2c=board.I2C())

        # initialise direction as forward
        self.direction = stepper.FORWARD
        
        # number of steps
        self.steps = 13

        # moving flag - so messages are not overloaded and all the required steps complete
        self.moving = False

        # shutdown flag
        self.SHUT_DOWN = False

        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        self.subscription_2 = self.create_subscription(Int16, 'tool_interchange_cmd', self.listener_callback, 10)

    def shutdown_callback(self, msg):
        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.SHUT_DOWN = True

    def listener_callback(self, msg):

        # check for current spike
        if self.SHUT_DOWN:
            self.kit.stepper1.release()
            return
    
        # check that the motor is not already stepping
        if self.moving:
            return

        else:
            if msg.data == 1:
                self.moving = True
                for _ in range(self.steps):
                        self.kit.stepper1.onestep(direction=self.direction)
                        sleep(0.01)  # 10 milliseconds delay

                if self.direction == stepper.FORWARD:
                    self.direction = stepper.BACKWARD

                else:
                    self.direction = stepper.FORWARD
                
                # set the flag back to false
                self.moving = False 

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