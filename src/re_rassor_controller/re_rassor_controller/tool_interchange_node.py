import rclpy
from rclpy.node import Node
from time import sleep
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import board
from std_msgs.msg import Bool
from custom_msgs.msg import Interchange
from gpiozero import DigitalInputDevice

class ToolInterchange(Node):
    def __init__(self):
        super().__init__('tool_interchange')

        self.kit = MotorKit(i2c=board.I2C())

        # # initialise direction as forward
        # self.direction = stepper.BACKWARD
        # number of steps
        self.steps = 30
        # moving flag - so messages are not overloaded and all the required steps complete
        self.moving = False
        # shutdown flag
        self.SHUT_DOWN = False

        # set up GPIO pin for proximity sensor
        # Set up GPIO pin 17 as an input
        self.proximity_sensor = DigitalInputDevice(pin=17)
 
        # set flag for whether it is connected
        self.is_connected = False

        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        self.subscription_2 = self.create_subscription(Interchange, 'tool_interchange_cmd', self.listener_callback, 10)

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
        
        if msg.mode.data == "MANUAL" and msg.toggle == 1:
            self.step()

            # set direction to reverse for next time it is called
            # if it was just connected
            # if self.direction == stepper.FORWARD:
            #     self.is_connected = True
            #     self.direction = stepper.BACKWARD
            # else: # if it was just disconnected
            #     self.direction = stepper.FORWARD
            #     self.is_connected = False

        elif msg.mode.data == "AUTO":
            if not self.proximity_sensor.value and not self.is_connected:
                self.step()
                self.is_connected = True
    
            if msg.toggle == 1 and self.is_connected:
                # self.direction = stepper.BACKWARD
                self.step()
                self.is_connected = False

    def step(self):
        # does the required amount of stepping for the interchange to connect/disconnect
        self.moving = True
        for _ in range(self.steps):
            self.kit.stepper1.onestep(direction=self.direction, style=stepper.INTERLEAVE)
            sleep(0.05)  # delay
        self.moving = False
         
def main(args=None):
    rclpy.init(args=args)
    node = ToolInterchange()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.kit.stepper1.release()
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()