from time import sleep

from adafruit_motor import stepper
from adafruit_motorkit import MotorKit
import board
from custom_msgs.msg import TJoint
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class TJointNode(Node):
    def __init__(self):

        super().__init__('t_joint')

        # number of steps for each callback
        self.steps = 5

        # variable to hold flag for shutting down motors
        self.SHUT_DOWN = False

        # stepper1 is front, stepper2 is back
        self.kit = MotorKit(i2c=board.I2C(), address=0x61)

        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd',
                                                       self.shutdown_callback, 10)
        self.subscription_2 = self.create_subscription(TJoint, 't_joint_cmd',
                                                       self.listener_callback, 10)

    def shutdown_callback(self, msg):
        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.SHUT_DOWN = True

    def listener_callback(self, msg):
        # shutdown flag true: immediately stop motor movement
        if self.SHUT_DOWN:
            self.kit.stepper1.release()
            self.kit.stepper2.release()
            return

        # set movement direction based on controller input
        if msg.up == 1:
            self.direction = stepper.FORWARD
        elif msg.down == 1:
            self.direction = stepper.BACKWARD

        # move
        if msg.up == 1 or msg.down == 1:
            if msg.t_joint.data == 'FRONT':
                for _ in range(self.steps):
                    self.kit.stepper1.onestep(direction=self.direction)
                    sleep(0.01)  # 10 milliseconds delay

            elif msg.t_joint.data == 'BACK':
                for _ in range(self.steps):
                    self.kit.stepper2.onestep(direction=self.direction)
                    sleep(0.01)  # 10 milliseconds delay

        # if no longer moving, release
        else:
            self.kit.stepper1.release()
            self.kit.stepper2.release()


def main(args=None):
    rclpy.init(args=args)

    node = TJointNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.kit.stepper1.release()
        node.kit.stepper2.release()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
