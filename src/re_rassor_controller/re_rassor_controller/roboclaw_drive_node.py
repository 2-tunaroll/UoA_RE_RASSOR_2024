import atexit
import time

from custom_msgs.msg import WheelSpeeds
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from re_rassor_controller.re_rassor_controller.lib.roboclaw_3 import Roboclaw

from std_msgs.msg import Bool, Float32


class RoboClawMotorDrive(Node):
    def __init__(self):

        super().__init__('roboclaw_drive')

        # motor controller boards
        self.addresses = [0x80]
        self.roboclaw_back = Roboclaw('/dev/ttyS0', 38400)

        # # initialise the boards
        self.roboclaw_back.Open()

        # shutdown flag
        self.SHUT_DOWN = False

        # speed multiplier: initialise to 25%
        self.speed_multiplier = 0.25

        # # store previous speed values
        # self.v_front_left = 0
        # self.v_back_left = 0
        # self.v_front_right = 0
        # self.v_back_right = 0

        # update time
        self.dt = 0.5

        self.last_called_time = time.time()

        # subscribe to current sensing command
        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd',
                                                       self.shutdown_callback, 10)
        # subscribe to velocity cmds
        self.subscription_2 = self.create_subscription(Twist, 'cmd_vel',
                                                       self.drive_cmd_callback, 10)
        # subscribe to speed mode
        self.subscription_3 = self.create_subscription(Float32, 'speed_mode', self.speed_mode_callback, 10)

        # publish wheel speeds
        self.speed_publisher_ = self.create_publisher(WheelSpeeds, 'wheel_speeds', 100)
        self.timer = self.create_timer(1, self.publish_wheel_speeds)

    def shutdown_callback(self, msg):

        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.SHUT_DOWN = True

    def speed_mode_callback(self, msg):

        # sets the speed multipler for driving
        self.speed_multiplier = msg.data

    def motor_shutdown(self):
        self.roboclaw_back.ForwardMixed(self.addresses[0], 0)
        self.roboclaw_back.TurnLeftMixed(self.addresses[0], 0)

    def drive_cmd_callback(self, msg):

        # chekck shutdown flag
        if self.SHUT_DOWN:
            # stop motors
            self.motor_shutdown()
            return

        current_time = time.time()

        # only send commands every 0.5 s
        if (current_time - self.last_called_time) > self.dt:

            x_cmd = int(msg.linear.x * 127 * self.speed_multiplier)
            z_cmd = int(msg.angular.z * 127 * self.speed_multiplier)

            self.last_called_time = time.time()

            if x_cmd <= 0: # drive forward (or stop)
                self.roboclaw_back.ForwardMixed(self.addresses[0], abs(x_cmd))
            else:
                self.roboclaw_back.BackwardMixed(self.addresses[0], abs(x_cmd))

            if z_cmd <= 0: # turn left
                self.roboclaw_back.TurnLeftMixed(self.addresses[0], abs(z_cmd))
            else: # turn right
                self.roboclaw_back.TurnRightMixed(self.addresses[0], z_cmd)

    def publish_wheel_speeds(self):

        # back wheels
        m1speed = self.roboclaw_back.ReadSpeedM1(self.addresses[0])
        m2speed = self.roboclaw_back.ReadSpeedM2(self.addresses[0])

        print(f"m1: {m1speed}, m2: {m2speed}")
        
def main(args=None):

    rclpy.init(args=args)

    node = RoboClawMotorDrive()

    atexit.register(node.motor_shutdown)
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
