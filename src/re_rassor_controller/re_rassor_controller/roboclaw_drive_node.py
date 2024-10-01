from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from custom_msgs.msg import WheelSpeeds
import board
import rclpy
import time
from math import pi


class RoboClawMotorDrive(Node):
    def __init__(self):

        super().__init__('roboclaw_drive')

        # motor controller boards

        # initialise the boards


        # shutdown flag
        self.SHUT_DOWN = False

        # speed multiplier: initialise to 25%
        self.speed_multiplier = 25

        # # store previous speed values
        # self.v_front_left = 0
        # self.v_back_left = 0
        # self.v_front_right = 0
        # self.v_back_right = 0

        # # update time
        # self.dt = 0.2

        # self.last_called_time = time.time()

        # subscribe to current sensing command
        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        # subscribe to velocity cmds
        self.subscription_2 = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        # subscribe to speed mode
        self.subscription_3 = self.create_subscription(Float32, 'speed_mode', self.speed_mode_callback, 10)
        # publish wheel velocities
        self.speed_publisher_ = self.create_publisher(WheelSpeeds, 'wheel_speeds', 100)

   

    def shutdown_callback(self, msg):

        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.SHUT_DOWN = True

    def speed_mode_callback(self, msg):

        # sets the speed multipler for driving
        self.speed_multiplier = msg.data

  

def main(args=None):

    rclpy.init(args=args)
    
    node = RoboClawMotorDrive()
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()