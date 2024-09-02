import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import re_rassor_controller.lib.controller_input_defs as inputs

class TestControlNode(Node):
    def __init__(self):
        super().__init__('test_control_node')

        # set up constants

        self.last_event_time = 0
        print(self.last_event_time)

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)


    def listener_callback(self, msg):
        
        velocities = self.calculate_motor_velocities(msg)
        print(velocities)

    def calculate_motor_velocities(self, msg):

        x_cmd = msg.linear.x
        z_cmd = msg.angular.z
        
        # constants
        speed_multiplier = 50
        turn_threshold = 0.2
        turn_component = 0

        # set turn component based on a threshold value
        if abs(z_cmd) > turn_threshold:

            if z_cmd > 0:
                turn_component = z_cmd - turn_threshold
            
            else:
                turn_component = z_cmd + turn_threshold

        # left wheels
        v_front_left = (-0.5*x_cmd + 0.5*turn_component)*speed_multiplier
        v_back_left = (-0.5*x_cmd + 0.5*turn_component)*speed_multiplier

        # right wheels
        v_front_right = (-0.5*x_cmd - 0.5*turn_component)*speed_multiplier
        v_back_right = (-0.5*x_cmd - 0.5*turn_component)*speed_multiplier

        return v_front_left, v_back_left, v_front_right, v_back_right

def main(args=None):

    rclpy.init(args=args)
    node = TestControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()