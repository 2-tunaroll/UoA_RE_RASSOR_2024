from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rclpy
import time

from re_rassor_controller.lib.DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC

class WheelMotorDrive(Node):
    def __init__(self):

        super().__init__('wheel_drive')

        self.left_board = DFRobot_DC_Motor_IIC(1, 0x10)
        self.right_board = DFRobot_DC_Motor_IIC(1, 0x12)

        self.initialise_boards(self.left_board, self.right_board)

        # store previous speed values
        self.prev_v_front_left = 0
        self.prev_v_back_left = 0
        self.prev_v_front_right = 0 
        self.prev_v_back_right = 0

        self.acceleration_threshold = 2

        # subscribe to velocity cmds
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

    def initialise_boards(self, left_board, right_board):

        for board in (left_board, right_board):

            while board.begin() != board.STA_OK:    # Board begin and check board status
                self.print_board_status(board)
                print("board begin failed")
                time.sleep(2)

            board.set_encoder_enable(board.ALL)
            board.set_moter_pwm_frequency(1000)

            print("board begin success")
            return True
        
        return False

    def print_board_status(self, board):

        if board.last_operate_status == board.STA_OK:
            print("board status: everything ok")
        elif board.last_operate_status == board.STA_ERR:
            print("board status: unexpected error")
        elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
            print("board status: device not detected")
        elif board.last_operate_status == board.STA_ERR_PARAMETER:
            print("board status: parameter error, last operate no effective")
        elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
            print("board status: unsupport board framware version")
        
    def listener_callback(self, msg):

        velocities = self.calculate_motor_velocities(msg)

        self.drive_front_left(velocities[0])
        self.drive_back_left(velocities[1])
        self.drive_front_right(velocities[2])
        self.drive_back_right(velocities[3])

    def calculate_motor_velocities(self, msg):

        x_cmd = msg.linear.x
        z_cmd = msg.angular.z
        
        # constants
        speed_multiplier = 10 # percent of max velocity
        turn_threshold = 0.2
        turn_component = 0

        # set turn component based on a threshold value
        if abs(z_cmd) > turn_threshold:

            if z_cmd > 0:
                turn_component = z_cmd - turn_threshold
            
            else:
                turn_component = z_cmd + turn_threshold

        # left wheels
        v_front_left = (0.5*x_cmd - 0.5*turn_component)*speed_multiplier
        v_back_left = (0.5*x_cmd - 0.5*turn_component)*speed_multiplier

        # self.prev_v_front_left = v_front_left
        # self.prev_v_back_left = v_front_left

        # right wheels
        v_front_right = (-0.5*x_cmd - 0.5*turn_component)*speed_multiplier
        v_back_right = (-0.5*x_cmd - 0.5*turn_component)*speed_multiplier

    
        # self.prev_v_front_left = v_front_left
        # self.prev_v_back_left = v_front_left

        return v_front_left, v_back_left, v_front_right, v_back_right

    def drive_front_left(self, vel):

        board = self.left_board

        if vel > 0:
            board.motor_movement([board.M1], board.CCW, vel)

        else:
            board.motor_movement([board.M1], board.CW, abs(vel))

    def drive_back_left(self, vel):
        
        board = self.left_board

        if vel > 0:
            board.motor_movement([board.M2], board.CCW, vel)

        else:
            board.motor_movement([board.M2], board.CW, abs(vel))

    def drive_front_right(self, vel):
        
        board = self.right_board

        if vel > 0:
            board.motor_movement([board.M1], board.CW, vel)

        else:
            board.motor_movement([board.M1], board.CCW, abs(vel))

    def drive_back_right(self, vel):
        
        board = self.right_board

        if vel > 0:
            board.motor_movement([board.M2], board.CW, vel)

        else:
            board.motor_movement([board.M2], board.CCW, abs(vel))


def main(args=None):

    rclpy.init(args=args)
    
    left_wheel_board = WheelMotorDrive()
    right_wheel_board = WheelMotorDrive()
    
    try:
        rclpy.spin(left_wheel_board)
        rclpy.spin(right_wheel_board)

    except KeyboardInterrupt:
        pass

    left_wheel_board.destroy_node()
    right_wheel_board.destroy_node()

    rclpy.shutdown()




