from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from custom_msgs.msg import WheelSpeeds
import rclpy
import time
from math import pi
import atexit

from numpy import sign

from re_rassor_controller.lib.DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC

SPEED_CONVERSION = 0.11*2*pi/60

class WheelMotorDrive(Node):
    def __init__(self):

        super().__init__('wheel_drive')

        # motor controller boards
        self.left_board = DFRobot_DC_Motor_IIC(1, 0x10)
        self.right_board = DFRobot_DC_Motor_IIC(1, 0x12)

        # initialise the boards
        self.initialise_board(self.left_board, "10. left wheels")
        self.initialise_board(self.right_board, "11. right wheels")

        self.motor_shutdown()

        # shutdown flag
        self.SHUT_DOWN = False

        # speed multiplier: initialise to 25%
        self.speed_multiplier = 25

        # wheel speeds topic
        self.wheel_speeds = WheelSpeeds()

        # store previous speed values
        self.motor_input_front_left = 0
        self.motor_input_back_left = 0
        self.motor_input_front_right = 0
        self.motor_input_back_right = 0

        # proportional gain
        self.k_p = 100

        # update time
        self.dt = 0.01

        self.last_called_time = time.time()

        # subscribe to current sensing command
        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        # subscribe to velocity cmds
        self.subscription_2 = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        # subscribe to speed mode
        self.subscription_3 = self.create_subscription(Float32, 'speed_mode', self.speed_mode_callback, 10)
        # publish wheel velocities
        self.speed_publisher_ = self.create_publisher(WheelSpeeds, 'wheel_speeds', 100)

    def initialise_board(self, board, id):

        while board.begin() != board.STA_OK:    # Board begin and check board status
            self.print_board_status(board)
            print(f"{id} board begin failed")
            time.sleep(2)
      
        board.set_encoder_enable(board.ALL)
        board.set_encoder_reduction_ratio(board.ALL, 90)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8
        board.set_moter_pwm_frequency(1000)

        print(f"{id} board begin success")

    def shutdown_callback(self, msg):

        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.SHUT_DOWN = True

    def speed_mode_callback(self, msg):

        # sets the speed multipler for driving
        self.speed_multiplier = msg.data

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

        if self.SHUT_DOWN:
            self.motor_shutdown()
            return
 
        current_time = time.time()

        # only send commands every 0.2 s
        if (current_time - self.last_called_time) > self.dt:

            self.calculate_motor_velocities(msg)
            self.last_called_time = current_time
            
            self.drive_front_left(self.motor_input_front_left)
            self.drive_back_left(self.motor_input_back_left)
            self.drive_front_right(self.motor_input_front_right)
            self.drive_back_right(self.motor_input_back_right)

    def calculate_motor_velocities(self, msg):

        x_cmd = msg.linear.x
        z_cmd = msg.angular.z
        
        # constants
        turn_threshold = 0.2
        turn_component = 0

        # set turn component based on a threshold value
        if abs(z_cmd) > turn_threshold:

            if z_cmd > 0:
                turn_component = z_cmd - turn_threshold
            
            else:
                turn_component = z_cmd + turn_threshold

        # calculate the target speeds, in m/s
        # left wheels 
        target_v_front_left = (-1*x_cmd + turn_component)*self.speed_multiplier*0.01
        target_v_back_left = (-1*x_cmd + turn_component)*self.speed_multiplier*0.01

        # right wheels
        target_v_front_right = (-1*x_cmd - turn_component)*self.speed_multiplier*0.01
        target_v_back_right = (-1*x_cmd - turn_component)*self.speed_multiplier*0.01

        # print(f"target FL: {target_v_front_left}, target BL: {target_v_back_left}, target FR: {target_v_front_right}, target BR: {target_v_back_right}")

        # get actual speeds from encoders
        self.get_velocity_feedback()

        # implement proportional control based on current actual speed
        delta_front_left = target_v_front_left - self.wheel_speeds.front_left
        delta_back_left = target_v_back_left - self.wheel_speeds.back_left
        delta_front_right = target_v_front_right - self.wheel_speeds.front_right
        delta_back_right = target_v_back_right - self.wheel_speeds.back_right
        # print(f"delta FL: {delta_front_left}, delta BL: {delta_back_left}, delta FR: {delta_front_right}, delta BR: {delta_back_right}")


        front_left_signal_response = self.apply_proportional_control(delta_front_left)
        back_left_signal_response = self.apply_proportional_control(delta_back_left)
        front_right_signal_response = self.apply_proportional_control(delta_front_right)
        back_right_signal_response = self.apply_proportional_control(delta_back_right)
       
        if abs(front_left_signal_response) < 5:
            front_left_signal_response = 5 * sign(front_left_signal_response)
        if abs(back_left_signal_response) < 5:
            back_left_signal_response = 5 * sign(back_left_signal_response)
        if abs(front_right_signal_response) < 5:
            front_right_signal_response = 5 * sign(front_right_signal_response)
        if abs(back_right_signal_response) < 5:
            back_right_signal_response = 5 * sign(back_right_signal_response)

        # ease the speeds
        self.motor_input_front_left = self.ease_speed(front_left_signal_response, self.motor_input_front_left)
        self.motor_input_back_left = self.ease_speed(back_left_signal_response, self.motor_input_back_left)
        self.motor_input_front_right = self.ease_speed(front_right_signal_response, self.motor_input_front_right)
        self.motor_input_back_right = self.ease_speed(back_right_signal_response, self.motor_input_back_right)    

        # print(f"input FL: {self.motor_input_front_left}, input BL: {self.motor_input_back_left}, input FR: {self.motor_input_front_right}, target BR: {self.motor_input_back_right}")
        print(f"LEFT FRONT: v_des: {target_v_front_left}, v: {self.wheel_speeds.front_left}, deltav: {delta_front_left}, input: {self.motor_input_front_left}")
        print(f"LEFT BACK: v_des: {target_v_back_left}, v: {self.wheel_speeds.back_left}, deltav: {delta_back_left}, input: {self.motor_input_back_left}")
        print(f"RIGHT FRONT: v_des: {target_v_front_right}, v: {self.wheel_speeds.front_right}, deltav: {delta_front_right}, input: {self.motor_input_front_right}")
        print(f"RIGHT BACK: v_des: {target_v_back_right}, v: {self.wheel_speeds.back_right}, deltav: {delta_back_right}, input: {self.motor_input_back_right}")

        
    def apply_proportional_control(self, delta):

        motor_signal = self.k_p * delta

        if (motor_signal < -80):
            motor_signal = -80
        if (motor_signal > 80):
            motor_signal = 80

        return motor_signal

    def ease_speed(self, new_speed, prev_speed):

        max_delta = 80 * self.dt

        if abs(new_speed - prev_speed) >= max_delta:

            if new_speed > prev_speed:
                # speeding up
                corrected_speed = prev_speed + max_delta

            else:
                # slowing down
                corrected_speed = prev_speed - max_delta

        else:
            corrected_speed = new_speed

        # update the speed and return
        return corrected_speed

    def drive_front_left(self, vel):

        board = self.left_board

        if vel > 0:
            board.motor_movement([board.M1], board.CW, vel)

        else:
            board.motor_movement([board.M1], board.CCW, abs(vel))

        # time.sleep(0.25)
        # speed = board.get_encoder_speed(board.M1)
        # self.wheel_speeds.front_left = SPEED_CONVERSION*speed

    def drive_back_left(self, vel):
        
        board = self.left_board

        if vel > 0:
            board.motor_movement([board.M2], board.CW, vel)

        else:
            board.motor_movement([board.M2], board.CCW, abs(vel))

        # time.sleep(0.25)
        # speed = board.get_encoder_speed(board.M2)
        # self.wheel_speeds.back_left = SPEED_CONVERSION*speed

    def drive_front_right(self, vel):
        
        board = self.right_board

        if vel > 0:
            board.motor_movement([board.M1], board.CCW, vel)

        else:
            board.motor_movement([board.M1], board.CW, abs(vel))

        # time.sleep(0.25)
        # speed = board.get_encoder_speed(board.M1)
        # self.wheel_speeds.front_right = SPEED_CONVERSION*speed

    def drive_back_right(self, vel):
        
        board = self.right_board

        if vel > 0:
            board.motor_movement([board.M2], board.CCW, vel)

        else:
            board.motor_movement([board.M2], board.CW, abs(vel))

        # time.sleep(0.25)
        # speed = board.get_encoder_speed(board.M2)
        # self.wheel_speeds.back_right = SPEED_CONVERSION*speed

    def get_velocity_feedback(self):

        left_board = self.left_board
        right_board = self.right_board

        left_front_speed, left_back_speed = left_board.get_encoder_speed(left_board.ALL)
        right_front_speed, right_back_speed = right_board.get_encoder_speed(right_board.ALL)

        right_front_speed = -1*right_front_speed
        right_back_speed = -1*right_back_speed

        # print(f"fl encoder: {left_front_speed}, bl encoder: {left_back_speed},fr encoder: {right_front_speed}, br encoder: {right_back_speed}")

        # convert from rpm to m/s
        speed_m_sec = lambda speed_rpm: 0.11*speed_rpm*2*pi/60

        self.wheel_speeds.front_left = speed_m_sec(left_front_speed)
        self.wheel_speeds.back_left = speed_m_sec(left_back_speed)
        self.wheel_speeds.front_right = speed_m_sec(right_front_speed)
        self.wheel_speeds.back_right = speed_m_sec(right_back_speed)

        # print(f"FL: {self.wheel_speeds.front_left}, BL: {self.wheel_speeds.back_left}, FR: {self.wheel_speeds.front_right}, BR: {self.wheel_speeds.back_right}")

        self.speed_publisher_.publish(self.wheel_speeds)

    def motor_shutdown(self):
        self.left_board.motor_stop(self.left_board.ALL)
        self.right_board.motor_stop(self.right_board.ALL)

def main(args=None):

    rclpy.init(args=args)
    
    node = WheelMotorDrive()

    atexit.register(node.motor_shutdown)
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()