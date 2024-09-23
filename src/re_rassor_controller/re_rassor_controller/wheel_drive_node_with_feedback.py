from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from custom_msgs.msg import WheelSpeeds
import rclpy
import time
from math import pi

from re_rassor_controller.lib.DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC

class WheelMotorDrive(Node):
    def __init__(self):

        super().__init__('wheel_drive')

        # motor controller boards
        self.left_board = DFRobot_DC_Motor_IIC(1, 0x10)
        self.right_board = DFRobot_DC_Motor_IIC(1, 0x11)

        # initialise the boards
        self.initialise_board(self.left_board, "10. left wheels")
        self.initialise_board(self.right_board, "11. right wheels")

        # shutdown flag
        self.SHUT_DOWN = False

        # speed multiplier: initialise to 25%
        self.speed_multiplier = 25

        # wheel speeds topic
        self.wheel_speeds = WheelSpeeds()

        # store previous speed values
        self.v_front_left = 0
        self.v_back_left = 0
        self.v_front_right = 0
        self.v_back_right = 0

        # proportional gain
        self.k_p = 150

        # update time
        self.dt = 0.2

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
            self.left_board.motor_stop(self.left_board.ALL)
            self.right_board.motor_stop(self.right_board.ALL)
            return
        
        current_time = time.time()

        # only send commands every 0.2 s
        if (current_time - self.last_called_time) > self.dt:

            self.v_front_left, self.v_back_left, self.v_front_right, self.v_back_right = self.calculate_motor_velocities(msg)
            self.last_called_time = current_time
            
            self.drive_front_left(self.v_front_left)
            self.drive_back_left(self.v_back_left)
            self.drive_front_right(self.v_front_right)
            self.drive_back_right(self.v_back_right)

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
        target_v_front_left = (-0.5*x_cmd + 0.5*turn_component)*self.speed_multiplier*0.01
        target_v_back_left = (-0.5*x_cmd + 0.5*turn_component)*self.speed_multiplier*0.01

        # right wheels
        target_v_front_right = (-0.5*x_cmd - 0.5*turn_component)*self.speed_multiplier*0.01
        target_v_back_right = (-0.5*x_cmd - 0.5*turn_component)*self.speed_multiplier*0.01

        # get actual speeds from encoders
        self.get_velocity_feedback()

        # implement proportional control based on current actual speed
        delta_front_left = target_v_front_left - self.wheel_speeds.front_left
        delta_back_left = target_v_back_left - self.wheel_speeds.back_left
        delta_front_right = target_v_front_right - self.wheel_speeds.front_right
        delta_back_right = target_v_back_right - self.wheel_speeds.back_right

        front_left_signal_response = self.apply_proportional_control(delta_front_left)
        back_left_signal_response = self.apply_proportional_control(delta_back_left)
        front_right_signal_response = self.apply_proportional_control(delta_front_right)
        back_right_signal_response = self.apply_proportional_control(delta_back_right)
       
        # ease the speeds
        self.v_front_left = self.ease_speed(front_left_signal_response, self.v_front_left)
        self.v_back_left = self.ease_speed(back_left_signal_response, self.v_back_left)
        self.v_front_right = self.ease_speed(front_right_signal_response, self.v_front_right)
        self.v_back_right = self.ease_speed(back_right_signal_response, self.v_back_right)

        return self.v_front_left, self.v_back_left, self.v_front_right, self.v_back_right
    
    def apply_proportional_control(self, delta):

        motor_signal = self.k_p * delta

        if (motor_signal < -80):
            motor_signal = -80
        if (motor_signal > 80):
            motor_signal = 80

        return motor_signal

    def ease_speed(self, new_speed, prev_speed):

        max_delta = 10 * self.dt

        if abs(new_speed - prev_speed) >= max_delta:

            if new_speed > prev_speed:
                # speeding up
                corrected_speed = prev_speed + max_delta
                print(f"corrected_speed: {corrected_speed}")

            else:
                # slowing down
                corrected_speed = prev_speed - max_delta
                print(f"corrected_speed: {corrected_speed}")

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

    def drive_back_left(self, vel):
        
        board = self.left_board

        if vel > 0:
            board.motor_movement([board.M2], board.CW, vel)

        else:
            board.motor_movement([board.M2], board.CCW, abs(vel))

    def drive_front_right(self, vel):
        
        board = self.right_board

        if vel > 0:
            board.motor_movement([board.M1], board.CCW, vel)

        else:
            board.motor_movement([board.M1], board.CW, abs(vel))

    def drive_back_right(self, vel):
        
        board = self.right_board

        if vel > 0:
            board.motor_movement([board.M2], board.CCW, vel)

        else:
            board.motor_movement([board.M2], board.CW, abs(vel))

    def get_velocity_feedback(self):

        left_board = self.left_board
        right_board = self.right_board

        left_front_speed, left_back_speed = left_board.get_encoder_speed(left_board.ALL)

        right_front_speed, right_back_speed = right_board.get_encoder_speed(right_board.ALL)

        # convert from rpm to m/s
        speed_m_sec = lambda speed_rpm: 0.11*speed_rpm*2*pi/60

        self.wheel_speeds.front_left = speed_m_sec(left_front_speed)
        self.wheel_speeds.back_left = speed_m_sec(left_back_speed)
        self.wheel_speeds.front_right = speed_m_sec(right_front_speed)
        self.wheel_speeds.back_right = speed_m_sec(right_back_speed)

        self.speed_publisher_.publish(self.wheel_speeds)

def main(args=None):

    rclpy.init(args=args)
    
    left_wheel_board = WheelMotorDrive()
    right_wheel_board = WheelMotorDrive()
    
    try:
        rclpy.spin(left_wheel_board)
        rclpy.spin(right_wheel_board)

    except KeyboardInterrupt:
        left_wheel_board.motor_stop(left_wheel_board.ALL)
        right_wheel_board.motor_stop(right_wheel_board.ALL)

    left_wheel_board.destroy_node()
    right_wheel_board.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()