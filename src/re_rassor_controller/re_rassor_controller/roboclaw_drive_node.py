from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16MultiArray, String
from geometry_msgs.msg import Twist
import rclpy
import time
from math import pi
from re_rassor_controller.lib.roboclaw_3 import Roboclaw
import atexit


class RoboClawMotorDrive(Node):
    def __init__(self):

        super().__init__('roboclaw_drive')

        # motor controller boards
        self.addresses = [0x80, 0x81]
        self.roboclaw = Roboclaw("/dev/serial1", 38400)

        # # initialise the boards
        self.roboclaw.Open()

        # set max current draw to 7A per motor
        self.roboclaw.SetM1MaxCurrent(self.addresses[0], 700)
        self.roboclaw.SetM2MaxCurrent(self.addresses[0], 700)
        self.roboclaw.SetM1MaxCurrent(self.addresses[1], 700)
        self.roboclaw.SetM2MaxCurrent(self.addresses[1], 700)

        # shutdown flag
        self.SHUT_DOWN = False

        # drive mode
        self.drive_mode = 'STANDARD'

        # speed multiplier: initialise to 10%
        self.speed_multiplier = 0.1

        # update time
        self.dt = 0.5

        self.last_called_time = time.time()

        # subscribe to current sensing command
        self.shutdown_subscription_ = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        # subscribe to drive mode command
        self.drive_mode_subscription_ = self.create_subscription(Float32, 'drive_mode', self.drive_mode_callback, 10)
        # subscribe to speed mode
        self.speed_mode_subscription_ = self.create_subscription(Float32, 'speed_mode', self.speed_mode_callback, 10)
        # subscribe to wheel selection command
        self.wheel_selection_subscription_ = self.create_subscription(String, 'wheel_selection', self.wheel_selection_callback, 10)
        # subscribe to velocity cmds
        self.drive_cmd_subscription_ = self.create_subscription(Twist, 'cmd_vel', self.drive_cmd_callback, 10)

        # publish wheel speeds
        self.speed_publisher_ = self.create_publisher(Float32MultiArray, 'wheel_speeds', 100)
        self.timer = self.create_timer(1, self.publish_wheel_speeds)

        self.current_publisher_ = self.create_publisher(Int16MultiArray, 'motor_currents', 100)
        self.timer = self.create_timer(1, self.publish_motor_currents)
     
    def shutdown_callback(self, msg):

        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.SHUT_DOWN = True

    def drive_mode_callback(self, msg):

        # subscribes to the drive mode topic to decide which drive mode to use
        self.drive_mode = msg.data

    def speed_mode_callback(self, msg):

        # sets the speed multipler for driving
        self.speed_multiplier = msg.data

    def wheel_selection_callback(self, msg):

        # gets the selected wheel for independent drive mode
        self.wheel_selection = msg.data

    def motor_shutdown(self):

        # sends 0 to each of the motors
        self.roboclaw.ForwardMixed(self.addresses[0], 0)
        self.roboclaw.TurnLeftMixed(self.addresses[0], 0)
        self.roboclaw.ForwardMixed(self.addresses[1], 0)
        self.roboclaw.TurnLeftMixed(self.addresses[1], 0)

    def drive_cmd_callback(self, msg):

        # check shutdown flag
        if self.SHUT_DOWN:
            # stop motors
            self.motor_shutdown()
            return
        
        current_time = time.time()

        # only send commands every 0.5 s
        if (current_time - self.last_called_time) > self.dt:

            # call relevant function based on the selected drive mode
            if self.drive_mode == 'STANDARD':

                # convert the inputs and pass to the drive function:
                # x_cmd is forward input and z_cmd is turn input
                x_cmd = int(msg.linear.x * 127 * self.speed_multiplier)
                z_cmd = int(msg.angular.z * 127 * self.speed_multiplier)
                self.drive_standard(x_cmd, z_cmd)

            else: # independent mode
                # convert the inputs and pass to the drive function:
                # left_cmd is left wheels and right_cmd is right wheels
                left_cmd = int(msg.linear.x * 127 * self.speed_multiplier)
                right_cmd = int(msg.linear.y * 127 * self.speed_multiplier)
                self.drive_independent(left_cmd, right_cmd)

            self.last_called_time = time.time()

    def drive_standard(self, x_cmd, z_cmd):

        # Drives the wheels using the RoboClaw functions in standard mode:
        # front and back wheels are sent the same commands, and the RoboClaw
        # handles turning

        for i in self.addresses:

            if x_cmd <= 0: # drive forward (or stop)
                self.roboclaw.ForwardMixed(i, abs(x_cmd))
            else:
                self.roboclaw.BackwardMixed(i, abs(x_cmd))
            if z_cmd <= 0: # turn left
                self.roboclaw.TurnLeftMixed(i, abs(z_cmd))
            else: # turn right
                self.roboclaw.TurnRightMixed(i, z_cmd)


    def drive_independent(self, left_cmd, right_cmd):

        # Drives the wheels using the RoboClaw functions in independent wheel
        # control mode: drives the front or back wheels forward or back (one at a time)
        # based on controller input

        if self.wheel_selection == 'FRONT':
            # use front wheels RoboClaw
            board = self.addresses[0]
        else: # use back wheels RoboClaw
            board = self.addresses[1]

        # Drive left wheels
        if left_cmd <= 0: # drive forward (or stop)
            self.roboclaw.ForwardM1(board, abs(left_cmd))
        else:
            self.roboclaw.BackwardM1(board, abs(left_cmd))
        # Drive right wheels
        if right_cmd <= 0: # drive forward (or stop)
            self.roboclaw.ForwardM2(board, abs(right_cmd))
        else:
            self.roboclaw.BackwardM2(board, abs(right_cmd))

    def publish_wheel_speeds(self):

        # order is: front left, front right, back left, back right
        wheel_speeds_msg = Float32MultiArray()
        wheel_speeds_msg.data = [0] * 4

        speed_ms = lambda encoder_speed: (encoder_speed/1440) * 2*pi*0.11

        wheel_speeds_msg.data[0] = speed_ms(self.roboclaw.ReadSpeedM1(self.addresses[0])[1])
        wheel_speeds_msg.data[1] = speed_ms(self.roboclaw.ReadSpeedM2(self.addresses[0])[1])
        wheel_speeds_msg.data[2] = speed_ms(self.roboclaw.ReadSpeedM1(self.addresses[1])[1])
        wheel_speeds_msg.data[3] = speed_ms(self.roboclaw.ReadSpeedM2(self.addresses[1])[1])

        print(f"front left: {wheel_speeds_msg.data[0]}")
        print(f"front right: {wheel_speeds_msg.data[1]}")
        print(f"back left: {wheel_speeds_msg.data[2]}")
        print(f"back right: {wheel_speeds_msg.data[3]}")

        self.speed_publisher_.publish(wheel_speeds_msg)

    def publish_motor_currents(self):

        motor_currents = Int16MultiArray()
        motor_currents.data = [0] * 4

        # left front, right front, left back, right back
        motor_currents.data[0] = self.roboclaw.ReadCurrents(self.addresses[0])[1]
        motor_currents.data[1] = self.roboclaw.ReadCurrents(self.addresses[0])[2]
        motor_currents.data[2] = self.roboclaw.ReadCurrents(self.addresses[1])[1]
        motor_currents.data[3] = self.roboclaw.ReadCurrents(self.addresses[1])[2]

        self.current_publisher_.publish(motor_currents)
        
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