from rclpy.node import Node
from std_msgs.msg import Int16
import rclpy
import time
from re_rassor_controller.lib.DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC

class VibratingRollerDrive(Node):
    def __init__(self):

        super().__init__('vibrating_roller_drive')

        # initialise and set i2c address
        self.board = DFRobot_DC_Motor_IIC(1, 0x14)
        self.initialise_board(self.board)

        # create subscription to corresponding topic
        self.subscription = self.create_subscription(Int16, 'vibrating_motor_cmd', self.listener_callback, 10)

    def initialise_board(self, board):

        while board.begin() != board.STA_OK:    # Board begin and check board status
            self.print_board_status(board)
            print("board begin failed")
            time.sleep(2)

            # board.set_encoder_enable(board.ALL)
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

        # implement ramping function when turning on/off

        board = self.board
        
        if msg.data == 1:
            # 10% duty cycle is placeholder for now
            board.motor_movement([board.M1], board.CW, 10)

        else:
            board.motor_stop(board.M1)
      
def main(args=None):

    rclpy.init(args=args)
    
    node = VibratingRollerDrive()
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()




