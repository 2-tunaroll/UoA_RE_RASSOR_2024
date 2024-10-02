import rclpy
from rclpy.node import Node
from time import sleep
from custom_msgs.msg import BucketDrum
from std_msgs.msg import Bool, Int16
from re_rassor_controller.lib.DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC
import time

class ToolControlNode(Node):
    def __init__(self):

        super().__init__('tool_control_node')

        # get instance of board
        self.board = DFRobot_DC_Motor_IIC(1, 0x10)

        # update time
        self.dt = 0.2
        self.bucket_last_called_time = time.time()
        self.roller_last_called_time = time.time()

        # set shutdown flag
        self.SHUT_DOWN = False

        # Call the initialisation method
        self.initialise_board(self.board, "12. tool control")

        self.subscription_1 = self.create_subscription(Bool, 'shutdown_cmd', self.shutdown_callback, 10)
        self.subscription_2 = self.create_subscription(BucketDrum, 'bucket_drum_cmd', self.bucket_drum_callback, 10)
        self.subscription_3 = self.create_subscription(Int16, 'vibrating_motor_cmd', self.vibrating_roller_callback, 10)

    def shutdown_callback(self, msg):
        # sets the shutdown flag to true if the current sensing chip detects a current spike
        if msg.data:
            self.SHUT_DOWN = True

    def initialise_board(self, id):

        board = self.board

        while board.begin() != board.STA_OK:    # Board begin and check board status
            self.print_board_status(board)
            print(f"{id} board begin failed")
            time.sleep(2)

        board.set_encoder_enable(board.ALL)
        board.set_moter_pwm_frequency(1000)

        print(f"{id} board begin success")

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

    def vibrating_roller_callback(self, msg):

        board = self.board

        if self.SHUT_DOWN:
            board.motor_stop(board.ALL)
            return

        current_time = time.time()
        # only send commands every time interval
        if (current_time - self.roller_last_called_time) > self.dt:
            
            if msg.data == 1:
                # 10% duty cycle is placeholder for now
                board.motor_movement([board.M1], board.CW, 10)
            
            else:
                board.motor_stop(board.M1)
            
            self.roller_last_called_time = current_time
      
    def bucket_drum_callback(self, msg):

        board = self.board

        if self.SHUT_DOWN:
            board.motor_stop(board.ALL)
            return

        current_time = time.time()
        # only send commands every time interval
        if (current_time - self.bucket_last_called_time) > self.dt:
        
            if msg.forward == 1:
                # have to verify directions
                # 10% duty cycle is placeholder for now
                board.motor_movement([board.M2], board.CW, 10)
            elif msg.backward == 1:
                board.motor_movement([board.M2], board.CCW, 10)
            else:
                board.motor_stop(board.M2)

            self.bucket_last_called_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ToolControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()