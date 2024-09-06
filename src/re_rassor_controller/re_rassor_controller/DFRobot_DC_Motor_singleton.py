from re_rassor_controller.lib.DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC
import time

class DFRobot_DC_Motor_Singleton():

    _instance = None
    _is_initialised = False

    @staticmethod
    def get_instance():
        if DFRobot_DC_Motor_Singleton._instance is None:
            # CHANGE ADDRESS TO 12
            DFRobot_DC_Motor_Singleton._instance = DFRobot_DC_Motor_IIC(1, 0x10)

        return DFRobot_DC_Motor_Singleton._instance
    
    def initialise_board(self, board):
        # Only initialize if it hasn't been done already
        if not DFRobot_DC_Motor_Singleton._is_initialised:
            
            while board.begin() != board.STA_OK:  # Assuming board has begin() method and status constants
                self.print_board_status(board)  # Add logic to print board status if needed
                print("Board initialization failed, retrying...")
                time.sleep(2)

            board.set_moter_pwm_frequency(1000)  # Adjust motor PWM frequency to 1000 Hz
            print("Board initialized successfully")
            DFRobot_DC_Motor_Singleton._is_initialised = True  # Set the flag to prevent re-initialization
        else:
            print("Board already initialized")
        return DFRobot_DC_Motor_Singleton._is_initialised
    
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