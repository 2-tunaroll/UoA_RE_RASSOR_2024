from rclpy.node import Node
from std_msgs.msg import Int16
import rclpy
from DFRobot_DC_Motor_singleton import DFRobot_DC_Motor_Singleton

class VibratingRollerDrive(Node):
    def __init__(self):

        super().__init__('vibrating_roller_drive')

        # get instance of board
        self.board = DFRobot_DC_Motor_Singleton.get_instance()

        # Call the initialization method
        self.board.initialise_board(self.board)

        # create subscription to corresponding topic
        self.subscription = self.create_subscription(Int16, 'vibrating_motor_cmd', self.listener_callback, 10)

    def listener_callback(self, msg):

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




