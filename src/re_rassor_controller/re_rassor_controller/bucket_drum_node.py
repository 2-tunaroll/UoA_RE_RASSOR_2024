import rclpy
from rclpy.node import Node
from time import sleep
from custom_msgs.msg import BucketDrum
from DFRobot_DC_Motor_singleton import DFRobot_DC_Motor_Singleton

class BucketDrumNode(Node):
    def __init__(self):

        super().__init__('bucket_drum_node')

        # get instance of board
        self.board = DFRobot_DC_Motor_Singleton.get_instance()

        # Call the initialization method
        self.board.initialise_board(self.board)

        self.subscription = self.create_subscription(BucketDrum, 'bucket_drum_cmd', self.listener_callback, 10)

    def listener_callback(self, msg):

        board = self.board
        
        if msg.forward == 1:
            # have to verify directions
            # 10% duty cycle is placeholder for now
            board.motor_movement([board.M2], board.CW, 10)
        elif msg.backward == 1:
            board.motor_movement([board.M2], board.CCW, 10)
        else:
            board.motor_stop(board.M2)

def main(args=None):
    rclpy.init(args=args)
    node = BucketDrumNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()