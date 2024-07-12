# ps4_controller_node.py

import rclpy
from rclpy.node import Node
import socket
import ast

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')

        # Network setup
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 8000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.listen(1)

        self.get_logger().info('Waiting for connection...')
        self.conn, self.addr = self.sock.accept()
        self.get_logger().info(f'Connected to {self.addr}')
        
        self.run()

    def run(self):
        while rclpy.ok():
            data = self.conn.recv(1024)
            if not data:
                break
            message = ast.literal_eval(data.decode())
            self.get_logger().info(f'Received data: {message}')

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
