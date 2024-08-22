import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import json
import re_rassor_controller.lib.controller_input_defs as inputs

class JsonPublisher(Node):
    def __init__(self):

        super().__init__('json_publisher')

        self.publisher_ = self.create_publisher(String, 'controller_state', 100)

        self.velocity_publisher_ = self.create_publisher(Twist, 'cmd_vel', 100)
        # self.speed_mode_publisher = self.create_publisher(Float32, 'speed_mode')

        self.receive_data()

    def receive_data(self):
        # Set the IP address and port for the server
        server_ip = '0.0.0.0'  # Listen on all available network interfaces
        server_port = 8000  # Choose a port number that is not in use

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind((server_ip, server_port))
            server_socket.listen(1)  # Listen for incoming connections
            self.get_logger().info('Server listening on port 8000...')

            while True:  # Keep the server running to accept multiple connections
                try:
                    client_socket, addr = server_socket.accept()
                    self.get_logger().info(f'Connection established with {addr}')
                    buffer = b""

                    with client_socket:
                        while True:  # Continuously receive data from the client
                            chunk = client_socket.recv(1024)
                            if not chunk:
                                break
                            buffer += chunk

                            # Process the buffer for complete JSON strings
                            while b'\n' in buffer:
                                msg_data, buffer = self.extract_json(buffer)
                                if msg_data:

                                    data_array = json.loads(msg_data)
                    
                                    # self.get_velocity_commands(msg_data)
                                    # ros_msg = String()
                                    # ros_msg.data = msg_data
                                    # self.publisher_.publish(ros_msg)
                                    # self.get_velocity_commands(ros_msg.data)
                                    self.get_velocity_commands(data_array)

                                    # self.get_logger().info(f'Published raw JSON: {ros_msg.data}')

                except socket.error as e:
                    self.get_logger().error(f'Socket error: {e}')
                except Exception as e:
                    self.get_logger().error(f'Unexpected error: {e}')

    def extract_json(self, buffer):
        """Extracts and returns a complete JSON string from the buffer."""
        parts = buffer.split(b'\n', 1)
        if len(parts) > 1:
            complete_json = parts[0].decode('utf-8').strip()
            remaining_buffer = parts[1]
            return complete_json, remaining_buffer
        else:
            return None, buffer
        
    def get_velocity_commands(self, data):

        velocity_msg = Twist()

        # must be pressing L2 and R2 to deliver power
        if data['axes'][inputs.RIGHT_TRIGGER] > -0.95 and data['axes'][inputs.LEFT_TRIGGER] > -0.95:

            # forward and back
            if data['axes'][inputs.LEFT_JOY_VERTICAL] > 0.05 or data['axes'][inputs.LEFT_JOY_VERTICAL] < -0.05:
                velocity_msg.linear.x = data['axes'][inputs.LEFT_JOY_VERTICAL]

            # left and right
            if (data['axes'][inputs.LEFT_JOY_HORIZONTAL] > 0.05 or data['axes'][inputs.LEFT_JOY_HORIZONTAL] < -0.05):
                velocity_msg.angular.z = data['axes'][inputs.LEFT_JOY_HORIZONTAL]

        self.velocity_publisher_.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JsonPublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
