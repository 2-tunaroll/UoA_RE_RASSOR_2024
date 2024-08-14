import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class JsonPublisher(Node):
    def __init__(self):
        super().__init__('json_publisher')
        self.publisher_ = self.create_publisher(String, 'controller_state', 10)
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
                                    ros_msg = String()
                                    ros_msg.data = msg_data
                                    self.publisher_.publish(ros_msg)
                                    self.get_logger().info(f'Published raw JSON: {ros_msg.data}')

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

def main(args=None):
    rclpy.init(args=args)
    node = JsonPublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
