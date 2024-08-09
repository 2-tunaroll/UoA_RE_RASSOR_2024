import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class PS4ControllerPublisher(Node):
    def __init__(self):
        super().__init__('ps4_controller_publisher')
        self.publisher_ = self.create_publisher(String, 'ps4_controller_topic', 10)

    def publish_controller_input(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerPublisher()

    # Set the IP address and port for the server
    server_ip = '0.0.0.0'  # Listen on all available network interfaces
    server_port = 8000  # Choose a port number that is not in use

    # Set up a socket server to accept incoming connections
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.bind((server_ip, server_port))
        server_socket.listen(1)

        print("Waiting for connection...")
        connection, client_address = server_socket.accept()

        try:
            print("Connection established:", client_address)

            while True:
                data = connection.recv(1024).decode('utf-8')
                if not data:
                    break
                node.publish_controller_input(data)

        except KeyboardInterrupt:
            pass
        finally:
            connection.close()
            print("Connection closed.")
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
