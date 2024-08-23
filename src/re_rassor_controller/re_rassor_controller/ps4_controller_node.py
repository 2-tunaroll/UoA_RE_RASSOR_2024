import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray

from geometry_msgs.msg import Twist
import json
import re_rassor_controller.lib.controller_input_defs as inputs
import time

class JsonPublisher(Node):
    def __init__(self):

        super().__init__('json_publisher')

        self.publisher_ = self.create_publisher(String, 'controller_state', 100)

        self.velocity_publisher_ = self.create_publisher(Twist, 'cmd_vel', 100)
        self.t_joint_publisher_ = self.create_publisher(Int32MultiArray, 't_joint_cmd', 100)
        # self.speed_mode_publisher = self.create_publisher(Float32, 'speed_mode')

        self.debounce_time = 0.5 #seconds
        self.circle_last_pressed_time = 0 
        self.t_joint_selection = 'FRONT'


        self.t_joint_msg = Int32MultiArray()
        self.t_joint_msg.data = [0, 0, 0]


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

                                    controller_msg = String()
                                    controller_msg.data = msg_data
                                    self.publisher_.publish(controller_msg)
                                    
                                    data_array = json.loads(msg_data)

                                    # convert raw json strings to meaningful commands
                                    self.get_robot_commands(data_array)
                                    self.get_t_joint_commands(data_array)

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
        
    def get_robot_commands(self, data):

        velocity_msg = Twist()

        # must be pressing L2 and R2 to deliver power
        if data['axes'][inputs.RIGHT_TRIGGER] > -0.95 and data['axes'][inputs.LEFT_TRIGGER] > -0.95:

            # velocity: forward and back
            if data['axes'][inputs.LEFT_JOY_VERTICAL] > 0.05 or data['axes'][inputs.LEFT_JOY_VERTICAL] < -0.05:
                velocity_msg.linear.x = data['axes'][inputs.LEFT_JOY_VERTICAL]

            # velocity: left and right
            if (data['axes'][inputs.LEFT_JOY_HORIZONTAL] > 0.05 or data['axes'][inputs.LEFT_JOY_HORIZONTAL] < -0.05):
                velocity_msg.angular.z = data['axes'][inputs.LEFT_JOY_HORIZONTAL]

        self.velocity_publisher_.publish(velocity_msg)

    def get_t_joint_commands(self, data):

        current_time = time.time()
        debounce_time = 0.2 # seconds

        if (data['buttons'][inputs.CIRCLE] == 1) and (current_time - self.circle_last_pressed_time > debounce_time):

            self.circle_last_pressed_time = current_time

            if self.t_joint_msg.data[0] == 0:
                self.t_joint_msg.data[0] = 1 # front t-joint

            else:
                self.t_joint_msg.data[0] = 0 # back t-joint

        # must be pressing L2 and R2 to deliver power
        if data['axes'][inputs.RIGHT_TRIGGER] > -0.95 and data['axes'][inputs.LEFT_TRIGGER] > -0.95:
            
            self.t_joint_msg.data[1] = data['buttons'][inputs.L1] # up
            self.t_joint_msg.data[2] = data['buttons'][inputs.R1] # down
        
        self.t_joint_publisher_.publish(self.t_joint_msg)
        
    def get_tool_commands(self, data):
        
        current_time = time.time()
        debounce_time = 0.2 # seconds

        # tool interchange
        if (data['buttons'][inputs.CROSS] == 1) and (current_time - self.circle_last_pressed_time > debounce_time):
            self.tool_interchange_msg = data['buttons'][inputs.CROSS]

        # bucket drum
        # must be pressing L2 and R2 to deliver power
        if data['axes'][inputs.RIGHT_TRIGGER] > -0.95 and data['axes'][inputs.LEFT_TRIGGER] > -0.95:
            
            self.bucket_drum_msg.data[0] = data['buttons'][inputs.UP] # up
            self.bucket_drum_msg.data[1] = data['buttons'][inputs.DOWN] # down
        

        # self.t_joint_publisher_.publish(self.t_joint_msg)
        





def main(args=None):
    rclpy.init(args=args)
    node = JsonPublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
