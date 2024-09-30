from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='re_rassor_controller',
            executable='drive',
            name='drive'),
        Node(
            package='re_rassor_controller',
            executable='tool_interchange',
            name='tool_interchange'),
        Node(
            package='re_rassor_controller',
            executable='t_joint',
            name='t_joint'),
        # Node(
        #     package='re_rassor_controller',
        #     executable='tool_control',
        #     name='tool_control'),
  ])