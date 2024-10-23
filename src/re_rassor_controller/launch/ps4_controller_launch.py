from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='re_rassor_controller',
            executable='controller',
            name='controller'),
        Node(
            package='re_rassor_controller',
            executable='controller_state_monitor',
            name='controller_state_monitor'),
  ])