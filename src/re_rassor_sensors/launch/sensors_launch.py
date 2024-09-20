
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='re_rassor_sensors',
            executable='current_and_power',
            name='current_and_power'),
        Node(
            package='re_rassor_sensors',
            executable='accelerometer',
            name='accelerometer'),
        Node(
            package='re_rassor_sensors',
            executable='external_temperature',
            name='external_temperature'),
        Node(
            package='re_rassor_sensors',
            executable='internal_temperature',
            name='internal_temperature'),
  ])