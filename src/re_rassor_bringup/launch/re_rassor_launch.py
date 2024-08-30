import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='re_rassor_sensors',
            executable='accelerometer',
            name='accelerometer'),
        launch_ros.actions.Node(
            package='re_rassor_sensors',
            executable='internal_temperature',
            name='internal_temperature'),
        launch_ros.actions.Node(
            package='re_rassor_sensors',
            executable='external_temperature',
            name='external_temperature'),
  ])

