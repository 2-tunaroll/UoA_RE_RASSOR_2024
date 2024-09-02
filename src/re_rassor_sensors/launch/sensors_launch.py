import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='re_rassor_sensors',
            executable='accelerometer',
            name='accelerometer'
        ),
        launch_ros.actions.Node(
            package='re_rassor_sensors',
            executable='temperature',
            name='temperature_sensor'
        ),
    ])