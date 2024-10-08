from setuptools import find_packages, setup
import os
from glob import glob

package_name = 're_rassor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teresavkelly',
    maintainer_email='teresavkelly@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['controller = re_rassor_controller.ps4_controller_node:main',
                            'controller_state_monitor = re_rassor_controller.controller_state_monitor_node:main',
                            'drive = re_rassor_controller.roboclaw_drive_node:main',
                            'tool_interchange = re_rassor_controller.tool_interchange_node:main',
                            't_joint = re_rassor_controller.t_joint_node:main',
                            'tool_control = re_rassor_controller.tool_control_node:main',
                            'test = re_rassor_controller.test_control_node:main'
        ],
    },
)
