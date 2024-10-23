from setuptools import find_packages, setup
import os
from glob import glob

package_name = 're_rassor_sensors'

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
        'console_scripts': ["accelerometer = re_rassor_sensors.accelerometer_node:main",
                            "external_temperature = re_rassor_sensors.external_temperature_node:main",
                            "internal_temperature = re_rassor_sensors.internal_temperature_node:main",
                            "current_and_power = re_rassor_sensors.current_and_power_node:main"
        ],
    },
)
