from setuptools import find_packages, setup

package_name = 're_rassor_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
                            "temperature = re_rassor_sensors.temperature_node:main"
        ],
    },
)
