from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'eureka_movement_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, package_name, 'csv'), glob(os.path.join(package_name,'csv', '*.csv'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei Smirnov',
    maintainer_email='andrey040902@gmail.com',
    description='nodes responsible for moving the EUREKA rover accoring to cmd_vel commands',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'ackermann_3 = eureka_movement_2.ackermann_3:main',
                'wheel_decoder = eureka_movement_2.wheel_decoder:main',
                'drivetrain_config = eureka_movement_2.drivetrain_config:main',
                'usb_movement_2 = eureka_movement_2.usb_movement_2:main',
        ],
    },
)
