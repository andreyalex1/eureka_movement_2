from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_movement_2',
            executable='ackermann',
            name='ackermann',
            shell=True,
        ),
        Node(
            package='eureka_movement_2',
            executable='usb_movement',
            name='usb_movement',
            shell=True,
        ),
    ])