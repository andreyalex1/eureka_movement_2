from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_movement_2',
            executable='ackermann_2',
            name='ackermann_2',
            shell=True,
        ),
        Node(
            package='eureka_movement_2',
            executable='usb_movement_2',
            name='usb_movement_2',
            shell=True,
        ),
    ])