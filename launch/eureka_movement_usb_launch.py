from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eureka_movement_2',
            executable='ackermann_3',
            name='ackermann_3',
            shell=True,
            respawn=True,
            respawn_delay=10,
        ),
        Node(
            package='eureka_movement_2',
            executable='usb_movement_2',
            name='usb_movement_2',
            shell=True,
            respawn=True,
            respawn_delay=10,
        ),
    ])