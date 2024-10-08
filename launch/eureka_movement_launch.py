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
            executable='wheel_decoder',
            name='wheel_decoder',
            shell=True,
        ),
         Node(
            package='eureka_movement_2',
            executable='drivetrain_config',
            name='drivetrain_config',
            shell=True,
        ),
    ])