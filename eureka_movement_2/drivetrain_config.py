#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.node import Node


lin_vel_gain = 1
ang_vel_gain = .6

wheel_diameter = .2

class config(Node):
    def __init__(self):
        super().__init__('drivetrain_config')
        self.pub = self.create_publisher(UInt8MultiArray, 'can_tx', 10)
        self.sub = self.create_subscription(JointState, "drivetrain_config", self.callback, 10)
        self.control_mode = 1
        self.power_saving = 0
        self.heartbeat = 1
    def __del__(self):
        self.get_logger().info("Drivetrain_Config Killed!")

    def send(self):
        msg = UInt8MultiArray()
        arr = np.array([self.control_mode, self.power_saving, self.heartbeat], dtype = np.uint8)
        data = bytes([10]) + arr.tobytes()
        msg.data = data
        print(msg.data)
        self.pub.publish(msg)

    def callback(self,data):
        names = data.name
        self.control_mode = data.position[names.index("control_mode")]
        self.power_saving = data.position[names.index("power_saving")]
        self.heartbeat = data.position[names.index("heartbeat")]
        self.send()




def main(args=None):
    rclpy.init()
    ack = config()
    rclpy.spin(ack)

    
    ack.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()