#!/usr/bin/env python3.10

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import threading
import rclpy
from rclpy.node import Node
import math

class wheel_decoder(Node):
    def __init__(self):
        super().__init__('wheel_decoder')
        self.velocities = [0.] * 6
        self.efforts = [0.] * 6
        self.positions = [0.] * 6
        self.update - [0] * 6
        self.pub = self.create_publisher(JointState,'wheel_states', 10)
        self.sub = self.sub = self.create_subscription( UInt8MultiArray, "can_rx", self.callback, 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publisher)
        self.get_logger().info("DC_Decoder Started!")
    def __del__(self):
        self.get_logger().info("DC_Decoder Killed!")
    def callback(self, arr):
        index = int(arr.data[0])
        if( 10 < index < 17):
            temp = np.frombuffer(arr.data[5:7], dtype=np.float16)[0]
            if(math.isnan(temp)):
                temp = 0
            self.positions[index - 11] = float(temp)
            self.velocities[index - 11] = float(np.frombuffer(arr.data[1:3], dtype=np.float16)[0])
            self.efforts[index - 11] = float(np.frombuffer(arr.data[3:5], dtype=np.float16)[0])
            self.update[index - 11] = 1
    def publisher(self):
        if(self.update == [1,1,1,1,1,1]):
            self.update = [0] * 6
            message = JointState()
            message.name = ['DC1','DC2','DC3','DC4','DC5','DC6']
    #      print(self.velocities)
            message.header.stamp = self.get_clock().now().to_msg()
            message.velocity = self.velocities
            message.effort = self.efforts
            message.position = self.positions
            self.pub.publish(message)

def main(args=None):
    rclpy.init()
    wd = wheel_decoder()
    rclpy.spin(wd)

    
    wd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()