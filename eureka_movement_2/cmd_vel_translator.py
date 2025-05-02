#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from math import atan, sqrt, pi
import rclpy
from rclpy.node import Node


lin_vel_gain = 1
ang_vel_gain = .6

wheel_diameter = .2

class translator(Node):
    def __init__(self):
        super().__init__('cmd_vel_translator')
        self.sub = self.create_subscription(Twist, "custom_cmd_vel", self.callback, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel",  10)
        self.get_logger().info("cmd_vel_translator Started!")
    def __del__(self):
        self.get_logger().info("cmd_vel_translator Killed!")
    def callback(self,data):
        msg = Twist()
        if( -0.02 < data.linear.x < 0.02):
            msg.linear.x = 0.0
        else:
            msg.linear.x = float((np.sign(data.linear.x) * np.clip(abs(data.linear.x), 0.4, 0.7)))
        msg.angular.z = data.angular.z
  #      if(abs(data.linear.x) > 0.02):
  #          msg.angular.z = float(float(data.angular.z) / float(data.linear.x) * 0.6)
  #      else:
   #         None
            #msg.angular.z = 100.0
        self.pub.publish(msg)




def main(args=None):
    rclpy.init()
    trn = translator()
    rclpy.spin(trn)

    
    trn.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()