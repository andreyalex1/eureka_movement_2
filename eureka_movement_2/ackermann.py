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
import pandas as pd


lin_vel_gain = 1
ang_vel_gain = 1

wheel_diameter = .2

class ackermann(Node):
    def __init__(self):
        super().__init__('ackermann')
        self.pub = self.create_publisher(UInt8MultiArray, 'can_tx', 10)
        self.sub = self.create_subscription(Twist, "cmd_vel", self.callback, 10)
        self.sub_2 = self.create_subscription(JointState, "wheel_states", self.callback_2, 10)
        self.pub_2 = self.create_publisher(JointState, "wheel_commands", 10)
        self.pub_3 = self.create_publisher(JointState, "dc_commands", 10)
        self.pub_4 = self.create_publisher(JointState, "stepper_commands", 10)
        self.ang_dataframe  = pd.read_csv('/home/eurekanuc/ros2_ws/src/eureka_movement_2/eureka_movement_2/csv/ang_wheel.csv')
        self.vel_dataframe  = pd.read_csv('/home/eurekanuc/ros2_ws/src/eureka_movement_2/eureka_movement_2/csv/vel_wheel.csv')
        self.ang_vel_dataframe  = pd.read_csv('/home/eurekanuc/ros2_ws/src/eureka_movement_2/eureka_movement_2/csv/ang_vel_wheel.csv')
        self.vel_wheel = [0] * 6
        self.vel_wheel_filt = [0] * 6
        self.ang_wheel = [0] * 6
        self.ang_wheel_filt = [0] * 6
        self.ang_vel_wheel = [0] * 6
        self.vel_wheel_temp = [0] * 6
        self.ang_vel_wheel_temp = [0] * 6
        self.l = [.41, 0, .385, .41, 0, .385]
        self.d = [.728, .780, .728, -.728, -.780, -.728]
        self.vel_lin = 0
        self.send_ctr = 0
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.filter)
        self.timer2 = self.create_timer(.015, self.send)
        self.get_logger().info("Ackermann Started!")
    def __del__(self):
        self.get_logger().info("Ackermann Killed!")

    def send(self):
        msg = UInt8MultiArray()
        arr = np.array([self.vel_wheel_filt[self.send_ctr], self.ang_wheel[self.send_ctr], self.ang_vel_wheel[self.send_ctr]], dtype = np.float16)
        data = bytes([self.send_ctr + 11]) + arr.tobytes()
        msg.data = data
         #   print(data)
        self.pub.publish(msg)
        self.send_ctr += 1
        self.send_ctr %= 6


    def callback_2(self,data):
        for c1 in range(0, 6):
            arr = self.ang_dataframe[str(c1)].to_numpy()
       #     print(arr)
            ang = data.position[c1]
            if(c1 == 0 or c1 == 3):
                ang = -ang
            if(c1 != 1 and c1 != 4):
                c2 = 0
                while( float(ang) > float(arr[c2])):
          #          print(ang, arr[c2])
                    c2+=1
                    if(c2 > 98):
                        break
         #   print("DONE")
            if(ang > 0):
                self.vel_wheel_temp[c1] = self.vel_dataframe[str(c1)][c2] * lin_vel_gain * self.vel_lin
                self.ang_vel_wheel_temp[c1] = self.ang_vel_dataframe[str(c1)][c2] * ang_vel_gain
            else:
                self.vel_wheel_temp[c1] = self.vel_dataframe[str(5 -c1)][99 - c2] * lin_vel_gain * self.vel_lin
                self.ang_vel_wheel_temp[c1] = self.ang_vel_dataframe[str(5 - c1)][99 - c2] * ang_vel_gain
        self.vel_wheel_temp[3] *= -1
        self.vel_wheel_temp[4] *= -1
        self.vel_wheel_temp[5] *= -1
            
        
    #    print( self.vel_wheel)
    #    print(self.ang_vel_wheel)
        print('-----------------')
        

    def callback(self,data):
        self.vel_lin = data.linear.x /(wheel_diameter /2) # convert to revs/sec from m/s
        vel_ang = -data.angular.z
        # go straingt
        
        if(vel_ang == 0):
            None
            self.vel_wheel = [self.vel_lin] * 6
            self.ang_wheel = [0] * 6
            self.vel_wheel[3] *= -1
            self.vel_wheel[4] *= -1
            self.vel_wheel[5] *= -1
        #turn
        else:
            None
            rad = .6/vel_ang
            #turn center outside
            if(abs(rad) > .58):
                for c in range(6):
                    None
     #               self.vel_wheel[c] = vel_lin / abs(rad) * sqrt(self.l[c]**2 + (rad + self.d[c]/2)**2)
                    self.ang_vel_wheel = self.ang_vel_wheel_temp
                    self.vel_wheel = self.vel_wheel_temp
                    self.ang_wheel[c] = atan(self.l[c]/ (rad + self.d[c] / 2)) * 180 / pi
                    if(c > 2):
                        None
     #                   self.vel_wheel[c] *= -1
            #turn center inside
            else:
                None
                for c in range(6):
                    None
                    self.ang_vel_wheel = [0.8] * 6
                    self.vel_wheel[c] = -self.vel_lin * sqrt(self.l[c]**2 + (rad + self.d[c]/2)**2)
                    self.ang_wheel[c] = atan(self.l[c]/ (rad + self.d[c] / 2)) * 180 / pi

        self.ang_wheel[0] *= -1
        self.ang_wheel[3] *= -1
        self.ang_wheel[0] -= 2
        self.ang_wheel[2] += 5
        self.ang_wheel[3] += 2
        self.ang_wheel[5] -= 0
    #    self.send()

 #   def filter(self):
 #       step = 0.05
 #       for c in range (6):
 #           self.vel_wheel_filt[c] += step * (self.vel_wheel[c] - self.vel_wheel[c])
    #    self.send()
 #       self.publish_jointstate()

    def filter(self):
        for c in range(6):
            if (self.vel_wheel[c] > self.vel_wheel_filt[c]  + 0.009):
                self.vel_wheel_filt[c] += 0.01
            if (self.vel_wheel[c] < self.vel_wheel_filt[c]  - 0.009):
                self.vel_wheel_filt[c] -= 0.01
        self.publish_jointstate()
        

    def publish_jointstate(self):
        message = JointState()
        message.name = ['DC1','DC2','DC3','DC4','DC5','DC6']
  #      print(self.velocities)
        message.header.stamp = self.get_clock().now().to_msg()
        message.position = np.array(self.ang_wheel, dtype=np.float32).tolist()
        message.velocity = np.array(self.vel_wheel_filt, dtype=np.float32).tolist()
        message.effort = [0.] * 6
        self.pub_2.publish(message)
        message.position = np.array(self.ang_wheel, dtype=np.float32).tolist()
        message.velocity = np.array(self.ang_vel_wheel, dtype=np.float32).tolist()
        message.effort = [0.] * 6
        self.pub_4.publish(message)
        message.position = [0.] * 6
        print(self.vel_wheel_filt)
        message.velocity = np.array(self.vel_wheel_filt, dtype=np.float32).tolist()
        message.effort = [0.] * 6
        self.pub_3.publish(message)




def main(args=None):
    rclpy.init()
    ack = ackermann()
    rclpy.spin(ack)

    
    ack.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()