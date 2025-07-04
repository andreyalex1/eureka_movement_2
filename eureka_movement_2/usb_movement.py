#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

"""
from sensor_msgs.msg import JointState
import numpy as np
import rclpy
from rclpy.node import Node
import serial
from neuron import h


class usb_movement(Node):
    def __init__(self):
        print("init")
        super().__init__('usb_movement')
        self.sub = self.create_subscription(JointState, "dc_commands", self.dc_callback, 10)
        self.sub_2 = self.create_subscription(JointState, "stepper_commands", self.stepper_callback, 10)
        self.sub_3 = self.create_subscription(JointState, "wheel_settings", self.settings_callback, 10)
        self.sub_4 = self.create_subscription(JointState, "light_commands", self.light_callback, 10)
        self.pub = self.create_publisher(JointState, "wheel_states", 10)
        #commands
        self.heartbeat = 1
        self.control_mode = 2
        self.power_saving = 0
        self.gain_p = 0.35
        self.gain_i = 0.2
        self.gain_d = 0.05
        self.stepper_pos_com = [0.] * 6
        self.stepper_vel_com = [0.] * 6
        self.dc_vel_com = [0.] * 6
        self.light_brightness_com = [0.0] * 2
        #feedback
        self.stepper_pos_fb = [0.] * 6
        self.dc_vel_fb = [0.] * 6
        print("check1")
        self.x = [h.ref(0) for i in range(20)]
        self.command_format = "global: heartbeat=%d, control_mode=%d, power_saving=%d gain_p=%.2f, gain_i=%.2f, gain_d=%.2f\r\n\
wheel1: stepper_pos=%.2f, stepper_vel=%.2f, dc_vel=%.2f\r\n\
wheel2: stepper_pos=%.2f, stepper_vel=%.2f, dc_vel=%.2f\r\n\
__end__"
        self.reply_format = '''wheel1: stepper_pos=%f,  dc_vel=%f\r\n\
wheel2: stepper_pos=%f,  dc_vel=%f\r\n\
__end__'''
        print("check2")
        flag = 0
        while flag < 1:
            print(flag)
            try:
                self.right = serial.Serial('/dev/dc_right', 9600, timeout=1)
                flag = 1
                continue
            except serial.serialutil.SerialException:
                None
        flag = 0
        print("got right")
        while flag < 1:
            try:
                self.left = serial.Serial('/dev/dc_left', 9600, timeout=1)
                flag = 1
                continue
            except serial.serialutil.SerialException:
                None
        print("got left")
        timer_period = 0.05  # seconds
        self.heartbeat_counter = 0
        self.timer = self.create_timer(timer_period, self.send)
        self.timer3 = self.create_timer(.1, self.heartbeat_function)
        self.send()
        self.get_logger().info("usb_movement Started!")
    def __del__(self):
        self.get_logger().info("usb_movement Killed!")

    def send(self):
            print(self.left.isOpen())
            print(self.right.isOpen())
            try:
                message = self.command_format % (self.heartbeat, self.control_mode, self.power_saving, self.gain_p, self.gain_i, self.gain_d,
                                                self.stepper_pos_com[3], 0.5, self.dc_vel_com[0] * 360 / 6.28,
                                                self.stepper_pos_com[5], 0.5, self.dc_vel_com[2] * 360 / 6.28)
                print(message)
                print(self.right.write(bytes(message, encoding='utf8')))
                reply = self.right.read_until(str.encode("__end__")).decode('utf-8')
                print(reply)
                
                num = h.sscanf(reply, self.reply_format, self.x[0], self.x[1], self.x[2], self.x[3])
                self.stepper_pos_fb[3] = float(self.x[0][0])
                self.stepper_pos_fb[5] = float(self.x[2][0])
                self.dc_vel_fb[3] = float(self.x[1][0])
                self.dc_vel_fb[5] = float(self.x[3][0])
                print(self.stepper_pos_fb)
                print(len(self.command_format))
                message = self.command_format % (self.heartbeat, self.control_mode, self.power_saving, self.gain_p, self.gain_i, self.gain_d,
                                                self.stepper_pos_com[0], 0.5, self.dc_vel_com[3] * 360 / 6.28,
                                                self.stepper_pos_com[2], 0.5, self.dc_vel_com[5] * 360 / 6.28)
                print(message)
                print(self.left.write(bytes(message, encoding='utf8')))
                reply = self.left.read_until(str.encode("__end__")).decode('utf-8')
                print(reply)
                num = h.sscanf(reply, self.reply_format, self.x[0], self.x[1], self.x[2], self.x[3])
                self.stepper_pos_fb[0] = float(self.x[0][0])
                self.stepper_pos_fb[2] = float(self.x[2][0])
                self.dc_vel_fb[0] = float(self.x[1][0])
                self.dc_vel_fb[2] = float(self.x[3][0])
                message = JointState()
                message.name = ['DC1','DC2','DC3','DC4','DC5','DC6']
                message.header.stamp = self.get_clock().now().to_msg()
                message.velocity = self.dc_vel_fb
                message.effort = [0.] * 6
                message.position = self.stepper_pos_fb
                self.pub.publish(message)
            except serial.serialutil.SerialException:
                self.get_logger().warning("No USB FS Connection to Drivetrain!")
                try:
                    self.right = serial.Serial('/dev/dc_right', 9600, timeout=1)
                    flag = 1
                except serial.serialutil.SerialException:
                    None
                try:
                    self.left = serial.Serial('/dev/dc_left', 9600, timeout=1)
                    flag = 1
                except serial.serialutil.SerialException:
                    None
                


    def dc_callback(self,data):
    #    print("CALLBACK_DC")
        self.heartbeat_counter = 0
        self.dc_vel_com = data.velocity
    def stepper_callback(self,data):
        self.stepper_pos_com = data.position
        self.stepper_vel_com = data.velocity
    def settings_callback(self,data):
        self.heartbeat = data.position[list(data.name).index('heartbeat')]
        self.control_mode = data.position[list(data.name).index('control_mode')]
        self.power_saving = data.position[list(data.name).index('power_saving')]
    #    self.gain_p = data.position[list(data.name).index('gain_p')]
    #    self.gain_i = data.position[list(data.name).index('gain_i')]
    #    self.gain_d = data.position[list(data.name).index('gain_d')]
    def publish_jointstate(self):
        message = JointState()
        message.name = ['DC1','DC2','DC3','DC4','DC5','DC6']
  #      print(self.velocities)
        message.header.stamp = self.get_clock().now().to_msg()
        message.velocity = np.array(self.vel_wheel_filt, dtype=np.float32).tolist()
        message.effort = [0.] * 6
        message.position = np.array(self.ang_wheel, dtype=np.float32).tolist()
        self.pub.publish(message)
    def light_callback(self,data):
  #      self.get_logger().info("Light Callback!")
        self.light_brightness_com[0] = data.position[data.name.index("ledstrip")]
        self.light_brightness_com[1] = data.position[data.name.index("spotlight")]
    def heartbeat_function(self):
        self.heartbeat_counter += 1
   #     print(self.heartbeat_counter)
        if(self.heartbeat_counter > 15):
            self.stepper_pos_com = [0] * 6
            self.stepper_vel_com = [0] * 6
            self.dc_vel_com = [0] * 6




def main(args=None):
    rclpy.init()
    usb = usb_movement()
    rclpy.spin(usb)

    
    usb.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

"""