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
#import pandas as pd



import math 



wheel_diameter = 0.3
wheelbase = 0.76
track_width = 0.94

def clamp(n, min, max):
    if n < min:
        return min
    elif n > max:
        return max
    else:
        return n
 
class RoverKinematics: 
    def __init__(self, wheelbase, track_width): 
        self.wheelbase = wheelbase 
        self.track_width = track_width 
        self.wheel_positions = { 
            'FL': (+wheelbase / 2, +track_width / 2), 
            'FR': (+wheelbase / 2, -track_width / 2), 
            'RL': (-wheelbase / 2, +track_width / 2), 
            'RR': (-wheelbase / 2, -track_width / 2) 
        } 
 
    def compute_steering_targets(self, turning_radius): 
        """Compute target steering angles for each wheel given turning radius.""" 
        angles = {} 
     #   print (self.wheel_positions)
        for wheel, (x, y) in self.wheel_positions.items(): 
            if(turning_radius == None):
                angles[wheel] = 0.0
            else:
                dy = y - turning_radius 
                angle_rad = math.atan2(x, abs(dy))
                if(dy > 0): 
                    angles[wheel] = math.degrees(angle_rad)
                else:
                    angles[wheel] = -math.degrees(angle_rad)
        return angles 
 
    def compute_stepper_steering_velocities(self, target_angles_deg, current_angles_deg, max_steering_speed_deg_s): 
        """Compute stepper motor angular speeds to reach target steering angles proportionally."""
        errors = {wheel: target_angles_deg[wheel] - current_angles_deg[wheel] for wheel in self.wheel_positions}
        max_error = max(abs(e) for e in errors.values())

        steering_speeds = {}
        if max_error < 1e-3:  # Threshold to avoid division by zero
            # All errors are very small, no motion needed
            for wheel in self.wheel_positions:
                steering_speeds[wheel] =  max_steering_speed_deg_s
        else:
            for wheel, error in errors.items():
                speed = (error / max_error) * max_steering_speed_deg_s
                steering_speeds[wheel] = speed
                
        return steering_speeds
 
    def compute_drive_velocities_from_steering(self, linear_velocity, current_steering_angles_deg): 
        """Compute propulsion velocities that match current steering angles to avoid slip.""" 
        drive_velocities = {} 
        turning_radius = self.estimate_current_turning_radius(current_steering_angles_deg)
     #   print(turning_radius)
        for wheel, (x, y) in self.wheel_positions.items(): 
            if(turning_radius == None):
                drive_velocities[wheel] = linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter)
            else:
                θ_deg = current_steering_angles_deg[wheel] 
                θ_rad = math.radians(θ_deg) 
    
                dir_x = math.cos(θ_rad) 
                dir_y = math.sin(θ_rad) 
    
                r_x = x 
                r_y = y - turning_radius 
    
                tangent_x = -r_y 
                tangent_y = r_x 
                tangent_mag = math.hypot(tangent_x, tangent_y) 
    
                dot = dir_x * tangent_x + dir_y * tangent_y 
                velocity_factor = dot / tangent_mag 
    
                drive_velocities[wheel] = linear_velocity * velocity_factor * 2 * 360.0 / (3.14 * wheel_diameter)

    #    print(drive_velocities)
        drive_velocities['FR'] *= -1
        drive_velocities['RR'] *= -1
        drive_velocities['FL'] *= -1
        drive_velocities['RL'] *= -1
        return drive_velocities 


    
    def estimate_current_turning_radius(self, current_steering_angles_deg):
        """
        Estimate current turning radius from both sides (left and right) of the rover.
        Returns average of left and right radius estimates.
        Returns None if both estimates are undefined.
        """

        def line_intersect_y(x1, y1, θ1_deg, x2, y2, θ2_deg):
            θ1 = math.radians(θ1_deg)
            θ2 = math.radians(θ2_deg)

            dir1 = (math.cos(θ1), math.sin(θ1))
            dir2 = (math.cos(θ2), math.sin(θ2))

            dx = x2 - x1
            dy = y2 - y1

            det = dir1[0] * dir2[1] - dir1[1] * dir2[0]
            if abs(det) < 1e-5:
                return None  # lines nearly parallel

            t = (dy * dir2[0] - dx * dir2[1]) / det
            y_int = y1 + t * dir1[1]
            return y_int

        # LEFT side (FL, RL)
        x_FL, y_FL = self.wheel_positions['FL']
        x_RL, y_RL = self.wheel_positions['RL']
        θ_FL = current_steering_angles_deg['FL']
        θ_RL = current_steering_angles_deg['RL']
    #    print(current_steering_angles_deg)
        left_radius = line_intersect_y(x_FL, y_FL, θ_FL, x_RL, y_RL, θ_RL)

        # RIGHT side (FR, RR)
        x_FR, y_FR = self.wheel_positions['FR']
        x_RR, y_RR = self.wheel_positions['RR']
        θ_FR = current_steering_angles_deg['FR']
        θ_RR = current_steering_angles_deg['RR']
        right_radius = line_intersect_y(x_FR, y_FR, θ_FR, x_RR, y_RR, θ_RR)

        # Average the two, if possible
        if left_radius is not None and right_radius is not None:
            return 0.5 * (left_radius + right_radius)
        elif left_radius is not None:
            return left_radius
        elif right_radius is not None:
            return right_radius
        else:
            return None  # no valid intersection
    


class ackermann(Node):
    def __init__(self):
        super().__init__('ackermann')
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.wheel_states_sub = self.create_subscription(JointState, "wheel_states", self.wheel_states_callback, 10)
        self.dc_commands_pub = self.create_publisher(JointState, "dc_commands", 10)
        self.stepper_commands_pub = self.create_publisher(JointState, "stepper_commands", 10)
        #heartbeat
        self.heartbeat_counter = 0
        #strafe flag
        self.strafe_angle_deg_estimated = 0.0
        #received from cmd_vel
        self.linear_velocity = 0.0
        self.strafe_angle_deg = 0.0
        self.turning_radius = None
        #command arrays
        self.drive_velocities = {
            'FL': 0.0,
            'FR': 0.0,
            'RL': 0.0,
            'RR': 0.0,
        }
        self.drive_velocities_filtered = {
            'FL': 0.0,
            'FR': 0.0,
            'RL': 0.0,
            'RR': 0.0,
        }
        self.steering_angles = {
            'FL': 0.0,
            'FR': 0.0,
            'RL': 0.0,
            'RR': 0.0,
        }
        self.steering_speeds = {
            'FL': 0.0,
            'FR': 0.0,
            'RL': 0.0,
            'RR': 0.0,
        }
        #feedback array
        self.current_steering_angles_deg = {
            'FL': 0.0,
            'FR': 0.0,
            'RL': 0.0,
            'RR': 0.0,
        }
        #kinematics class
        self.kinematics = RoverKinematics(wheelbase, track_width)
        self.heartbeat_counter = 0
        self.timer = self.create_timer(0.01, self.filter)
        self.timer2 = self.create_timer(0.015, self.send)
        self.timer3 = self.create_timer(0.1, self.heartbeat_function)
        self.get_logger().info("Ackermann Started!")
    def __del__(self):
        self.get_logger().info("Ackermann Killed!")


    def wheel_states_callback(self, message):
        # Map from joint names to angles (in degrees)
        angles_deg = {}
        for name, pos in zip(message.name, message.position):
            if name in self.current_steering_angles_deg:
                angles_deg[name] = pos

        # Only update if all expected joints are present
        if all(wheel in angles_deg for wheel in self.current_steering_angles_deg):
            self.current_steering_angles_deg = angles_deg
            mean = 0
            for key in self.current_steering_angles_deg:
                mean += self.current_steering_angles_deg[key]
            self.strafe_angle_deg_estimated = mean / len(self.current_steering_angles_deg)

    def cmd_vel_callback(self, message):
        self.heartbeat_counter = 0
        self.linear_velocity = message.linear.x
        self.strafe_angle_deg = message.linear.y
        inverse_turning_radius = message.angular.z
        if(abs(inverse_turning_radius) < 0.01):
            self.turning_radius = None
        elif(abs(inverse_turning_radius) > 90):
            self.turning_radius = 0.0
        else:
            self.turning_radius = 1.0 / clamp(message.angular.z, -1.4, 1.4)

    def calculate_controls(self):
     #   print(self.strafe_angle_deg_estimated)
        if(abs(self.strafe_angle_deg) > 1 or abs(self.strafe_angle_deg_estimated) > 5):
            self.drive_velocities = {
                'FL': self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'FR': -self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'RL': self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'RR': -self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
            }
            self.steering_angles = {
                'FL': self.strafe_angle_deg,
                'FR': self.strafe_angle_deg,
                'RL': self.strafe_angle_deg,
                'RR': self.strafe_angle_deg,
            }
            self.steering_speeds = {
                'FL': 25.0,
                'FR': 25.0,
                'RL': 25.0,
                'RR': 25.0,
            }
        else:
            if(self.turning_radius == None or abs(self.turning_radius) > track_width / 2 + 0.2):
                self.steering_angles = self.kinematics.compute_steering_targets(self.turning_radius)
                self.drive_velocities = self.kinematics.compute_drive_velocities_from_steering(self.linear_velocity, self.current_steering_angles_deg)
                self.steering_speeds = self.kinematics.compute_stepper_steering_velocities(self.steering_angles, self.current_steering_angles_deg, 25.0)
            else:
                self.drive_velocities = {
                'FL': self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'FR': self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'RL': self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'RR': self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                }
                self.steering_angles = {
                'FL': 39.0,
                'FR': -39.0,
                'RL': -39.0,
                'RR': 39.0,
                }
                self.steering_speeds = {
                'FL': 25.0,
                'FR': 25.0,
                'RL': 25.0,
                'RR': 25.0,
                }

    def send(self):
        self.calculate_controls()

        message = JointState()
     #   print(self.drive_velocities_filtered)
    #    print(list(self.steering_angles.values()))

        message.name = list(self.steering_angles.keys())
        message.position = list(self.steering_angles.values())
        message.velocity = list(self.steering_speeds.values())
        self.stepper_commands_pub.publish(message)

        message.position = [0.0, 0.0, 0.0, 0.0]
        message.velocity = list(self.drive_velocities_filtered.values())
        self.dc_commands_pub.publish(message)
        return 1

    def filter(self):
        filter_step = 3.0
        for key, value in self.drive_velocities.items():
            if(abs(self.drive_velocities_filtered[key] - value) > filter_step * 0.9):
                if(self.drive_velocities_filtered[key] > value):
                    self.drive_velocities_filtered[key] -= filter_step
                else:
                    self.drive_velocities_filtered[key] += filter_step
        return 1

    def heartbeat_function(self):
        self.heartbeat_counter += 1
  #      print(self.heartbeat_counter)
        if(self.heartbeat_counter > 10):
            self.turning_radius = None
            self.linear_velocity = 0.0


def main(args=None):
    rclpy.init()
    ack = ackermann()
    rclpy.spin(ack)

    
    ack.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()