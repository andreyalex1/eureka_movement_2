#!/usr/bin/env python3

#Developed by Andrei Smirnov. 2024
#MSU Rover Team. Voltbro. NIIMech 

import math

class RoverSteeringLookupTable:
    def __init__(self, wheelbase, track_width,
                 max_steering_speed_deg_s=20.0,
                 dt=0.05,
                 resolution=0.01,
                 max_radius=10.0):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.max_steering_speed_deg_s = max_steering_speed_deg_s
        self.dt = dt
        self.resolution = resolution
        self.max_radius = max_radius

        self.wheel_positions = {
            'FL': (+wheelbase / 2, +track_width / 2),
            'FR': (+wheelbase / 2, -track_width / 2),
            'RL': (-wheelbase / 2, +track_width / 2),
            'RR': (-wheelbase / 2, -track_width / 2)
        }

        self.lookup_table = self._generate_lookup_table()
   #     for instance in self.lookup_table:
     #       print(instance)
     #   self.print_lookup_table()

    def compute_steering_targets(self, turning_radius):
        angles = {}
        for wheel, (x, y) in self.wheel_positions.items():
            if turning_radius is None:
                angles[wheel] = 0.0
            else:
                dy = y - turning_radius
                angle_rad = math.atan2(x, abs(dy))
                if dy > 0:
                    angles[wheel] = math.degrees(angle_rad)
                else:
                    angles[wheel] = -math.degrees(angle_rad)
        return angles

    def _generate_lookup_table(self):
        lookup_table = []

        min_radius = max(self.track_width / 2 + 0.05, self.resolution)
        r = min_radius
        while r <= self.max_radius:
            angles = self.compute_steering_targets(r)

            # === Steering velocity scaling ===
            raw_velocities = {wheel: angles[wheel] / 0.5 for wheel in self.wheel_positions}
            max_raw = max(abs(v) for v in raw_velocities.values())
            if max_raw < 1e-3:
                scaled_velocities = {wheel: 0.0 for wheel in self.wheel_positions}
            else:
                scale = self.max_steering_speed_deg_s / max_raw
                scaled_velocities = {wheel: v * scale for wheel, v in raw_velocities.items()}

            # === Drive multipliers ===
            drive_multipliers = {}
            for wheel, (x, y) in self.wheel_positions.items():
                θ_deg = angles[wheel]
                θ_rad = math.radians(θ_deg)

                # Direction the wheel is pointing
                dir_x = math.cos(θ_rad)
                dir_y = math.sin(θ_rad)

                # Vector from CoR to wheel
                r_x = x
                r_y = y - r  # turning center at (0, r)

                # Tangent direction at that point (perpendicular to radius vector)
                tangent_x = -r_y
                tangent_y = r_x

                tangent_mag = math.hypot(tangent_x, tangent_y)
                if tangent_mag < 1e-5:
                    velocity_factor = 0.0
                else:
                    dot = dir_x * tangent_x + dir_y * tangent_y
                    velocity_factor = dot / tangent_mag

                drive_multipliers[wheel] = velocity_factor

            lookup_table.append({
                'radius': r,
                'angles': angles,
                'velocities': scaled_velocities,
                'drive_multipliers': drive_multipliers
            })

            r += self.resolution

        # === Mirror for negative radii ===
        mirrored_table = []
        for entry in lookup_table:
            mirrored_table.append({
                'radius': -entry['radius'],
                'angles': {"FL":-entry['angles']['FR'], "FR":-entry['angles']['FL'], "RL":-entry['angles']['RR'], "RR":-entry['angles']['RL']},
                'velocities': {"FL":-entry['velocities']['FR'], "FR":-entry['velocities']['FL'], "RL":-entry['velocities']['RR'], "RR":-entry['velocities']['RL']},
                'drive_multipliers': {"FL":entry['drive_multipliers']['FR'], "FR":-entry['drive_multipliers']['FL'], "RL":-entry['drive_multipliers']['RR'], "RR":-entry['drive_multipliers']['RL']},
            })

        # === Straight-line case ===
        zero_entry = {
            'radius': None,
            'angles': {w: 0.0 for w in self.wheel_positions},
            'velocities': {w: 0.0 for w in self.wheel_positions},
            'drive_multipliers': {w: 1.0 for w in self.wheel_positions}
        }

        return mirrored_table + [zero_entry] + lookup_table


    def compute_steering_velocities(self, current_radius, current_motor_angles, Kp=0.5):
        """
        Compute steering motor velocities based on desired radius and current motor angles.
        Applies proportional control and scales so max motor uses full speed.
        """
        def radius_distance(r1, r2):
            if r1 is None or r2 is None:
                return 0 if r1 is None and r2 is None else float('inf')
            return abs(r1 - r2)

        # Find closest lookup entry
        closest = min(self.lookup_table, key=lambda entry: radius_distance(entry['radius'], current_radius))
        target_angles = closest['angles']
        base_velocities = closest['velocities']

        raw_velocities = {}
        for wheel in self.wheel_positions:
            error = target_angles[wheel] - current_motor_angles.get(wheel, 0.0)
            correction = Kp * error
            raw_velocities[wheel] = base_velocities[wheel] + correction

        # Scale so the fastest wheel uses full max velocity
        max_raw = max(abs(v) for v in raw_velocities.values())
        if max_raw < 1e-3:
            return {wheel: 0.0 for wheel in self.wheel_positions}

        scale = self.max_steering_speed_deg_s / max_raw
        scaled_velocities = {wheel: v * scale for wheel, v in raw_velocities.items()}

        return raw_velocities


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
        self.turning_radius_filtered = 0.0
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
        self.kinematics = RoverSteeringLookupTable(wheelbase, track_width)
        self.heartbeat_counter = 0
        self.timer = self.create_timer(0.01, self.filter)
        self.timer4 = self.create_timer(0.01, self.radius_filter_function)
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
        print("_________________________________")
        print(self.turning_radius_filtered)
        if(self.turning_radius != None):
            print(self.current_steering_angles_deg)
            closest_entry = min(self.kinematics.lookup_table, key=lambda e: abs((e['radius'] or 1e9) - self.turning_radius_filtered))
            print(closest_entry)
            drive_multipliers = closest_entry['drive_multipliers']
            closest_entry = min(self.kinematics.lookup_table, key=lambda e: abs((e['radius'] or 1e9) - self.turning_radius))
            self.steering_angles = closest_entry['angles']
            self.drive_velocities = multiplied_dict = {key: value * self.linear_velocity for key, value in drive_multipliers.items()}
            self.steering_speeds = self.kinematics.compute_steering_velocities(self.turning_radius_filtered, self.current_steering_angles_deg)
            print(self.steering_speeds)
           

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

    def radius_filter_function(self):
        if self.turning_radius is None:
        #    self.turning_radius_filtered = 1e9  # Simulate straight motion
            return

        # Current and target inverse radii
        current_inv = 1.0 / self.turning_radius_filtered if abs(self.turning_radius_filtered) > 1e-5 else 0.0
        target_inv = 1.0 / self.turning_radius if abs(self.turning_radius) > 1e-5 else 0.0

        # Nonlinear gain scaling (faster when small radius)
        alpha = 0.002 + 0.002 * min(1.0, abs(target_inv) / 1.0)  # max gain = 0.22, min = 0.02

        # Low-pass filter in inverse radius space
        filtered_inv = (1 - alpha) * current_inv + alpha * target_inv

        # Prevent divide by zero
        if abs(filtered_inv) < 1e-5:
            self.turning_radius_filtered = 1e9  # practically straight
        else:
            self.turning_radius_filtered = 1.0 / filtered_inv

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
