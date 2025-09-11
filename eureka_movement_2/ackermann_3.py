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

class RoverSteeringLookupTable:
    def __init__(self, wheelbase, track_width,
                 max_steering_speed_deg_s=15.0,
                 dt=0.05,
                 resolution=0.1,
                 max_radius=10.0
                 , max_correction_deg_s=5.0):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.max_steering_speed_deg_s = max_steering_speed_deg_s
        self.dt = dt
        self.resolution = resolution
        self.max_radius = max_radius
        self.max_steering_speed_deg_s = max_steering_speed_deg_s

        self.wheel_positions = {
            'FL': (+wheelbase / 2, +track_width / 2),
            'FR': (+wheelbase / 2, -track_width / 2),
            'RL': (-wheelbase / 2, +track_width / 2),
            'RR': (-wheelbase / 2, -track_width / 2)
        }

        self.lookup_table = self._generate_lookup_table()
        c = 0
        for instance in self.lookup_table:
            c+=1
            if(c % 100 == 0):
                print(instance)
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
        prev_angles = None

        while r <= self.max_radius:
            angles = self.compute_steering_targets(r)

            # === Steering velocity per wheel ===
            if prev_angles is not None:
                # True angular velocity = delta / dt

                raw_velocities = {
                    wheel:  (angles[wheel] - prev_angles[wheel])
                    for wheel in self.wheel_positions
                }
            else:
                # Dummy zero velocities for the first entry
                raw_velocities = {wheel: 0.0 for wheel in self.wheel_positions}

            max_raw = max(abs(v) for v in raw_velocities.values())
            if max_raw < 1e-3:
                scaled_velocities = {wheel: 0.0 for wheel in self.wheel_positions}
            else:
                # Uniformly scale to match max allowed speed
                scale = self.max_steering_speed_deg_s / max_raw
                scaled_velocities = {
                    wheel: v * scale for wheel, v in raw_velocities.items()
                }

            # === Drive multipliers ===
            drive_multipliers_raw = {}
            for wheel, (x, y) in self.wheel_positions.items():
                θ_deg = angles[wheel]
                θ_rad = math.radians(θ_deg)

                # Direction wheel is pointing
                dir_x = math.cos(θ_rad)
                dir_y = math.sin(θ_rad)

                # Vector from CoR to wheel
                r_x = x
                r_y = y - r  # CoR at (0, r)

                tangent_x = -r_y
                tangent_y = r_x

                tangent_mag = math.hypot(tangent_x, tangent_y)
                if tangent_mag < 1e-5:
                    velocity_factor = 0.0
                else:
                    dot = dir_x * tangent_x + dir_y * tangent_y
                    velocity_factor = dot / tangent_mag

                drive_multipliers_raw[wheel] = velocity_factor

            # Normalize drive multipliers
            mean_factor = sum(abs(m) for m in drive_multipliers_raw.values()) / len(drive_multipliers_raw)
            if mean_factor < 1e-5:
                normalized_drive_multipliers = {wheel: 0.0 for wheel in self.wheel_positions}
            else:
                norm_scale = 1.0 / mean_factor
                normalized_drive_multipliers = {
                    wheel: m * norm_scale for wheel, m in drive_multipliers_raw.items()
                }

            lookup_table.append({
                'radius': r,
                'angles': angles,
                'velocities': scaled_velocities,
                'drive_multipliers': normalized_drive_multipliers
            })

            prev_angles = angles
            r += self.resolution

        # Remove the dummy first point
        lookup_table = lookup_table[1:]

        # === Mirror negative radii ===
        mirrored_entries = []
        for entry in lookup_table:
            mirrored_entries.append({
                'radius': -entry['radius'],
                'angles': {
                    'FL': -entry['angles']['FR'],
                    'FR': -entry['angles']['FL'],
                    'RL': -entry['angles']['RR'],
                    'RR': -entry['angles']['RL'],
                },
                'velocities': {
                    'FL': -entry['velocities']['FR'],
                    'FR': -entry['velocities']['FL'],
                    'RL': -entry['velocities']['RR'],
                    'RR': -entry['velocities']['RL'],
                },
                'drive_multipliers': {
                    'FL': entry['drive_multipliers']['FR'],
                    'FR': entry['drive_multipliers']['FL'],
                    'RL': entry['drive_multipliers']['RR'],
                    'RR': entry['drive_multipliers']['RL'],
                }
            })

        # === Straight-line case ===
        zero_entry = {
            'radius': None,
            'angles': {w: 0.0 for w in self.wheel_positions},
            'velocities': {w: 20.0 for w in self.wheel_positions},  # constant speed toward center
            'drive_multipliers': {w: 1.0 for w in self.wheel_positions}
        }

        return mirrored_entries[::-1] + [zero_entry] + lookup_table



    def compute_synchronized_steering_velocities_by_index(self, target_radius, current_angles_deg, Kp_index=0.5):
        """
        Synchronize all wheels based on their relative lookup table positions (indices), 
        not angle errors. Applies time-based path synchronization.
        """
        # Step 1: For each wheel, find best-matching table index
        wheel_indices = {}
    #    print(current_angles_deg)
        for wheel, current_angle in current_angles_deg.items():
            best_index = None
            best_error = float('inf')

            for i, entry in enumerate(self.lookup_table):
                table_angle = entry['angles'][wheel]
                error = abs(current_angle - table_angle)
                if error < best_error:
                    best_error = error
                    best_index = i

            wheel_indices[wheel] = best_index
   #         print(self.lookup_table[wheel_indices[wheel]])

        # Step 2: Compute average index
        index_values = list(wheel_indices.values())
        average_index = sum(index_values) / len(index_values)

        print(index_values)
        # Step 3: Get final entry for target radius (we still need the base velocities)

        target_entry = min(self.lookup_table, key=lambda e: self._radius_distance(e['radius'], target_radius))

        # Step 4: For each wheel, combine base velocity with correction
        raw_velocities = {}
        for wheel, current_index in wheel_indices.items():
            current_entry = self.lookup_table[current_index]
            base_velocity = current_entry['velocities'][wheel]
            current_radius = current_entry['radius']

            if target_radius is None:
                direction = -1.0  # Always steer toward straight
            elif current_radius is None:
                direction = 1.0   # Always steer away from straight
            else:
                # Positive if current is turning too wide or opposite direction
                direction = 1.0 if (current_radius - target_radius) > 0 else -1.0


            index_error = (average_index - current_index)
            correction = Kp_index * index_error * direction
            print(correction)

            raw_velocities[wheel] = base_velocity #+ clamp(correction, -5.0, 5.0)

        # Step 5: Scale so max wheel uses full allowed speed
        max_raw = max(abs(v) for v in raw_velocities.values())
        if max_raw < 1e-3:
            return {wheel: 0.0 for wheel in self.wheel_positions}

        scale = self.max_steering_speed_deg_s / max_raw
        scaled_velocities = {wheel: v * scale for wheel, v in raw_velocities.items()}
        return scaled_velocities


    def get_angles_for_radius(self, radius):
        """Return steering angles dict from the closest lookup entry."""
        closest_entry = min(self.lookup_table, key=lambda e: self._radius_distance(e['radius'], radius))
        return closest_entry['angles']

    def get_steering_velocities_for_radius(self, radius):
        """Return base steering velocities from the closest lookup entry."""
        closest_entry = min(self.lookup_table, key=lambda e: self._radius_distance(e['radius'], radius))
        return closest_entry['velocities']

    def get_drive_multipliers_for_radius(self, radius):
        """Return linear velocity multipliers from the closest lookup entry."""
        closest_entry = min(self.lookup_table, key=lambda e: self._radius_distance(e['radius'], radius))
        return closest_entry['drive_multipliers']

    def _radius_distance(self, r1, r2):
        """Internal helper to handle None (straight line) and out-of-bounds radii."""
        STRAIGHT_RADIUS = self.max_radius + 1.0  # Treat None as large-radius straight-line motion

        r1_val = STRAIGHT_RADIUS if r1 is None else r1
        r2_val = STRAIGHT_RADIUS if r2 is None else r2

        return abs(r1_val - r2_val)



    def get_drive_multipliers_per_wheel_from_angles(self, current_angles_deg):
        """
        For each wheel, find the lookup entry with the closest steering angle,
        and return a dictionary of the corresponding drive multipliers.

        Args:
            current_angles_deg (dict): Current angles per wheel in degrees.
            angle_threshold (float): Max deviation to count as 'straight'.

        Returns:
            dict: Drive multipliers per wheel, matched individually.
        """

        drive_multipliers = {}

        for wheel in self.wheel_positions:
            best_entry = None
            best_error = float('inf')

            for entry in self.lookup_table:
                error = abs(entry['angles'][wheel] - current_angles_deg[wheel])
                if error < best_error:
                    best_error = error
                    best_entry = entry

            drive_multipliers[wheel] = best_entry['drive_multipliers'][wheel]

        return drive_multipliers


    


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
        self.turning_radius = 0.0
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
        self.filter_step = 1.0
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
        print("_________________________________")
        if(abs(self.strafe_angle_deg) > 1 or abs(self.strafe_angle_deg_estimated) > 15):
            print("strafe")
            self.filter_step = 2.0
            self.drive_velocities = {
                'FL': -self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'FR': self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'RL': -self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
                'RR': self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter),
            }
            self.steering_angles = {
                'FL': self.strafe_angle_deg,
                'FR': self.strafe_angle_deg,
                'RL': self.strafe_angle_deg,
                'RR': self.strafe_angle_deg,
            }
            self.steering_speeds = {
                'FL': 15.0,
                'FR': 15.0,
                'RL': 15.0,
                'RR': 15.0,
            }
        else:
            if(self.turning_radius == None or abs(self.turning_radius) > track_width / 2 + 0.2):
                print("corner")
                self.filter_step = 2.0
                self.steering_angles = self.kinematics.get_angles_for_radius(self.turning_radius)
                self.drive_velocities = {
                    key: multiplier * self.linear_velocity * 2 * 360.0 / (3.14 * wheel_diameter)
                    for key, multiplier in self.kinematics.get_drive_multipliers_per_wheel_from_angles(self.current_steering_angles_deg).items()
                }
                self.drive_velocities["FL"] *= -1
                self.drive_velocities["RL"] *= -1
                self.steering_speeds = self.kinematics.compute_synchronized_steering_velocities_by_index(
                    self.turning_radius, self.current_steering_angles_deg
                )
            else:
                print("in-place")
                self.filter_step = 2.0
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
                'FL': 15.0,
                'FR': 15.0,
                'RL': 15.0,
                'RR': 15.0,
                }

  #      print(self.steering_speeds)

           

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
        self.filter_step = 1.5
        for key, value in self.drive_velocities.items():
            if(abs(self.drive_velocities_filtered[key] - value) > self.filter_step * 0.9):
                if(self.drive_velocities_filtered[key] > value):
                    self.drive_velocities_filtered[key] -= self.filter_step
                else:
                    self.drive_velocities_filtered[key] += self.filter_step
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
