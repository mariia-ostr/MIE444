#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import random
import matplotlib.pyplot as plt
#from std_msgs.msg import String
from geometry_msgs.msg import Twist


class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node')
        rospy.loginfo("Now initalizing the localization node.")
        #self.pf = ParticleFilter(number_of_particles=1000, movement_noise_std=[0.3, 0.3])
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.serial_pub = rospy.Publisher('/motor_speeds', Twist, queue_size=10)
        self.lidar_readings = []
        self.desired_distance = 0.15  # distance to maintain from the left wall
        self.dist_error = 0.05 # deviation from dist we're ok being within
        
    def initialize_distances(self):
        # Initialize distances to infinity (indicating no obstacles)
        self.front_wall_distance = float('inf')
        self.back_wall_distance = float('inf')
        self.left_wall_distance = float('inf')
        self.left_fr_diag_distance = float('inf')
        self.left_b_diag_distance = float('inf')
        self.right_wall_distance = float('inf')
        self.right_fr_diag_distance = float('inf')
        self.right_b_diag_distance = float('inf')

        for angle, distance in self.lidar_readings:
            if -math.pi/6 <= angle < math.pi/6:
                self.front_wall_distance = min(self.front_wall_distance, distance)
            elif math.pi/6 <= angle < math.pi/3:
                self.left_fr_diag_distance = min(self.left_fr_diag_distance, distance)
            elif math.pi/3 <= angle < 2 * math.pi / 3:
                self.left_wall_distance = min(self.left_wall_distance, distance)
            elif 2 * math.pi / 3 <= angle < 5 * math.pi / 6:
                self.left_b_diag_distance = min(self.left_b_diag_distance, distance)
            elif angle >= 5 * math.pi / 6 or angle <= -5 * math.pi / 6:
                self.back_wall_distance = min(self.back_wall_distance, distance)
            elif -5 * math.pi / 6 < angle <= -2 * math.pi / 3:
                self.right_b_diag_distance = min(self.right_b_diag_distance, distance)
            elif -2 * math.pi / 3 < angle <= -math.pi / 3:
                self.right_wall_distance = min(self.right_wall_distance, distance)
            elif -math.pi / 3 < angle <= -math.pi / 6:
                self.right_fr_diag_distance = min(self.right_fr_diag_distance, distance)

    
    def send_motor_command(self, left_speed, right_speed):
        """
        Publish motor command as a formatted message to the Arduino.
        """
        # 3-character string with leading zeros
        left_speed_str = f"{left_speed:03d}"
        right_speed_str = f"{right_speed:03d}"
        
        command = f"<{left_speed_str} {right_speed_str}>"
        
        # Publish the command message to the motor topic
        twist_msg = Twist()
        twist_msg.linear.x = left_speed
        twist_msg.angular.z = right_speed
        rospy.loginfo(f"Sending motor command: {command}")
        self.serial_pub.publish(twist_msg)

    def lidar_callback(self, scan_data):
        """
        Callback function to handle incoming LiDAR data (LaserScan messages).
        """
        # convert the scan data into (theta, distance) pairs
        #rospy.loginfo("Processing lidar data now.")
        self.lidar_readings = []
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment
        
        for i, distance in enumerate(scan_data.ranges):
            theta = angle_min + i * angle_increment
            if not math.isinf(distance) and not math.isnan(distance):
                self.lidar_readings.append((theta, distance))


    def go_empty_dir(self):
        """
        Moves the robot in the direction of the largest free space based on LiDAR readings.
        """
        # define reg_speed, faster, slower, fast, slow?
        reg_speed = 100
        faster = 120
        slower = 80
        fast = 150
        slow = 50
        
        region_distances = {
            'front': self.front_wall_distance,
            'back': self.back_wall_distance,
            'left': self.left_wall_distance,
            'left_front_diag': self.left_fr_diag_distance,
            'left_back_diag': self.left_b_diag_distance,
            'right': self.right_wall_distance,
            'right_front_diag': self.right_fr_diag_distance,
            'right_back_diag': self.right_b_diag_distance
        }

        max_region = max(region_distances, key=region_distances.get)
        max_distance = region_distances[max_region]
        rospy.loginfo(f"Largest free space direction: {max_region} with distance: {max_distance}")
        


        if max_region == 'front':
            # Move forward
            self.send_motor_command(reg_speed, reg_speed)
        # elif max_region == 'back':
        #     # right turn
        #     self.send_motor_command()
        elif max_region == 'left':
            # Turn slightly left
            rospy.loginfo("turning slightly left")
            self.send_motor_command(slower, faster)
        elif max_region == 'right':
            # Turn slightly right
            rospy.loginfo("turning slightly right")
            self.send_motor_command(faster, slower)
        elif max_region == 'left_front_diag':
            # Move diagonally to the left (favoring a left turn)
            rospy.loginfo("turning left")
            self.send_motor_command(slower, reg_speed)
        elif max_region == 'right_front_diag':
            # Move diagonally to the right (favoring a right turn)
            rospy.loginfo("turning right")
            self.send_motor_command(reg_speed, slower)
        elif max_region == 'left_back_diag':
            # Turn left more sharply
            rospy.loginfo("quickly turning left")
            self.send_motor_command(slow, fast)
        elif max_region == 'right_back_diag':
            # Turn right more sharply
            rospy.loginfo("quickly turning right")
            self.send_motor_command(fast, slow)
            
    def forward_with_adjustments(self):
        """
        Moves the robot in the direction of the largest free space while staying centered in a 12-inch passage.
        Eliminates backward movement; prioritizes forward or side movement based on free space.
        """
        regular_speed = 150
        slow_turn_speed = 130  # Slightly slower to enable a gentle turn
        adjustment_speed = 20   # Small adjustment for left/right balancing

        region_distances = {
            'front': self.front_wall_distance,
            'left': self.left_wall_distance,
            'right': self.right_wall_distance,
            'left_front_diag': self.left_fr_diag_distance,
            'right_front_diag': self.right_fr_diag_distance,
        }

        max_region = max(region_distances, key=region_distances.get)
        max_distance = region_distances[max_region]

        large_distance_threshold = 0.4  # Adjust based on environment

        # if there is sufficient space in front, go there
        if region_distances['front'] > large_distance_threshold:
            rospy.loginfo("Moving forward, front is clear")
            self.send_motor_command(regular_speed, regular_speed)
            return

        # if diagonals have larger space, gentle turns towards them
        if max_distance > large_distance_threshold:
            if max_region == 'left_front_diag':
                rospy.loginfo("Turning slightly left towards larger space in left diagonal")
                self.send_motor_command(slow_turn_speed, regular_speed)
            elif max_region == 'right_front_diag':
                rospy.loginfo("Turning slightly right towards larger space in right diagonal")
                self.send_motor_command(regular_speed, slow_turn_speed)
            return

        # adjust based on distance to left and right walls if moving forward is not an option
        left_right_balance = self.left_wall_distance - self.right_wall_distance

        if abs(left_right_balance) < 0.1:  # Already centered, small tolerance
            rospy.loginfo("Centered, moving forward")
            self.send_motor_command(regular_speed, regular_speed)
        elif left_right_balance > 0:
            # Closer to right wall; adjust to move slightly left
            rospy.loginfo("Adjusting to center, too close to right wall")
            self.send_motor_command(regular_speed + adjustment_speed, regular_speed)
        else:
            # Closer to left wall; adjust to move slightly right
            rospy.loginfo("Adjusting to center, too close to left wall")
            self.send_motor_command(regular_speed, regular_speed + adjustment_speed)



    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.lidar_readings:
                self.initialize_distances()
                self.go_empty_dir()
            rate.sleep()

def main():# 
    navigation_node = NavigationNode()
    navigation_node.run()

if __name__ == '__main__':
    main()
