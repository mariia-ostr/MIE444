#!/usr/bin/env python3

import rospy
import math
import numpy as np
import sys
from geometry_msgs.msg import Point
from std_msgs.msg import String
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

FACTOR = 12  # Scaling factor for map
THRESHOLD = 2
IN_TO_CM = 2.54

class NavigationNode:
    def __init__(self, goals):
        # Define map (static for now)
        self.map_data = np.array([
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 1, 0, 1],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 0, 1, 0, 1]
        ])
        self.received_data = False
        rospy.init_node('navigation_node')
        self.current_x, self.current_y, self.current_theta = 0, 0, 0
        self.goals = goals
        self.active_goal = self.goals.pop(0) if self.goals else None

        rospy.Subscriber('/predicted_position', Point, self.position_callback)
        #self.current_x, self.current_y, self.current_theta = 30, 30, -90
        self.bearing_pub = rospy.Publisher('/bearing_to_goal', String, queue_size=10)
        
        rospy.loginfo("Navigation node initialized.")
        if self.active_goal:
            rospy.loginfo(f"Current goal is {self.active_goal}.")

    def position_callback(self, msg):
        """Update the robot's current estimated position."""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.z
        self.received_data = True
        
    def init_plot(self):
        """Initialize the plot."""
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.cmap = ListedColormap(['grey', 'white'])  # 0 = obstacle, 1 = free

        # Scale the map
        self.scaled_map = np.kron(self.map_data, np.ones((FACTOR, FACTOR)))

        # Display the scaled map
        self.ax.imshow(self.scaled_map, cmap=self.cmap, origin="lower")

        # Initialize an empty scatter plot for particles and the robot
        self.robot_plot, = self.ax.plot([], [], 'ro', label='Robot')
        self.goal_plot, = self.ax.plot([], [], 'go', label='Goal')
        self.path_plot, = self.ax.plot([], [], 'b--', label='Path')

        # Configure the plot
        self.ax.set_xlim(0, self.scaled_map.shape[1])
        self.ax.set_ylim(0, self.scaled_map.shape[0])
        self.ax.legend()
        plt.ion()
        plt.show()

    def update_plot(self):
        """Update the plot dynamically with the robot's position, goal, and path."""
        # Clear existing dynamic elements (robot, goal, path)
        self.robot_plot.set_data(self.current_x, self.current_y)

        if self.active_goal:
            # Scale the goal coordinates
            goal_x_scaled = self.active_goal[0] * FACTOR + FACTOR // 2
            goal_y_scaled = self.active_goal[1] * FACTOR + FACTOR // 2
            self.goal_plot.set_data(goal_x_scaled, goal_y_scaled)

            # Plot the path (straight line from robot to goal)
            self.path_plot.set_data(
                [self.current_x, goal_x_scaled],
                [self.current_y, goal_y_scaled]
            )
        plt.pause(0.001)

    def calculate_bearing(self):
        """Calculate the bearing and forward command to the goal."""
        goal_x_scaled = self.active_goal[0] * FACTOR + FACTOR // 2
        goal_y_scaled = self.active_goal[1] * FACTOR + FACTOR // 2
        rospy.loginfo(f"goal_y_scaled is {goal_x_scaled}, y is {goal_y_scaled}")
        delta_x = goal_x_scaled - self.current_x
        delta_y = goal_y_scaled - self.current_y
        rospy.loginfo(f"dx is {delta_x}, dy is {delta_y}")
        dist_to_goal = math.sqrt(delta_x**2 + delta_y**2) * IN_TO_CM # in cm
        rospy.loginfo(f"dist to goal is {dist_to_goal} in cm")
        goal_bearing = math.atan2(delta_y, delta_x)
        angle_to_goal = (goal_bearing - self.current_theta) % (2 * math.pi)
        rospy.loginfo(f"angle to goal is {np.degrees(angle_to_goal)}")
        return dist_to_goal, angle_to_goal
    
    def send_command(self, dist_to_goal, angle_to_goal):
        """Formulate and send the command"""
        #dist_to_goal, angle_to_goal = self.calculate_bearing()

        if angle_to_goal > math.pi:
            angle_to_goal -= 2 * math.pi
            rospy.loginfo(f"angle to goal is now {np.degrees(angle_to_goal)}")
        if abs(np.degrees(angle_to_goal)) < 5:
            rospy.loginfo(f"Discarding the angle {np.degrees(angle_to_goal)}, rounding to zero")
            angle_to_goal = 0

        if angle_to_goal > 0:
            turn_command = f"a:{int(np.degrees(angle_to_goal))}"
        else:
            turn_command = f"d:{int(abs(np.degrees(angle_to_goal)))}"
        forward_command = f"w:{int(dist_to_goal)}"
        combined_command = f"{turn_command},{forward_command}"

        rospy.loginfo(f"Commands to goal: {combined_command}")
        self.bearing_pub.publish(combined_command)


    def run(self):
        """Main loop for navigation."""
        rate = rospy.Rate(10)
        
        command_sent = False    # only send one command
        self.init_plot()
        while not rospy.is_shutdown():
            if self.active_goal:
                if distance < THRESHOLD*IN_TO_CM:
                    rospy.loginfo(f"Goal reached: {self.active_goal}")
                if not command_sent:
                    distance, angle = self.calculate_bearing()
                    self.send_command(distance, angle)
                    command_sent = True

            self.update_plot()
            rate.sleep()

if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            rospy.logerr("Usage: navigation_node.py x,y")
            sys.exit(1)

        try:
            goal_whole = sys.argv[1]
            x, y = map(int, goal_whole.split(","))
            goals = [(x, y)]
        except ValueError:
            rospy.logerr("Invalid goal format. Use x,y.")
            sys.exit(1)

        navigation_node = NavigationNode(goals)
        navigation_node.run()
    except rospy.ROSInterruptException:
        pass
