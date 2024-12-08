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


class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.path = None
        self.g = 0  # Cost from start node to current node
        self.h = 0  # Heuristic cost to goal
        self.f = 0  # Total cost

    def __eq__(self, other):
        return self.position == other.position


class NavigationNode:
    def __init__(self, goals):
        # Define map (static for now)
        self.map_data = np.array([
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 1, 0, 1],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 0, 1, 0, 1]
        ])
        
        rospy.init_node('navigation_node')
        self.current_x, self.current_y, self.current_theta = 0, 0, 0
        self.goals = goals
        self.active_goal = self.goals.pop(0) if self.goals else None

        #rospy.Subscriber('/predicted_position', Point, self.position_callback)
        self.current_x, self.current_y, self.current_theta = 45, 90, -90
        self.bearing_pub = rospy.Publisher('/bearing_to_goal', String, queue_size=10)
        
        rospy.loginfo("Navigation node initialized.")
        if self.active_goal:
            rospy.loginfo(f"Current goal is {self.active_goal}.")

    def position_callback(self, msg):
        """Update the robot's current estimated position."""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.z
        rospy.loginfo(f"Received new position: ({self.current_x}, {self.current_y}) at {np.degrees(self.current_theta)}°.")
        

    def calculate_bearing(self):
        """Calculate the bearing and forward command to the goal."""
        goal_x_scaled = self.active_goal[0] * FACTOR + FACTOR // 2
        goal_y_scaled = self.active_goal[1] * FACTOR + FACTOR // 2

        delta_x = goal_x_scaled - self.current_x
        delta_y = goal_y_scaled - self.current_y
        dist_to_goal = math.sqrt(delta_x**2 + delta_y**2) * IN_TO_CM # in cm

        goal_bearing = math.atan2(delta_y, delta_x)
        angle_to_goal = (goal_bearing - self.current_theta) % (2 * math.pi)
        
        return dist_to_goal, angle_to_goal
    
    def send_command(self, dist_to_goal, angle_to_goal):
        """Formulate and send the command"""
        #dist_to_goal, angle_to_goal = self.calculate_bearing()
        
        if angle_to_goal > math.pi:
            angle_to_goal -= 2 * math.pi
            
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
        
    def send_command_to_step(self, step):
        """Send movement commands to reach the next step in the path."""
        goal_x_scaled = step[1] * FACTOR + FACTOR // 2
        goal_y_scaled = step[0] * FACTOR + FACTOR // 2

        delta_x = goal_x_scaled - self.current_x
        delta_y = goal_y_scaled - self.current_y
        dist_to_step = math.sqrt(delta_x**2 + delta_y**2) * IN_TO_CM

        step_bearing = math.atan2(delta_y, delta_x)
        angle_to_step = (step_bearing - self.current_theta) % (2 * math.pi)

        self.send_command(dist_to_step, angle_to_step)

  
    # start with an empty list
    # add the starting node to the list
    # look for the lowest F square on the list (curr node)
    # look at the 8 cells around us. if an obstacle or a wall - skip
    # else, add it to the open list if it's not already there. 
    def a_star(self, start, end):
        """A* Pathfinding to find the shortest path from start to end."""
        rospy.loginfo(f"Attempting A* from {start} to {end}.")

        # Check if start and end are walkable
        if self.map_data[start[0]][start[1]] != 1:
            rospy.logerr(f"Starting cell {start} is not walkable.")
            return None

        if self.map_data[end[0]][end[1]] != 1:
            rospy.logerr(f"Goal cell {end} is not walkable.")
            return None

        # Create start and end nodes
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        open_list = []
        closed_list = []
        open_list.append(start_node)

        while open_list:
            current_node = min(open_list, key=lambda node: node.f)
            open_list.remove(current_node)
            closed_list.append(current_node)

            if current_node.position == end_node.position:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                rospy.loginfo(f"Path found: {path[::-1]}")
                return path[::-1]

            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
                node_position = (
                    current_node.position[0] + new_position[0],
                    current_node.position[1] + new_position[1],
                )

                if (
                    node_position[0] < 0 or node_position[0] >= len(self.map_data) or
                    node_position[1] < 0 or node_position[1] >= len(self.map_data[0])
                ):
                    continue

                if self.map_data[node_position[0]][node_position[1]] == 0:
                    continue

                new_node = Node(current_node, node_position)
                children.append(new_node)

            for child in children:
                if child in closed_list:
                    continue

                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2 +
                        (child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                for open_node in open_list:
                    if child == open_node and child.g >= open_node.g:
                        break
                else:
                    open_list.append(child)
                    rospy.loginfo(f"Adding child {child.position} to the open list.")

        rospy.logwarn(f"No path found to {end}!")
        return None

    def init_plot(self):
        """Initialize the plot."""
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.cmap = ListedColormap(['grey', 'white'])  # 0 = obstacle, 1 = free

        # Scale the map
        self.scaled_map = np.kron(self.map_data, np.ones((FACTOR, FACTOR)))

        # Debug scaled map values
        rospy.loginfo(f"Scaled map:\n{self.scaled_map}")

        # Display the scaled map
        self.ax.imshow(self.scaled_map, cmap=self.cmap, origin="lower")

        # Initialize scatter plots
        self.robot_plot, = self.ax.plot([], [], 'ro', label='Robot')  # Robot position
        self.goal_plot, = self.ax.plot([], [], 'go', label='Goal')    # Goal position
        self.path_plot, = self.ax.plot([], [], 'b--', label='Path')   # Planned path

        # Configure the plot
        self.ax.set_xlim(0, self.scaled_map.shape[1])
        self.ax.set_ylim(0, self.scaled_map.shape[0])
        self.ax.legend()
        plt.ion()
        plt.show()

    def update_plot(self, path=None):
        """Update the plot dynamically with the robot's position, goal, and path."""
        self.robot_plot.set_data(self.current_x, self.current_y)  # Robot position

        if self.active_goal:
            goal_x_scaled = self.active_goal[0] * FACTOR + FACTOR // 2
            goal_y_scaled = self.active_goal[1] * FACTOR + FACTOR // 2
            self.goal_plot.set_data(goal_x_scaled, goal_y_scaled)

        if path:
            scaled_path = [(p[1] * FACTOR + FACTOR // 2, p[0] * FACTOR + FACTOR // 2) for p in path]
            path_x, path_y = zip(*scaled_path)
            self.path_plot.set_data(path_x, path_y)

        plt.pause(0.001)

    def run(self):
        """Main loop for navigation."""
        rate = rospy.Rate(10)
        self.init_plot()

        while not rospy.is_shutdown():
            if self.active_goal:
                start = (self.current_y // FACTOR, self.current_x // FACTOR)
                rospy.loginfo(f"Starting cell is {start}")
                self.path = self.a_star(start, self.active_goal)

                if self.path:
                    rospy.loginfo(f"Path to goal: {self.path}")
                    self.update_plot(self.path)

                    for step in self.path[1:]:
                        self.send_command_to_step(step)
                else:
                    rospy.logwarn(f"No path found to {self.active_goal}. Skipping goal.")
                    self.active_goal = self.goals.pop(0) if self.goals else None

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
