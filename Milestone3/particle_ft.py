#!/usr/bin/env python3

import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
import std_msgs.msg
from matplotlib.colors import ListedColormap
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from matplotlib.patches import Circle, FancyArrow

FACTOR = 12 # scale the map
NUM_PARTICLES = 200
LIDAR_NOISE_STD = 6
MOVE_NOISE_STD = np.array([0.1, 0.1])
M_TO_IN = 39.3701
BIAS = 0.6

class ParticleFilterNode:
    def __init__(self):
        rospy.init_node('particle_ft_node')
        self.predicted_position_pub = rospy.Publisher('/predicted_position', Point, queue_size=10)
        rospy.Subscriber('/navigation_bearing', String, self.bearing_callback)
        rospy.loginfo("Now initializing the particle_ft node.")
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.fixed_angles = np.linspace(-np.pi, np.pi, 36, endpoint=False) # linspace with angles, 10 deg a>
        self.best_particle = np.array([40, 20, 0])
        self.lidar_dir = np.zeros(2)
        self.robot_orientation = 0
        self.navigation_bearing = None
        self.nav_dist = 0
        self.nav_angle = 0

        self.map_data = np.array([
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 1, 0, 0, 1, 0, 1],
            [1, 1, 0, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 0, 1, 0, 1]
        ])

        self.scaled_map = np.kron(self.map_data, np.ones((FACTOR, FACTOR)))

        self.particle_count = NUM_PARTICLES
        self.particles = self.initialize_particles()
        self.move_noise_std = MOVE_NOISE_STD
        self.lidar_noise_std = LIDAR_NOISE_STD
        # Store lidar readings
        self.weights = np.ones(self.particle_count) / self.particle_count
        self.drivable_areas = np.argwhere(self.scaled_map > 0)
        self.lidar_readings = []

        self.init_plot()

    def initialize_particles(self):
        """Initialize particles randomly within the free space of the map."""
        height, width = self.scaled_map.shape
        particles = np.zeros((self.particle_count, 3))  # x, y, theta
        for i in range(self.particle_count):
            while True:
                x = np.random.randint(0, width)
                y = np.random.randint(0, height)
                if self.scaled_map[y, x] == 1:  # only in free space
                    particles[i, :] = [x, y, np.random.uniform(0, 360)]
                    break
        return particles

    def insert_random_particles(self, num_to_inject=10):
        """
        Add new random particles into the maze for diversity.
        """
        new_particles = []
        height, width = self.scaled_map.shape
        for _ in range(num_to_inject):
            while True:
                x = np.random.randint(0, width)
                y = np.random.randint(0, height)
                if self.scaled_map[y, x] == 1:  # free space
                    theta = np.random.uniform(0, 360)
                    new_particles.append([x, y, theta])
                    break
        new_particles = np.array(new_particles)
        self.particles = np.vstack((self.particles, new_particles))
        self.weights = np.hstack((self.weights, np.ones(len(new_particles)) / len(new_particles)))
        self.weights /= np.sum(self.weights)
        
    def bearing_callback(self, msg):
        """Update the bearing for the motion model."""
        combined_cmd = msg.data
        rospy.loginfo(f"Received a new bearing command: {combined_cmd}")
        
        # parse
        commands = combined_cmd.split(",")
        for command in commands:
            dir, value = command.split(":")
            value = float(value)
            
            if dir == "w":  # forward
                self.nav_dist = value
            elif dir == "a":  # CCW
                self.nav_angle = value
            elif dir == "d":  # CW
                self.nav_angle = -value


    def lidar_callback(self, scan_data):
        """Process lidar data."""
        self.lidar_readings = []
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment

        for angle in self.fixed_angles:
            idx = int((angle - angle_min) / angle_increment)
            if 0 <= idx < len(scan_data.ranges):
                distance = scan_data.ranges[idx]
                corrected_angle = angle #- np.pi/2
                if not math.isinf(distance) and not math.isnan(distance) and distance > 0.1:
                    self.lidar_readings.append((corrected_angle, distance))


    def simulate_lidar(self, x, y, particle_theta, measured_theta):
        """
        Simulate lidar reading for a particle at (x, y) with orientation particle_theta,
        looking in the direction of measured_theta.
        """
        global_angle = (particle_theta + np.degrees(measured_theta)) % 360
        dx, dy = np.cos(np.radians(global_angle)), np.sin(np.radians(global_angle))

        distance = 2
        max_distance = min(self.scaled_map.shape[0], self.scaled_map.shape[1])

        while distance < max_distance:
            check_x = int(x + distance * dx)
            check_y = int(y + distance * dy)

            if check_x < 0 or check_x >= self.scaled_map.shape[1] or check_y < 0 or check_y >= self.scaled_>
                return distance  # wall
            if self.scaled_map[check_y, check_x] == 0:  # obstacle
                return distance
            distance += 0.5  # add to the ray distance

        return max_distance # wall
    
    def update_weights(self):
        """
        Update particle weights based on the likelihood of observed lidar readings
        compared to simulated readings for each particle.
        """
        if not self.lidar_readings:
            rospy.logwarn("No lidar readings available to update weights.")
            return

        for i, particle in enumerate(self.particles):
            x, y, theta_particle = particle
            weight = 1.0

            for theta, observed_dist in self.lidar_readings:
                simulated_dist = self.simulate_lidar(x, y, theta_particle, theta)
                likelihood = np.exp(-((simulated_dist - observed_dist * M_TO_IN) ** 2) / (2 * self.lidar_noise))
                weight *= likelihood

                self.weights[i] = weight

        # normalize weights to sum to 1
        self.weights += 1e-10  # Avoid division by zero
        self.weights /= np.sum(self.weights)
        best_idx = np.argmax(self.weights)
        self.best_particle = self.particles[best_idx]


    def motion_model(self):
        """
        Update particle positions based on sensor data and noise.
        Particles move independently (noise).
        """
        for i, particle in enumerate(self.particles):
            x, y, theta = particle
            x += np.random.normal(0, self.move_noise_std[0])
            y += np.random.normal(0, self.move_noise_std[1])
            theta += np.random.normal(0, 5)  # angular noise
            # if we have a bearing to target
            if hasattr(self, "nav_dist"):
                x += BIAS * self.nav_dist * np.cos(np.radians(theta))
                y += BIAS * self.nav_dist * np.sin(np.radians(theta))

            if hasattr(self, "nav_angle"):
                theta += self.nav_angle
                
            self.particles[i] = [x, y, theta % 360]
            

    def systematic_resampling(self):
        """
        Resample particles systematically based on their weights.
        """
        cumsum_weights = np.cumsum(self.weights)
        indices = []
        step = 1.0 / self.particle_count
        rand_offset = np.random.uniform(0, step)
        i = 0

        for particle in range(self.particle_count):
            target = rand_offset + particle * step
            while i < len(cumsum_weights) and target > cumsum_weights[i]:
                i += 1
            if i >= len(cumsum_weights):
                i = len(cumsum_weights) - 1
            indices.append(i)

        for particle in self.particles:
            particle[0] += np.random.normal(0, self.move_noise_std[0]) 
            particle[1] += np.random.normal(0, self.move_noise_std[1])
            particle[2] = (particle[2] + np.random.normal(0, 5)) % 360

        self.particles = self.particles[indices]
        self.weights = np.ones(self.particle_count) / self.particle_count
    
    def check_particle_spread(self):
        """
        Check if particles are too clustered and inject new ones if needed.
        """
        spread_x = np.std(self.particles[:, 0])
        spread_y = np.std(self.particles[:, 1])
        if spread_x < 5 or spread_y < 5:  # Thresholds in map units
            self.insert_random_particles(num_to_inject=10)

    def find_estimate(self):
        """
        Estimate the robot's position based on the weighted average of the particles.
        """
        particles_with_weights = np.column_stack((self.particles, self.weights))
        # Sort by weight in descending order
        particles_with_weights = particles_with_weights[particles_with_weights[:, -1].argsort()[::-1]]

        # Select the top 40-50% of particles
        top_fraction = 0.5  # Adjust as needed
        top_count = int(len(particles_with_weights) * top_fraction)
        top_particles = particles_with_weights[:top_count, :-1]  # Exclude weights
        top_weights = particles_with_weights[:top_count, -1]  # Extract weights

        # Normalize the selected weights
        top_weights /= np.sum(top_weights)

        mean_x = np.average(self.particles[:, 0], weights=self.weights)
        mean_y = np.average(self.particles[:, 1], weights=self.weights)
        mean_theta = np.arctan2(
            np.average(np.sin(np.radians(self.particles[:, 2])), weights=self.weights),
            np.average(np.cos(np.radians(self.particles[:, 2])), weights=self.weights)
        )
        return mean_x, mean_y, mean_theta


    def publish_position(self, est_x, est_y, est_theta):
        predicted_position = Point()
        predicted_position.x = est_x
        predicted_position.y = est_y
        predicted_position.z = est_theta  # Use z for theta (yaw)
        self.predicted_position_pub.publish(predicted_position)


    def init_plot(self):
        """Initialize the plot."""
        # initialize the plot and axes
        self.fig, self.ax = plt.subplots(figsize=(16, 8))
        self.cmap = ListedColormap(['grey', 'white'])  # 0 = obstacle, 1 = free
        self.ax.imshow(self.scaled_map, cmap=self.cmap, origin="lower")
        # initialize an empty scatter plot with blue dots as particles
        self.particles_plot, = self.ax.plot([], [], 'bo', markersize=2, label='Particles')
        self.robot_plot, = self.ax.plot([], [], 'ro', label='Robot')

        plt.xlim(0, self.scaled_map.shape[1])   # x
        plt.ylim(0, self.scaled_map.shape[0])   # y
        self.lidar_lines = []
        # add live updates
        plt.ion()
        plt.show()


    def update_plot(self, est_x, est_y, est_theta):
        """Update the plot dynamically with new data."""
        self.ax.clear()
        self.ax.imshow(self.scaled_map, cmap=self.cmap, origin="lower")

        # update particles
        x_particles = self.particles[:, 0]
        y_particles = self.particles[:, 1]
        self.ax.scatter(x_particles, y_particles, color='blue', s=5, label='Particles')

        # real lidar data        
        for theta, distance in self.lidar_readings:
            full_theta = theta + est_theta
            distance_in = distance * M_TO_IN
            end_x = est_x + distance_in * np.cos(full_theta)
            end_y = est_y + distance_in * np.sin(full_theta)
            color = 'blue'
            if -5 < np.degrees(theta) < 5:
                color = 'orange'
                self.lidar_dir[0] = end_x
                self.lidar_dir[1] = end_y
            self.ax.plot([est_x, end_x], [est_y, end_y], color=color, linewidth=0.5)

        plt.xlim(0, self.scaled_map.shape[1])
        plt.ylim(0, self.scaled_map.shape[0])

        self.ax.legend(loc="upper right")
        plt.pause(0.001)

        # robot circle
        robot_circle = Circle((est_x, est_y), radius=4, color='green', fill=False, label='Estimated Position")
        self.ax.add_patch(robot_circle)

        # heading arrow
        arrow_length = 10  # Adjust arrow length as needed
        arrow_dx = self.lidar_dir[0] - est_x
        arrow_dy = self.lidar_dir[1] - est_y

        # Normalize the arrow direction vector
        arrow_magnitude = math.sqrt(arrow_dx**2 + arrow_dy**2)
        arrow_dx = (arrow_dx / arrow_magnitude) * arrow_length
        arrow_dy = (arrow_dy / arrow_magnitude) * arrow_length
        heading_arrow = FancyArrow(est_x, est_y, arrow_dx, arrow_dy, color='red', width=1, label='Heading')
        self.ax.add_patch(heading_arrow)

        # refresh
        plt.xlim(0, self.scaled_map.shape[1])
        plt.ylim(0, self.scaled_map.shape[0])
        plt.legend(loc="upper right")
        plt.pause(0.001)


    def run(self):
        """Main loop to visualize lidar data and particles."""        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.motion_model()
            self.update_weights()
            self.systematic_resampling()
            x, y, z = self.find_estimate()
            self.publish_position(x, y, z)
            self.check_particle_spread()
            self.update_plot(x, y, z)
            rate.sleep()

if __name__ == '__main__':
    try:
        pf_node = ParticleFilterNode()
        pf_node.run()
    except rospy.ROSInterruptException:
        pass
