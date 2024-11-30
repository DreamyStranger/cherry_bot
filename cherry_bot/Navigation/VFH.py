import numpy as np
from math import atan2, pi

class VectorFieldHistogram:
    def __init__(self, num_bins=36, smoothing_factor=0.8, valley_threshold=0.1, obstacle_threshold=1, max_linear_velocity=0.2, angular_gain=3.0):
        self.num_bins = num_bins  # Number of bins in the polar histogram
        self.smoothing_factor = smoothing_factor  # Low-pass filter smoothing factor
        self.valley_threshold = valley_threshold  # Threshold to identify valleys
        self.obstacle_threshold = obstacle_threshold  # Maximum range to consider obstacles
        self.max_linear_velocity = max_linear_velocity  # Maximum forward velocity
        self.angular_gain = angular_gain  # Gain for angular velocity
        self.histogram = np.zeros(self.num_bins)  # Initialize histogram
        self.previous_histogram = np.zeros(self.num_bins)  # For smoothing
        self.goal = None  # Goal position (x, y)

    def set_goal(self, goal_x, goal_y):
        """Set the goal position."""
        self.goal = (goal_x, goal_y)

    def compute(self, robot_pose, lidar_ranges, angle_min, angle_increment):
        """
        Main VFH method to compute velocity commands.
        :param robot_pose: Current robot pose (x, y, theta).
        :param lidar_ranges: LiDAR range data.
        :param angle_min: Minimum angle of the LiDAR scan.
        :param angle_increment: Angular increment between LiDAR beams.
        :return: (linear_velocity, angular_velocity)
        """
        if self.goal is None:
            return 0.0

        # Step 2: Build the polar histogram
        raw_histogram = self.build_polar_histogram(lidar_ranges, angle_min, angle_increment)

        # Step 3: Smooth the histogram
        self.histogram = self.smoothing_factor * self.previous_histogram + (1 - self.smoothing_factor) * raw_histogram
        self.previous_histogram = self.histogram.copy()

        # Step 4: Find valleys
        valleys = self.find_valleys(self.histogram)

        # Step 5: Compute the goal angle
        goal_angle = self.compute_goal_angle(robot_pose)

        # Step 6: Select the best steering angle
        steering_angle = self.select_steering_angle(valleys, goal_angle)

        # Step 7: Compute velocity commands
        angular_velocity = self.compute_velocity(robot_pose, steering_angle)

        return angular_velocity

    def build_polar_histogram(self, ranges, angle_min, angle_increment):
        """
        Build a polar histogram from LiDAR data.
        :param ranges: LiDAR range data.
        :param angle_min: Start angle of the scan.
        :param angle_increment: Increment between consecutive rays.
        :return: List of histogram values.
        """
        histogram = np.zeros(self.num_bins)
        bin_width = 2 * pi / self.num_bins  # Bin width in radians

        for i, distance in enumerate(ranges):
            if distance < self.obstacle_threshold:  # Only consider obstacles within threshold
                angle = angle_min + i * angle_increment
                bin_index = int((angle + pi) // bin_width) % self.num_bins
                histogram[bin_index] += 1 / (distance ** 2)  # Inverse distance weight

        return histogram

    def find_valleys(self, histogram):
        """
        Identify valleys (collision-free regions) in the histogram.
        :param histogram: Polar histogram values.
        :return: List of valleys as (start_bin, end_bin).
        """
        valleys = []
        in_valley = False
        start_bin = None

        for i, value in enumerate(histogram):
            if value < self.valley_threshold and not in_valley:
                start_bin = i
                in_valley = True
            elif value >= self.valley_threshold and in_valley:
                valleys.append((start_bin, i - 1))
                in_valley = False

        if in_valley:  # Handle case where valley wraps around
            valleys.append((start_bin, len(histogram) - 1))

        return valleys

    def compute_goal_angle(self, robot_pose):
        """
        Compute the angle to the goal relative to the robot.
        :param robot_pose: Current robot pose (x, y, theta).
        :return: Goal angle in radians.
        """
        dx = self.goal[0] - robot_pose[0]
        dy = self.goal[1] - robot_pose[1]
        return atan2(dy, dx)

    def select_steering_angle(self, valleys, goal_angle):
        """
        Select the best steering angle based on valleys and goal direction.
        :param valleys: List of valleys as (start_bin, end_bin).
        :param goal_angle: Goal direction in radians.
        :return: Selected steering angle in radians.
        """
        def normalize_angle(angle):
            return (angle + pi) % (2 * pi) - pi

        best_valley = None
        min_cost = float('inf')

        for start, end in valleys:
            center_bin = (start + end) // 2
            center_angle = (center_bin / self.num_bins) * 2 * pi - pi

            # Cost based on alignment with the goal
            cost = abs(normalize_angle(center_angle - goal_angle))
            if cost < min_cost:
                min_cost = cost
                best_valley = center_bin

        if best_valley is not None:
            return (best_valley / self.num_bins) * 2 * pi - pi  # Convert bin index to angle
        else:
            return goal_angle  # Default to goal angle if no valleys found

    def compute_velocity(self, robot_pose, steering_angle):
        """
        Compute velocity commands based on the steering angle.
        :param robot_pose: Current robot pose (x, y, theta).
        :param steering_angle: Selected steering angle in radians.
        :return: Linear and angular velocities.
        """
        def normalize_angle(angle):
            return (angle + pi) % (2 * pi) - pi
        
        angular_velocity = self.angular_gain * normalize_angle(steering_angle - robot_pose[2])
        return angular_velocity
