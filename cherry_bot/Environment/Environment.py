import pygame
import numpy as np
import math
import random

from cherry_bot.globals import goal_x, goal_y
class Environment:
    # Class variables
    width = 8  # Default width in meters
    height = 6  # Default height in meters
    scale = 100  # Default scale (pixels per meter)

    # Predefined walls (known map)
    wall_thickness = 0.1
    walls = [
            # Top wall
            {'x': 0, 'y': 0, 'width': width, 'height': wall_thickness},
            # Bottom wall
            {'x': 0, 'y': height - wall_thickness, 'width': width, 'height': wall_thickness},
            # Left wall
            {'x': 0, 'y': 0, 'width': wall_thickness, 'height': height},
            # Right wall
            {'x': width - wall_thickness, 'y': 0, 'width': wall_thickness, 'height': height},
            # Horizontal wall (center-left)
            {'x': 0, 'y': height // 2, 'width': width // 2.5, 'height': wall_thickness},
            # Horizontal wall (center-right)
            {'x': 2 * width // 3, 'y': height // 2, 'width': width // 3, 'height': wall_thickness},
            # Vertical wall (center-top)
            {'x': width // 2, 'y': 0, 'width': wall_thickness, 'height': height // 3},
            # Vertical wall (center-bottom)
            {'x': width // 2, 'y': 2 * height // 3, 'width': wall_thickness, 'height': height // 3},
            # Horizontal wall (bottom-left)
            {'x': 0, 'y': 2 * height // 3, 'width': width // 3, 'height': wall_thickness},
            # Vertical wall (top-right)
            {'x': 3 * width // 4, 'y': 0, 'width': wall_thickness, 'height': height // 3},
        ]

    # Predefined dynamic obstacles (unknown to the robot)
    obstacles = [
        {'x': 2, 'y': 0.3, 'width': 0.5, 'height': 0.5},
        {'x': 2, 'y': 2, 'width': 0.5, 'height': 0.5},
        {'x': 2, 'y': 5, 'width': 0.5, 'height': 0.5},
        {'x': 6, 'y': 4, 'width': 0.5, 'height': 1},
        {'x': 7, 'y': 1, 'width': 0.5, 'height': 1},
    ]

    true_pose = ()
    ekf_pose = ()
    true_path = []
    ekf_path = []

    goal = (goal_x, goal_y)

    lidar_range = 2
    lidar_rays = 180
    lidar_data = []

    @classmethod
    def update_true_pose(cls, pose):
        """Update the true pose of the robot."""
        if cls.true_pose:
            cls.true_path.append(cls.true_pose[:2])
        cls.true_pose = pose

    @classmethod
    def update_ekf_pose(cls, pose):
        """Update the EKF-estimated pose of the robot."""
        if cls.ekf_pose:
            cls.ekf_path.append(cls.ekf_pose[:2])
        cls.ekf_pose = pose

    @classmethod
    def update_goal(cls, goal):
        """Update the goal position."""
        cls.goal = goal
    
    @classmethod
    def simulate_lidar(cls, pose, noise_std=0.01):
        """
        Simulate LiDAR rays from the given pose.
        :param pose: The true pose of the robot (x, y, theta).
        :param num_rays: Number of rays (default: 180 for half-circle).
        :param noise_std: Standard deviation of Gaussian noise to add to distances.
        :return: List of distances for each ray (with noise).
        """
        distances = []
        num_rays = cls.lidar_rays
        max_range = cls.lidar_range + 0.5

        if not pose:
            return distances
        
        # Apply the environment shift (if robot is centered on the screen)
        x, y, theta = pose
        x += cls.width // 2 # Shift horizontally
        y += cls.height // 2 # Shift vertically

        # Compute angles for the rays
        angles = cls._compute_ray_angles(theta, num_rays)
        

        for angle in angles:
            # Check for collisions and compute the intersection distance
            distance = cls._process_ray((x, y), angle, max_range)
            # Add Gaussian noise to simulate real-world imperfections
            distance_with_noise = max(0, distance + random.gauss(0, noise_std))
            distances.append(distance_with_noise)

        cls.lidar_data = distances
        return distances


    @staticmethod
    def _compute_ray_angles(theta, num_rays):
        """
        Compute the global angles for all rays based on the robot's orientation.
        :param theta: Robot's orientation in radians.
        :param num_rays: Number of rays.
        :return: List of global angles for each ray.
        """
        return np.linspace(-math.pi / 2, math.pi / 2, num_rays) + theta

    @classmethod
    def _process_ray(cls, start, angle, max_range):
        """
        Process a single ray to find the closest intersection.
        :param start: The starting point of the ray (x, y).
        :param angle: The direction of the ray in radians.
        :param max_range: The maximum range of the ray.
        :return: Distance to the closest intersection or max_range if no intersection.
        """
        x1, y1 = start
        x2 = x1 + max_range * math.cos(angle)
        y2 = y1 + max_range * math.sin(angle)

        min_distance = max_range + 0.5

        # Check intersections with walls (treated as rectangles)
        for wall in cls.walls:
            wall_distance = cls._check_ray_rectangle_collision((x1, y1), (x2, y2), wall)
            if wall_distance:
                min_distance = min(min_distance, wall_distance)

        # Check intersections with obstacles (also rectangles)
        for obstacle in cls.obstacles:
            obstacle_distance = cls._check_ray_rectangle_collision((x1, y1), (x2, y2), obstacle)
            if obstacle_distance:
                min_distance = min(min_distance, obstacle_distance)

        return min_distance


    @staticmethod
    def _check_ray_line_collision(ray_start, ray_end, line_segment):
        """
        Check if a ray intersects with a line segment.
        :param ray_start: Start of the ray (x1, y1).
        :param ray_end: End of the ray (x2, y2).
        :param line_segment: Line segment as (x3, y3, x4, y4).
        :return: Intersection point (xi, yi) or None if no intersection.
        """
        x1, y1 = ray_start
        x2, y2 = ray_end
        x3, y3, x4, y4 = line_segment

        # Compute intersection using line equations
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if abs(denominator) < 1e-6:  # Parallel or coincident lines
            return None

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
        u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator

        if 0 <= t <= 1 and 0 <= u <= 1:
            # Intersection point
            xi = x1 + t * (x2 - x1)
            yi = y1 + t * (y2 - y1)
            return xi, yi

        return None

    @staticmethod
    def _check_ray_rectangle_collision(ray_start, ray_end, rect):
        """
        Check if a ray intersects with a rectangular region.
        :param ray_start: Start of the ray (x1, y1).
        :param ray_end: End of the ray (x2, y2).
        :param rect: Rectangle as a dictionary {'x', 'y', 'width', 'height'}.
        :return: Distance to the rectangle or None if no intersection.
        """
        x, y, width, height = rect['x'], rect['y'], rect['width'], rect['height']
        corners = [
            (x, y),  # Bottom-left
            (x + width, y),  # Bottom-right
            (x + width, y + height),  # Top-right
            (x, y + height)  # Top-left
        ]

        edges = [
            (corners[0][0], corners[0][1], corners[1][0], corners[1][1]),  # Bottom edge
            (corners[1][0], corners[1][1], corners[2][0], corners[2][1]),  # Right edge
            (corners[2][0], corners[2][1], corners[3][0], corners[3][1]),  # Top edge
            (corners[3][0], corners[3][1], corners[0][0], corners[0][1])   # Left edge
        ]

        min_distance = None
        for edge in edges:
            intersection = Environment._check_ray_line_collision(ray_start, ray_end, edge)
            if intersection:
                distance = math.hypot(intersection[0] - ray_start[0], intersection[1] - ray_start[1])
                if min_distance is None or distance < min_distance:
                    min_distance = distance

        return min_distance

