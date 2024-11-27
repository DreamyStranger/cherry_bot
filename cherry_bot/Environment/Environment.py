import pygame
import numpy as np
import math
import random

class Environment:
    # Class variables
    width = 8  # Default width in meters
    height = 6  # Default height in meters
    scale = 100  # Default scale (pixels per meter)

    # Predefined walls (known map)
    walls = []

    # Predefined dynamic obstacles (unknown to the robot)
    obstacles = [
        {'x': 2, 'y': 0.3, 'width': 0.5, 'height': 0.5},
        {'x': 2, 'y': 2, 'width': 0.5, 'height': 0.5},
        {'x': 2, 'y': 5, 'width': 0.5, 'height': 0.5},
        {'x': 6, 'y': 4, 'width': 0.5, 'height': 1},
        {'x': 7, 'y': 1, 'width': 0.5, 'height': 1},
    ]

    robot_radius = 10
    true_pose = ()
    ekf_pose = ()
    true_path = []
    ekf_path = []

    goal = None
    lidar_data = []

    @classmethod
    def set_dimensions(cls, width, height, scale=100):
        """Set environment dimensions and scale."""
        cls.width = width
        cls.height = height
        cls.scale = scale
        cls.update_walls()

    @classmethod
    def update_true_pose(cls, x, y):
        """Update the true pose of the robot."""
        if cls.true_pose:
            cls.true_path.append(cls.true_pose)
        cls.true_pose = (x, y)

    @classmethod
    def update_ekf_pose(cls, x, y):
        """Update the EKF-estimated pose of the robot."""
        if cls.ekf_pose:
            cls.ekf_path.append(cls.ekf_pose)
        cls.ekf_pose = (x, y)

    @classmethod
    def update_goal(cls, goal):
        """Update the goal position."""
        cls.goal = goal

    @classmethod
    def update_walls(cls):
        """Recompute wall positions if dimensions change."""
        cls.walls = [
            (0, 0, cls.width, 0),
            (0, cls.height, cls.width, cls.height),
            (0, 0, 0, cls.height),
            (cls.width, 0, cls.width, cls.height),
            (0, cls.height // 2, cls.width // 2.5, cls.height // 2),
            (2 * cls.width // 3, cls.height // 2, cls.width, cls.height // 2),
            (cls.width // 2, 0, cls.width // 2, cls.height // 3),
            (cls.width // 2, 2 * cls.height // 3, cls.width // 2, cls.height),
            (0, 2 * cls.height // 3, cls.width // 3, 2 * cls.height // 3),
            (3 * cls.width // 4, 0, 3 * cls.width // 4, cls.height // 3),
        ]

    @classmethod
    def simulate_lidar(cls, num_rays=360, max_range=5.0):
        """Simulate a LiDAR scan."""
        cls.lidar_data = cls._compute_lidar_scan(num_rays, max_range)
        return cls.lidar_data

    @staticmethod
    def _compute_lidar_scan(num_rays, max_range):
        """Dummy method to compute LiDAR scans."""
        return [max_range] * num_rays

    @classmethod
    def render(cls, screen):
        """Render the environment."""
        cls._draw_walls(screen)
        cls._draw_obstacles(screen)
        cls._draw_goal(screen)
        cls._draw_paths(screen)
        cls._draw_robots(screen)

    @classmethod
    def _draw_robots(cls, screen):
        """Draw true and estimated locations of the robot."""
        if cls.true_pose:
            pygame.draw.circle(screen, (0, 0, 255), cls.true_pose, cls.robot_radius)
        if cls.ekf_pose:
            pygame.draw.circle(screen, (128, 0, 128), cls.ekf_pose, cls.robot_radius)

    @classmethod
    def _draw_goal(cls, screen):
        """Draw the goal position."""
        if cls.goal:
            pygame.draw.circle(screen, (0, 255, 0), cls.goal, cls.robot_radius // 3)

    @classmethod
    def _draw_walls(cls, screen):
        """Draw static walls on the screen."""
        wall_thickness = 20  # Wall thickness in pixels
        for x1, y1, x2, y2 in cls.walls:
            pygame.draw.line(
                screen, (0, 0, 0),  # Black color
                cls._scale_point((x1, y1)),
                cls._scale_point((x2, y2)),
                wall_thickness
            )

    @classmethod
    def _draw_obstacles(cls, screen):
        """Draw dynamic obstacles as black rectangles."""
        for obstacle in cls.obstacles:
            rect = pygame.Rect(
                obstacle['x'] * cls.scale,
                obstacle['y'] * cls.scale,
                obstacle['width'] * cls.scale,
                obstacle['height'] * cls.scale
            )
            pygame.draw.rect(screen, (0, 0, 0), rect)

    @classmethod
    def _draw_paths(cls, screen):
        """Draw the paths of the true and EKF poses."""
        if len(cls.true_path) > 1:
            pygame.draw.lines(
                screen, (0, 0, 255), False, [p for p in cls.true_path], 3
            )
        if len(cls.ekf_path) > 1:
            pygame.draw.lines(
                screen, (128, 0, 128), False, [p for p in cls.ekf_path], 3
            )

    @classmethod
    def _scale_point(cls, point):
        """Scale a point from meters to pixels."""
        x, y = point
        return int(x * cls.scale), int(y * cls.scale)
    
    @classmethod
    def simulate_lidar(cls, pose, num_rays=180, max_range=5.0, noise_std=0.01):
        """
        Simulate LiDAR rays from the given pose.
        :param pose: The true pose of the robot (x, y, theta).
        :param num_rays: Number of rays (default: 180 for half-circle).
        :param max_range: Maximum range of the LiDAR (default: 5 meters).
        :param noise_std: Standard deviation of Gaussian noise to add to distances.
        :return: List of distances for each ray (with noise).
        """
        x, y, theta = pose
        angles = cls._compute_ray_angles(theta, num_rays)
        distances = []

        for angle in angles:
            # Check for collisions and compute the intersection distance
            distance = cls._process_ray((x, y), angle, max_range)
            # Add Gaussian noise to simulate real-world imperfections
            distance_with_noise = max(0, distance + random.gauss(0, noise_std))
            distances.append(distance_with_noise)

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

        min_distance = max_range

        # Check intersections with walls
        for wall in cls.walls:
            intersection = cls._check_ray_wall_collision((x1, y1), (x2, y2), wall)
            if intersection:
                distance = math.hypot(intersection[0] - x1, intersection[1] - y1)
                min_distance = min(min_distance, distance)

        # Check intersections with obstacles
        for obstacle in cls.obstacles:
            obstacle_distance = cls._check_ray_obstacle_collision((x1, y1), (x2, y2), obstacle)
            if obstacle_distance:
                min_distance = min(min_distance, obstacle_distance)

        return min_distance

    @staticmethod
    def _check_ray_wall_collision(ray_start, ray_end, wall):
        """
        Check if a ray intersects with a wall and return the intersection point.
        :param ray_start: Start of the ray (x1, y1).
        :param ray_end: End of the ray (x2, y2).
        :param wall: Wall as a line segment (x3, y3, x4, y4).
        :return: Intersection point (xi, yi) or None if no intersection.
        """
        x1, y1 = ray_start
        x2, y2 = ray_end
        x3, y3, x4, y4 = wall

        # Compute intersection using line equations
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denominator == 0:  # Parallel lines
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
    def _check_ray_obstacle_collision(ray_start, ray_end, obstacle):
        """
        Check if a ray intersects with a rectangular obstacle.
        :param ray_start: Start of the ray (x1, y1).
        :param ray_end: End of the ray (x2, y2).
        :param obstacle: Obstacle as a dictionary {'x', 'y', 'width', 'height'}.
        :return: Distance to the obstacle or None if no intersection.
        """
        x, y, width, height = obstacle['x'], obstacle['y'], obstacle['width'], obstacle['height']
        corners = [
            (x, y),  # Bottom-left
            (x + width, y),  # Bottom-right
            (x + width, y + height),  # Top-right
            (x, y + height)  # Top-left
        ]

        edges = [
            (corners[0], corners[1]),
            (corners[1], corners[2]),
            (corners[2], corners[3]),
            (corners[3], corners[0])
        ]

        min_distance = None
        for edge in edges:
            intersection = Environment._check_ray_wall_collision(ray_start, ray_end, edge)
            if intersection:
                distance = math.hypot(intersection[0] - ray_start[0], intersection[1] - ray_start[1])
                if min_distance is None or distance < min_distance:
                    min_distance = distance

        return min_distance
    
    @classmethod
    def render_lidar(cls, screen, ekf_pose, lidar_data):
        """
        Render LiDAR rays on the screen.
        :param screen: Pygame screen for rendering.
        :param ekf_pose: EKF-estimated pose (scaled, (x, y, theta)).
        :param lidar_data: List of distances for each ray.
        """
        x, y, theta = ekf_pose  # EKF pose as starting point
        angles = np.linspace(-math.pi / 2, math.pi / 2, len(lidar_data)) + theta  # Global angles for rays

        for i, distance in enumerate(lidar_data):
            # Compute end point of the ray
            x_end = x + distance * math.cos(angles[i]) * cls.scale
            y_end = y + distance * math.sin(angles[i]) * cls.scale

            # Draw the ray as a line
            pygame.draw.line(screen, (255, 0, 0), (x, y), (x_end, y_end), 1)

