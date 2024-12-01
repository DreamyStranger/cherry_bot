import pygame
import numpy as np
import math
import random
from .Environment import Environment
from cherry_bot.globals import robot_radius

class Renderer:
    @classmethod
    def render(cls, screen):
        """Render the environment."""
        cls._draw_walls(screen)
        cls._draw_obstacles(screen)
        cls._draw_lidar(screen, Environment.ekf_pose, Environment.lidar_data)
        cls._draw_paths(screen)
        cls._draw_robots(screen)
        cls._draw_goal(screen)

    @classmethod
    def _draw_lidar(cls, screen, ekf_pose, lidar_data):
        """
        Render LiDAR rays on the screen.
        :param screen: Pygame screen for rendering.
        :param ekf_pose: EKF-estimated pose (unscaled, (x, y, theta)).
        :param lidar_data: List of distances for each ray.
        """
        if not lidar_data or not ekf_pose:
            return
        
        x, y, theta = ekf_pose
        # Scale EKF pose
        screen_width = screen.get_width()
        screen_height = screen.get_height()

        x = screen_width // 2 + int(x * Environment.scale)
        y = screen_height // 2 + int(y * Environment.scale)


        # Compute global angles for the rays
        angles = np.linspace(-math.pi / 2, math.pi / 2, len(lidar_data)) + theta

        for i, distance in enumerate(lidar_data):
            # Scale distance
            distance = min(distance, Environment.lidar_range)
            distance *= Environment.scale

            # Compute the end point of the ray
            x_end = int(x + distance * math.cos(angles[i]))
            y_end = int(y + distance * math.sin(angles[i]))

            # Draw the ray as a line
            pygame.draw.line(screen, (255, 0, 0), (x, y), (x_end, y_end), 1)

    @classmethod
    def _draw_robots(cls, screen):
        """Draw true and estimated locations of the robot."""
        screen_width = screen.get_width()
        screen_height = screen.get_height()

        if Environment.true_pose:
            # Scale and shift true pose
            x = screen_width // 2 + int(Environment.true_pose[0] * Environment.scale)
            y = screen_height // 2 + int(Environment.true_pose[1] * Environment.scale)
            pygame.draw.circle(screen, (0, 0, 255), (x, y), robot_radius * Environment.scale)

        if Environment.ekf_pose:
            # Scale and shift EKF pose
            x = screen_width // 2 + int(Environment.ekf_pose[0] * Environment.scale)
            y = screen_height // 2 + int(Environment.ekf_pose[1] * Environment.scale)
            pygame.draw.circle(screen, (128, 0, 128), (x, y), robot_radius * Environment.scale)

    @classmethod
    def _draw_paths(cls, screen):
        """Draw the paths of the true and EKF poses."""
        screen_width = screen.get_width()
        screen_height = screen.get_height()

        if len(Environment.true_path) > 1:
            true_path_scaled = [
                (
                    screen_width // 2 + int(p[0] * Environment.scale),
                    screen_height // 2 + int(p[1] * Environment.scale),
                )
                for p in Environment.true_path
            ]
            pygame.draw.lines(screen, (0, 0, 255), False, true_path_scaled, 3)

        if len(Environment.ekf_path) > 1:
            ekf_path_scaled = [
                (
                    screen_width // 2 + int(p[0] * Environment.scale),
                    screen_height // 2 + int(p[1] * Environment.scale),
                )
                for p in Environment.ekf_path
            ]
            pygame.draw.lines(screen, (128, 0, 128), False, ekf_path_scaled, 3)

    @classmethod
    def _draw_goal(cls, screen):
        """Draw the goal position."""
        if Environment.goal:
            # Scale and shift goal position
            x = screen.get_width() // 2 + int(Environment.goal[0] * Environment.scale)
            y = screen.get_height() // 2 + int(Environment.goal[1] * Environment.scale)
            pygame.draw.circle(screen, (0, 255, 0), (x, y), robot_radius * Environment.scale // 3)

    @classmethod
    def _draw_walls(cls, screen):
        """Draw static walls as rectangles on the screen."""
        for wall in Environment.walls:
            rect = pygame.Rect(
                int(wall['x'] * Environment.scale),  # Scale x-coordinate
                int(wall['y'] * Environment.scale),  # Scale y-coordinate
                int(wall['width'] * Environment.scale),  # Scale width
                int(wall['height'] * Environment.scale)  # Scale height
            )
            pygame.draw.rect(screen, (0, 0, 0), rect)  # Black color for walls

    @classmethod
    def _draw_obstacles(cls, screen):
        """Draw dynamic obstacles as black rectangles."""
        for obstacle in Environment.obstacles:
            rect = pygame.Rect(
                obstacle['x'] * Environment.scale,
                obstacle['y'] * Environment.scale,
                obstacle['width'] * Environment.scale,
                obstacle['height'] * Environment.scale
            )
            pygame.draw.rect(screen, (0, 0, 0), rect)