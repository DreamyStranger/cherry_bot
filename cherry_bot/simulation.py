import pygame

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer

from .Environment.Environment import Environment
from .Environment.Renderer import Renderer
from cherry_bot.globals import synch_tolerance

from math import atan2
import json


class PygameVisualizer(Node):
    def __init__(self):
        super().__init__('pygame')

        # Visualization settings
        self.width = Environment.width * Environment.scale
        self.height = Environment.height * Environment.scale

        # ROS topic subscriptions
        self.init_subscribers()

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Pygame ROS2 Visualization')
        self.clock = pygame.time.Clock()

    def init_subscribers(self):
        """Initialize all ROS subscribers with synchronization."""
        # Subscribers for message filters
        self.true_pose_sub = Subscriber(self, PoseStamped, 'true_pose')
        self.ekf_pose_sub = Subscriber(self, PoseStamped, 'ekf_pose')
        self.lidar_sub = Subscriber(self, LaserScan, 'lidar_scan')

        # Synchronize topics
        self.sync = ApproximateTimeSynchronizer(
            [self.true_pose_sub, self.ekf_pose_sub, self.lidar_sub], 
            queue_size=10, 
            slop= synch_tolerance # Allow a small time difference
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Separate subscriber for goal
        #self.create_subscription(String, 'path', self.path_callback, 10)

    def synchronized_callback(self, true_pose_msg, ekf_pose_msg, lidar_msg):
        """Handle synchronized true_pose, ekf_pose, and lidar_scan messages."""
        # Process true_pose
        x_true = true_pose_msg.pose.position.x
        y_true = true_pose_msg.pose.position.y
        theta_true = 2 * atan2(true_pose_msg.pose.orientation.z, true_pose_msg.pose.orientation.w)
        true_pose = (x_true, y_true, theta_true)
        Environment.update_true_pose(true_pose)

        # Process ekf_pose
        x_ekf = ekf_pose_msg.pose.position.x
        y_ekf = ekf_pose_msg.pose.position.y
        theta_ekf = 2 * atan2(ekf_pose_msg.pose.orientation.z, ekf_pose_msg.pose.orientation.w)
        ekf_pose = (x_ekf, y_ekf, theta_ekf)
        Environment.update_ekf_pose(ekf_pose)

        # Process LiDAR ranges
        ranges = lidar_msg.ranges
        Environment.lidar_data = ranges  # Store LiDAR ranges for rendering

        self.get_logger().info(f"True Pose: {true_pose_msg.pose.position.x}, {true_pose_msg.pose.position.y}")
        self.get_logger().info(f"EKF Pose: {ekf_pose_msg.pose.position.x}, {ekf_pose_msg.pose.position.y}")
        self.get_logger().info(f"LiDAR Ranges: {len(lidar_msg.ranges)}")


    def path_callback(self, msg):
        """Update the goal position."""
        path = json.loads(msg.data)
        if path:
            goal = path[len(path) - 1]
            #Environment.update_goal(goal)

    def render(self):
        """Render the scene."""
        # Clear the screen
        self.screen.fill((255, 255, 255))
        # Render the environment
        Renderer.render(self.screen)
        # Update the display
        pygame.display.flip()

    def run(self):
        """Run the Pygame event loop while processing ROS2 callbacks."""
        running = True
        while running:
            # Process Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Render the current state
            self.render()

            # Process ROS2 callbacks (non-blocking)
            rclpy.spin_once(self, timeout_sec=0.01)

            # Limit the frame rate
            self.clock.tick(30)

        pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    visualizer = PygameVisualizer()

    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("Pygame Renderer stopped.")
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
