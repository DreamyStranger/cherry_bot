import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from math import atan2
from .Environment.Environment import Environment

class PygameVisualizer(Node):
    def __init__(self):
        super().__init__('pygame_visualizer')

        # Visualization settings
        self.width = 800
        self.height = 600
        self.scale = 100  # Scale ROS coordinates to pixels

        # Set up Environment dimensions (using the static class)
        Environment.set_dimensions(self.width // self.scale, self.height // self.scale, self.scale)

        # ROS topic subscriptions
        self.init_subscribers()

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption('Pygame ROS2 Visualization')
        self.clock = pygame.time.Clock()

    def init_subscribers(self):
        """Initialize all ROS subscribers."""
        self.create_subscription(PoseStamped, 'true_pose', self.true_pose_callback, 10)
        self.create_subscription(PoseStamped, 'ekf_pose', self.ekf_pose_callback, 10)
        self.create_subscription(PointStamped, 'goals', self.goal_pose_callback, 10)

    def true_pose_callback(self, msg):
        """Update the robot's true position and orientation."""
        x = self.width // 2 + int(msg.pose.position.x * self.scale)
        y = self.height // 2 - int(msg.pose.position.y * self.scale)
        Environment.update_true_pose(x, y)

    def goal_pose_callback(self, msg):
        """Update the goal position."""
        goal= (
            self.width // 2 + int(msg.point.x * self.scale),
            self.height // 2 - int(msg.point.y * self.scale)
        )
        Environment.update_goal(goal)

    def ekf_pose_callback(self, msg):
        """Update the robot's estimated position from EKF."""
        self.get_logger().info(f"Received EKF pose: x={msg.pose.position.x}, y={msg.pose.position.y}")
        x = self.width // 2 + int(msg.pose.position.x * self.scale)
        y = self.height // 2 - int(msg.pose.position.y * self.scale)

        Environment.update_ekf_pose(x, y)

    def render(self):
        """Render the scene."""
        # Clear the screen
        self.screen.fill((255, 255, 255))

        # Render the environment
        Environment.render(self.screen)

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
