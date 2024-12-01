import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Use Float32MultiArray for lists of floats
from cherry_bot.Environment.Environment import Environment

from cherry_bot.Navigation.Astar import AStarPathPlanner
from cherry_bot.globals import goal_x, goal_y


class GoalHandler(Node):
    def __init__(self):
        super().__init__('goals')

        # Publisher for the combined x and y coordinates
        self.path_publisher = self.create_publisher(Float32MultiArray, 'goals', 10)

        # Predefined goal (in meters)
        self.goal_x = goal_x + Environment.width / 2
        self.goal_y = goal_y + Environment.height / 2

        # Initialize A* path planner
        self.planner = AStarPathPlanner()

        # Current robot position (for simplicity, assume a known start position)
        self.start_x = Environment.width / 2
        self.start_y = Environment.height / 2

        # Timer to wait for 4 seconds before publishing the path
        self.timer = self.create_timer(2.0, self.publish_path)

    def publish_path(self):
        """Generate the path using A* and publish it as a combined list of x and y coordinates."""
        self.timer.cancel()  # Cancel the timer after publishing the path

        path = self.planner.find_path(self.start_x, self.start_y, self.goal_x, self.goal_y)

        if path:
            # Flatten the path into a single list: [x1, y1, x2, y2, ...]
            combined_path = [coord for point in path for coord in point]

            # Create Float32MultiArray message
            path_msg = Float32MultiArray()
            path_msg.data = combined_path

            # Publish the path
            self.path_publisher.publish(path_msg)

            self.get_logger().info(f"Published path with {len(path)} waypoints.")
        else:
            self.get_logger().warn("No path found to the goal.")


def main(args=None):
    rclpy.init(args=args)
    goal_handler = GoalHandler()

    try:
        rclpy.spin(goal_handler)
    except KeyboardInterrupt:
        print("GoalHandler interrupted.")
    finally:
        goal_handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
