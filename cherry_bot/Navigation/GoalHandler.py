import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class GoalHandler(Node):
    def __init__(self):
        super().__init__('goals')

        # Publisher for the goal
        self.goal_publisher = self.create_publisher(PointStamped, 'goals', 10)

        # Timer to periodically publish the goal (e.g., every 5 seconds)
        self.timer = self.create_timer(5.0, self.publish_goal)

        # Predefined goal (in meters)
        self.goal_x = -3.0
        self.goal_y = -2.0

    def publish_goal(self):
        """Publish the predefined goal."""
        goal_msg = PointStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'  # Frame of reference for the goal
        goal_msg.point.x = self.goal_x
        goal_msg.point.y = self.goal_y
        goal_msg.point.z = 0.0  # Assuming a 2D plane

        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal: x={self.goal_x}, y={self.goal_y}")

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
