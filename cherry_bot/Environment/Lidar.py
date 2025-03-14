import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from math import atan2, pi
from .Environment import Environment
from cherry_bot.globals import lidar_rate

class Lidar(Node):
    def __init__(self):
        super().__init__('LiDAR')

        self.pose = (0, 0, 0)  # Robot's true pose (x, y, theta)
        self.num_rays = 180  # Number of LiDAR rays
        self.max_range = 2.0  # Maximum LiDAR range in meters

        # Subscribe to the true_pose topic
        self.create_subscription(PoseStamped, 'true_pose', self.pose_callback, 10)

        # Publisher for LiDAR scan
        self.lidar_publisher = self.create_publisher(LaserScan, 'lidar_scan', 10)

        # Timer to publish LiDAR scans at regular intervals (100 Hz)
        self.timer = self.create_timer(lidar_rate, self.publish_lidar_scan)

    def pose_callback(self, msg):
        """Update the robot's true position and orientation."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = 2 * atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.pose = (x, y, theta)

    def publish_lidar_scan(self):
        """Simulate and publish a LiDAR scan."""
        if self.pose is None:
            self.get_logger().warn("Pose not received yet. Skipping LiDAR scan.")
            return

        # Compute LiDAR scan using the Environment class
        ranges = Environment.simulate_lidar(self.pose)

        # Populate the LaserScan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link"  # Replace with a configurable parameter if needed
        scan.angle_min = -pi / 2  # Half-circle in front of the robot
        scan.angle_max = pi / 2
        scan.angle_increment = (scan.angle_max - scan.angle_min) / self.num_rays
        scan.range_min = 0.0
        scan.range_max = self.max_range
        scan.ranges = ranges

        # Publish the LiDAR scan
        self.lidar_publisher.publish(scan)
        self.get_logger().debug(f"Published LiDAR scan with {len(ranges)} rays.")

def main(args=None):
    rclpy.init(args=args)
    lidar_node = Lidar()

    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        lidar_node.get_logger().info("LiDAR Node stopped.")
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
