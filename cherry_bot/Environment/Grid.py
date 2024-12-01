import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Map configuration
        self.width = 800  # Map width in cells
        self.height = 600  # Map height in cells
        self.resolution = 0.01  # 1 cm per cell
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)  # Log-odds map

        # Log odds parameters
        self.log_free = -0.4  # Decrease log-odds for free cells
        self.log_occ = 0.85   # Increase log-odds for occupied cells
        self.min_log = -5.0   # Minimum log-odds (strong belief in free)
        self.max_log = 5.0    # Maximum log-odds (strong belief in occupied)

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 10)

        # Subscribers for synchronized EKF pose and LiDAR scan
        self.ekf_sub = Subscriber(self, PoseStamped, 'ekf_pose')
        self.lidar_sub = Subscriber(self, LaserScan, 'lidar_scan')
        self.sync = ApproximateTimeSynchronizer(
            [self.ekf_sub, self.lidar_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.update_map)

        # Timer to publish the map periodically
        self.create_timer(1.0, self.publish_map)

    def update_map(self, ekf_pose, lidar_scan):
        """
        Update the occupancy grid map using EKF pose and LiDAR data.
        """
        # Convert EKF pose from odom frame to map frame
        robot_x = ekf_pose.pose.position.x + (self.width * self.resolution / 2)
        robot_y = ekf_pose.pose.position.y + (self.height * self.resolution / 2)

        # Extract robot orientation (theta) from quaternion
        qz = ekf_pose.pose.orientation.z
        qw = ekf_pose.pose.orientation.w
        robot_theta = 2 * np.arctan2(qz, qw)  # Robot orientation in radians

        # Process LiDAR data
        angle_min = lidar_scan.angle_min
        angle_increment = lidar_scan.angle_increment
        lidar_ranges = lidar_scan.ranges
        max_range = lidar_scan.range_max

        for i, distance in enumerate(lidar_ranges):
            if distance > max_range or distance <= 0.0:
                continue  # Ignore invalid ranges

            # Compute the global angle of the LIDAR ray
            angle = robot_theta + (angle_min + i * angle_increment)

            # Calculate the coordinates of the LIDAR hit
            hit_x = robot_x + distance * np.cos(angle)
            hit_y = robot_y + distance * np.sin(angle)

            # Convert robot position and hit point to grid coordinates
            robot_grid_x = int(robot_x / self.resolution)
            robot_grid_y = int(robot_y / self.resolution)
            grid_x = int(hit_x / self.resolution)
            grid_y = int(hit_y / self.resolution)

            # Use Bresenham's line algorithm to mark free cells along the ray
            free_cells = self.bresenham(robot_grid_x, robot_grid_y, grid_x, grid_y)
            for fx, fy in free_cells:
                if 0 <= fx < self.width and 0 <= fy < self.height:
                    self.log_odds[fy, fx] = max(self.min_log, self.log_odds[fy, fx] + self.log_free)

            # Mark the hit cell as occupied
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                self.log_odds[grid_y, grid_x] = min(self.max_log, self.log_odds[grid_y, grid_x] + self.log_occ)

    def bresenham(self, x0, y0, x1, y1):
        """
        Bresenham's Line Algorithm to compute grid cells along a line.
        """
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = err * 2
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    def publish_map(self):
        """
        Publish the updated occupancy grid map.
        """
        grid = OccupancyGrid()

        # Fill in the header
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        # Fill in the map metadata
        grid.info.resolution = self.resolution  # meters per cell
        grid.info.width = self.width
        grid.info.height = self.height

        # Origin of the map
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Convert log odds to probabilities and assign to map data
        probabilities = 1 - 1 / (1 + np.exp(self.log_odds))  # Convert log odds to [0, 1]
        self.map_data = (probabilities * 100).astype(np.int8)  # Scale to [0, 100]
        grid.data = np.flipud(self.map_data).flatten().tolist()

        # Publish the map
        self.map_pub.publish(grid)
        self.get_logger().info('Published updated occupancy grid map.')


def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()

    try:
        rclpy.spin(map_publisher)
    except KeyboardInterrupt:
        print("MapPublisher interrupted.")
    finally:
        map_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
