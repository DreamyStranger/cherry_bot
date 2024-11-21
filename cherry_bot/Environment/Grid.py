import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Parameters for map dimensions and resolution
        self.map_width = 800  # In cells (8 meters, assuming 1 cm resolution)
        self.map_height = 600  # In cells (6 meters, assuming 1 cm resolution)
        self.resolution = 0.01  # 1 cm per cell
        self.map_update_rate = 1.0  # Publish map every 1 second

        # Initialize the map (static walls + dynamic obstacles placeholder)
        self.static_map = self.initialize_static_map()
        self.dynamic_obstacles = []  # List of dynamic obstacles [(x, y), ...]

        # Publisher for OccupancyGrid
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)

        # Timer to publish map periodically
        self.create_timer(self.map_update_rate, self.publish_map)

    def initialize_static_map(self):
        """Initialize the static part of the map with walls."""
        grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        # Create static walls
        # Top and bottom walls
        grid[0, :] = 100
        grid[-1, :] = 100
        # Left and right walls
        grid[:, 0] = 100
        grid[:, -1] = 100

        return grid

    def update_dynamic_obstacles(self):
        """Add dynamic obstacles to the static map."""
        # Create a copy of the static map
        updated_map = self.static_map.copy()

        # Add dynamic obstacles
        for obstacle in self.dynamic_obstacles:
            x, y = obstacle
            # Convert real-world coordinates to grid indices
            col = int((x + self.map_width * self.resolution / 2) / self.resolution)
            row = int((y + self.map_height * self.resolution / 2) / self.resolution)
            if 0 <= row < self.map_height and 0 <= col < self.map_width:
                updated_map[row, col] = 100  # Mark as occupied

        return updated_map

    def publish_map(self):
        """Publish the OccupancyGrid map."""
        # Update the map with dynamic obstacles
        final_map = self.update_dynamic_obstacles()

        # Create the OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height

        # Align the origin to center the robot (0, 0) on the map
        msg.info.origin = Pose()
        msg.info.origin.position.x = -self.map_width * self.resolution / 2
        msg.info.origin.position.y = -self.map_height * self.resolution / 2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0  # No rotation

        # Flatten the map and assign it to the OccupancyGrid data
        msg.data = final_map.flatten().tolist()
        self.map_publisher.publish(msg)

        self.get_logger().info('Published updated map')

    def add_dynamic_obstacle(self, x, y):
        """Add a dynamic obstacle to the map."""
        self.dynamic_obstacles.append((x, y))

def main(args=None):
    rclpy.init(args=args)
    map_publisher = MapPublisher()

    # Example: Add some dynamic obstacles
    #map_publisher.add_dynamic_obstacle(1.0, 1.0)  # At (1.0, 1.0) meters
    #map_publisher.add_dynamic_obstacle(-2.0, 0.5)  # At (-2.0, 0.5) meters

    try:
        rclpy.spin(map_publisher)
    except KeyboardInterrupt:
        print("Map Publisher stopped by user")
    finally:
        map_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
