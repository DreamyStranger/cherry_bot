class Environment:
    def __init__(self, width, height, scale=100):
        """Initialize the environment."""
        # Basic environment parameters
        self.width = width        # Width of the environment in meters
        self.height = height      # Height of the environment in meters
        self.scale = scale        # Pixels per meter for rendering

        # Walls (known map)
        self.walls = []           # List of wall segments [(x1, y1, x2, y2), ...]

        # Dynamic obstacles (unknown to the robot)
        self.obstacles = []       # List of obstacle objects

        # Robot state
        self.robot_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
        self.lidar_data = []      # Lidar scan results

    def add_wall(self, x1, y1, x2, y2):
        """Add a static wall to the environment."""
        self.walls.append((x1, y1, x2, y2))

    def add_obstacle(self, obstacle):
        """Add a dynamic obstacle to the environment."""
        self.obstacles.append(obstacle)

    def update_robot_pose(self, x, y, theta):
        """Update the robot's pose in the environment."""
        self.robot_pose = (x, y, theta)

    def simulate_lidar(self, num_rays=360, max_range=5.0):
        """Simulate a lidar scan."""
        # Cast rays and detect intersections with walls and obstacles
        self.lidar_data = self._compute_lidar_scan(num_rays, max_range)
        return self.lidar_data

    def render(self, screen):
        """Render the environment."""
        self._draw_walls(screen)
        self._draw_obstacles(screen)
        self._draw_robot(screen)
