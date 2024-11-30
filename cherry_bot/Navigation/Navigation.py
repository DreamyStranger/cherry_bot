import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from math import sqrt, atan2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from .VFH import VectorFieldHistogram
from .Steering import SteeringController


class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # Parameters
        self.distance_threshold = 0.1  # Stop when within 10 cm of the goal
        self.linear_velocity = 0.2  # Default linear velocity for forward motion

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Goal state
        self.goal_x = None
        self.goal_y = None

        # Steering Controllers
        self.vfh = VectorFieldHistogram()
        self.steering_controller = SteeringController()  # Ensure SteeringController is valid

        # Subscriber for dynamic goals
        self.create_subscription(PointStamped, 'goals', self.goal_callback, 10)

        # Synchronize EKF pose and LiDAR scan
        self.ekf_sub = Subscriber(self, PoseStamped, 'ekf_pose')
        self.lidar_sub = Subscriber(self, LaserScan, 'lidar_scan')
        self.sync = ApproximateTimeSynchronizer(
            [self.ekf_sub, self.lidar_sub],
            queue_size=10,
            slop=0.01  # Allow slight time differences
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def goal_callback(self, msg):
        """Callback to update the current goal."""
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.vfh.set_goal(self.goal_x, self.goal_y)
        self.steering_controller.set_goal(self.goal_x, self.goal_y)

    def synchronized_callback(self, ekf_msg, lidar_msg):
        """Synchronized callback for EKF pose and LiDAR data."""
        # Update the robot's position and orientation
        self.current_x = ekf_msg.pose.position.x
        self.current_y = ekf_msg.pose.position.y
        qz = ekf_msg.pose.orientation.z
        qw = ekf_msg.pose.orientation.w
        self.current_theta = 2 * atan2(qz, qw)

        # Process the LiDAR data
        lidar_ranges = lidar_msg.ranges
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment

        # Navigate to the goal using VFH
        self.navigate_to_goal(lidar_ranges, angle_min, angle_increment)

    def navigate_to_goal(self, lidar_ranges, angle_min, angle_increment):
        """Compute and publish velocity commands to reach the goal."""
        # Do nothing if no goal is set
        if self.goal_x is None or self.goal_y is None:
            self.get_logger().info('Waiting for a goal...')
            return

        # Compute the distance to the goal
        distance_to_goal = sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

        # Stop the robot if close enough to the goal
        if distance_to_goal < self.distance_threshold:
            self.stop_robot()
            self.get_logger().info('Goal reached!')
            self.goal_x = None  # Clear the goal once reached
            self.goal_y = None
            return

        # Use VFH to compute the velocities
        pose = (self.current_x, self.current_y, self.current_theta)
        #angular_velocity = self.vfh.compute(pose, lidar_ranges, angle_min, angle_increment)
        angular_velocity = self.steering_controller.compute(pose)

        # Publish velocity commands
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

        # Log the control outputs
        self.get_logger().info(f'Distance: {distance_to_goal:.2f}')
        self.get_logger().info(f'Linear vel: {twist.linear.x:.2f}, Angular vel: {twist.angular.z:.2f}')

    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    # Create the SimpleNavigator node
    navigator_node = SimpleNavigator()

    try:
        rclpy.spin(navigator_node)
    except KeyboardInterrupt:
        print("\nNavigator node interrupted by user")
    finally:
        navigator_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
