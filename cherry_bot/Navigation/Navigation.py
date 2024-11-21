import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from math import sqrt, atan2
from .Steering import SteeringController

class SimpleNavigator(Node):
    def __init__(self, steering_controller):
        super().__init__('simple_navigator')

        # Parameters
        self.distance_threshold = 0.1  # Stop when within 10 cm of the goal
        self.linear_velocity = 0.3  # Fixed linear velocity for forward motion

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Goal state
        self.goal_x = None
        self.goal_y = None

        # Steering controller
        self.steering_controller = steering_controller

        # Subscriber for dynamic goals
        self.create_subscription(PointStamped, 'goals', self.goal_callback, 10)
        
        # Subscriber for ekf pose (PoseStamped)
        self.create_subscription(PoseStamped, 'ekf_pose', self.ekf_pose_callback, 10)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer for control loop (10 Hz)
        self.create_timer(0.1, self.control_loop)

    def ekf_pose_callback(self, msg):
        """Callback to update the robot's position and orientation."""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        # Extract yaw (theta) from quaternion
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.current_theta = 2 * atan2(qz, qw)

    def goal_callback(self, msg):
        """Callback to update the current goal."""
        self.goal_x = msg.point.x
        self.goal_y = msg.point.y
        self.get_logger().info(f'New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}')

    def control_loop(self):
        """Control loop to navigate towards the goal."""
        # Do nothing if no goal is set
        if self.goal_x is None or self.goal_y is None:
            self.get_logger().info('Waiting for a goal...')
            return

        # Compute the distance to the goal
        distance_to_goal = sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)

        # Stop the robot if close enough to the goal
        if distance_to_goal < self.distance_threshold:
            self.stop_robot()
            self.get_logger().info('Goal reached!')
            self.goal_x = None  # Clear the goal once reached
            self.goal_y = None
            return

        # Use the steering controller to calculate the angular velocity
        steering_angle = self.steering_controller.compute(
            goal=(self.goal_x, self.goal_y),
            pose=(self.current_x, self.current_y, self.current_theta)
        )

        # Publish velocity commands
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = steering_angle
        self.cmd_vel_pub.publish(twist)

        # Log the control outputs
        self.get_logger().info(f'Distance: {distance_to_goal:.2f}, Steering angle: {steering_angle:.2f}')
        self.get_logger().info(f'Linear vel: {twist.linear.x:.2f}, Angular vel: {twist.angular.z:.2f}')

    def stop_robot(self):
        """Stop the robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the SteeringController
    steering_controller = SteeringController(K_h=2.0, max_steering=1.0)

    # Pass it to the SimpleNavigator node
    navigator_node = SimpleNavigator(steering_controller=steering_controller)

    try:
        rclpy.spin(navigator_node)
    except KeyboardInterrupt:
        print("\nNavigator node interrupted by user")
    finally:
        navigator_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
