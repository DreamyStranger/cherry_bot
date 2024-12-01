import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from cherry_bot.globals import true_pose_rate

import math

class TruePose(Node):
    def __init__(self):
        super().__init__('grount_truth')
        
        # Publisher for ground truth pose (PoseStamped)
        self.publisher = self.create_publisher(PoseStamped, 'true_pose', 10)
        
        # Subscriber to receive velocity commands
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Robot's initial ground truth state
        self.x = 0.0  # Position x
        self.y = 0.0  # Position y
        self.theta = 0.0  # Orientation (radians)
        self.linear_velocity = 0.0  # Initial linear velocity (m/s)
        self.angular_velocity = 0.0  # Initial angular velocity (rad/s)

        # Timer to publish ground truth 
        self.timer = self.create_timer(true_pose_rate, self.publish_ground_truth)

    def cmd_vel_callback(self, msg):
        """Update the robot's true velocities from cmd_vel messages."""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def publish_ground_truth(self):
        """Calculate and publish the robot's ground truth pose."""
        delta_t = true_pose_rate 
        
        # Update position based on velocities
        self.x += self.linear_velocity * math.cos(self.theta) * delta_t
        self.y += self.linear_velocity * math.sin(self.theta) * delta_t
        self.theta += self.angular_velocity * delta_t
        
        # Normalize theta to the range [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # Create and populate a PoseStamped message
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        pose_stamped_msg.header.frame_id = "odom"  # Coordinate frame

        # Set position
        pose_stamped_msg.pose.position.x = self.x
        pose_stamped_msg.pose.position.y = self.y
        pose_stamped_msg.pose.position.z = 0.0  # Assuming 2D simulation
        
        # Set orientation as a quaternion
        pose_stamped_msg.pose.orientation.x = 0.0
        pose_stamped_msg.pose.orientation.y = 0.0
        pose_stamped_msg.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_stamped_msg.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Publish the PoseStamped message
        self.publisher.publish(pose_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    ground_truth_publisher_node = TruePose()
    
    try:
        rclpy.spin(ground_truth_publisher_node)
    except KeyboardInterrupt:
        print("\nGround Truth Publisher node interrupted by user")
    finally:
        ground_truth_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
