import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from cherry_bot.globals import noisy_pose_rate

import numpy as np

class PoseSensor(Node):
    def __init__(self):
        super().__init__('gps_and_imu')
        
        # Publisher for the simulated PoseStamped data
        self.publisher = self.create_publisher(PoseStamped, 'noisy_pose', 10)
        
        # Subscriber to receive true odometry data
        self.create_subscription(PoseStamped, 'true_pose', self.odom_callback, 10)
        
        # Noise parameters for position and orientation
        self.position_noise_stddev = 0.03  # 3 cm noise
        self.orientation_noise_stddev = 0.01  # Small orientation noise

        # Latest ground truth values
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.latest_theta = 0.0

        self.timer = self.create_timer(noisy_pose_rate, self.publish_pose_data)

    def odom_callback(self, msg):
        # Update the latest ground truth position and orientation from PoseStamped
        self.latest_x = msg.pose.position.x
        self.latest_y = msg.pose.position.y
        self.latest_theta = 2 * np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)

    def publish_pose_data(self):
        # Add Gaussian noise to position and orientation
        noisy_x = self.latest_x + np.random.normal(0, self.position_noise_stddev)
        noisy_y = self.latest_y + np.random.normal(0, self.position_noise_stddev)
        noisy_theta = self.latest_theta + np.random.normal(0, self.orientation_noise_stddev)
        noisy_theta = (noisy_theta + np.pi) % (2 * np.pi) - np.pi

        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        pose_msg.header.frame_id = "odom"  # Coordinate frame
        
        # Populate the position
        pose_msg.pose.position.x = noisy_x
        pose_msg.pose.position.y = noisy_y
        pose_msg.pose.position.z = 0.0  # Assuming 2D simulation
        
        # Populate the orientation (quaternion)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = np.sin(noisy_theta / 2.0)
        pose_msg.pose.orientation.w = np.cos(noisy_theta / 2.0)

        # Publish the noisy pose
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    pose_sensor_node = PoseSensor()
    
    try:
        rclpy.spin(pose_sensor_node)
    except KeyboardInterrupt:
        print("\nPose Sensor node interrupted by user")
    finally:
        pose_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
