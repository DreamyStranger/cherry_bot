import numpy as np
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import rclpy

class EKF(Node):
    def __init__(self):
        super().__init__('ekf')
        
        # Initialize state vector [x, y, theta] and covariance matrix P
        self.state = np.array([0.0, 0.0, 0.0])  # State: [x, y, theta]
        self.P = np.eye(3) * 0.1  # Initial covariance with small uncertainty

        # Process noise covariance matrix Q
        self.Q = np.diag([0.001, 0.001, 0.0005]) 

        # Measurement noise covariance matrix for PoseStamped
        self.R_pose = np.diag([0.03, 0.03, 0.01])  # Noise for x, y, and theta

        # Linear and angular velocities (default)
        self.v = 0.0
        self.w = 0.0

        # Subscriber for noisy pose and velocity commands
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(PoseStamped, 'noisy_pose', self.noisy_pose_callback, 10)

        # Timer for fixed-rate prediction (100 Hz)
        self.create_timer(0.01, self.fixed_rate_predict)

        # Publisher for EKF pose
        self.pose_publisher = self.create_publisher(PoseStamped, 'ekf_pose', 10)

    def cmd_vel_callback(self, msg):
        """Callback to update the current velocity commands."""
        self.v = msg.linear.x
        self.w = msg.angular.z

    def noisy_pose_callback(self, msg):
        """Callback for PoseStamped measurements."""
        # Extract position and orientation from noisy_pose
        pose_measurement = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            2 * np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)
        ])
        self.correct_pose(pose_measurement)

        # Publish pose after correction
        self.publish_pose()

    def fixed_rate_predict(self):
        """Fixed-rate prediction step based on velocities."""
        delta_t = 0.01  # Time step for 100 Hz
        self.predict(self.v, self.w, delta_t)

    def predict(self, v, w, delta_t):
        """Predict step of the EKF."""
        theta = self.state[2]
        
        # State transition function (f)
        f = np.array([
            v * np.cos(theta) * delta_t,
            v * np.sin(theta) * delta_t,
            w * delta_t
        ])
        
        # Jacobian of the state transition function (A)
        A = np.array([
            [1, 0, -v * np.sin(theta) * delta_t],
            [0, 1, v * np.cos(theta) * delta_t],
            [0, 0, 1]
        ])
        
        # Update state and covariance
        self.state += f
        self.P = A @ self.P @ A.T + self.Q

    def correct_pose(self, pose_measurement):
        """Correction step using PoseStamped data."""
        # Measurement matrix H for pose (measures x, y, and theta)
        H = np.array([
            [1, 0, 0],  # x position
            [0, 1, 0],  # y position
            [0, 0, 1]   # theta (orientation)
        ])
        
        # Innovation (residual) between actual and predicted measurements
        y = pose_measurement - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_pose
        
        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update the state with measurement
        self.state = self.state + K @ y
        
        # Update the covariance
        self.P = (np.eye(3) - K @ H) @ self.P

    def publish_pose(self):
        """Publish the EKF's estimated state as a PoseStamped message."""
        pose_msg = PoseStamped()
        
        # Populate pose (x, y, theta)
        pose_msg.pose.position.x = self.state[0]
        pose_msg.pose.position.y = self.state[1]
        pose_msg.pose.position.z = 0.0  # Assuming 2D
        
        # Populate orientation (convert theta to quaternion)
        pose_msg.pose.orientation.z = math.sin(self.state[2] / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.state[2] / 2.0)
        
        # Set the header with timestamp and frame_id
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"  # Reference frame
        
        # Publish the PoseStamped message
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    ekf = EKF()
    
    try:
        rclpy.spin(ekf)
    except KeyboardInterrupt:
        print("\nEKF node interrupted by user")
    finally:
        ekf.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
