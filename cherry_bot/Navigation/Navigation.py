import json
import os

import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer

from cherry_bot.globals import synch_tolerance, goal_tolerance, goal_x, goal_y
from cherry_bot.Environment.Environment import Environment
from .Steering import SteeringController

from math import sqrt, atan2

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # Parameters
        self.distance_threshold = goal_tolerance  # Stop when within x cm of the waypoint
        self.linear_velocity = 0.2  # Default linear velocity for forward motion

        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Waypoints
        self.waypoint_list = []
        self.current_waypoint_index = 0

        # Final goal
        self.goal = (goal_x + Environment.width / 2, goal_y + Environment.height /2)

        # Steering Controller
        self.steering_controller = SteeringController()

        # Logging parameters
        self.data_log_path = "training_data.json"
        self.data_log = []

        # Subscriber for the path
        self.create_subscription(Float32MultiArray, 'goals', self.path_callback, 10)

        # Synchronize EKF pose and LiDAR scan
        self.ekf_sub = Subscriber(self, PoseStamped, 'ekf_pose')
        self.lidar_sub = Subscriber(self, LaserScan, 'lidar_scan')
        self.sync = ApproximateTimeSynchronizer(
            [self.ekf_sub, self.lidar_sub],
            queue_size=10,
            slop=synch_tolerance
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def path_callback(self, msg):
        """
        Callback to update the list of waypoints from the received path.
        """
        self.waypoint_list = [
            (msg.data[i], msg.data[i + 1]) for i in range(0, len(msg.data), 2)
        ]
        self.current_waypoint_index = 0
        self.get_logger().info(f"Received path with {len(self.waypoint_list)} waypoints.")

    def synchronized_callback(self, ekf_msg, lidar_msg):
        """
        Synchronized callback for EKF pose and LiDAR data.
        """
        # Update the robot's position and orientation
        self.current_x = ekf_msg.pose.position.x + Environment.width // 2
        self.current_y = ekf_msg.pose.position.y + Environment.height // 2
        qz = ekf_msg.pose.orientation.z
        qw = ekf_msg.pose.orientation.w
        self.current_theta = 2 * atan2(qz, qw)

        # Process the LiDAR data
        lidar_ranges = lidar_msg.ranges
        angle_min = lidar_msg.angle_min
        angle_increment = lidar_msg.angle_increment

        lidar_data = [
            (lidar_ranges[i], angle_min + i * angle_increment)
            for i in range(len(lidar_ranges))
        ]

        # Navigate to the goal using SteeringController
        self.navigate_to_goal(lidar_data)

    def navigate_to_goal(self, lidar_data):
        """
        Compute and publish velocity commands to navigate through the waypoints.
        """
        if not self.waypoint_list or self.current_waypoint_index >= len(self.waypoint_list):
            self.get_logger().info('No more waypoints to navigate.')
            self.stop_robot()
            return

        # Get the current goal from the waypoint list
        goal_x, goal_y = self.waypoint_list[self.current_waypoint_index]

        # Compute the distance to the current goal
        distance_to_goal = sqrt((goal_x - self.current_x) ** 2 + (goal_y - self.current_y) ** 2)

        # Stop the robot and move to the next waypoint if close enough to the current goal
        if distance_to_goal < self.distance_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoint_list)}.")
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoint_list):
                self.get_logger().info('All waypoints reached.')
                self.stop_robot()
            return

        # Use the SteeringController to compute the angular velocity
        pose = (self.current_x, self.current_y, self.current_theta)
        angular_velocity = self.steering_controller.compute(pose, (goal_x, goal_y))

        # Publish velocity commands
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = angular_velocity

        # Save data to log
        self.save_training_data(lidar_data, pose, self.goal, angular_velocity)

        self.cmd_vel_pub.publish(twist)

        # Log the control outputs
        self.get_logger().info(f"Distance to waypoint: {distance_to_goal:.2f}")
        self.get_logger().info(f"Linear vel: {twist.linear.x:.2f}, Angular vel: {twist.angular.z:.2f}")

    def save_training_data(self, lidar_data, pose, goal, steering):
        """
        Save LiDAR hits (distances), EKF pose, goal, and steering angle to a log for NN training.

        :param lidar_data: List of (distance, angle) tuples from LiDAR.
        :param pose: Current robot pose (x, y, theta).
        :param goal: Final goal (x, y).
        :param steering: Computed steering angle [rad].
        """
        # Extract only distances from the LiDAR data
        lidar_hits = [distance for distance, angle in lidar_data]

        # Prepare the data dictionary
        data_entry = {
            "lidar": lidar_hits,
            "pose": {"x": pose[0], "y": pose[1], "theta": pose[2]},
            "goal": {"x": goal[0], "y": goal[1]},
            "steering": steering
        }

        # Append to the log
        self.data_log.append(data_entry)

        # Save to a file periodically or at the end
        if len(self.data_log) % 10 == 0:  # Save every 10 entries
            self.write_to_file()


    def write_to_file(self):
        """
        Write the accumulated log to a JSON file.
        """
        try:
            with open(self.data_log_path, "w") as f:
                json.dump(self.data_log, f, indent=4)
            self.get_logger().info(f"Saved {len(self.data_log)} data entries to {self.data_log_path}.")
        except Exception as e:
            self.get_logger().error(f"Error saving training data: {e}")

    def stop_robot(self):
        """
        Stop the robot.
        """
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
