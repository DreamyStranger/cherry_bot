import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
from math import sin, cos, atan2


class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('transform_broadcaster')

        # Initialize transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Map to odom translation (static translation)
        self.map_to_odom_translation = {
            'x': 8 / 2,  # Half width of the map
            'y': 6 / 2,  # Half height of the map
            'z': 0.0
        }

        # Robot's pose (odom frame)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Path storage
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Subscribe to EKF pose
        self.create_subscription(PoseStamped, 'ekf_pose', self.ekf_pose_callback, 10)

        # Path publisher
        self.path_pub = self.create_publisher(Path, 'path', 10)

        # Publish transforms at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_transforms)

    def ekf_pose_callback(self, msg):
        """Callback to update the robot's pose and add it to the path."""
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y  # Flip the Y-axis for correct orientation in RViz

        # Extract yaw (theta) from quaternion
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.robot_theta = 2 * atan2(qz, qw)

        # Add pose to path
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.robot_x
        pose.pose.position.y = self.robot_y
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.poses.append(pose)

        # Publish the updated path
        self.path_pub.publish(self.path)

    def publish_transforms(self):
        """Publish map->odom and odom->base_link transforms."""

        # Map to Odom Transform (Static, with flip in Y-axis)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = self.map_to_odom_translation['x']
        map_to_odom.transform.translation.y = self.map_to_odom_translation['y']
        map_to_odom.transform.translation.z = self.map_to_odom_translation['z']
        # Rotate 180° around X to reflect Y-axis
        map_to_odom.transform.rotation.x = 1.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 0.0

        # Odom to Base Link Transform (Dynamic)
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = self.get_clock().now().to_msg()
        odom_to_base_link.header.frame_id = 'odom'
        odom_to_base_link.child_frame_id = 'base_link'
        odom_to_base_link.transform.translation.x = self.robot_x
        odom_to_base_link.transform.translation.y = self.robot_y
        odom_to_base_link.transform.translation.z = 0.0
        odom_to_base_link.transform.rotation.x = 0.0
        odom_to_base_link.transform.rotation.y = 0.0
        odom_to_base_link.transform.rotation.z = sin(self.robot_theta / 2)
        odom_to_base_link.transform.rotation.w = cos(self.robot_theta / 2)

        # Broadcast the transforms
        self.broadcaster.sendTransform([map_to_odom, odom_to_base_link])



def main(args=None):
    rclpy.init(args=args)
    node = TransformBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
