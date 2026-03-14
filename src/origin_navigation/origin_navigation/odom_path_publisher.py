import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node


class OdomPathPublisher(Node):

    def __init__(self):

        super().__init__('odom_path_publisher')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            Path,
            '/odom_path',
            10
        )

        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Keep memory and RViz rendering cost bounded.
        self.max_points = 2000
        self.min_distance = 0.02

    def odom_callback(self, msg: Odometry):

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose

        if self.path.poses:
            prev = self.path.poses[-1].pose.position
            cur = pose.pose.position
            dx = cur.x - prev.x
            dy = cur.y - prev.y

            # Skip near-duplicate points to reduce noise and message size.
            if math.hypot(dx, dy) < self.min_distance:
                return

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)

        if len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points:]

        self.publisher.publish(self.path)


def main():

    rclpy.init()

    node = OdomPathPublisher()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
