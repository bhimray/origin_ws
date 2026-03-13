import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

from .path_config import get_waypoints


class WaypointPublisher(Node):

    def __init__(self):

        super().__init__('waypoint_publisher')

        self.publisher = self.create_publisher(PoseArray, '/waypoints', 10)

        self.timer = self.create_timer(1.0, self.publish_waypoints)

        self.waypoints = get_waypoints()


    def publish_waypoints(self):

        msg = PoseArray()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.waypoints:

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)

            msg.poses.append(pose)

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = WaypointPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
