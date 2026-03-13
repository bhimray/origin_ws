import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import numpy as np


class WaypointPublisher(Node):

    def __init__(self):

        super().__init__('waypoint_publisher')

        self.publisher = self.create_publisher(PoseArray,'/waypoints',10)

        self.timer = self.create_timer(1.0,self.publish_waypoints)

        self.radius = 2.0
        self.num_points = 12

        self.waypoints = []

        for i in range(self.num_points):

            theta = 2*np.pi*i/self.num_points

            x = self.radius*np.cos(theta)
            y = self.radius*np.sin(theta)

            self.waypoints.append((x,y))


    def publish_waypoints(self):

        msg = PoseArray()
        msg.header.frame_id = "odom"

        for x,y in self.waypoints:

            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)

            msg.poses.append(pose)

        self.publisher.publish(msg)


def main():

    rclpy.init()

    node = WaypointPublisher()

    rclpy.spin(node)

    rclpy.shutdown()