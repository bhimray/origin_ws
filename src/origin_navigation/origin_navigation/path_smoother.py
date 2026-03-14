"""Convert sparse waypoints into a dense smoothed path with estimated headings.

This node subscribes to `/waypoints`, fits a periodic spline through the input
points, resamples the curve at approximately uniform spacing, estimates the
tangent heading at each sample, and publishes the result as `/smooth_path`.
"""

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from .trajectory_math import (
    estimate_headings,
    quaternion_from_yaw,
    smooth_waypoints,
)


class PathSmoother(Node):

    def __init__(self):

        super().__init__('path_smoother')

        self.declare_parameter('sample_spacing', 0.05)
        self.declare_parameter('spline_smoothing', 0.02)

        self.subscription = self.create_subscription(
            PoseArray,
            '/waypoints',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            Path,
            '/smooth_path',
            10
        )


    def callback(self, msg):

        waypoints = [
            (pose.position.x, pose.position.y)
            for pose in msg.poses
        ]

        if len(waypoints) < 2:
            return

        sample_spacing = self.get_parameter(
            'sample_spacing'
        ).get_parameter_value().double_value
        spline_smoothing = self.get_parameter(
            'spline_smoothing'
        ).get_parameter_value().double_value
        smoothed_points = smooth_waypoints(
            waypoints,
            sample_spacing=sample_spacing,
            spline_smoothing=spline_smoothing,
        )
        headings = estimate_headings(smoothed_points)

        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        for (x_pos, y_pos), heading in zip(smoothed_points, headings):

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x_pos
            pose.pose.position.y = y_pos
            _, _, qz, qw = quaternion_from_yaw(heading)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path.poses.append(pose)

        if len(path.poses) > 0:
            path.poses.append(path.poses[0])

        self.publisher.publish(path)


def main():

    rclpy.init()

    node = PathSmoother()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
