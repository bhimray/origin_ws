"""Attach timing, heading, and desired speed information to the smoothed path.

This node subscribes to `/smooth_path`, computes a time-parameterized trajectory
using the configured cruise speed and acceleration, stores desired speed in the
`z` position field, updates pose orientation from the path heading, and
publishes the result on `/trajectory`.
"""

import rclpy
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node

from .trajectory_math import (
    generate_timed_trajectory,
    quaternion_from_yaw,
)


class TrajectoryGenerator(Node):

    def __init__(self):

        super().__init__('trajectory_generator')

        self.declare_parameter('cruise_speed', 0.3)
        self.declare_parameter('acceleration', 0.5)

        self.subscription = self.create_subscription(
            Path,
            '/smooth_path',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            Path,
            '/trajectory',
            10
        )

    def callback(self, msg):

        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

        if len(points) < 2:
            return

        cruise_speed = self.get_parameter(
            'cruise_speed'
        ).get_parameter_value().double_value
        acceleration = self.get_parameter(
            'acceleration'
        ).get_parameter_value().double_value
        trajectory_points = generate_timed_trajectory(
            points,
            cruise_speed=cruise_speed,
            acceleration=acceleration,
        )

        start_time = self.get_clock().now()
        trajectory = Path()
        trajectory.header.frame_id = 'odom'
        trajectory.header.stamp = start_time.to_msg()

        for point, source_pose in zip(trajectory_points, msg.poses):
            source_pose.header.stamp = (
                start_time +
                Duration(seconds=point['time_from_start'])
            ).to_msg()
            source_pose.pose.position.z = point['desired_speed']
            _, _, qz, qw = quaternion_from_yaw(point['heading'])
            source_pose.pose.orientation.z = qz
            source_pose.pose.orientation.w = qw
            trajectory.poses.append(source_pose)

        self.publisher.publish(trajectory)


def main():

    rclpy.init()

    node = TrajectoryGenerator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
