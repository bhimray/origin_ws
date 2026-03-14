"""Log path-tracking metrics from odometry against the reference path."""

import csv
import math
import os

from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node


class NavigationMetricsLogger(Node):

    def __init__(self):

        super().__init__('navigation_metrics_logger')

        self.declare_parameter(
            'output_path',
            '/home/bim/origin_ws/results/navigation_metrics.csv'
        )

        self.output_path = self.get_parameter(
            'output_path'
        ).get_parameter_value().string_value

        self.path_points = []
        self.records = []

        self.create_subscription(
            Path,
            '/smooth_path',
            self.path_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'Logging navigation metrics to {self.output_path}'
        )

    def path_callback(self, msg: Path):

        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

    def odom_callback(self, msg: Odometry):

        if len(self.path_points) < 2:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = self.yaw_from_quaternion(q.x, q.y, q.z, q.w)

        nearest_index, path_error, desired_heading = self.find_nearest_segment(
            x,
            y,
        )
        ref_x, ref_y = self.path_points[nearest_index]
        heading_error = self.normalize_angle(desired_heading - yaw)

        linear_velocity = msg.twist.twist.linear.x
        angular_velocity = msg.twist.twist.angular.z

        stamp = (
            float(msg.header.stamp.sec) +
            float(msg.header.stamp.nanosec) * 1e-9
        )

        self.records.append({
            'time_sec': stamp,
            'x': x,
            'y': y,
            'ref_x': ref_x,
            'ref_y': ref_y,
            'path_error_m': path_error,
            'heading_error_rad': heading_error,
            'heading_error_deg': math.degrees(heading_error),
            'linear_velocity_mps': linear_velocity,
            'angular_velocity_radps': angular_velocity,
        })

    def find_nearest_segment(self, x, y):

        nearest_index = 0
        min_distance_sq = float('inf')
        nearest_heading = 0.0

        for index in range(len(self.path_points) - 1):
            start_x, start_y = self.path_points[index]
            end_x, end_y = self.path_points[index + 1]
            seg_dx = end_x - start_x
            seg_dy = end_y - start_y
            seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy

            if seg_len_sq < 1e-9:
                dx = start_x - x
                dy = start_y - y
                distance_sq = dx * dx + dy * dy
            else:
                projection = (
                    (x - start_x) * seg_dx + (y - start_y) * seg_dy
                ) / seg_len_sq
                projection = max(0.0, min(1.0, projection))
                closest_x = start_x + projection * seg_dx
                closest_y = start_y + projection * seg_dy
                dx = closest_x - x
                dy = closest_y - y
                distance_sq = dx * dx + dy * dy

            if distance_sq < min_distance_sq:
                min_distance_sq = distance_sq
                nearest_index = index
                nearest_heading = math.atan2(seg_dy, seg_dx)

        return nearest_index, math.sqrt(min_distance_sq), nearest_heading

    @staticmethod
    def yaw_from_quaternion(x, y, z, w):

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):

        return math.atan2(math.sin(angle), math.cos(angle))

    def write_csv(self):

        if not self.records:
            self.get_logger().warning('No metrics collected, skipping CSV write.')
            return

        output_dir = os.path.dirname(self.output_path)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        fieldnames = list(self.records[0].keys())

        with open(self.output_path, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.records)

        self.get_logger().info(
            f'Wrote {len(self.records)} metric samples to {self.output_path}'
        )


def main():

    rclpy.init()

    node = NavigationMetricsLogger()

    try:
        rclpy.spin(node)
    finally:
        node.write_csv()
        node.destroy_node()
        rclpy.shutdown()
