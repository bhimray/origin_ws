import csv
import math
import os

import rclpy
from nav_msgs.msg import Odometry, Path
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

        nearest_index, path_error = self.find_nearest_point(x, y)
        ref_x, ref_y = self.path_points[nearest_index]
        target_index = (nearest_index + 1) % len(self.path_points)

        tx, ty = self.path_points[target_index]
        desired_heading = math.atan2(ty - y, tx - x)
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

    def find_nearest_point(self, x, y):

        nearest_index = 0
        min_distance = float('inf')

        for index, (px, py) in enumerate(self.path_points):
            dx = x - px
            dy = y - py
            distance = math.hypot(dx, dy)

            if distance < min_distance:
                min_distance = distance
                nearest_index = index

        return nearest_index, min_distance

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
