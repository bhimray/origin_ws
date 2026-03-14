"""
Track the generated trajectory using odometry feedback and Stanley-style steering.

This node subscribes to `/trajectory` for the reference path and `/odom` for the
robot state. It selects the nearest path segment to the front-axle control
point, computes heading and cross-track errors, then publishes linear and
angular velocity commands on `/cmd_vel`.
"""

import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node

from .trajectory_math import normalize_angle, quaternion_to_yaw


class TrajectoryController(Node):

    def __init__(self):

        super().__init__('trajectory_controller')

        self.declare_parameter('front_axle_offset', 0.20)
        self.declare_parameter('heading_gain', 2.0)
        self.declare_parameter('stanley_gain', 3.2)
        self.declare_parameter('softening_velocity', 0.05)
        self.declare_parameter('speed_gain', 1.2)
        self.declare_parameter('max_linear_velocity', 0.16)
        self.declare_parameter('max_angular_velocity', 2.2)

        self.subscription_path = self.create_subscription(
            Path,
            '/trajectory',
            self.path_callback,
            10
        )

        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.trajectory = []
        self.current_index = 0
        self.state = None

    def path_callback(self, msg):

        self.trajectory = [
            {
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'desired_speed': pose_stamped.pose.position.z,
                'heading': quaternion_to_yaw(pose_stamped.pose.orientation),
            }
            for pose_stamped in msg.poses
        ]

        self.current_index = 0

    def odom_callback(self, msg):

        if len(self.trajectory) < 2:
            return

        pose = msg.pose.pose
        twist = msg.twist.twist
        self.state = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': quaternion_to_yaw(pose.orientation),
            'linear_velocity': twist.linear.x,
        }

        front_x, front_y = self.front_axle_position()
        segment_index = self.find_nearest_segment_index(front_x, front_y)
        self.current_index = segment_index

        start = self.trajectory[segment_index]
        end = self.next_point(segment_index)

        seg_dx = end['x'] - start['x']
        seg_dy = end['y'] - start['y']
        seg_len = math.hypot(seg_dx, seg_dy)

        if seg_len < 1e-6:
            self.publish_command(0.0, 0.0)
            return

        path_heading = math.atan2(seg_dy, seg_dx)
        heading_error = normalize_angle(path_heading - self.state['yaw'])

        rel_x = front_x - start['x']
        rel_y = front_y - start['y']
        cross_track_error = (
            rel_x * seg_dy - rel_y * seg_dx
        ) / seg_len

        desired_speed = self.reference_speed(segment_index)
        commanded_speed = self.command_speed(desired_speed, heading_error)

        stanley_term = math.atan2(
            self.get_parameter(
                'stanley_gain'
            ).get_parameter_value().double_value * cross_track_error,
            commanded_speed + self.get_parameter(
                'softening_velocity'
            ).get_parameter_value().double_value,
        )

        angular_velocity = (
            self.get_parameter(
                'heading_gain'
            ).get_parameter_value().double_value * heading_error +
            stanley_term
        )

        max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).get_parameter_value().double_value
        angular_velocity = max(
            -max_angular_velocity,
            min(max_angular_velocity, angular_velocity),
        )

        self.publish_command(commanded_speed, angular_velocity)

    def front_axle_position(self):

        front_axle_offset = self.get_parameter(
            'front_axle_offset'
        ).get_parameter_value().double_value
        return (
            self.state['x'] + front_axle_offset * math.cos(self.state['yaw']),
            self.state['y'] + front_axle_offset * math.sin(self.state['yaw']),
        )

    def next_point(self, index):

        next_index = index + 1
        if next_index >= len(self.trajectory):
            return self.trajectory[0]
        return self.trajectory[next_index]

    def reference_speed(self, index):

        return self.trajectory[index]['desired_speed']

    def command_speed(self, desired_speed, heading_error):

        max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).get_parameter_value().double_value
        desired_speed = min(desired_speed, max_linear_velocity)

        heading_scale = max(0.15, 1.0 - abs(heading_error) / 1.6)
        desired_speed *= heading_scale

        current_speed = max(self.state['linear_velocity'], 0.0)
        speed_gain = self.get_parameter(
            'speed_gain'
        ).get_parameter_value().double_value
        commanded_speed = current_speed + speed_gain * (
            desired_speed - current_speed
        )

        return max(0.0, min(max_linear_velocity, commanded_speed))

    def find_nearest_segment_index(self, x_pos, y_pos):

        nearest_index = 0
        min_distance_sq = float('inf')

        for index in range(len(self.trajectory)):
            start = self.trajectory[index]
            end = self.next_point(index)
            seg_dx = end['x'] - start['x']
            seg_dy = end['y'] - start['y']
            seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy

            if seg_len_sq < 1e-9:
                dx = start['x'] - x_pos
                dy = start['y'] - y_pos
                distance_sq = dx * dx + dy * dy
            else:
                projection = (
                    (x_pos - start['x']) * seg_dx +
                    (y_pos - start['y']) * seg_dy
                ) / seg_len_sq
                projection = max(0.0, min(1.0, projection))
                closest_x = start['x'] + projection * seg_dx
                closest_y = start['y'] + projection * seg_dy
                dx = closest_x - x_pos
                dy = closest_y - y_pos
                distance_sq = dx * dx + dy * dy

            if distance_sq < min_distance_sq:
                min_distance_sq = distance_sq
                nearest_index = index

        return nearest_index

    def publish_command(self, linear_velocity, angular_velocity):

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.linear.x = linear_velocity
        cmd.twist.angular.z = angular_velocity
        self.publisher.publish(cmd)


def main():

    rclpy.init()

    node = TrajectoryController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
