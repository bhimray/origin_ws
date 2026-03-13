import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from .trajectory_math import normalize_angle, quaternion_to_yaw


class TrajectoryController(Node):

    def __init__(self):

        super().__init__('trajectory_controller')

        self.declare_parameter('closed_path', True)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('front_axle_offset', 0.20)
        self.declare_parameter('heading_gain', 1.6)
        self.declare_parameter('stanley_gain', 2.8)
        self.declare_parameter('softening_velocity', 0.05)
        self.declare_parameter('speed_gain', 1.0)
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
        self.closed_path = True
        self.state = None

    def path_callback(self, msg):

        self.trajectory = [
            {
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'heading': quaternion_to_yaw(pose_stamped.pose.orientation),
                'time_from_start': (
                    float(pose_stamped.header.stamp.sec) +
                    float(pose_stamped.header.stamp.nanosec) * 1e-9
                ),
            }
            for pose_stamped in msg.poses
        ]

        self.closed_path = self.get_parameter(
            'closed_path'
        ).get_parameter_value().bool_value
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

        if (
            not self.closed_path and
            self.goal_reached(self.trajectory[-1]['x'], self.trajectory[-1]['y'])
        ):
            self.publish_command(0.0, 0.0)
            return

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
            if self.closed_path:
                return self.trajectory[0] # start from initial point
            return self.trajectory[-1] # stay at last point
        return self.trajectory[next_index] # return next point

    def reference_speed(self, index):

        current = self.trajectory[index]
        nxt = self.next_point(index)
        #dis btwn two points (trajectory)
        distance = math.hypot(
            nxt['x'] - current['x'],
            nxt['y'] - current['y'],
        )
        delta_time = nxt['time_from_start'] - current['time_from_start']

        if self.closed_path and index == len(self.trajectory) - 1:
            previous = self.trajectory[max(0, len(self.trajectory) - 2)]
            delta_time = max(
                current['time_from_start'] - previous['time_from_start'],
                1e-6,
            )

        if delta_time <= 1e-6:
            return 0.0

        return distance / delta_time

    def command_speed(self, desired_speed, heading_error):

        max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).get_parameter_value().double_value
        desired_speed = min(desired_speed, max_linear_velocity)

        # scaling parameter for speed based on heading error
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
        segment_count = len(self.trajectory)

        if not self.closed_path:
            segment_count -= 1

        for index in range(segment_count):
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

    def goal_reached(self, goal_x, goal_y):

        tolerance = self.get_parameter(
            'goal_tolerance'
        ).get_parameter_value().double_value
        return math.hypot(
            goal_x - self.state['x'],
            goal_y - self.state['y'],
        ) <= tolerance

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
