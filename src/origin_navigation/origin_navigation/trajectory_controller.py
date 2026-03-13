import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node

from .trajectory_math import (
    normalize_angle,
    quaternion_to_yaw,
    target_speed_from_trajectory,
)


class TrajectoryController(Node):

    def __init__(self):

        super().__init__('trajectory_controller')

        self.declare_parameter('closed_path', True)
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('lookahead_distance', 0.35)
        self.declare_parameter('lookahead_gain', 0.8)
        self.declare_parameter('heading_gain', 1.8)
        self.declare_parameter('cross_track_gain', 1.0)
        self.declare_parameter('max_linear_velocity', 0.35)
        self.declare_parameter('max_angular_velocity', 1.8)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('search_window', 60)

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

        control_rate = self.get_parameter(
            'control_rate_hz'
        ).get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.trajectory = []
        self.closed_path = True
        self.current_index = 0
        self.state = None

    def path_callback(self, msg):

        trajectory = []

        for pose_stamped in msg.poses:
            stamp = pose_stamped.header.stamp
            time_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            trajectory.append({
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'heading': quaternion_to_yaw(pose_stamped.pose.orientation),
                'time_from_start': time_sec,
            })

        self.trajectory = trajectory
        self.closed_path = self.get_parameter(
            'closed_path'
        ).get_parameter_value().bool_value
        self.current_index = 0

    def odom_callback(self, msg):

        pose = msg.pose.pose
        twist = msg.twist.twist

        self.state = {
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': quaternion_to_yaw(pose.orientation),
            'linear_velocity': twist.linear.x,
        }

    def control_loop(self):

        if self.state is None or len(self.trajectory) < 2:
            return

        if (
            not self.closed_path and
            self.goal_reached(self.state, self.trajectory[-1])
        ):
            self.publish_command(0.0, 0.0)
            return

        reference_index = self.find_reference_index()
        self.current_index = reference_index

        desired_speed = target_speed_from_trajectory(
            self.trajectory,
            reference_index,
            self.closed_path,
        )
        max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).get_parameter_value().double_value
        desired_speed = min(desired_speed, max_linear_velocity)

        lookahead_distance = self.compute_lookahead_distance(desired_speed)
        target_index = self.find_lookahead_index(
            reference_index,
            lookahead_distance,
        )
        target_point = self.trajectory[target_index]
        reference_point = self.trajectory[reference_index]

        dx = target_point['x'] - self.state['x']
        dy = target_point['y'] - self.state['y']
        distance_to_target = math.hypot(dx, dy)
        heading_to_target = math.atan2(dy, dx)
        alpha = normalize_angle(heading_to_target - self.state['yaw'])
        heading_error = normalize_angle(
            target_point['heading'] - self.state['yaw']
        )
        cross_track_error = self.compute_cross_track_error(reference_point)

        if distance_to_target < 1e-3:
            curvature = 0.0
        else:
            curvature = (
                2.0 *
                math.sin(alpha) /
                max(distance_to_target, lookahead_distance)
            )

        linear_velocity = desired_speed * max(0.2, math.cos(alpha))
        if not self.closed_path:
            linear_velocity *= min(
                1.0,
                distance_to_target / max(lookahead_distance, 1e-6),
            )

        angular_velocity = (
            linear_velocity * curvature +
            self.get_parameter(
                'heading_gain'
            ).get_parameter_value().double_value * heading_error +
            self.get_parameter(
                'cross_track_gain'
            ).get_parameter_value().double_value *
            cross_track_error
        )

        max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).get_parameter_value().double_value
        angular_velocity = max(
            -max_angular_velocity,
            min(max_angular_velocity, angular_velocity),
        )

        self.publish_command(linear_velocity, angular_velocity)

    def compute_lookahead_distance(self, desired_speed):

        base_distance = self.get_parameter(
            'lookahead_distance'
        ).get_parameter_value().double_value
        lookahead_gain = self.get_parameter(
            'lookahead_gain'
        ).get_parameter_value().double_value
        return max(base_distance, base_distance + lookahead_gain * desired_speed)

    def goal_reached(self, state, goal):

        distance = math.hypot(goal['x'] - state['x'], goal['y'] - state['y'])
        tolerance = self.get_parameter(
            'goal_tolerance'
        ).get_parameter_value().double_value
        return distance <= tolerance

    def compute_cross_track_error(self, reference_point):

        dx = self.state['x'] - reference_point['x']
        dy = self.state['y'] - reference_point['y']
        return (
            -math.sin(reference_point['heading']) * dx +
            math.cos(reference_point['heading']) * dy
        )

    def find_reference_index(self):

        search_window = self.get_parameter(
            'search_window'
        ).get_parameter_value().integer_value

        if self.closed_path:
            candidate_indices = [
                (self.current_index + offset) % len(self.trajectory)
                for offset in range(search_window)
            ]
        else:
            end_index = min(
                len(self.trajectory),
                max(self.current_index + search_window, search_window),
            )
            candidate_indices = list(range(self.current_index, end_index))
            if not candidate_indices:
                candidate_indices = list(range(len(self.trajectory)))

        nearest_index = candidate_indices[0]
        nearest_distance = float('inf')

        for index in candidate_indices:
            point = self.trajectory[index]
            distance = math.hypot(
                point['x'] - self.state['x'],
                point['y'] - self.state['y'],
            )
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index

        return nearest_index

    def find_lookahead_index(self, start_index, lookahead_distance):

        traversed_distance = 0.0
        current_index = start_index

        while traversed_distance < lookahead_distance:
            next_index = current_index + 1

            if next_index >= len(self.trajectory):
                if not self.closed_path:
                    return len(self.trajectory) - 1
                next_index = 0

            current_point = self.trajectory[current_index]
            next_point = self.trajectory[next_index]
            traversed_distance += math.hypot(
                next_point['x'] - current_point['x'],
                next_point['y'] - current_point['y'],
            )

            if next_index == start_index:
                break

            current_index = next_index

        return current_index

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
