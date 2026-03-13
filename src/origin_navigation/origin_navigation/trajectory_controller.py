import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseArray
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion


class TrajectoryController(Node):

    def __init__(self):

        super().__init__('trajectory_controller')

        self.subscription_path = self.create_subscription(
            PoseArray,
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

        self.path = []
        self.index = 0
        self.has_odom = False
        self.current_x = 0.0
        self.current_y = 0.0

        self.velocity = 0.25
        self.front_axle_offset = 0.20
        self.heading_gain = 1.2
        self.cross_track_gain = 1.0
        self.stanley_gain = 0.8
        self.softening_velocity = 0.05
        self.goal_tolerance = 0.15
        self.max_angular_velocity = 1.5


    def path_callback(self,msg): # receive trajectory points

        self.path = msg.poses

        if len(self.path) == 0:
            self.index = 0
            return

        # Keep progress when refreshed trajectory arrives.
        if self.has_odom:
            self.index = self.find_nearest_segment_index(
                self.current_x,
                self.current_y
            )
        else:
            self.index = 0


    def odom_callback(self,msg): # receive current position and orientation

        if len(self.path) < 2:
            return

        if self.index >= len(self.path) - 1:
            self.index = 0

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_x = x
        self.current_y = y
        self.has_odom = True

        q = msg.pose.pose.orientation
        quat = [q.x,q.y,q.z,q.w]

        _,_,theta = euler_from_quaternion(quat)

        goal = self.path[-1].position
        goal_distance = math.hypot(goal.x - x, goal.y - y)

        if goal_distance < self.goal_tolerance:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_footprint'
            self.publisher.publish(cmd)
            return

        front_x = x + self.front_axle_offset * math.cos(theta)
        front_y = y + self.front_axle_offset * math.sin(theta)

        self.index = self.find_nearest_segment_index(front_x, front_y)

        start = self.path[self.index].position
        end = self.path[self.index + 1].position

        seg_dx = end.x - start.x
        seg_dy = end.y - start.y
        seg_len = math.hypot(seg_dx, seg_dy)

        if seg_len < 1e-6:
            return

        path_heading = math.atan2(seg_dy, seg_dx)
        heading_error = self.normalize_angle(path_heading - theta)

        rel_x = front_x - start.x
        rel_y = front_y - start.y

        cross_track_error = (
            rel_x * seg_dy - rel_y * seg_dx
        ) / seg_len

        cross_track_term = math.atan2(
            self.stanley_gain * cross_track_error,
            self.velocity + self.softening_velocity
        )

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'

        total_error = (
            self.heading_gain * heading_error +
            self.cross_track_gain * cross_track_term
        )

        # Slow down when the path error is large so the robot can settle.
        if abs(total_error) > 0.8:
            cmd.twist.linear.x = 0.05
        elif abs(total_error) > 0.4:
            cmd.twist.linear.x = 0.12
        else:
            cmd.twist.linear.x = self.velocity

        cmd.twist.angular.z = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, total_error)
        )

        self.publisher.publish(cmd) # publish velocity command


    def find_nearest_segment_index(self, x, y):

        nearest_index = 0
        min_dist_sq = float('inf')

        for i in range(len(self.path) - 1):
            start = self.path[i].position
            end = self.path[i + 1].position
            seg_dx = end.x - start.x
            seg_dy = end.y - start.y
            seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy

            if seg_len_sq < 1e-9:
                dx = start.x - x
                dy = start.y - y
                dist_sq = dx * dx + dy * dy
            else:
                proj = (
                    (x - start.x) * seg_dx + (y - start.y) * seg_dy
                ) / seg_len_sq
                proj = max(0.0, min(1.0, proj))
                closest_x = start.x + proj * seg_dx
                closest_y = start.y + proj * seg_dy
                dx = closest_x - x
                dy = closest_y - y
                dist_sq = dx * dx + dy * dy

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                nearest_index = i

        return nearest_index


    def normalize_angle(self, angle):

        return math.atan2(math.sin(angle), math.cos(angle))


def main():

    rclpy.init()

    node = TrajectoryController()

    rclpy.spin(node)
