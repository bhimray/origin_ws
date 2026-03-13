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

        self.lookahead = 0.5
        self.velocity = 0.25


    def path_callback(self,msg): # receive trajectory points

        self.path = msg.poses

        if len(self.path) == 0:
            self.index = 0
            return

        # Keep progress when refreshed trajectory arrives.
        if self.has_odom:
            self.index = self.find_nearest_index(self.current_x, self.current_y)
        else:
            self.index = 0


    def odom_callback(self,msg): # receive current position and orientation

        if len(self.path)==0:
            return

        if self.index >= len(self.path):
            self.index = 0

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_x = x
        self.current_y = y
        self.has_odom = True

        q = msg.pose.pose.orientation
        quat = [q.x,q.y,q.z,q.w]

        _,_,theta = euler_from_quaternion(quat)

        target = self.path[self.index]

        tx = target.position.x
        ty = target.position.y

        dx = tx - x
        dy = ty - y

        distance = math.sqrt(dx*dx + dy*dy)

        # switch waypoint
        if distance < 0.2:
            self.index = (self.index + 1) % len(self.path)
            return

        desired_heading = math.atan2(dy,dx)

        heading_error = desired_heading - theta

        # normalize angle
        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'

        # Slow down on large heading errors to avoid cutting through the path.
        if abs(heading_error) > 0.8:
            cmd.twist.linear.x = 0.05
        elif abs(heading_error) > 0.4:
            cmd.twist.linear.x = 0.12
        else:
            cmd.twist.linear.x = self.velocity

        cmd.twist.angular.z = 2.0 * heading_error

        self.publisher.publish(cmd) # publish velocity command


    def find_nearest_index(self, x, y):

        nearest_index = 0
        min_dist_sq = float('inf')

        for i, pose in enumerate(self.path):
            dx = pose.position.x - x
            dy = pose.position.y - y
            dist_sq = dx * dx + dy * dy

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                nearest_index = i

        return nearest_index


def main():

    rclpy.init()

    node = TrajectoryController()

    rclpy.spin(node)
