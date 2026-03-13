import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, Pose
import math


class TrajectoryGenerator(Node):

    def __init__(self):

        super().__init__('trajectory_generator')

        self.subscription = self.create_subscription(
            Path,
            '/smooth_path',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseArray,
            '/trajectory',
            10
        )

        self.velocity = 0.3


    def callback(self,msg):

        traj = PoseArray()
        traj.header.frame_id="odom"

        prev=None
        time=0.0

        for pose in msg.poses:

            p=Pose()
            p.position.x=pose.pose.position.x
            p.position.y=pose.pose.position.y

            if prev:

                dx=p.position.x-prev.position.x
                dy=p.position.y-prev.position.y

                dist=math.sqrt(dx*dx+dy*dy)

                time+=dist/self.velocity

            traj.poses.append(p)

            prev=p

        self.publisher.publish(traj)


def main():

    rclpy.init()

    node = TrajectoryGenerator()

    rclpy.spin(node)