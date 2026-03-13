import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
import numpy as np
from scipy.interpolate import splprep, splev


class PathSmoother(Node):

    def __init__(self):

        super().__init__('path_smoother')

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


    def callback(self,msg):

        x=[]
        y=[]

        for pose in msg.poses:

            x.append(pose.position.x)
            y.append(pose.position.y)

        if len(x)<3:
            return

        tck,u = splprep([x,y],s=0,per=True)

        u_new = np.linspace(0,1,100,endpoint=False)

        x_new,y_new = splev(u_new,tck)

        path = Path()
        path.header.frame_id = "odom"

        for i in range(len(x_new)):

            pose = PoseStamped()
            pose.pose.position.x = float(x_new[i])
            pose.pose.position.y = float(y_new[i])

            path.poses.append(pose)

        # Append first point so the rendered path closes the loop in RViz.
        if len(path.poses) > 0:
            path.poses.append(path.poses[0])

        self.publisher.publish(path)


def main():

    rclpy.init()

    node = PathSmoother()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
