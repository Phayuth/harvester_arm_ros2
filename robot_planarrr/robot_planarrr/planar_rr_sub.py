#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .planar_rr import PlanarRR


class PlanarSub(Node):

    def __init__(self):
        super().__init__(node_name="planar_sub")
        self.sub = self.create_subscription(msg_type=JointState, topic="/jointstate", callback=self.sub_cartesian, qos_profile=10)
        self.robot = PlanarRR()

    def sub_cartesian(self, msg: JointState):
        x = msg.position[0]
        y = msg.position[1]
        theta = self.robot.inverse_kinematic_geometry(np.array([x, y]).reshape(2, 1), elbow_option=0)
        th1 = theta[0, 0]
        th2 = theta[1, 0]
        self.get_logger().info(f"From IK : th1 :{th1}, th2 :{th2}")


def main(args=None):
    rclpy.init(args=args)
    subnode = PlanarSub()
    rclpy.spin(node=subnode)
    rclpy.shutdown()


if __name__ == "__main__":
    main()