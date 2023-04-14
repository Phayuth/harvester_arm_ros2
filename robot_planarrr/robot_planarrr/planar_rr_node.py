#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from .planar_rr import PlanarRR


class PlanarRRNode(Node):

    def __init__(self):
        super().__init__(node_name="planar_node")
        self.robot = PlanarRR()
        self.th1 = 0
        self.th2 = 0
        self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

    def timer_callback(self):
        theta = np.array([self.th1, self.th2]).reshape(2, 1)
        xy = self.robot.forward_kinematic(theta)
        self.get_logger().info(f"Forward Kinematic X: {xy[0,0]}, Y: {xy[1,0]}")
        self.th1 += 0.1
        self.th2 += 0.1


def main(args=None):

    rclpy.init(args=args)
    node = PlanarRRNode()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()