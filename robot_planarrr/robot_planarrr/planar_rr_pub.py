#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .planar_rr import PlanarRR


class PlanarPub(Node):

    def __init__(self):
        super().__init__(node_name="planar_pub")
        self.pub = self.create_publisher(msg_type=JointState, topic="/jointstate", qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=1, callback=self.pub_jointstate)
        self.get_logger().info('Start Publisher')
        self.robot = PlanarRR()
        self.th1 = 0
        self.th2 = 0
        self.joint_msg = JointState()

    def pub_jointstate(self):
        theta = np.array([self.th1, self.th2]).reshape(2, 1)
        xy = self.robot.forward_kinematic(theta)
        x = xy[0, 0]
        y = xy[1, 0]
        self.joint_msg.header.frame_id = "cartesian_space"
        self.joint_msg.name = ['x', 'y']
        self.joint_msg.position = [x, y]
        self.pub.publish(self.joint_msg)
        self.th1 += 0.1
        self.th2 += 0.1


def main(args=None):

    rclpy.init(args=args)
    pubndoe = PlanarPub()
    rclpy.spin(node=pubndoe)
    rclpy.shutdown()


if __name__ == "__main__":
    main()