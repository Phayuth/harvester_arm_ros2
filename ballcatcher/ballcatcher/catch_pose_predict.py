#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


class CatchPosePrediction(Node):

    def __init__(self):
        super().__init__(node_name="catch_pose_prediction")

        # Publisher and Subscriber
        self.catchpose_pub = self.create_publisher(msg_type=Pose, topic="/catch_pose", qos_profile=10)
        self.joint_current_sub = self.create_subscription(msg_type=JointState, topic="/joint_state", callback=self.joint_callback, qos_profile=10)
        self.ballpose_sub = self.create_subscription(msg_type=Pose, topic="/current_ballpose", callback=self.pose_callback, qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=0.01, callback=self.pose_pub)

        # Log Info
        self.get_logger().info("Estimate Pose to Catch")

        # Data to Publish
        self.pose_msg = Pose()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def pose_pub(self):
        self.pose_msg.position.x = self.x
        self.pose_msg.position.y = self.y
        self.pose_msg.position.z = self.z

        self.catchpose_pub.publish(self.pose_msg)

        self.x += 0.1
        self.y += 0.1
        self.z += 0.1

    def joint_callback(self, msg):
        pass

    def pose_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    posenode = CatchPosePrediction()
    rclpy.spin(node=posenode)
    rclpy.shutdown()