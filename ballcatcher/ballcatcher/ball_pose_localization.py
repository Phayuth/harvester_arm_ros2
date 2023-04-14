#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Pose


class BallPose(Node):

    def __init__(self):
        super().__init__(node_name="ball_pose_localization")

        # Publisher and Subscriber
        self.pose_localization_pub = self.create_publisher(msg_type=Pose, topic="/current_ballpose", qos_profile=10)
        self.joint_current_sub = self.create_subscription(msg_type=JointState, topic="/joint_state", callback=self.joint_callback, qos_profile=10)
        self.image_current_sub = self.create_subscription(msg_type=Image, topic="/image_rgb", callback=self.image_callback, qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=0.01, callback=self.pose_pub)

        # Log Info
        self.get_logger().info("Start Ball Pose Localization")

        # Data to Publish
        self.pose_msg = Pose()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def pose_pub(self):
        self.pose_msg.position.x = self.x
        self.pose_msg.position.y = self.y
        self.pose_msg.position.z = self.z

        self.pose_localization_pub.publish(self.pose_msg)

        self.x += 0.1
        self.y += 0.1
        self.z += 0.1

    def image_callback(self, msg):
        pass

    def joint_callback(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    posenode = BallPose()
    rclpy.spin(node=posenode)
    rclpy.shutdown()