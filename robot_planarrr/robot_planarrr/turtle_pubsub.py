#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class CloseLoopController(Node):

    def __init__(self):
        super().__init__(node_name='turtlebot_controller')
        self.pub = self.create_publisher(msg_type=Twist, topic="/turtle1/cmd_vel", qos_profile=10)
        self.sub = self.create_subscription(msg_type=Pose, topic="/turtle1/pose", callback=self.pose_callback, qos_profile=10)
        self.get_logger().info("Start controller for turtlebot")

    def pose_callback(self, msg: Pose):
        cmd = Twist()

        if msg.x > 9.0 or msg.x < 2.0 or msg.y > 9.0 or msg.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    controlnode = CloseLoopController()
    rclpy.spin(controlnode)
    rclpy.shutdown()
