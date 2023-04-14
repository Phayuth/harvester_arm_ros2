#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeJointStatePublisher(Node):

    def __init__(self):
        super().__init__(node_name="joint_state_publisher")

        # Publisher and Subscriber
        self.pub = self.create_publisher(msg_type=JointState, topic="/joint_state", qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=1, callback=self.pub_callback)

        # Log Info
        self.get_logger().info('Start Fake Joint State Publisher')

        # Data to Publish
        self.joint_msg = JointState()
        self.th = 0.0

    def pub_callback(self):
        self.joint_msg.header.frame_id = "joint_state"
        self.joint_msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_msg.position = [self.th, self.th, self.th, self.th, self.th, self.th]
        self.pub.publish(self.joint_msg)
        self.th += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = FakeJointStatePublisher()
    rclpy.spin(node=node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()