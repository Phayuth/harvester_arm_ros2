import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):

    def __init__(self):
        super().__init__(node_name="publish_joint_trajectory_position_controller")
        self.joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        self.pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 1)
        self.timer = self.create_timer(1, callback=self.timer_callback)

    def timer_callback(self):
        thetaInit = np.deg2rad([0.0, -90.0, -90.0, 0.0, 0.0, 0.0])
        pos = thetaInit.flatten().tolist()

        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = pos
        point.time_from_start = Duration(sec=7)
        traj.points.append(point)
        self.pub.publish(traj)
        self.get_logger().info("Mesg is published")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    pubJointTrajNode = PublisherJointTrajectory()
    rclpy.spin(pubJointTrajNode)
    pubJointTrajNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
