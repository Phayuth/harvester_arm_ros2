import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):

    def __init__(self):
        super().__init__(node_name="publish_joint_trajectory_position_controller")
        self.joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        self.pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 1)
        self.timer = self.create_timer(timer_period_sec=6, callback=self.timer_callback)

    def timer_callback(self):
        thetaInit = np.array([np.deg2rad(-27.68), np.deg2rad(2.95), np.deg2rad(-26.58), np.deg2rad(-15.28), np.deg2rad(87.43), np.deg2rad(0.0)]).reshape(6, 1)
        pos = [thetaInit[0, 0], thetaInit[1, 0], thetaInit[2, 0], thetaInit[3, 0], thetaInit[4, 0], thetaInit[5, 0]]

        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = pos
        point.time_from_start = Duration(sec=7)
        traj.points.append(point)
        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    pubJointTrajNode = PublisherJointTrajectory()
    rclpy.spin(pubJointTrajNode)
    pubJointTrajNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
