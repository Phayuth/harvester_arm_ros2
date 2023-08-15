import numpy as np

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__(node_name="publish_joint_trajectory_position_controller")
        self.controller_name = 'joint_trajectory_controller'
        self.wait_sec_between_publish = 6
        self.joints = ['shoulder_pan_joint',
                       'shoulder_lift_joint',
                       'elbow_joint',
                       'wrist_1_joint',
                       'wrist_2_joint',
                       'wrist_3_joint']

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        thetaInit = np.array([np.deg2rad(-27.68), np.deg2rad(2.95), np.deg2rad(-26.58), np.deg2rad(-15.28), np.deg2rad(87.43), np.deg2rad(0.0)]).reshape(6, 1)

        self.pos1 = [thetaInit[0, 0], thetaInit[1, 0], thetaInit[2, 0], thetaInit[3, 0], thetaInit[4, 0], thetaInit[5, 0]]

        publish_topic = "/" + self.controller_name + "/" + "joint_trajectory"

        self.get_logger().info(f'Publishing goals on topic "{publish_topic}" every {self.wait_sec_between_publish} s')

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(self.wait_sec_between_publish, self.timer_callback)

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.pos1
        point.time_from_start = Duration(sec=7)

        traj.points.append(point)
        self.publisher_.publish(traj)

def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
