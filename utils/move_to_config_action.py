import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration as RCLDuration
from builtin_interfaces.msg import Duration as MsgDuration
import numpy as np


class JointTrajectoryControllerClient(rclpy.node.Node):

    def __init__(self):
        super().__init__("joint_trajectory_controller_client")
        self.actcli = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
        self.jntNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        # trajectory data
        self.pos = np.array([[0.0, -2.0, 1.0, 0.5, -1.0, 0.0],
                             [1.0, -1.0, -1.0, 1.0, 1.0, 1.0],
                             [0.0, -1.0, -1.0, -1.0, 0.0, 0.0],
                             [1.0, -0.5, -1.0, 1.0, 1.0, -1.0],]).T

        # if provide, will use cubic continuous
        # self.velo = np.array([[0.1, 1.0, 1.0, 1.0, 1.0, 1.0],
        #                       [0.1, 1.0, 1.0, 1.0, 1.0, 1.0],
        #                       [0.1, 1.0, 1.0, 1.0, 1.0, 1.0],
        #                       [0.1, 1.0, 1.0, 1.0, 1.0, 1.0],]).T

        self.velo = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],]).T # all zeros, mean stop and each knot same as if not provide

        # self.velo = np.array([[0.0, 0.4, 0.4, 0.1, 0.1, 0.1],
        #                       [0.0, 0.4, 0.4, 0.1, 0.1, 0.1],
        #                       [0.0, 0.4, 0.4, 0.1, 0.1, 0.1],
        #                       [0.0, 0.4, 0.4, 0.1, 0.1, 0.1],]).T

        # if provide, will use quintic countinous
        self.acc = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],]).T

        self.times = np.linspace(10.2, 40.8, 4)

    def send_goal(self):
        gMsg = FollowJointTrajectory.Goal()
        gMsg.trajectory = JointTrajectory()
        gMsg.trajectory.joint_names = self.jntNames
        for i in range(self.pos.shape[1]):
            point = JointTrajectoryPoint()
            point.positions = self.pos[:, i].tolist()
            point.velocities = self.velo[:,i].tolist() if self.velo is not None else [0.0]*6
            point.accelerations = self.acc[:,i].tolist() if self.acc is not None else [0.0]*6
            point.time_from_start = RCLDuration(seconds=self.times[i]).to_msg()
            gMsg.trajectory.points.append(point)

        self.actcli.wait_for_server()
        self.sendGoalFuture = self.actcli.send_goal_async(gMsg, feedback_callback=self.feedback_callback)
        self.sendGoalFuture.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goalHandle = future.result()
        if not goalHandle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self.getResultFuture = goalHandle.get_result_async()
        self.getResultFuture.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received Feedback{feedback.actual}")

    def get_result_callback(self, future):
        result = future.result().result

        # SUCCESSFUL = 0
        # INVALID_GOAL = -1
        # INVALID_JOINTS = -2
        # OLD_HEADER_TIMESTAMP = -3
        # PATH_TOLERANCE_VIOLATED = -4
        # GOAL_TOLERANCE_VIOLATED = -5

        if result.error_code == 0:
            self.get_logger().info("Goal succeeded!")
        else:
            self.get_logger().info("Goal failed with status: %d" % result.error_string)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryControllerClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
