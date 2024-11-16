import sys
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

from trajectory_generator.traj_interpolator import CubicSplineInterpolationIndependant, MonotoneSplineInterpolationIndependant, BSplineSmoothingUnivariant, BSplineInterpolationIndependant, SmoothSpline

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration as RCLDuration
from builtin_interfaces.msg import Duration as MsgDuration
import numpy as np
import pickle


class JointTrajectoryControllerClient(rclpy.node.Node):

    def __init__(self):
        super().__init__("joint_trajectory_controller_client")
        self.actcli = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
        self.jntNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        # trajectory data
        pikf = "/home/yuth/ws_yuthdev/robotics_manipulator/datasave/new_data/initial_to_grasp.pkl"
        # pikf = "/home/yuth/ws_yuthdev/robotics_manipulator/datasave/new_data/pregrasp_to_drop.pkl"
        with open(pikf, "rb") as file:
            self.data = pickle.load(file)

        self.tf = 6
        # self.pos = data
        # self.velo = None # if provide, will use cubic continuous
        # self.acc = None # if provide, will use quintic countinous
        # self.times = np.linspace(0.0, self.pos.shape[1]*0.5, self.pos.shape[1])

        self.posdes = []
        self.velodes = []
        self.posreal = []
        self.veloreal = []

        self.pos, self.velo, self.acc, self.times = self.interp(self.data)

    def interp(self, data):
        vmin = -np.pi
        vmax = np.pi
        time = np.linspace(0, self.tf, data.shape[1])
        # cb = CubicSplineInterpolationIndependant(time, data)
        # cb = MonotoneSplineInterpolationIndependant(time, data, 2)
        # cb = BSplineInterpolationIndependant(time, data, degree=3)
        cb = BSplineSmoothingUnivariant(time, data, smoothc=0.01, degree=5)
        # cb = SmoothSpline(time, data, lam=0.001)

        timenew = np.linspace(0, self.tf, 1001)
        p = cb.eval_pose(timenew)
        v = cb.eval_velo(timenew)
        a = cb.eval_accel(timenew)

        if isinstance(p, list):
            p = np.array(p)
            v = np.array(v)
            a = np.array(a)
        return p, v, a, timenew

    def move_to_starting(self):
        startp = self.pos[:, 0, np.newaxis]
        self.send_goal(startp, None, None, [10])

    def move_traj(self):
        self.send_goal(self.pos, self.velo, self.acc, self.times)

    def send_goal(self, pos, velo, acc, times):
        gMsg = FollowJointTrajectory.Goal()
        gMsg.trajectory = JointTrajectory()
        gMsg.trajectory.joint_names = self.jntNames
        for i in range(pos.shape[1]):
            point = JointTrajectoryPoint()
            point.positions = pos[:, i].tolist()
            # point.velocities = velo[:, i].tolist() if velo is not None else [0.0] * 6
            # point.accelerations = acc[:, i].tolist() if acc is not None else [0.0] * 6
            point.time_from_start = RCLDuration(seconds=times[i]).to_msg()
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
        # self.get_logger().info(f"Received Feedback{feedback}")
        # self.get_logger().info(f"Received Feedback{feedback.actual}")
        self.posdes.append(feedback.desired.positions)
        self.velodes.append(feedback.desired.velocities)

        self.posreal.append(feedback.actual.positions)
        self.veloreal.append(feedback.actual.velocities)

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
    node.move_to_starting()
    # node.move_traj()
    rclpy.spin(node)

    posdes = np.array(node.posdes).T
    velodes = np.array(node.velodes).T
    posreal = np.array(node.posreal).T
    veloreal = np.array(node.veloreal).T
    tt = np.linspace(0, node.tf,posdes.shape[1])

    import matplotlib.pyplot as plt
    # position
    fig, axs = plt.subplots(6, 1, figsize=(10, 15), sharex=True)
    for i in range(6):
        # axs[i].plot(time, data[i], "k*", label=f"Joint Pos value {i+1}")
        axs[i].plot(tt, posdes[i], "b-", label=f"Joint Pos Cmd {i+1}")
        axs[i].plot(tt, posreal[i], "r-", label=f"Joint Pos Real {i+1}")
        axs[i].set_ylabel(f"JPos {i+1}")
        axs[i].set_xlim(tt[0], tt[-1])
        axs[i].legend(loc="upper right")
        axs[i].grid(True)
    axs[-1].set_xlabel("Time")
    fig.suptitle("Joint Position")
    plt.tight_layout(rect=[0, 0, 1, 0.96])


    # velocity
    fig1, axs1 = plt.subplots(6, 1, figsize=(10, 15), sharex=True)
    for i in range(6):
        axs1[i].plot(tt, velodes[i], "g-", label=f"Joint Velo Cmd {i+1}")
        axs1[i].plot(tt, veloreal[i], "r-", label=f"Joint Velo Real {i+1}")
        axs1[i].plot(tt, [-np.pi] * len(tt), "m", label=f"Velo min : {-np.pi:.3f}")
        axs1[i].plot(tt, [np.pi] * len(tt), "c", label=f"Velo max : {np.pi:.3f}")
        axs1[i].set_ylabel(f"JVelo {i+1}")
        axs1[i].set_xlim(tt[0], tt[-1])
        axs1[i].legend(loc="upper right")
        axs1[i].grid(True)
    axs1[-1].set_xlabel("Time")
    fig1.suptitle("Joint Velo")
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    plt.show()


if __name__ == "__main__":
    main()
