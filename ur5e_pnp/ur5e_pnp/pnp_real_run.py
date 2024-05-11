import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")
path_to_src = get_package_share_directory("relaxed_ik_ros2") + "/relaxed_ik_core"
sys.path.insert(1, path_to_src + "/wrappers")
os.chdir(path_to_src)
from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt
from python_wrapper import RelaxedIKRust

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RCLDuration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from onrobotsg_interfaces.srv import OnrobotSG


class UR5ePnP(Node):

    def __init__(self):
        super().__init__("pnp_real")
        self.cbg = ReentrantCallbackGroup()
        self.cbgtimer = ReentrantCallbackGroup()

        # ik solver
        setting_file_path = path_to_src + "/configs/example_settings/ur5e_ik.yaml"
        self.relaxed_ik = RelaxedIKRust(setting_file_path)

        # robot controller
        self.jntcsub = self.create_subscription(JointState, "/joint_states", self.jntc_cb, 2, callback_group=self.cbg)
        self.trajAct = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory", callback_group=self.cbg)
        self.jntName = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.jntc = None

        # traformation
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        # gripper
        self.grprcli = self.create_client(OnrobotSG, "gripper_command")
        self.gripopen = 100.0 # mm
        self.gripclose = 50.0 # mm

        # grasp
        self.graspSub = self.create_subscription(PoseArray, "/grasp_pose", self.grasp_callback, 1, callback_group=self.cbg)
        self.graspPoseCamlink = None
        self.preGraspPoseCamlink = None

        # constant and application specific
        self.timer_ = self.create_timer(7, self.cb, callback_group=self.cbgtimer)

        self.dropj = np.array([0.3218642473220825, -2.3846870861449183, -0.8765149116516113, -1.3730150026134034, 1.5501008033752441, 0.06399798393249512]).reshape(6, 1)
        self.homej = np.array([-1.7186427116394043, -0.8601207298091431, -1.533746067677633, -1.4423204225352784, 1.5499086380004883, 0.03100419044494629]).reshape(6, 1)
        self.initj = np.deg2rad([0.0, -90.0, -90.0, 0.0, 0.0, 0.0]).reshape(6, 1)

        self.isReady = True

    def cb(self) -> None:
        self.timer_.cancel()

        # home
        self.action_forward(self.homej, t=[2])
        time.sleep(1)
        while self.isReady is False:
            self.get_logger().info("Moving to home")

        # open gripp
        self.send_request_gripper(self.gripopen)
        time.sleep(2)

        # compute grasp pose
        pg, ppg = self.get_grasp_and_pregrasp()
        self.reset_ik()
        pgj = self.handle_ik_pose(pg)
        ppgj = self.handle_ik_pose(ppg)
        pgj = np.array(pgj).reshape(6, 1)
        ppgj = np.array(ppgj).reshape(6, 1)

        # pregrasp
        self.action_forward(ppgj, t=[10])
        time.sleep(1)
        while self.isReady is False:
            self.get_logger().info("Moving to pregrasp")

        # grasp
        self.action_forward(pgj, t=[5])
        time.sleep(1)
        while self.isReady is False:
            self.get_logger().info("Moving to grasp")

        # close gripp
        self.send_request_gripper(self.gripclose)
        time.sleep(2)

        # pregrasp
        self.action_forward(ppgj, t=[3])
        time.sleep(1)
        while self.isReady is False:
            self.get_logger().info("Moving to pregrasp")

        # home
        self.action_forward(self.homej, t=[7])
        time.sleep(1)
        while self.isReady is False:
            self.get_logger().info("Moving to home")

        # drop
        self.action_forward(self.dropj, t=[10])
        time.sleep(1)
        while self.isReady is False:
            self.get_logger().info("Moving to drop off")

        # open gripp
        self.send_request_gripper(self.gripopen)
        time.sleep(2)

        # home
        self.action_forward(self.homej, t=[10])
        time.sleep(1)
        while self.isReady is False:
            self.get_logger().info("Moving to home")

        self.timer_.reset()

    def jntc_cb(self, msg: JointState):
        self.jntc = msg.position

    def action_forward(self, path, velo=None, acc=None, t=None, reverse=False):  # (dof, seq), t
        goalMsg = FollowJointTrajectory.Goal()
        goalMsg.trajectory = JointTrajectory()
        goalMsg.trajectory.joint_names = self.jntName
        for i in range(path.shape[1]):
            point = JointTrajectoryPoint()
            point.positions = path[:, i].tolist()  # path[:, -i - 1].tolist() reverse
            point.velocities = velo[:, i].tolist() if velo is not None else [0.0] * 6
            point.accelerations = acc[:, i].tolist() if acc is not None else [0.0] * 6
            point.time_from_start = RCLDuration(seconds=t[i]).to_msg()
            goalMsg.trajectory.points.append(point)

        self.trajAct.wait_for_server()
        self.sendGoalFuture = self.trajAct.send_goal_async(goalMsg, feedback_callback=self.feedback_callback)
        self.sendGoalFuture.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goalHandle = future.result()
        if not goalHandle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self.isReady = False
        print(f"> self.isReady: {self.isReady}")

        self.getResultFuture = goalHandle.get_result_async()
        self.getResultFuture.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedbackMsg):
        feedback = feedbackMsg.feedback.error.positions
        error = np.linalg.norm(feedback)
        self.get_logger().info(f"Error value : {error}")

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info("Goal succeeded!")
            self.isReady = True
        else:
            self.get_logger().info(f"Goal failed with status: {result.error_string}!")

    def get_transformation(self, toFrameRel, fromFrameRel):
        while True:
            try:
                now = rclpy.time.Time()
                trans = self.tfBuffer.lookup_transform(toFrameRel, fromFrameRel, now)
                translation = trans.transform.translation
                rotation = trans.transform.rotation
                return [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
            except:
                self.get_logger().info("Waiting for transformation")

    def handle_ik_pose(self, pose, tolerances=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):  # xyzqxqyqzqw  # linear xyz, angular xyz
        ik_solution = self.relaxed_ik.solve_position(pose[0:3], pose[3:7], tolerances)
        return ik_solution

    def handle_ik_posevel(self, posevel, tolerances=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):  # linear xyz  # angular xyz
        ik_solution = self.relaxed_ik.solve_velocity(posevel[0:3], posevel[3:7], tolerances)
        return ik_solution

    def reset_ik(self):  # always call with current joint value
        self.relaxed_ik.reset(self.jntc)

    def send_request_gripper(self, width):
        reqMsg = OnrobotSG.Request()
        reqMsg.desiredwidth = width
        future = self.grprcli.call_async(reqMsg)

    def grasp_callback(self, msg):
        g = msg.poses[0]
        pg = msg.poses[1]
        self.graspPoseCamlink = [g.position.x, g.position.y, g.position.z, g.orientation.x, g.orientation.y, g.orientation.z, g.orientation.w]
        self.preGraspPoseCamlink = [pg.position.x, pg.position.y, pg.position.z, pg.orientation.x, pg.orientation.y, pg.orientation.z, pg.orientation.w]

    def get_grasp_and_pregrasp(self):
        pcamtobase = self.get_transformation("base_link", "camera_link")
        Hcamtobase = rbt.conv_t_and_quat_to_h(translation=pcamtobase[0:3], quaternion=pcamtobase[3:7])

        while True:
            if self.graspPoseCamlink is not None:
                Hgrasptocam = rbt.conv_t_and_quat_to_h(translation=self.graspPoseCamlink[0:3], quaternion=self.graspPoseCamlink[3:7])
                Hpregrasptocam = rbt.conv_t_and_quat_to_h(translation=self.preGraspPoseCamlink[0:3], quaternion=self.preGraspPoseCamlink[3:7])

                Hgrasptobase = Hcamtobase @ Hgrasptocam
                Hpregrasptobase = Hcamtobase @ Hpregrasptocam

                pg = np.hstack(rbt.conv_h_to_t_and_quat(Hgrasptobase)).tolist()
                ppg = np.hstack(rbt.conv_h_to_t_and_quat(Hpregrasptobase)).tolist()

                return pg, ppg


def main(args=None):
    rclpy.init(args=args)
    node = UR5ePnP()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        node.destroy_node()
        exe.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
