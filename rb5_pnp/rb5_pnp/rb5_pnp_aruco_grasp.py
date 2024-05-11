import sys
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robotiq2f_interfaces.srv import Robotiq2FCmd
from rb_interfaces.srv import ReqJnt, ReqTcp
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt


class RB5PnPRun(Node):

    def __init__(self):
        super().__init__("pnp_run_node")
        self.cbg = ReentrantCallbackGroup()
        self.cbgtimer = ReentrantCallbackGroup()
        self.ccb = ReentrantCallbackGroup()

        # req
        self.r85cli = self.create_client(Robotiq2FCmd, "/gripper_command", callback_group=self.cbg)
        self.jntctrl = self.create_client(ReqJnt, "/req_jnt", callback_group=self.cbg)
        self.tcpctrl = self.create_client(ReqTcp, "/req_tcp", callback_group=self.cbg)

        # constant
        self.gripperOpen = 0.085  # 0, 0.085
        self.gripperClose = 0.068
        self.gripperVel = 0.05  # 0.013, 0.1
        self.gripperForce = 100.0  # 5.0, 220.0
        self.homej = [-1.5707963267948966, -0.5235987755982988, 2.2689280275926285, -0.8726646259971648, 1.5707963267948966, 0.0]
        self.dropj = [-0.0010280520263029234, 0.0737424229192113, 2.3063949757045172, -0.8086622736788628, 1.5701094975497651, -0.04206219096092846]
        self.retrj1 = [-1.4696863533747848, -0.2114430562616566, 1.840798783383708, -0.0578864361644487, 1.571919648142692, -0.03920966247258778]
        self.retrj2 = [0.0, -0.2114430562616566, 1.840798783383708, -0.0578864361644487, 1.571919648142692, -0.03920966247258778]

        # realtime data
        self.jntcsub = self.create_subscription(JointState, "/joint_states", self.jntc_cb, 2, callback_group=self.ccb)
        self.tcpcsub = self.create_subscription(PoseStamped, "/tcp_states", self.tcpc_cb, 2, callback_group=self.ccb)
        self.arccsub = self.create_subscription(PoseStamped, "/arc_pose", self.arc_cb, 2, callback_group=self.ccb)
        self.gsppub = self.create_publisher(PoseStamped, "/grasp_pose", 2, callback_group=self.ccb)
        self.tcpcmdpub = self.create_publisher(PoseStamped, "/tcp_cmd", 2, callback_group=self.ccb)
        self.jntc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.tcpc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Hgrippose = None
        self.HgripperTtcp = rbt.ht(0.0, 0.0, -0.1628) @ rbt.hrz(-np.pi) @ rbt.hrx(-np.pi/2)
        self.Hcmd = None

        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

        # run loop
        self.timer_ = self.create_timer(5, self.run_loop, callback_group=self.cbgtimer)

    def run_loop(self):
        self.timer_.cancel()

        # move to home
        self.send_req_jnt(self.homej)
        while not self.is_near(self.homej, self.jntc):
            self.get_logger().info("moving to home")

        # open gripper
        self.get_logger().info("open gripper")
        self.send_req_gripper(self.gripperOpen, self.gripperVel, self.gripperForce)
        time.sleep(1)

        # calculate pregrasp and grasp
        Hpgsp = self.Hcmd @ rbt.ht(0.0, 0.2, 0.0)

        # move to pregrasp
        tpgsp, qpgsp = rbt.conv_h_to_t_and_quat(Hpgsp)
        ppgsp = [tpgsp[0], tpgsp[1], tpgsp[2], qpgsp[0], qpgsp[1], qpgsp[2], qpgsp[3]]
        self.send_req_tcp(ppgsp)
        while not self.is_near(ppgsp, self.tcpc):
            self.get_logger().info("moving to pregrasp")

        # move to grasp
        tcmd, qcmd = rbt.conv_h_to_t_and_quat(self.Hcmd)
        pcmd = [tcmd[0], tcmd[1], tcmd[2], qcmd[0], qcmd[1], qcmd[2], qcmd[3]]
        self.send_req_tcp(pcmd)
        while not self.is_near(pcmd, self.tcpc):
            self.get_logger().info("moving to grasp")

        # close gripper
        time.sleep(2)
        self.get_logger().info("close gripper")
        self.send_req_gripper(self.gripperClose, self.gripperVel, self.gripperForce)
        time.sleep(1)

        # move to pregrasp
        self.send_req_tcp(ppgsp)
        while not self.is_near(ppgsp, self.tcpc):
            self.get_logger().info("moving to pregrasp")

        # move retract 1
        self.send_req_jnt(self.retrj1)
        while not self.is_near(self.retrj1, self.jntc):
            self.get_logger().info("moving to retract 1")

        # move retract 2
        self.send_req_jnt(self.retrj2)
        while not self.is_near(self.retrj2, self.jntc):
            self.get_logger().info("moving to retract 2")

        # move to drop off
        self.send_req_jnt(self.dropj)
        while not self.is_near(self.dropj, self.jntc):
            self.get_logger().info("moving to drop off")

        # open gripper
        self.get_logger().info("open gripper")
        self.send_req_gripper(self.gripperOpen, self.gripperVel, self.gripperForce)
        time.sleep(1)

        # move retract 2
        self.send_req_jnt(self.retrj2)
        while not self.is_near(self.retrj2, self.jntc):
            self.get_logger().info("moving to retract 2")

        # move to home
        self.send_req_jnt(self.homej)
        while not self.is_near(self.homej, self.jntc):
            self.get_logger().info("moving to home")

        # self.timer_.reset()

    def send_req_gripper(self, pos, vel, force):
        rqm = Robotiq2FCmd.Request()
        rqm.pos = pos
        rqm.vel = vel
        rqm.force = force
        future = self.r85cli.call_async(rqm)
        # rclpy.spin_until_future_complete(self, future)
        # return future.result()

    def send_req_tcp(self, tcp, spd=-1.0, acc=5.0, movetype=1):
        req = ReqTcp.Request()
        req.tcppose.pose.position = Point(x=tcp[0], y=tcp[1], z=tcp[2])
        req.tcppose.pose.orientation = Quaternion(x=tcp[3], y=tcp[4], z=tcp[5], w=tcp[6])
        req.movetype = movetype
        req.spd = spd
        req.acc = acc
        future = self.tcpctrl.call_async(req)
        # while not future.done():
        #     time.sleep(0.5)
        #     print("Waiting...")
        # rclpy.spin_until_future_complete(self, future)
        # return future.result()

    def send_req_jnt(self, jnt, spd=-1.0, acc=5.0):
        req = ReqJnt.Request()
        req.jntstate.position = jnt
        req.spd = spd
        req.acc = acc
        future = self.jntctrl.call_async(req)
        # rclpy.spin_until_future_complete(self, future) # this thing block other
        # return future.result()

    def jntc_cb(self, msg:JointState):
        self.jntc = msg.position

    def tcpc_cb(self, msg:PoseStamped):
        self.tcpc = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

    def arc_cb(self, msg:PoseStamped):
        arcc = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w]

        HboxTarc = np.array([[1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, -0.07/2],
                             [0.0, 0.0, 0.0, 1.0]])

        HcamopticalTbase = self.get_transformation("Body_Base", "camera_color_optical_frame")
        HcamopticalTbase = rbt.conv_t_and_quat_to_h(HcamopticalTbase[0:3], HcamopticalTbase[3:7])
        HarcTcamoptical = rbt.conv_t_and_quat_to_h(arcc[0:3], arcc[3:7])
        HboxTbase = HcamopticalTbase @ HarcTcamoptical @ HboxTarc

        self.Hgrippose = HboxTbase @ rbt.hrx(-np.pi) @ rbt.hrz(-np.pi/2)
        t, q = rbt.conv_h_to_t_and_quat(self.Hgrippose)
        arcp = PoseStamped()
        arcp.header.frame_id = "Body_Base"
        arcp.pose.position.x = t[0]
        arcp.pose.position.y = t[1]
        arcp.pose.position.z = t[2]
        arcp.pose.orientation.x = q[0]
        arcp.pose.orientation.y = q[1]
        arcp.pose.orientation.z = q[2]
        arcp.pose.orientation.w = q[3]
        self.gsppub.publish(arcp)

        self.Hcmd = self.Hgrippose @ self.HgripperTtcp @ rbt.ht(0.0, 0.0, 0.015) # offset a bit for center
        tg, qg = rbt.conv_h_to_t_and_quat(self.Hcmd)
        tcpcmd = PoseStamped()
        tcpcmd.header.frame_id = "Body_Base"
        tcpcmd.pose.position.x = tg[0]
        tcpcmd.pose.position.y = tg[1]
        tcpcmd.pose.position.z = tg[2]
        tcpcmd.pose.orientation.x = qg[0]
        tcpcmd.pose.orientation.y = qg[1]
        tcpcmd.pose.orientation.z = qg[2]
        tcpcmd.pose.orientation.w = qg[3]
        self.tcpcmdpub.publish(tcpcmd)

    def is_near(self, jntd, jntc):
        e = np.array(jntd) - np.array(jntc)
        if np.linalg.norm(e) < 0.05:
            return True
        else:
            return False

    def get_transformation(self, frameId, ChildFrameId):
        while True:
            try:
                now = rclpy.time.Time()
                trans = self.tfBuffer.lookup_transform(frameId, ChildFrameId, now)
                translation = trans.transform.translation
                rotation = trans.transform.rotation
                return [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
            except:
                self.get_logger().info("Waiting for transformation")


def main(args=None):
    rclpy.init(args=args)
    node2 = RB5PnPRun()
    exe = MultiThreadedExecutor()
    exe.add_node(node2)
    try:
        exe.spin()
    finally:
        node2.destroy_node()
        exe.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
