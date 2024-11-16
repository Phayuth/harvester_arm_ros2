import sys
import os
sys.path.append(str(wd=os.path.abspath(os.getcwd())))
from xtras.spatial_transformation import RigidBodyTransformation as rbt
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import rtde_control
import rtde_receive
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from rclpy.duration import Duration
from rclpy.time import Time
from robotiq2f_interfaces.srv import Robotiq2FCmd
import time
import cv2


class SimpleSystem(Node):

    def __init__(self) -> None:
        super().__init__("system")
        self.cbg = ReentrantCallbackGroup()  # req cb
        self.cbgtimer = ReentrantCallbackGroup()  # main loop cb
        self.ccb = ReentrantCallbackGroup()  # realtime data update cb

        # req
        self.r85cli = self.create_client(Robotiq2FCmd, "/gripper_command", callback_group=self.cbg)

        # constant
        self.ztablelimit = 0.11  # m
        self.inWSdist = 1.0  # m
        self.gripperOpen = 0.085  # 0, 0.085
        self.gripperClose = 0.02
        self.gripperVel = 0.05  # 0.013, 0.1
        self.gripperForce = 30.0  # 5.0, 220.0
        self.homej = [3.4809112548828125e-05, -0.8096572917750855, -1.9074101448059082, -3.6404277286925257, -1.5708077589618128, -3.1415932814227503]
        self.homet = [0.15594326155634256, -0.13457440350544858, 0.5256641098226548, -1.2664412512623242, 1.2635574967973842, -1.1795720169587576]
        self.fixedorientation = self.homet[3:6]

        self.dropzonet = [0.8781918310724031, -0.039869372802589065, -0.022375079856074787]

        # self.dropzoffset = 0.17
        # self.dropt = [0.8781918310724031, -0.039869372802589065, -0.022375079856074787 + self.dropzoffset] + self.fixedorientation  # calucated from perception

        # self.dropj = [0.16559672355651855, -1.9429060421385707, -2.0326151847839355, -2.386176725427145, -1.4103148619281214, -3.134151283894674]
        # self.dropt = [0.5268100809090901, -0.06395943130935745, 0.16152765181404297, -1.2773341136449956, 1.2542592095008442, -1.1738317174684596]
        # self.fakegraspj = [0.632285475730896, -1.5457076591304322, -1.2139649391174316, -3.5172182522215785, -0.9690817038165491, -3.134223286305563]
        # self.fakegraspt = [0.4642602103488932, 0.10377248246679507, 0.6354979900135115, -1.211801008167565, 1.1849909033855954, -1.1877744865830127]

        self.centsub = self.create_subscription(PointStamped, "/p_cent_3d", self.get_cent, 1, callback_group=self.ccb)
        self.topsub = self.create_subscription(PointStamped, "/p_top_3d", self.get_top, 1, callback_group=self.ccb)
        self.botsub = self.create_subscription(PointStamped, "/p_bot_3d", self.get_bot, 1, callback_group=self.ccb)
        self.leftsub = self.create_subscription(PointStamped, "/p_left_3d", self.get_left, 1, callback_group=self.ccb)
        self.rightsub = self.create_subscription(PointStamped, "/p_right_3d", self.get_right, 1, callback_group=self.ccb)

        self.pointc = None
        self.pointb = None
        self.pointt = None
        self.pointl = None
        self.pointr = None

        self.pseudoMatchTime = Duration(seconds=0, nanoseconds=70000000)  # 0.07 sec
        self.iscupmoving = False

        # realtime control
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.0.3")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.3")

        # run loop
        self.timer_ = self.create_timer(0.5, self.run_loop, callback_group=self.cbgtimer)

    def get_cent(self, msg: PointStamped):
        self.pointc = msg

    def get_top(self, msg: PointStamped):
        self.pointt = msg

    def get_bot(self, msg: PointStamped):
        self.pointb = msg

    def get_left(self, msg: PointStamped):
        self.pointl = msg

    def get_right(self, msg: PointStamped):
        self.pointr = msg

    def get_drop_tcp(self):
        h = np.abs(self.pointc.point.z - self.pointb.point.z)
        dropt = self.dropzonet.copy()
        dropt[2] += h + 0.1
        return dropt + self.fixedorientation

    def get_actual_tcp_h(self):
        t = self.rtde_r.getActualTCPPose()
        R, _ = cv2.Rodrigues(np.array(t[3:6]))
        H = np.eye(4) @ rbt.ht(t[0], t[1], t[2])
        H[:3, :3] = R
        return H

    def get_grasp(self):
        Hgrasp = np.eye(4)
        Hgrasp[0:3, 3] = np.array([self.pointc.point.x, self.pointc.point.y, self.pointc.point.z])
        Hgrasp[:3, :3], _ = cv2.Rodrigues(np.array(self.fixedorientation))
        Hgrasp = Hgrasp @ rbt.ht(0.0, 0.0, 0.02)

        graspPointToBase = np.array([self.pointc.point.x, self.pointc.point.y, self.pointc.point.z, 1.00])
        HtcpToBase = self.get_actual_tcp_h()
        PpointTotcp = rbt.hinverse(HtcpToBase) @ graspPointToBase
        z = np.array([0.0, 1.0]).reshape(2, 1)
        p = np.array([PpointTotcp[0], PpointTotcp[2]]).reshape(2, 1)
        dotp = np.sum(z * p)
        normz = np.linalg.norm(z)
        normp = np.linalg.norm(p)
        alpha = np.arccos(dotp / (normz * normp))
        alphasign = alpha * np.sign(PpointTotcp[0])
        HorientCorrection = rbt.hry(alphasign)

        HgraspFinal = Hgrasp @ HorientCorrection
        tvec = HgraspFinal[0:3, 3].tolist()
        rvec, _ = cv2.Rodrigues(HgraspFinal[:3, :3])
        return tvec + rvec.flatten().tolist()

    def is_object_in_ws(self):
        pointxyz = [self.pointc.point.x, self.pointc.point.y, self.pointc.point.z]
        if (dist := np.linalg.norm(pointxyz)) > self.inWSdist:  # if outside of workspace, don't do anything
            self.get_logger().info(f"outside of workdspace at {pointxyz} with distance {dist}")
            return False
        else:
            return True

    def is_object_on_table(self):
        self.get_logger().info(f"z is at {self.pointb.point.z}")
        if self.pointb.point.z < self.ztablelimit:
            self.get_logger().info(f"cup is on table with z at {self.pointb.point.z}")
            return True
        else:
            return False

    def is_object_moving(self):
        if True:
            return True
        else:
            return False

    def is_object_updated(self):
        timetolatest = self.get_clock().now() - Time.from_msg(self.pointc.header.stamp)
        if timetolatest > self.pseudoMatchTime:
            self.get_logger().info("no new centroid data")
            return False
        else:
            return True

    def get_retract(self):
        t = self.rtde_r.getActualTCPPose()
        t[0] -= 0.1
        return t

    def run_loop(self):
        self.timer_.cancel()
        # move to home
        self.move_joint(self.homej)
        self.get_logger().info("moving to home")

        if self.pointc is None:
            self.timer_.reset()
            return

        if not self.is_object_updated():
            self.timer_.reset()
            return

        if not self.is_object_in_ws():
            self.timer_.reset()
            return

        if self.is_object_on_table():
            self.timer_.reset()
            return

        # move to home
        self.move_joint(self.homej)
        self.get_logger().info("moving to home")

        # open gripper
        self.get_logger().info("open gripper")
        self.send_req_gripper(self.gripperOpen, self.gripperVel, self.gripperForce)
        time.sleep(1)

        # move to grasp
        centroidgrasp = self.get_grasp()
        self.move_tcp(centroidgrasp)
        self.get_logger().info("moving to grasp")

        # close gripper
        self.get_logger().info("close gripper")
        self.send_req_gripper(self.gripperClose, self.gripperVel, self.gripperForce)
        time.sleep(1)

        # move to drop off
        # self.move_tcp(self.dropt)
        dropt = self.get_drop_tcp()
        self.move_tcp(dropt)
        self.move_until_contact([0.0, 0.0, -0.1, 0.0 ,0.0 ,0.0])
        # self.move_until_contact_control()
        self.get_logger().info("moving to drop off")

        # open gripper
        self.get_logger().info("open gripper")
        self.send_req_gripper(self.gripperOpen, self.gripperVel, self.gripperForce)
        time.sleep(1)

        # retract motion
        retr = self.get_retract()
        self.move_tcp(retr)
        self.get_logger().info("moving to retract")

        # move to home
        self.move_joint(self.homej)
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

    def get_joint_value(self):
        return self.rtde_r.getActualQ()

    def get_tcp_value(self):
        return self.rtde_r.getActualTCPPose()

    def move_tcp(self, tcp, asyn=False):
        self.rtde_c.moveL(tcp, 0.25, 0.5, asyn)
        # self.rtde_c.stopL(0.5)
        # self.rtde_c.stopScript()

    def move_joint(self, jnt, asyn=False):
        self.rtde_c.moveJ(jnt, 1.05, 1.4, asyn)
        # self.rtde_c.stopJ(0.5)
        # self.rtde_c.stopScript()

    def move_until_contact(self, xd=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        self.rtde_c.moveUntilContact(xd)

    # def move_until_contact_control(self):
    #     e = np.zeros(6)
    #     t = self.get_tcp_value()[0:3]
    #     e[0:3] = np.array(self.dropzonet) - np.array(t)
    #     elimit = np.clip(e, -0.05, 0.05)
    #     while
    #     ec = elimit.tolist() + [0.0, 0.0, 0.0]

    #     self.move_until_contact(ec)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSystem()
    exe = MultiThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        node.rtde_c.servoStop()
        node.rtde_c.stopScript()
        node.destroy_node()
        exe.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
