import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from robotiq2f_interfaces.srv import Robotiq2FCmd
from rb_interfaces.srv import ReqJnt, ReqTcp


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
        self.gripperClose = 0.01
        self.gripperVel = 0.05  # 0.013, 0.1
        self.gripperForce = 100.0  # 5.0, 220.0
        self.homej = [-1.5707963267948966, -0.5235987755982988, 2.2689280275926285, -0.8726646259971648, 1.5707963267948966, 0.0]
        self.dropj = [-0.0010280520263029234, 0.0737424229192113, 2.3063949757045172, -0.8086622736788628, 1.5701094975497651, -0.04206219096092846]
        self.pgspj = [-1.5112975796170003, 0.6240719872017728, 1.4529076395589584, -0.5057010680714485, 1.5711342819372276, -7.593964674665714e-05]
        self.gspj = [-1.5114223487140706, 0.7315973825298311, 1.5095179222185549, -0.6698345489512039, 1.5705999186645032, -0.0007429750007164083]
        self.retrj1 = [-1.4696863533747848, -0.2114430562616566, 1.840798783383708, -0.0578864361644487, 1.571919648142692, -0.03920966247258778]
        self.retrj2 = [0.0, -0.2114430562616566, 1.840798783383708, -0.0578864361644487, 1.571919648142692, -0.03920966247258778]

        # joint now
        self.jntcsub = self.create_subscription(JointState, "/joint_states", self.jntc_cb, 10, callback_group=self.ccb)
        self.jntc = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

        # move to pregrasp
        self.send_req_jnt(self.pgspj)
        while not self.is_near(self.pgspj, self.jntc):
            self.get_logger().info("moving to pregrasp")

        # move to grasp
        self.send_req_jnt(self.gspj)
        while not self.is_near(self.gspj, self.jntc):
            self.get_logger().info("moving to grasp")

        # close gripper
        self.get_logger().info("close gripper")
        self.send_req_gripper(self.gripperClose, self.gripperVel, self.gripperForce)
        time.sleep(1)

        # move to pregrasp
        self.send_req_jnt(self.pgspj)
        while not self.is_near(self.pgspj, self.jntc):
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

        self.timer_.reset()

    def send_req_gripper(self, pos, vel, force):
        rqm = Robotiq2FCmd.Request()
        rqm.pos = pos
        rqm.vel = vel
        rqm.force = force
        future = self.r85cli.call_async(rqm)
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

    def jntc_cb(self, msg):
        self.jntc = msg.position

    def is_near(self, jntd, jntc):
        e = np.array(jntd) - np.array(jntc)
        if np.linalg.norm(e) < 0.05:
            return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = RB5PnPRun()
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
