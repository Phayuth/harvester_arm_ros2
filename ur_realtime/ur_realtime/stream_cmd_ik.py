import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState

import rtde_control
import rtde_receive
import numpy as np


class ReadWriteThread(Node):

    def __init__(self) -> None:
        super().__init__("cam_tracking")
        self.ccb = ReentrantCallbackGroup()
        self.cbgloop = ReentrantCallbackGroup()

        # vision
        self.jntsub = self.create_subscription(JointState, "/relaxed_ik/joint_angle_solutions", self.jnt_desired_cb, 1, callback_group=self.ccb)
        self.jntdesired = None

        # realtime control
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.0.3")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.3")

        # # servoj param
        # self.velocity = 0.2  # rad/s
        # self.acceleration = 1.5  # rad/ss
        # self.dt = 1.0 / 500  # 2ms
        # self.lookahead_time = 0.1  # s
        # self.gain = 300
        # self.kp = 3.0

        # speedj param
        self.acceleration = 1.5  # rad/ss
        self.dt = 1.0 / 500  # 2ms
        self.kp = 3.0

        # control loop
        self.controllooop = self.create_timer(self.dt, self.loop, self.cbgloop)

        self.get_logger().info(f"Streaming")

    def jnt_desired_cb(self, msg: JointState):
        self.jntdesired = msg.position

    def loop(self):
        # servoj control
        # self.rtde_c.servoJ(jntcmd, self.velocity, self.acceleration, self.dt, self.lookahead_time, self.gain)
        # self.rtde_c.waitPeriod(t_start)

        # speedj control
        if self.jntdesired is not None:
            t_start = self.rtde_c.initPeriod()

            # p controller
            jntc = self.rtde_r.getActualQ()
            e = self.kp * (np.array(self.jntdesired) - np.array(jntc))

            self.rtde_c.speedJ(e, self.acceleration, self.dt)
            self.rtde_c.waitPeriod(t_start)


def main(args=None):
    rclpy.init(args=args)
    node = ReadWriteThread()
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
