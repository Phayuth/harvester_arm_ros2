import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import cv2
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup

path_to_src = get_package_share_directory("relaxed_ik_ros2") + "/relaxed_ik_core"
sys.path.insert(1, path_to_src + "/wrappers")
from python_wrapper import RelaxedIKRust

import rtde_control
import rtde_receive
import time
import numpy as np

os.chdir(path_to_src)  # for some reason the library dont let user give urdf path directly

setting_file_path = path_to_src + "/configs/example_settings/ur5e_calibrated.yaml"


class ReadWriteThread(Node):

    def __init__(self) -> None:
        super().__init__("cam_tracking")
        self.ccb = ReentrantCallbackGroup()
        self.cbgloop = ReentrantCallbackGroup()

        # kinematic # start pose and joint
        self.relaxed_ik = RelaxedIKRust(setting_file_path)
        self.j = [-1.57, -1.57, -1.57, 0.0, 1.57, 0.0]
        self.p = [-0.13294, -0.63148, 0.68919]
        self.fixedorient = [0.0014894, -0.70651, 0.7077, 0.0018552]
        self.tol = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.maxtranslationvelo = 0.00005

        # vision
        self.cvbride = CvBridge()

        # realtime control
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.0.3")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.3")

        self.velocity = 0.3  # rad/s
        self.acceleration = 0.3  # rad/ss
        self.dt = 1.0 / 500  # 2ms
        self.lookahead_time = 0.1  # s
        self.gain = 300

        # move to initial pose
        self.rtde_c.moveJ(self.j, speed=0.1, acceleration=0.1)
        # self.ik_reset(self.j)

        self.acceleration = 1.5  # rad/ss
        self.dt = 1.0 / 500  # 2ms
        self.kp = 3.0

        # control loop
        self.controllooop = self.create_timer(self.dt, self.loop, self.cbgloop)

        self.get_logger().info(f"Perception initialized.")

    def ik_pose(self, positions):
        return self.relaxed_ik.solve_position(positions, self.fixedorient, self.tol)

    def ik_vel(self, linear_vels):
        angular_vels = [0.0, 0.0, 0.0]  # angular xyz
        return self.relaxed_ik.solve_velocity(linear_vels, angular_vels, self.tol)

    def ik_reset(self, current_joint):
        self.relaxed_ik.reset(current_joint)

    def loop_servoj(self):
        if self.ttarget is not None:
            xcmdv = np.clip(self.ttarget[0], -self.maxtranslationvelo, self.maxtranslationvelo)
            ycmdv = 0.0  # self.ttarget[2]
            zcmdv = np.clip(-self.ttarget[1], -self.maxtranslationvelo, self.maxtranslationvelo)

            jntcmd = self.ik_vel([xcmdv, ycmdv, zcmdv])
            self.get_logger().info(f"{jntcmd}")

            t_start = self.rtde_c.initPeriod()
            self.rtde_c.servoJ(jntcmd, self.velocity, self.acceleration, self.dt, self.lookahead_time, self.gain)
            self.rtde_c.waitPeriod(t_start)

    def loop_speedj(self):
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
