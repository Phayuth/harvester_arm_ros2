import sys

sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")
from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt

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
import time
import cv2
import pytransform3d.transformations as pt
import matplotlib.pyplot as plt
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
from sensor_msgs.msg import PointCloud2


class SimpleSystem(Node):

    def __init__(self) -> None:
        super().__init__("system")
        self.cbg = ReentrantCallbackGroup()  # req cb
        self.cbgtimer = ReentrantCallbackGroup()  # main loop cb
        self.ccb = ReentrantCallbackGroup()  # realtime data update cb

        # constant
        self.homej = [3.4809112548828125e-05, -0.8096572917750855, -1.9074101448059082, -3.6404277286925257, -1.5708077589618128, -3.1415932814227503]
        self.homet = [0.15594326155634256, -0.13457440350544858, 0.5256641098226548, -1.2664412512623242, 1.2635574967973842, -1.1795720169587576]
        self.tcpposessave = []
        self.i = 0

        # data stream
        self.pointcloudSub = self.create_subscription(PointCloud2, "/zed/zed_node/point_cloud/cloud_registered", self.update_pcd_array, 1)
        self.pseudoMatchTime = Duration(seconds=0, nanoseconds=70000000)  # 0.07 sec

        # realtime control
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.0.3")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.3")

        # run loop
        self.timer_ = self.create_timer(0.5, self.run_loop, callback_group=self.cbgtimer)

    def get_actual_tcp_h(self):
        t = self.get_tcp_value()
        return self.tcp_to_h(t)

    def tcp_to_h(self, tcp):
        R, _ = cv2.Rodrigues(np.array(tcp[3:6]))
        H = np.eye(4) @ rbt.ht(tcp[0], tcp[1], tcp[2])
        H[:3, :3] = R
        return H

    def h_to_tcp(self, H):
        tvec = H[0:3, 3].tolist()
        rvec, _ = cv2.Rodrigues(H[:3, :3])
        return tvec + rvec.flatten().tolist()

    def get_joint_value(self):
        return self.rtde_r.getActualQ()

    def get_tcp_value(self):
        return self.rtde_r.getActualTCPPose()

    def move_tcp(self, tcp, asyn=False):
        self.rtde_c.moveL(tcp, 0.25, 0.5, asyn)

    def move_joint(self, jnt, asyn=False):
        self.rtde_c.moveJ(jnt, 1.05, 1.4, asyn)

    def scan_motion(self, callablefunc=None):
        x1 = np.array([0.27807259335283196, -0.30019781096384157, 0.6473946314428765, -1.6166917736115658, 1.4040813480080727, -1.0089979438086432])
        x2 = np.array([0.27805100450130654, 0.0019913585353435563, 0.6473984922940137, -1.616586865451318, 1.4041997172743135, -1.0090335750800299])
        x3 = np.array([0.27806122057702065, 0.0020114963649783164, 0.559859533380667, -1.6166187065971134, 1.4041989683614604, -1.0090056371005565])
        x4 = np.array([0.27806122057702065, -0.30019781096384157, 0.559859533380667, -1.6166187065971134, 1.4041989683614604, -1.0090056371005565])

        seg1 = np.linspace(x1, x2, num=3, endpoint=False)
        seg2 = np.linspace(x2, x3, num=1, endpoint=False)
        seg3 = np.linspace(x3, x4, num=3, endpoint=False)
        seg4 = np.linspace(x4, x1, num=1, endpoint=False)

        tcplists = np.vstack((seg1, seg2, seg3, seg4))

        for tcpi in tcplists:
            self.move_tcp(tcpi)
            time.sleep(2)
            if callablefunc is not None:
                callablefunc()
                tcpnow = self.get_tcp_value()
                self.tcpposessave.append(tcpnow)
                self.i += 1

        # # plot
        # ax = pt.plot_transform(name="base")
        # for tcpi in tcplists:
        #     H = self.tcp_to_h(tcpi)
        #     pt.plot_transform(ax, H, name="TCP")
        # plt.show()

    def save_single_pcd(self):
        np.save(f"/home/yuth/pcd_leftframe_id_{self.i}.npy", self.pcd)

    def convert_pointcloud_to_array(self, msg: PointCloud2):
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = list(gen)
        data = np.array(int_data, dtype=np.float32)

        pcd = np.empty((data.shape[0], 6), dtype=np.float32)
        pcd[:, 0:3] = data[:, 0:3]

        colorfloat = data[:, 3].tobytes()
        uint32_val = np.frombuffer(colorfloat, dtype=np.uint32)
        pcd[:, 3] = (uint32_val & 0x00FF0000) >> 16
        pcd[:, 4] = (uint32_val & 0x0000FF00) >> 8
        pcd[:, 5] = uint32_val & 0x000000FF
        return pcd

    def update_pcd_array(self, msg: PointCloud2):
        self.pcd = self.convert_pointcloud_to_array(msg)
        self.get_logger().info("New PCD received")

    def run_loop(self):
        self.timer_.cancel()

        self.scan_motion(self.save_single_pcd)

        np.save(f"/home/yuth/tool0_to_base.npy", np.array(self.tcpposessave))
        self.get_logger().info("Finished Scan")

        # self.timer_.reset()


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
