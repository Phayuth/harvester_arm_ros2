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
        self.imgcsub = self.create_subscription(Image, "/camera/color/image_raw", self.imgc_cb, 1, callback_group=self.ccb)
        self.infcsub = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.infc_cb, 1, callback_group=self.ccb)
        self.imgdpub = self.create_publisher(Image, "/detected_aruco", 1, callback_group=self.ccb)
        self.cameraMatrix = None
        self.distCoeffs = None
        self.cvbride = CvBridge()
        self.ttarget = None
        self.rtarget = None

        self.markerLength = 0.120  # m
        self.detectorParams = cv2.aruco.DetectorParameters()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detectorParams)

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

        # control loop
        self.controllooop = self.create_timer(self.dt, self.loop, self.cbgloop)

        self.get_logger().info(f"Perception initialized.")

    def pose_estimate(self, imageRaw, cameraMatrix, distCoeffs):
        h, w = imageRaw.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (w, h), 1, (w, h))
        imageUndst = cv2.undistort(imageRaw, cameraMatrix, distCoeffs, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        imageUndst = imageUndst[y : y + h, x : x + w]

        corners, ids, rej = self.detector.detectMarkers(imageUndst)
        retrvc = None
        rettvc = None
        if not ids is None:

            cv2.aruco.drawDetectedMarkers(imageUndst, corners, ids)  # aruco corner
            for i in range(len(ids)):
                marker_points = np.array([[-self.markerLength / 2.0,  self.markerLength / 2.0, 0],
                                          [ self.markerLength / 2.0,  self.markerLength / 2.0, 0],
                                          [ self.markerLength / 2.0, -self.markerLength / 2.0, 0],
                                          [-self.markerLength / 2.0, -self.markerLength / 2.0, 0]], dtype=np.float32)
                retval, rvc, tvc = cv2.solvePnP(marker_points, corners[i], cameraMatrix, distCoeffs, None, None, False, cv2.SOLVEPNP_IPPE_SQUARE)
                if retval:
                    cv2.drawFrameAxes(imageUndst, cameraMatrix, distCoeffs, rvc, tvc, self.markerLength, 3)

                    if ids[i] == 0:  # interested on id 0 only
                        retrvc = rvc
                        rettvc = tvc

        return imageUndst, retrvc, rettvc

    def imgc_cb(self, msg: Image):
        imgBGR = self.cvbride.imgmsg_to_cv2(msg)
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        if self.cameraMatrix is not None:
            imageUndst, retrvc, rettvc = self.pose_estimate(imgRGB, self.cameraMatrix, self.distCoeffs)
            if retrvc is not None:
                self.rtarget = retrvc
                self.ttarget = rettvc
            else:
                self.rtarget = np.array([[0.0],[0.0],[0.0]])
                self.ttarget = np.array([[0.0],[0.0],[0.0]])
                # rM, _ = cv2.Rodrigues(retrvc)
                # H = rbt.conv_rotmat_and_t_to_h(rM, rettvc.reshape(3,))
                # t, q = rbt.conv_h_to_t_and_quat(H)
            imgBGR = cv2.cvtColor(imageUndst, cv2.COLOR_RGB2BGR)
            self.imgdpub.publish(self.cvbride.cv2_to_imgmsg(imgBGR, encoding="rgb8"))

    def infc_cb(self, msg: CameraInfo):
        self.cameraMatrix = msg.k.reshape(3, 3)
        self.distCoeffs = np.array(msg.d)

    def ik_pose(self, positions):
        return self.relaxed_ik.solve_position(positions, self.fixedorient, self.tol)

    def ik_vel(self, linear_vels):
        angular_vels = [0.0, 0.0, 0.0]  # angular xyz
        return self.relaxed_ik.solve_velocity(linear_vels, angular_vels, self.tol)

    def ik_reset(self, current_joint):
        self.relaxed_ik.reset(current_joint)

    def loop(self):
        self.get_logger().info(f"{self.ttarget}")
        if self.ttarget is not None:
            xcmdv = np.clip(self.ttarget[0], -self.maxtranslationvelo, self.maxtranslationvelo)
            ycmdv = 0.0 # self.ttarget[2]
            zcmdv = np.clip(-self.ttarget[1], -self.maxtranslationvelo, self.maxtranslationvelo)

            jntcmd = self.ik_vel([xcmdv, ycmdv, zcmdv])
            self.get_logger().info(f"{jntcmd}")

            t_start = self.rtde_c.initPeriod()
            self.rtde_c.servoJ(jntcmd, self.velocity, self.acceleration, self.dt, self.lookahead_time, self.gain)
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
