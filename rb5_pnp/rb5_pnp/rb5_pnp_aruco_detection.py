import sys
import os
sys.path.append(str(wd=os.path.abspath(os.getcwd())))
from xtras.spatial_transformation import RigidBodyTransformation as rbt
import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image


class ArUcoSimplePoseEstimator(Node):

    def __init__(self) -> None:
        super().__init__("perception")
        self.ccb = ReentrantCallbackGroup()

        self.cvbride = CvBridge()
        self.imgcsub = self.create_subscription(Image, "/camera/color/image_raw", self.imgc_cb, 2, callback_group=self.ccb)
        self.infcsub = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.infc_cb, 2, callback_group=self.ccb)
        self.imgdpub = self.create_publisher(Image, "/detected_aruco", 2, callback_group=self.ccb)
        self.arcpub = self.create_publisher(PoseStamped, "/arc_pose", 2, callback_group=self.ccb)
        self.cameraMatrix = None
        self.distCoeffs = None

        self.markerLength = 0.025  # m
        self.detectorParams = cv2.aruco.DetectorParameters()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detectorParams)

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

                    if ids[i] == 1: # interested on id 1 only
                        retrvc = rvc
                        rettvc = tvc

        return imageUndst, retrvc, rettvc

    def imgc_cb(self, msg:Image):
        imgBGR = self.cvbride.imgmsg_to_cv2(msg)
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        if self.cameraMatrix is not None:
            imageUndst, retrvc, rettvc = self.pose_estimate(imgRGB, self.cameraMatrix, self.distCoeffs)
            if retrvc is not None:
                rM, _ = cv2.Rodrigues(retrvc)
                H = rbt.conv_rotmat_and_t_to_h(rM, rettvc.reshape(3,))
                t, q = rbt.conv_h_to_t_and_quat(H)
                arcp = PoseStamped()
                arcp.header.frame_id = "camera_color_optical_frame"
                arcp.pose.position.x = t[0]
                arcp.pose.position.y = t[1]
                arcp.pose.position.z = t[2]
                arcp.pose.orientation.x = q[0]
                arcp.pose.orientation.y = q[1]
                arcp.pose.orientation.z = q[2]
                arcp.pose.orientation.w = q[3]
                self.arcpub.publish(arcp)
            imgBGR = cv2.cvtColor(imageUndst, cv2.COLOR_RGB2BGR)
            self.imgdpub.publish(self.cvbride.cv2_to_imgmsg(imgBGR, encoding="rgb8"))

    def infc_cb(self, msg:CameraInfo):
        self.cameraMatrix = msg.k.reshape(3,3)
        self.distCoeffs = np.array(msg.d)


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoSimplePoseEstimator()
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
