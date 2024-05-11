import rclpy
from rclpy.node import Node
import cv2
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from geometry_msgs.msg import Twist
from relaxed_ik_ros2.msg import EEPoseGoals, EEVelGoals


class ReadWriteThread(Node):

    def __init__(self) -> None:
        super().__init__("cam_tracking")
        self.ccb = ReentrantCallbackGroup()
        self.cbgloop = ReentrantCallbackGroup()

        # kinematic # start pose and joint
        self.j = [-1.57, -1.57, -1.57, 0.0, 1.57, 0.0]
        self.p = [0.11056, 0.74868, 0.46499]
        self.fixedorient = [-0.85027, 0.013521, -0.048529, 0.52394]
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

        self.ee_vel_goals_pub = self.create_publisher(EEVelGoals, "relaxed_ik/ee_vel_goals", 5)

        self.markerLength = 0.120  # m
        self.detectorParams = cv2.aruco.DetectorParameters()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detectorParams)

        self.dt = 1.0 / 500  # 2ms
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
                # arcp.pose.position.x = t[0]
                # arcp.pose.position.y = t[1]
                # arcp.pose.position.z = t[2]
            imgBGR = cv2.cvtColor(imageUndst, cv2.COLOR_RGB2BGR)
            self.imgdpub.publish(self.cvbride.cv2_to_imgmsg(imgBGR, encoding="rgb8"))

    def infc_cb(self, msg: CameraInfo):
        self.cameraMatrix = msg.k.reshape(3, 3)
        self.distCoeffs = np.array(msg.d)

    def loop(self):
        self.get_logger().info(f"{self.ttarget}")

        msg = EEVelGoals()

        if self.ttarget is not None:
            xcmdv = np.clip(self.ttarget[0], -self.maxtranslationvelo, self.maxtranslationvelo)
            ycmdv = 0.0 # self.ttarget[2]
            zcmdv = np.clip(-self.ttarget[1], -self.maxtranslationvelo, self.maxtranslationvelo)

            for i in range(1):  # 1 chain
                twist = Twist()
                twist.linear.x = float(xcmdv)
                twist.linear.y = float(ycmdv)
                twist.linear.z = float(zcmdv)

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0

                tolerance = Twist()
                tolerance.linear.x = 0.0
                tolerance.linear.y = 0.0
                tolerance.linear.z = 0.0
                tolerance.angular.x = 0.0
                tolerance.angular.y = 0.0
                tolerance.angular.z = 0.0

                msg.ee_vels.append(twist)
                msg.tolerances.append(tolerance)

            self.ee_vel_goals_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReadWriteThread()
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
