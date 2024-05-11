import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time
from loadcam_data import camInfoLeft, camInfoRight


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")
        self.cbg = ReentrantCallbackGroup()

        self.cvb = CvBridge()

        # camera info
        self.camLeftInfoMsg = camInfoLeft
        self.camRightInfoMsg = camInfoRight

        # camera data
        self.imgLeftRawSub = self.create_subscription(Image, "/camera_left/color/image_raw", self.cam_left_image, 1, callback_group=self.cbg)
        self.imgRightRawSub = self.create_subscription(Image, "/camera_right/color/image_raw", self.cam_right_image, 1, callback_group=self.cbg)
        self.imgLeftMsg = None
        self.imgRightMsg = None
        self.pseudoMatchTime = Duration(seconds=0, nanoseconds=1000000)  # 0.01 sec

        # debug data
        self.imgLeftDebugPub = self.create_publisher(Image, "/camera_left/image_debug", 5, callback_group=self.cbg)
        self.imgRightDebugPub = self.create_publisher(Image, "/camera_right/image_debug", 5, callback_group=self.cbg)

        # detection
        self.markerLength = 0.120  # m
        self.detectorParams = cv2.aruco.DetectorParameters()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detectorParams)

        # processing data
        self.timer_ = self.create_timer(0.040, self.process_images, callback_group=self.cbg)
        self.Once = True

    def cam_left_image(self, msg: Image):
        self.imgLeftMsg = msg

    def cam_right_image(self, msg: Image):
        self.imgRightMsg = msg

    def process_images(self):
        if self.imgLeftMsg is None or self.imgRightMsg is None:
            return

        if Time.from_msg(self.imgLeftMsg.header.stamp) - Time.from_msg(self.imgRightMsg.header.stamp) < self.pseudoMatchTime:
            if self.Once is True:
                self.und_rec_map()
                self.Once = False

            elif self.Once is False:
                imgl = self.cvb.imgmsg_to_cv2(self.imgLeftMsg, desired_encoding="rgb8")
                imgr = self.cvb.imgmsg_to_cv2(self.imgRightMsg, desired_encoding="rgb8")

                lremap = self.remap(imgl, self.leftmapx, self.leftmapy)
                rremap = self.remap(imgr, self.Rightmapx, self.Rightmapy)

                centerLeft = self.aruco_pixels(lremap)
                centerRight = self.aruco_pixels(rremap)
                if centerLeft is not None and centerRight is not None:

                    point3d = self.triangulate(np.array(centerLeft), np.array(centerRight))
                    self.get_logger().info(f"point in 3d: {point3d}")

                self.imgLeftDebugPub.publish(self.cvb.cv2_to_imgmsg(lremap, encoding="rgb8"))
                self.imgRightDebugPub.publish(self.cvb.cv2_to_imgmsg(rremap, encoding="rgb8"))

    def aruco_pixels(self, imageRectified):
        corners, ids, rej = self.detector.detectMarkers(imageRectified)
        centerx = None
        centery = None
        if not ids is None:
            cv2.aruco.drawDetectedMarkers(imageRectified, corners, ids)  # aruco corner
            for i in range(len(ids)):
                centerx = (corners[i][0][0][0] + corners[i][0][2][0]) / 2
                centery = (corners[i][0][0][1] + corners[i][0][2][1]) / 2
                cv2.circle(imageRectified, (int(centerx), int(centery)), 15, (200, 2, 1), 3)

        return (centerx, centery)

    def triangulate(self, point1, point2):
        if point1.dtype != "float64":
            point1 = point1.astype(np.float64)

        if point2.dtype != "float64":
            point2 = point2.astype(np.float64)

        point3d = cv2.triangulatePoints(self.camLeftInfoMsg["p"], self.camRightInfoMsg["p"], point1.reshape(2, 1), point2.reshape(2, 1), None).flatten()
        point3d /= point3d[-1]
        return point3d

    def und_rec_map(self):
        """
        Computes the undistortion and rectification transformation map.
        we get output:
        lmapx, lmapy, rmapx, rmapy
        """
        if self.camLeftInfoMsg is not None and self.camRightInfoMsg is not None:
            self.leftmapx, self.leftmapy = cv2.initUndistortRectifyMap(self.camLeftInfoMsg["k"], self.camLeftInfoMsg["d"], self.camLeftInfoMsg["r"], self.camLeftInfoMsg["p"], (self.camLeftInfoMsg["width"], self.camLeftInfoMsg["height"]), cv2.CV_32FC1, None, None)
            self.Rightmapx, self.Rightmapy = cv2.initUndistortRectifyMap(self.camRightInfoMsg["k"], self.camRightInfoMsg["d"], self.camRightInfoMsg["r"], self.camRightInfoMsg["p"], (self.camRightInfoMsg["width"], self.camRightInfoMsg["height"]), cv2.CV_32FC1, None, None)

    def remap(self, src, mapx, mapy):
        return cv2.remap(src, mapx, mapy, cv2.INTER_LINEAR)

    def undistort_points(self, src, intrinsics, distortion, R, P):
        return cv2.undistortPoints(src, intrinsics, distortion, R=R, P=P)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
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

# https://github.com/AttilioLughetta/Stereo-images-to-3D-reconstruction/blob/master/reconstruction.py