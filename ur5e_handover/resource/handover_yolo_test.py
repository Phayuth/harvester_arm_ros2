import sys

sys.path.append("/home/yuth/ws_yuthdev/neural_network")
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time

from projects.yolo_detection.yolo_centroid import CVYOLOCentroid


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")
        self.cbg = ReentrantCallbackGroup()

        self.cvb = CvBridge()

        # camera info
        self.imgLeftInfoSub = self.create_subscription(CameraInfo, "/camera_left/camera_info", self.cam_left_info_cb, 1)
        self.imgRightInfoSub = self.create_subscription(CameraInfo, "/camera_right/camera_info", self.cam_right_info_cb, 1)

        # camera data
        self.imgLeftRawSub = self.create_subscription(Image, "/camera_left/image_raw", self.cam_left_image, 1, callback_group=self.cbg)
        self.imgRightRawSub = self.create_subscription(Image, "/camera_right/image_raw", self.cam_right_image, 1, callback_group=self.cbg)
        self.imgLeftMsg = None
        self.imgRightMsg = None
        self.pseudoMatchTime = Duration(seconds=0, nanoseconds=1000000)  # 0.01 sec

        # debug data
        self.imgLeftDebugPub = self.create_publisher(Image, "/camera_left/image_debug", 5, callback_group=self.cbg)
        self.imgRightDebugPub = self.create_publisher(Image, "/camera_right/image_debug", 5, callback_group=self.cbg)

        # detection
        self.yoloWeightDir = "/home/yuth/ws_yuthdev/neural_network/datasave/neural_weight/yolov8x-seg.pt"
        self.cvc = CVYOLOCentroid(self.yoloWeightDir, interestNames=["wine glass", "cup"])

        # processing data
        self.timer_ = self.create_timer(0.040, self.process_images, callback_group=self.cbg)
        self.Once = True

    def cam_left_info_cb(self, msg: CameraInfo):
        self.camLeftInfoMsg = {
            "height": msg.height,
            "width": msg.width,
            "distm": msg.distortion_model,
            "d": np.array(msg.d),
            "k": msg.k.reshape(3, 3),
            "r": msg.r.reshape(3, 3),
            "p": msg.p.reshape(3, 4),
        }
        self.imgLeftInfoSub.destroy()

    def cam_right_info_cb(self, msg: CameraInfo):
        self.camRightInfoMsg = {
            "height": msg.height,
            "width": msg.width,
            "distm": msg.distortion_model,
            "d": np.array(msg.d),
            "k": msg.k.reshape(3, 3),
            "r": msg.r.reshape(3, 3),
            "p": msg.p.reshape(3, 4),
        }
        self.imgRightInfoSub.destroy()

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

                centerLeft = self.cvc.detect_centroid(lremap, edgeErode=True, drawImg=True)
                centerRight = self.cvc.detect_centroid(rremap, edgeErode=True, drawImg=True)

                if centerLeft is not None and centerRight is not None:

                    point3d = self.triangulate(np.array(centerLeft), np.array(centerRight))
                    self.get_logger().info(f"point in 3d: {point3d}")

                self.imgLeftDebugPub.publish(self.cvb.cv2_to_imgmsg(lremap, encoding="rgb8"))
                self.imgRightDebugPub.publish(self.cvb.cv2_to_imgmsg(rremap, encoding="rgb8"))

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
        we get output: lmapx, lmapy, rmapx, rmapy
        """
        if self.camLeftInfoMsg is not None and self.camRightInfoMsg is not None:
            self.leftmapx, self.leftmapy = cv2.initUndistortRectifyMap(self.camLeftInfoMsg["k"], self.camLeftInfoMsg["d"], self.camLeftInfoMsg["r"], self.camLeftInfoMsg["p"], (self.camLeftInfoMsg["width"], self.camLeftInfoMsg["height"]), cv2.CV_32FC1, None, None)
            self.Rightmapx, self.Rightmapy = cv2.initUndistortRectifyMap(self.camRightInfoMsg["k"], self.camRightInfoMsg["d"], self.camRightInfoMsg["r"], self.camRightInfoMsg["p"], (self.camRightInfoMsg["width"], self.camRightInfoMsg["height"]), cv2.CV_32FC1, None, None)

    def remap(self, src, mapx, mapy):
        return cv2.remap(src, mapx, mapy, cv2.INTER_LINEAR)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    exe = SingleThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        node.destroy_node()
        exe.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
