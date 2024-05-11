import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_system import Camera
import pathlib


class ImageStream(Node):

    def __init__(self):
        super().__init__("image_stream")
        path = pathlib.Path(__file__).parent.resolve()
        self.camleft = Camera(4, str(path) + "/left.yaml")
        self.br = CvBridge()

        # message
        self.frame = "/camera_color_optical_frame"
        self.infomsg = self.compose_camera_info()
        self.imgpub = self.create_publisher(Image, "/camera/color/image_raw", 1)
        self.infopub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 1)
        self.timer = self.create_timer(0.01, callback=self.timer_callback)

    def timer_callback(self):
        timenow = self.get_clock().now().to_msg()
        _, imglraw = self.camleft.read()

        imgmsg = self.br.cv2_to_imgmsg(imglraw, encoding="bgr8")
        imgmsg.header.stamp = timenow
        imgmsg.header.frame_id = self.frame
        self.imgpub.publish(imgmsg)
        self.infomsg.header.stamp = timenow
        self.infopub.publish(self.infomsg)

    def compose_camera_info(self):
        infomsg = CameraInfo()
        infomsg.header.frame_id = self.frame
        infomsg.height = self.camleft.info["height"]
        infomsg.width = self.camleft.info["width"]
        infomsg.distortion_model = self.camleft.info["distm"]
        infomsg.d = self.camleft.info["d"].tolist()
        infomsg.k = self.camleft.info["k"].flatten().tolist()
        infomsg.r = self.camleft.info["r"].flatten().tolist()
        infomsg.p = self.camleft.info["p"].flatten().tolist()
        return infomsg

def main(args=None):
    rclpy.init(args=args)
    imageNode = ImageStream()
    try:
        rclpy.spin(imageNode)
    except KeyboardInterrupt:
        imageNode.camleft.release()
    finally:
        imageNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
