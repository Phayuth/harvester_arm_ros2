import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.imgPublisher = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(timer_period_sec=0.1, callback=self.timer_callback)
        self.cap = cv2.VideoCapture(4)
        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.imgPublisher.publish(self.br.cv2_to_imgmsg(frame))
        self.get_logger().info('Publishing video frame')


def main(args=None):

    rclpy.init(args=args)
    imagePublisherNode = ImagePublisher()
    rclpy.spin(imagePublisherNode)
    imagePublisherNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
