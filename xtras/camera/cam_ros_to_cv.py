import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSubscriber(Node):

    def __init__(self):

        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.listener_callback, 10)
        self.br = CvBridge()
        self.get_logger().info('Receiving video frame')

    def listener_callback(self, data):
        imgBGR = self.br.imgmsg_to_cv2(data)
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        cv2.imshow("camera", imgRGB)
        cv2.waitKey(1)  # not waitkey lead to black screen on imshow


def main(args=None):
    rclpy.init(args=args)
    imageSubscriberNode = ImageSubscriber()
    rclpy.spin(imageSubscriberNode)
    imageSubscriberNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
