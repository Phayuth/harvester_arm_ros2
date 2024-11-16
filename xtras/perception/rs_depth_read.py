import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthInfoNode(Node):

    def __init__(self):
        super().__init__("depth_info_node")
        self.depthsub = self.create_subscription(Image, "/camera/depth/image_rect_raw", self.process_depth_image, 10)

    def process_depth_image(self, msg):
        px = 320  # X coordinate of the pixel
        py = 240  # Y coordinate of the pixel
        indx = (py * msg.width) + px
        depth = float(msg.data[indx * 2] + (msg.data[indx * 2 + 1] << 8)) / 1000.0
        self.get_logger().info(f"Depth at pixel {px}, {py}, {depth}")


def main(args=None):
    rclpy.init(args=args)
    dnode = DepthInfoNode()
    rclpy.spin(dnode)
    dnode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
