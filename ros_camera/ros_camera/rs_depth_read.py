import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthInfoNode(Node):

    def __init__(self):
        super().__init__('depth_info_node')
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.process_depth_image, 10)

    def process_depth_image(self, msg):
        pixel_x = 320  # X coordinate of the pixel
        pixel_y = 240  # Y coordinate of the pixel

        image_index = (pixel_y * msg.width) + pixel_x
        depth = float(msg.data[image_index * 2] + (msg.data[image_index*2 + 1] << 8)) / 1000.0

        self.get_logger().info(f"Depth at pixel {pixel_x}, {pixel_y}, {depth}")


def main(args=None):
    rclpy.init(args=args)
    depth_info_node = DepthInfoNode()
    rclpy.spin(depth_info_node)
    depth_info_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
