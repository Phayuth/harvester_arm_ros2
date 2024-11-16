import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__("frame_listener")
        self.tfbuf = Buffer()
        self.tflisten = TransformListener(self.tfbuf, self)
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        child = "camera_color_optical_frame"
        parent = "camera_link"
        try:
            now = rclpy.time.Time()
            trans = self.tfbuf.lookup_transform(parent, child, now)
            translation = trans.transform.translation
            print(f"> translation: {translation}")
            rotation = trans.transform.rotation
            print(f"> rotation: {rotation}")

        except TransformException as ex:
            self.get_logger().info(f"Could not transform {parent} to {child}: {ex}")


def main(args=None):
    rclpy.init(args=args)
    imageSubscriber = FrameListener()
    rclpy.spin(imageSubscriber)
    imageSubscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
