import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class TimerRestart(Node):

    def __init__(self):
        super().__init__("crop_harvest_pose_node")
        self.pub_ = self.create_publisher(String, "/string", 10)
        self.pub_timer = self.create_timer(1, self.string_cb)

    def string_cb(self):
        self.pub_timer.cancel()
        st = String()
        st.data = "saturdays"
        self.pub_.publish(st)
        self.get_logger().info("data is pub")
        time.sleep(3)
        self.pub_timer.reset()


def main_pub(args=None):
    rclpy.init(args=args)
    node = TimerRestart()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main_pub()
