import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int64
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class NodeA(Node):

    def __init__(self):
        super().__init__("Subscriber_Node")
        self.cbgsub = ReentrantCallbackGroup()
        self.subscription_1 = self.create_subscription(Int64, "topic_1", self.callback1, 1)
        # self.subscription_2 = self.create_subscription(Int64, "topic_2", self.callback2, 2)
        self.j = 0
    def callback1(self, msg):
        # time.sleep(1)
        self.get_logger().info(f"Callback1: {msg.data} at {hex(id(msg))} and is matched {msg.data == self.j}")
        self.j += 1

    def callback2(self, msg):
        # time.sleep(1)
        self.get_logger().info(f"Callback2: {msg.data} at {hex(id(msg))}")


class NodeB(Node):

    def __init__(self):
        super().__init__("Publisher_Node")
        self.cbgpub = ReentrantCallbackGroup()
        self.publisher_1 = self.create_publisher(Int64, "topic_1", 1)
        # self.publisher_2 = self.create_publisher(Int64, "topic_2", 10)

        self.timer1 = self.create_timer(0.0000000000000001, self.timer_callback1)
        # self.timer2 = self.create_timer(3, self.timer_callback2)

        self.i1 = 0
        self.i2 = 0

    def timer_callback1(self):
        msg = Int64()
        msg.data = self.i1
        # time.sleep(1)
        self.publisher_1.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data} at {hex(id(msg))}')
        self.i1 += 1

    def timer_callback2(self):
        msg = Int64()
        msg.data = self.i2
        time.sleep(5)
        self.publisher_2.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data} at {hex(id(msg))}')
        self.i2 += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        subs_node = NodeA()
        pub_node = NodeB()

        executor = SingleThreadedExecutor()
        # executor = MultiThreadedExecutor()
        executor.add_node(subs_node)
        executor.add_node(pub_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            subs_node.destroy_node()
            pub_node.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
