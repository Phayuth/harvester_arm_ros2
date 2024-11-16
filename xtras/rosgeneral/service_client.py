from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__("minimal_client_async")
        self.cb1 = MutuallyExclusiveCallbackGroup()
        self.cb2 = MutuallyExclusiveCallbackGroup()

        self.cli = self.create_client(AddTwoInts, "add_two_ints", callback_group=self.cb1)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = AddTwoInts.Request()
        self.tt = self.create_timer(0.1, self.send_request, self.cb2)

        self.a = 0
        self.b = 0

    def send_request(self):
        self.req.a = self.a
        self.req.b = self.b

        self.rr = self.cli.call(self.req)
        self.get_logger().info(f"called")
        self.get_logger().info(f"resp {self.rr.sum}")

        self.a += 2
        self.b += 3

        # self.future = self.cli.call_async(self.req)
        # self.get_logger().info(f"called")
        # rclpy.spin_until_future_complete(self, self.future)
        # res = self.future.result()
        # self.get_logger().info(f"resp {res.sum}")


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()

    exe = MultiThreadedExecutor()
    exe.add_node(minimal_client)
    exe.spin()
    exe.spin_until_future_complete()
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
