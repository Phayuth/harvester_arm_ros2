"""https://gist.github.com/driftregion/14f6da05a71a57ef0804b68e17b06de5"""
from threading import Event
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_srvs.srv import Trigger


class ServiceFromService(Node):

    def __init__(self):
        super().__init__('action_from_service')
        self.service_done_event = Event()
        self.callback_group = ReentrantCallbackGroup()

        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.client = self.create_client(AddTwoInts, '/add_two_ints', callback_group=self.callback_group)

        self.srv_prox = self.create_service(Trigger, 'add_two_ints_proxy', self.add_two_ints_proxy_callback, callback_group=self.callback_group)

    def add_two_ints_callback(self, request, response): # main server
        self.get_logger().info('Request received: {} + {}'.format(request.a, request.b))
        response.sum = request.a + request.b
        return response

    def add_two_ints_proxy_callback(self, request, response):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('No action server available')

        self.service_done_event.clear()
        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        requestadd = AddTwoInts.Request()
        requestadd.a = 5
        requestadd.b = 5
        future = self.client.call_async(requestadd)
        future.add_done_callback(done_callback)

        # Wait for action to be done
        # self.service_done_event.wait()
        event.wait()
        respo = future.result()
        response.success = True
        response.message = f"{respo.sum}"
        return response

    # def get_result_callback(self, future):
    #     # Signal that action is done
    #     self.service_done_event.set()


def main(args=None):
    rclpy.init(args=args)
    service_from_service = ServiceFromService()
    executor = MultiThreadedExecutor()
    rclpy.spin(service_from_service, executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()