import rclpy
from rclpy.node import Node

from onrobotsg_interfaces.srv import GripperSgSrv


class client_call(Node):

    def __init__(self):
        super().__init__('gripper_client_call')

        self.gripper_call = self.create_client(GripperSgSrv, 'gripper_Command')
        while not self.gripper_call.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GripperSgSrv.Request()

    def send_request(self):
        self.req.desiredwidth = 500  # desired width
        self.future = self.gripper_call.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    client = client_call()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info('Status is : %s' % response.status)
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
