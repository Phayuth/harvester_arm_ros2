import rclpy
from rclpy.node import Node

from onrobotsg_interfaces.srv import GripperSgSrv

from .driver import SG


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Service(Node):

    def __init__(self):
        super().__init__('service')

        self.ip = "192.168.1.1"
        self.port = 502
        self.model_id = 3
        self.gent = True

        self.get_logger().info(bcolors.OKGREEN + "Setting Up Connection" + bcolors.ENDC)
        self.sg = SG(self.ip, self.port)
        self.sg.set_model_id(self.model_id)
        self.sg.set_init()
        self.sg.set_gentle(self.gent)
        self.get_logger().info("Gripper width is 110 to 750")

        self.srv = self.create_service(GripperSgSrv, 'gripper_Command', self.execute_callback)

    def execute_callback(self, request, response):
        self.get_logger().info('Incoming request %d' % request.desiredwidth)
        self.sg.set_target(request.desiredwidth)
        self.sg.set_move()
        response.status = "SUCCESS"
        return response

    def close_connection(self):
        self.sg.close_connection()
        self.get_logger().info("Gripper connection is disconnected")


def main(args=None):
    rclpy.init(args=args)
    service = Service()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':

    try:
        main()

    except Exception as e:
        raise e

    finally:
        service.close_connection()
