import rclpy
from rclpy.node import Node
from robotiq2f_interfaces.srv import GripperR2FCmd, GripperR2FInfo
from .robotiq_85_gripper import Robotiq85Gripper
import time


class Robitiq2FServiceNode(Node):

    def __init__(self):
        super().__init__('service')
        self.gripper = Robotiq85Gripper()
        self.startup_routine()

        self.getInfoSrv = self.create_service(GripperR2FInfo, 'gripper_get_info', self.get_info_callback)
        self.cmdSrv = self.create_service(GripperR2FCmd, 'gripper_cmd', self.cmd_callback)

    def startup_routine(self):
        self.get_logger().info("Gripper connection is opened")
        time.sleep(1)
        self.gripper.deactivate_gripper()
        self.gripper.process_act_cmd()
        time.sleep(1)
        self.gripper.activate_gripper()
        self.gripper.process_act_cmd()
        time.sleep(2)
        self.gripper.deactivate_gripper()
        self.gripper.process_act_cmd()
        time.sleep(2)
        self.gripper.goto(pos=255)
        self.gripper.process_act_cmd()
        time.sleep(5)
        self.gripper.goto(pos=0)
        self.gripper.process_act_cmd()
        time.sleep(5)

    def close_connection(self):
        self.gripper.shutdown()
        self.get_logger().info("Gripper connection is disconnected")

    def get_info_callback(self, request, response):
        response.is_ready = self.gripper.is_ready
        response.is_reset = self.gripper.is_reset()
        response.is_moving = self.gripper.is_moving()
        response.is_stopped = self.gripper.is_stopped()
        response.object_detected = self.gripper.object_detected()
        response.get_fault_status = self.gripper.get_fault_status()
        response.get_pos = self.gripper.get_pos()
        response.get_req_pos = self.gripper.get_req_pos()
        response.get_current = self.gripper.get_current()
        return response

    def cmd_callback(self, request, response):
        self.gripper.goto(pos=request.pos, vel=request.vel, force=request.force)
        self.gripper.process_act_cmd()
        response.success = True
        response.message = "done"
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        serviceNode = Robitiq2FServiceNode()
        rclpy.spin(serviceNode)
    finally:
        serviceNode.close_connection()
        rclpy.shutdown()


if __name__ == '__main__':
    main()