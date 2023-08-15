import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from control_msgs.msg import JointTrajectoryControllerState


class DataSubscriberNode(Node):

    def __init__(self):
        super().__init__('data_subscriber_node')
        self.subscription = self.create_subscription(JointTrajectoryControllerState, '/joint_trajectory_controller/state', self.joint_state_callback, 10)
        self.trigger_service = self.create_service(Trigger, 'trigger_data_subscription', self.trigger_callback)
        self.joint = None

    def trigger_callback(self, request, response):
        if request:
            response.success = True
            response.message = str(self.joint)
        else:
            response.success = False
            response.message = "Invalid trigger request."
        return response

    def joint_state_callback(self, msg):
        joint_positions = msg.actual.positions
        self.joint = joint_positions
        self.get_logger().info("Received joint positions: {}".format(joint_positions))


def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
