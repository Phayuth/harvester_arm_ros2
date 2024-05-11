import numpy as np
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState


class JointValueRecord(Node):

    def __init__(self):
        super().__init__('joint_value_record')
        self.get_logger().info("Start Recoding...")
        self.jcontrolsub = self.create_subscription(JointTrajectoryControllerState, '/joint_trajectory_controller/state', self.jccb, 10)
        self.jstatesub = self.create_subscription(JointState, '/joint_states', self.jscb, 1)

        self.qname = []
        self.q = []

    def jccb(self, msg:JointTrajectoryControllerState):
        qn = msg.joint_names
        q = msg.actual.positions
        self.qname.append(qn)
        self.q.append(q)

    def jscb(self, msg:JointState):
        qn = msg.name
        q = msg.position
        self.qname.append(qn)
        self.q.append(q)

    def display(self):
        jointNP = np.array(self.q)
        print(f"> joint name: {self.qname}")
        print(f"> jointNP.shape: {jointNP.shape}")
        print(f"> jointNP: {jointNP}")


def main(args=None):
    rclpy.init(args=args)
    jointRecorder = JointValueRecord()
    try:
        rclpy.spin(jointRecorder)
    except KeyboardInterrupt:
        jointRecorder.display()
        jointRecorder.get_logger().info("KeyboardInterrupt received, shutting down...")
        jointRecorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
