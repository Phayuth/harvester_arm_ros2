import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_joint1 = self.create_publisher(JointState, '/vs/joint_states', 10)
        self.publisher_joint2 = self.create_publisher(JointState, '/va/joint_states', 10)
        self.publisher_joint3 = self.create_publisher(JointState, '/vg/joint_states', 10)
        self.timer_ = self.create_timer(0.1, self.publish_joint_states)

    def publish_joint_states(self):
        now = self.get_clock().now().to_msg()

        joint_state_msg1 = JointState()
        joint_state_msg1.header.stamp = now
        joint_state_msg1.name = ['vs_shoulder_pan_joint', 'vs_shoulder_lift_joint', 'vs_elbow_joint', 'vs_wrist_1_joint', 'vs_wrist_2_joint', 'vs_wrist_3_joint']
        joint_state_msg1.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        joint_state_msg2 = JointState()
        joint_state_msg2.header.stamp = now
        joint_state_msg2.name = ['va_shoulder_pan_joint', 'va_shoulder_lift_joint', 'va_elbow_joint', 'va_wrist_1_joint', 'va_wrist_2_joint', 'va_wrist_3_joint']
        joint_state_msg2.position = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

        joint_state_msg3 = JointState()
        joint_state_msg3.header.stamp = now
        joint_state_msg3.name = ['vg_shoulder_pan_joint', 'vg_shoulder_lift_joint', 'vg_elbow_joint', 'vg_wrist_1_joint', 'vg_wrist_2_joint', 'vg_wrist_3_joint']
        joint_state_msg3.position = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]

        self.publisher_joint1.publish(joint_state_msg1)
        self.publisher_joint2.publish(joint_state_msg2)
        self.publisher_joint3.publish(joint_state_msg3)


def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
