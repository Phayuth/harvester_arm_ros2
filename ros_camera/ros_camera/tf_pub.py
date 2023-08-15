import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StaticTransformPublisherNode(Node):

    def __init__(self):
        super().__init__('static_transform_publisher_node')
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_camera_to_tool0_static_transform()
        # self.publish_gripper_to_tool0_static_transform()

    def publish_camera_to_tool0_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'tool0'
        static_transform.child_frame_id = 'camera_link'
        static_transform.transform.translation.x = -0.0175  # m
        static_transform.transform.translation.y = -0.1  # m
        static_transform.transform.translation.z =  0.0258  # m
        static_transform.transform.rotation.x = 0.5
        static_transform.transform.rotation.y = -0.5
        static_transform.transform.rotation.z = 0.5
        static_transform.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(static_transform)

    # def publish_gripper_to_tool0_static_transform(self):
    #     static_transform = TransformStamped()
    #     static_transform.header.stamp = self.get_clock().now().to_msg()
    #     static_transform.header.frame_id = 'tool0'
    #     static_transform.child_frame_id = 'gripper_link'
    #     static_transform.transform.translation.x = 0.0
    #     static_transform.transform.translation.y = 0.0
    #     static_transform.transform.translation.z = 0.14  # m
    #     static_transform.transform.rotation.x = 0.0
    #     static_transform.transform.rotation.y = 0.0
    #     static_transform.transform.rotation.z = 0.0
    #     static_transform.transform.rotation.w = 1.0
    #     self.tf_broadcaster.sendTransform(static_transform)


def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
