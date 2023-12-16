import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


class PoseArraySubscriberNode(Node):

    def __init__(self):
        super().__init__('pose_array_subscriber_node')
        self.subscription = self.create_subscription(PoseArray, '/crop_pose_array', self.pose_array_callback, 10)

    def pose_array_callback(self, msg):
        poses = msg.poses
        num_poses = len(poses)

        xyz_values = np.zeros((num_poses, 3))
        for i, pose in enumerate(poses):
            xyz_values[i, 0] = pose.position.x
            xyz_values[i, 1] = pose.position.y
            xyz_values[i, 2] = pose.position.z

        np.save('pose_array.npy', xyz_values)
        print("Done")

def main(args=None):
    rclpy.init(args=args)
    node = PoseArraySubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
