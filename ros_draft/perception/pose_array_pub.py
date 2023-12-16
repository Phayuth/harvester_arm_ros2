import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header


class CropHarvestingPose(Node):

    def __init__(self):
        super().__init__('crop_harvest_pose_node')
        self.poseArrayPub = self.create_publisher(PoseArray, '/pose_array_pub', 10)
        self.poseArrayTimer = self.create_timer(0.1, self.publish_pose_array)

    def publish_pose_array(self):
        self.point = np.random.random((3, 10))
        if self.point is not None:
            harvestPoseArrayMsg = PoseArray()
            harvestPoseArrayMsg.header = Header()
            harvestPoseArrayMsg.header.stamp = self.get_clock().now().to_msg()
            harvestPoseArrayMsg.header.frame_id = 'base_link'

            for i in range(self.point.shape[1]):
                xGoal = self.point[0, i]
                yGoal = self.point[1, i]
                zGoal = self.point[2, i]
                harvestPose = Pose()
                harvestPose.position.x = xGoal
                harvestPose.position.y = yGoal
                harvestPose.position.z = zGoal
                harvestPose.orientation.x = 0.0
                harvestPose.orientation.y = 0.0
                harvestPose.orientation.z = 0.0
                harvestPose.orientation.w = 1.0
                harvestPoseArrayMsg.poses.append(harvestPose)

            self.poseArrayPub.publish(harvestPoseArrayMsg)


def main(args=None):
    rclpy.init(args=args)
    node = CropHarvestingPose()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
