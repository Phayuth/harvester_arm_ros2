import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class CropHarvestingPose(Node):

    def __init__(self):
        super().__init__('crop_harvest_to_base_node')

        self.callbackGroup = ReentrantCallbackGroup()
        self.goalPoseSubs = self.create_subscription(PoseArray, '/crop_harvest_goal_pose', self.goal_pose_array_callback, 10, callback_group=self.callbackGroup)
        self.auxiPoseSubs = self.create_subscription(PoseArray, '/crop_harvest_app_pose', self.auxi_pose_array_callback, 10, callback_group=self.callbackGroup)

        self.goalPosePub = self.create_publisher(PoseArray, '/crop_harvest_to_base_goal_pose', 10)
        self.auxiPosePub = self.create_publisher(PoseArray, '/crop_harvest_to_base_app_pose', 10)

        self.goalPickedPosePub = self.create_publisher(PoseStamped, '/crop_harvest_goal_picked', 10)
        self.auxiPickedPosePub = self.create_publisher(PoseStamped, '/crop_harvest_aux_picked', 10)

        self.pickedGoalIndex = 10  # we have to score but for now just picked it
        self.picketAuxIndex = self.pickedGoalIndex + 1
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Initialize Ready. --> Start Publishing Harvesting Pose')

    def goal_pose_array_callback(self, msg):
        poses = msg.poses
        num_poses = len(poses)

        poseList = np.zeros((num_poses, 7))
        for i, pose in enumerate(poses):
            poseList[i, 0] = pose.position.x
            poseList[i, 1] = pose.position.y
            poseList[i, 2] = pose.position.z
            poseList[i, 3] = pose.orientation.x
            poseList[i, 4] = pose.orientation.y
            poseList[i, 5] = pose.orientation.z
            poseList[i, 6] = pose.orientation.w

        from_frame_rel = 'camera_link'
        to_frame_rel = 'base_link'

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            tCamToBase = np.array([translation.x, translation.y, translation.z])
            RCamToBase = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
            TCamToBase = self.compose_transformation_matrix(tCamToBase, RCamToBase)

        except:
            pass

        harvestPoseArrayMsg = PoseArray()
        harvestPoseArrayMsg.header = Header()
        harvestPoseArrayMsg.header.stamp = self.get_clock().now().to_msg()
        harvestPoseArrayMsg.header.frame_id = 'base_link'

        for j in range(num_poses):
            tPointToCam = np.array([poseList[j, 0], poseList[j, 1], poseList[j, 2]])
            RPointToCam = np.array([poseList[j, 3], poseList[j, 4], poseList[j, 5], poseList[j, 6]])
            TPointToCam = self.compose_transformation_matrix(tPointToCam, RPointToCam)

            TPointToBase = TCamToBase @ TPointToCam
            tPointToBase, qPointToBase = self.transformation_to_quaternion_and_translation(TPointToBase)

            harvestPose = Pose()
            harvestPose.position.x = tPointToBase[0]
            harvestPose.position.y = tPointToBase[1]
            harvestPose.position.z = tPointToBase[2]
            harvestPose.orientation.x = qPointToBase[0]
            harvestPose.orientation.y = qPointToBase[1]
            harvestPose.orientation.z = qPointToBase[2]
            harvestPose.orientation.w = qPointToBase[3]
            harvestPoseArrayMsg.poses.append(harvestPose)

            if j == self.pickedIndex:
                goalPicked = PoseStamped()
                goalPicked.header = harvestPoseArrayMsg.header
                goalPicked.pose = harvestPose
                self.goalPickedPosePub.publish(goalPicked)

        self.goalPosePub.publish(harvestPoseArrayMsg)

    def auxi_pose_array_callback(self, msg):
        poses = msg.poses
        num_poses = len(poses)

        poseList = np.zeros((num_poses, 7))
        for i, pose in enumerate(poses):
            poseList[i, 0] = pose.position.x
            poseList[i, 1] = pose.position.y
            poseList[i, 2] = pose.position.z
            poseList[i, 3] = pose.orientation.x
            poseList[i, 4] = pose.orientation.y
            poseList[i, 5] = pose.orientation.z
            poseList[i, 6] = pose.orientation.w

        from_frame_rel = 'camera_link'
        to_frame_rel = 'base_link'

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, now)
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            tCamToBase = np.array([translation.x, translation.y, translation.z])
            RCamToBase = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
            TCamToBase = self.compose_transformation_matrix(tCamToBase, RCamToBase)

        except:
            pass

        harvestPoseArrayMsg = PoseArray()
        harvestPoseArrayMsg.header = Header()
        harvestPoseArrayMsg.header.stamp = self.get_clock().now().to_msg()
        harvestPoseArrayMsg.header.frame_id = 'base_link'

        for j in range(num_poses):
            tPointToCam = np.array([poseList[j, 0], poseList[j, 1], poseList[j, 2]])
            RPointToCam = np.array([poseList[j, 3], poseList[j, 4], poseList[j, 5], poseList[j, 6]])
            TPointToCam = self.compose_transformation_matrix(tPointToCam, RPointToCam)

            TPointToBase = TCamToBase @ TPointToCam
            tPointToBase, qPointToBase = self.transformation_to_quaternion_and_translation(TPointToBase)

            harvestPose = Pose()
            harvestPose.position.x = tPointToBase[0]
            harvestPose.position.y = tPointToBase[1]
            harvestPose.position.z = tPointToBase[2]
            harvestPose.orientation.x = qPointToBase[0]
            harvestPose.orientation.y = qPointToBase[1]
            harvestPose.orientation.z = qPointToBase[2]
            harvestPose.orientation.w = qPointToBase[3]
            harvestPoseArrayMsg.poses.append(harvestPose)

            if j == self.pickedIndex:
                auxPicked = PoseStamped()
                auxPicked.header = harvestPoseArrayMsg.header
                auxPicked.pose = harvestPose
                self.auxiPickedPosePub.publish(auxPicked)

        self.auxiPosePub.publish(harvestPoseArrayMsg)

    def compose_transformation_matrix(self, translation, rotation):
        rot = R.from_quat(rotation)
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rot.as_matrix()
        transformation_matrix[:3, 3] = translation
        return transformation_matrix

    def transformation_to_quaternion_and_translation(self, transformation_matrix):
        rotation_matrix = transformation_matrix[:3, :3]
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()
        translation = transformation_matrix[:3, 3]
        return translation, quaternion


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CropHarvestingPose()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
