import sys
sys.path.append("/home/yuth/ws_yuthdev/neural_network")
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion

from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt
from projects.yolo_detection.yolo_mask import CVYOLOMask
from grasp_poses.normal_parallel import NormalVectorParallelGraspPose


class CropLocalization(Node):

    def __init__(self):
        super().__init__("crop_detection_node")
        self.cbgr = ReentrantCallbackGroup()
        self.cbgm = MutuallyExclusiveCallbackGroup()

        # detection
        self.yoloWeightDir = "/home/yuth/ws_yuthdev/neural_network/datasave/neural_weight/yolov8x-seg.pt"
        self.cvc = CVYOLOMask(self.yoloWeightDir, interestNames=["apple"])

        # data input
        self.topicImageSub = "/camera/color/image_raw"
        self.topicPointCloudSub = "/camera/depth/color/points"
        self.imageSub = self.create_subscription(Image, self.topicImageSub, self.get_target_detection, 1)
        self.pointCloudSub = self.create_subscription(PointCloud2, self.topicPointCloudSub, self.get_pointcloud, 1, callback_group=self.cbgr)

        # topic for debug
        self.topicArrayPub = "/debug_crop_point"
        self.topicMaskPub = "/debug_crop_detection_mask"
        self.poseArrayPub = self.create_publisher(PoseArray, self.topicArrayPub, 1)
        self.maskPub = self.create_publisher(Image, self.topicMaskPub, 1)

        # get data
        self.topicHarvestPosePub = "/g_n_pg_pose"
        self.gnpgposepub = self.create_publisher(PoseArray, self.topicHarvestPosePub, 1)
        self.poseArrayTimer = self.create_timer(1, self.grasp_pregrasp_poses)

        self.br = CvBridge()

        self.image = None
        self.mask = None
        self.pointcloud = None

    def get_target_detection(self, data:Image):
        imgBGR = self.br.imgmsg_to_cv2(data)
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        indvMask, debugMaskImg = self.cvc.detect_mask(imgRGB, edgeErode=True, drawImg=True)
        if len(indvMask) != 0:
            self.debug_mask(debugMaskImg)
            # self.debug_target_points(indvMask[0])  # only interested in the first mask
            self.mask = indvMask[0]

    def debug_mask(self, maskImg):
        imgBGR = cv2.cvtColor(maskImg, cv2.COLOR_RGB2BGR)
        self.maskPub.publish(self.br.cv2_to_imgmsg(imgBGR, encoding="rgb8"))

    def debug_target_points(self, mask):
        if self.pointcloud is not None:
            posarmsg = PoseArray()
            posarmsg.header.frame_id = "camera_color_optical_frame"
            xyz = self.segment_pointcloud_from_img(mask, self.pointcloud)[:, 0:3]
            for p in xyz:
                pose = Pose()
                # pose.position = Point(x=float(z), y = float(-x), z = float(-y))
                pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                posarmsg.poses.append(pose)
            self.poseArrayPub.publish(posarmsg)

    def grasp_pregrasp_poses(self):
        if (self.pointcloud is not None) and (self.mask is not None):
            posearmsg = PoseArray()
            posearmsg.header.frame_id = "camera_color_optical_frame"
            xyz = self.segment_pointcloud_from_img(self.mask, self.pointcloud)[:, 0:3]
            harvest = NormalVectorParallelGraspPose(xyz, fixedOrientation=False)
            grasps, preGrasps = harvest.get_grasp_and_pregrasp_poses()
            for i in range(len(grasps)):
                # if i in [0, 10]:
                gt, gq = rbt.conv_h_to_t_and_quat(grasps[i])
                graspPoseMsg = Pose()
                graspPoseMsg.position = Point(x=float(gt[0]), y=float(gt[1]), z=float(gt[2]))
                graspPoseMsg.orientation = Quaternion(x=float(gq[0]), y=float(gq[1]), z=float(gq[2]), w=float(gq[3]))
                posearmsg.poses.append(graspPoseMsg)

                pgt, pgq = rbt.conv_h_to_t_and_quat(preGrasps[i])
                preGraspPoseMsg = Pose()
                preGraspPoseMsg.position = Point(x=float(pgt[0]), y=float(pgt[1]), z=float(pgt[2]))
                preGraspPoseMsg.orientation = Quaternion(x=float(pgq[0]), y=float(pgq[1]), z=float(pgq[2]), w=float(pgq[3]))
                posearmsg.poses.append(preGraspPoseMsg)

            self.gnpgposepub.publish(posearmsg)

    def get_pointcloud(self, msg:PointCloud2):
        self.pointcloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :4]

    def segment_pointcloud_from_img(self, mask, pointcloud):
        rows, cols = np.nonzero(mask)
        index = (rows * 640) + cols
        xyzc = pointcloud[index]
        return xyzc


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CropLocalization()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
