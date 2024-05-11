import sys
sys.path.append("/home/yuth/ws_yuthdev/neural_network")
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import numpy as np
import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

from projects.yolo_detection.yolo_centroid import CVYOLOCentroid
from grasp_poses.gp_single_fixed_orientation import SingleFixedOrientationGraspPose
from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt


class PerceptionNode(Node):

    def __init__(self) -> None:
        super().__init__("perception_node")
        # Image
        self.yoloWeightDir = "/home/yuth/ws_yuthdev/neural_network/datasave/neural_weight/yolov8x-seg.pt"
        self.topicImageSub = "/camera/color/image_raw"
        self.topicPointCloudSub = "/camera/depth/color/points"
        self.topicMaskPub = "/crop_detection"
        self.topicGraspPub = "/grasp_pose"
        self.sensorLinkName = "camera_link"

        self.imageSub = self.create_subscription(Image, self.topicImageSub, self.get_masked_image, 3)
        self.pointCloudSub = self.create_subscription(PointCloud2, self.topicPointCloudSub, self.get_point_cloud, 3)
        self.maskPub = self.create_publisher(Image, self.topicMaskPub, 3)
        self.graspPub = self.create_publisher(PoseArray, self.topicGraspPub, 3)

        self.br = CvBridge()
        self.pointCloud = None
        self.cvc = CVYOLOCentroid(self.yoloWeightDir, interestNames=["apple"])

        self.get_logger().info(f"Perception initialized.")

    def get_masked_image(self, msg:Image):
        imgBGR = self.br.imgmsg_to_cv2(msg)
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        indvCentroids = self.cvc.detect_centroid(imgRGB, edgeErode=True, drawImg=True)
        self.publish_mask_image(imgRGB)
        self.publish_grasp_pose(indvCentroids)

    def get_point_cloud(self, msg:PointCloud2):
        self.pointCloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :4]

    def publish_mask_image(self, image):
        imgBGR = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.maskPub.publish(self.br.cv2_to_imgmsg(imgBGR, encoding="rgb8"))

    def publish_grasp_pose(self, centroid):
        if self.pointCloud is not None:
            poseArrayMsg = PoseArray()
            poseArrayMsg.header.stamp = self.get_clock().now().to_msg()
            poseArrayMsg.header.frame_id = self.sensorLinkName
            if centroid is not None:
                col, row = centroid[0]
                index = (row * 640) + col
                xyzc = self.pointCloud[index]
                xyzcRotated = np.array([xyzc[2], -xyzc[0], -xyzc[1]])

                grasp, preGrasp = SingleFixedOrientationGraspPose(xyzcRotated, distanceOffset=0.1).get_grasp_and_pregrasp_poses()

                gt, gq = rbt.conv_h_to_t_and_quat(grasp)
                graspPoseMsg = Pose()
                graspPoseMsg.position = Point(x=float(gt[0]), y=float(gt[1]), z=float(gt[2]))
                graspPoseMsg.orientation = Quaternion(x=float(gq[0]), y=float(gq[1]), z=float(gq[2]), w=float(gq[3]))
                poseArrayMsg.poses.append(graspPoseMsg)

                pgt, pgq = rbt.conv_h_to_t_and_quat(preGrasp)
                preGraspPoseMsg = Pose()
                preGraspPoseMsg.position = Point(x=float(pgt[0]), y=float(pgt[1]), z=float(pgt[2]))
                preGraspPoseMsg.orientation = Quaternion(x=float(pgq[0]), y=float(pgq[1]), z=float(pgq[2]), w=float(pgq[3]))
                poseArrayMsg.poses.append(preGraspPoseMsg)

                self.graspPub.publish(poseArrayMsg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PerceptionNode()
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
