import cv2
import numpy as np
import rclpy
import ultralytics
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from scipy.ndimage import binary_dilation, binary_erosion
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header


class CropLocalization(Node):

    def __init__(self):
        super().__init__('crop_detection_node')
        # name
        self.yoloWeightDir = './weight/yolov8x-seg.pt'
        self.topicImageSub = '/camera/color/image_raw'
        self.topicPointCloudSub = '/camera/depth/color/points'
        self.topicArrayPub = '/crop_point'
        self.topicMaskPub = '/crop_detection'
        self.sensorLinkName = 'camera_link'

        self.detectionModel = ultralytics.YOLO(self.yoloWeightDir)
        self.imageSub = self.create_subscription(Image, self.topicImageSub, self.masked_image, 10)
        self.pointCloudSub = self.create_subscription(PointCloud2, self.topicPointCloudSub, self.get_point_cloud, 10)
        self.poseArrayPub = self.create_publisher(PoseArray, self.topicArrayPub, 10)
        self.maskPub = self.create_publisher(Image, self.topicMaskPub, 10)

        self.br = CvBridge()

        self.image = None
        self.mask = None
        self.pointCloud = None

        self.get_logger().info(f'\nperception initialized. \
                                 \nimage subscribe on {self.topicImageSub}. \
                                 \npoint cloud subscribe on {self.topicPointCloudSub}. \
                                 \nmask image published on {self.topicMaskPub}. \
                                 \ncrop point published on {self.topicArrayPub}.')

    def masked_image(self, data):
        imgBGR = self.br.imgmsg_to_cv2(data)
        imgRGB = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2RGB)
        imgMask = np.zeros((imgRGB.shape[0], imgRGB.shape[1]))
        results = self.detectionModel(imgRGB, stream=True, conf=0.5, verbose=False)
        for r in results:
            boxes = r.boxes
            masks = r.masks
            for bi, box in enumerate(boxes):
                classesNumber = int(box.cls[0].item())
                classesName = self.detectionModel.names[classesNumber]
                if classesName == "apple":
                    appleMask = masks.data[bi, :, :].cpu().numpy()
                    appleMask = appleMask.reshape(appleMask.shape[0], appleMask.shape[1])
                    imgMask = imgMask + appleMask
        self.image = imgRGB
        self.mask = imgMask

        self.pub_mask_image()
        self.publish_pose_array()

    def get_point_cloud(self, msg):
        self.pointCloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :4]

    def pub_mask_image(self):
        if self.image is not None:
            imgshow = np.where(self.mask[..., None], self.image, 0)
            imgBGR = cv2.cvtColor(imgshow, cv2.COLOR_RGB2BGR)
            imgData = self.br.cv2_to_imgmsg(imgBGR, encoding="rgb8")
            self.maskPub.publish(imgData)

    def publish_pose_array(self):
        if self.pointCloud is not None:
            poseArrayMsg = PoseArray()
            poseArrayMsg.header = Header()
            poseArrayMsg.header.stamp = self.get_clock().now().to_msg()
            poseArrayMsg.header.frame_id = self.sensorLinkName

            # no dilation
            # mask = self.mask

            # with dilation
            mask = binary_erosion(self.mask, iterations=10).astype(self.mask.dtype)

            rows, cols = np.nonzero(mask)
            for row, col in zip(rows, cols):
                index = (row*640) + col
                xyzc = self.pointCloud[index]
                x = xyzc[0]
                y = xyzc[1]
                z = xyzc[2]
                pose = Pose()
                pose.position.x = float(z)
                pose.position.y = float(-x)
                pose.position.z = float(-y)
                poseArrayMsg.poses.append(pose)

            self.poseArrayPub.publish(poseArrayMsg)


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


if __name__ == '__main__':
    main()