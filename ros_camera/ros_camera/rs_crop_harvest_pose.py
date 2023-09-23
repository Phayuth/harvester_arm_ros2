import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger


class CropHarvestingPose(Node):

    def __init__(self):
        super().__init__('crop_harvest_pose_node')
        # name
        self.topicCropPointSub = '/crop_point'
        self.topicHarvestPosePub = '/crop_harvest_pose'
        self.servUpdatePoint = '/trigger_update_harvest_pose'
        self.sensorLinkName = 'camera_link'

        # get point on crop
        self.subscription = self.create_subscription(PoseArray, self.topicCropPointSub, self.pose_array_callback, 10)

        # publish point to camera frame
        self.poseArrayPub = self.create_publisher(PoseArray, self.topicHarvestPosePub, 10)
        self.poseArrayTimer = self.create_timer(0.2, self.publish_pose_array)
        self.triggerUpdatePoint = self.create_service(Trigger, self.servUpdatePoint, self.trigger_update_point)

        self.point = None
        self.harvestPoseArrayMsg = None

        self.get_logger().info(f'\nharvest node initialized. \
                               \ncrop point subscribe on {self.topicCropPointSub}. \
                               \ncrop harvest published on {self.topicHarvestPosePub}.')
        self.get_logger().info(f'\ncall -> ros2 service call {self.servUpdatePoint} std_srvs/srv/Trigger {{}} to update havest pose')

    def pose_array_callback(self, msg):
        poses = msg.poses
        numPoses = len(poses)
        if numPoses != 0:
            xyz_values = np.zeros((numPoses, 3))
            for i, pose in enumerate(poses):
                xyz_values[i, 0] = pose.position.x
                xyz_values[i, 1] = pose.position.y
                xyz_values[i, 2] = pose.position.z

            self.point = xyz_values

    def publish_pose_array(self):
        if self.harvestPoseArrayMsg is not None:
            self.poseArrayPub.publish(self.harvestPoseArrayMsg)
    
    def trigger_update_point(self, request, response):
        if self.point is not None:

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.point)
            pcd.estimate_normals()
            pcd.normalize_normals()
            pcd.orient_normals_towards_camera_location()
            pcdDownSample = pcd.voxel_down_sample(voxel_size=0.01)

            harvestPoint = np.asarray(pcdDownSample.points).T
            normalPointOut = np.asarray(pcdDownSample.normals).T
            auxPoint, normalPointIntoCrop = self.auxilary_pose_offset(harvestPoint, -1 * normalPointOut, distanceOffset=0.1)

            harvestPoseArrayMsg = PoseArray()
            harvestPoseArrayMsg.header = Header()
            harvestPoseArrayMsg.header.stamp = self.get_clock().now().to_msg()
            harvestPoseArrayMsg.header.frame_id = self.sensorLinkName

            for i in range(harvestPoint.shape[1]):
                rM = self.rotation_matrix_align_z_axis(normalPointIntoCrop[:, i])
                r = R.from_matrix(rM)
                rq = r.as_quat().astype(dtype=float)
                harvestPose = Pose()
                harvestPose.position.x = float(harvestPoint[0, i])
                harvestPose.position.y = float(harvestPoint[1, i])
                harvestPose.position.z = float(harvestPoint[2, i])
                harvestPose.orientation.x = rq[0]
                harvestPose.orientation.y = rq[1]
                harvestPose.orientation.z = rq[2]
                harvestPose.orientation.w = rq[3]
                harvestPoseArrayMsg.poses.append(harvestPose)

                auxPose = Pose()
                auxPose.position.x = float(auxPoint[0, i])
                auxPose.position.y = float(auxPoint[1, i])
                auxPose.position.z = float(auxPoint[2, i])
                auxPose.orientation.x = rq[0]
                auxPose.orientation.y = rq[1]
                auxPose.orientation.z = rq[2]
                auxPose.orientation.w = rq[3]
                harvestPoseArrayMsg.poses.append(auxPose)

            self.harvestPoseArrayMsg = harvestPoseArrayMsg
            response.success = True
            response.message = f"New Pose Is Updated. Available on {self.topicHarvestPosePub}"
        else:
            response.success = False
            response.message = "Wait Until Point is available"
        return response
    
    def rotation_matrix_align_z_axis(self, unit_vector):
        z_axis = np.array([0, 0, 1])  # Z-axis
        rotation_axis = np.cross(z_axis, unit_vector)
        rotation_axis /= np.linalg.norm(rotation_axis) + 0.0001  # to avoid devide by 0
        rotation_cos = np.dot(z_axis, unit_vector)
        rotation_sin = np.sqrt(1 - rotation_cos**2)
        rotation_matrix = np.eye(3) + rotation_sin * self.skew_symmetric_matrix(rotation_axis) + (1-rotation_cos) * self.skew_symmetric_matrix(rotation_axis) @ self.skew_symmetric_matrix(rotation_axis)
        return rotation_matrix

    def skew_symmetric_matrix(self, vector):
        matrix = np.array([[0, -vector[2], vector[1]], [vector[2], 0, -vector[0]], [-vector[1], vector[0], 0]])
        return matrix

    def auxilary_pose_offset(self, point, normalPointIntoCrop, distanceOffset):  # assume the normal vector direction is pointing into the crop when pass in
        if point.shape[0] != 3:
            point = point.T
            normalPointIntoCrop = normalPointIntoCrop.T
        auxPoint = distanceOffset * (-1 * normalPointIntoCrop) + point
        return auxPoint, normalPointIntoCrop


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CropHarvestingPose()
        executor = MultiThreadedExecutor()
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
