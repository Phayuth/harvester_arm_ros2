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

        # get point on crop
        self.subscription = self.create_subscription(PoseArray, '/crop_pose_array', self.pose_array_callback, 10)

        # publish point to camera frame
        self.poseArrayPub = self.create_publisher(PoseArray, '/crop_harvest_goal_pose', 10)
        self.poseArrayAppPub = self.create_publisher(PoseArray, '/crop_harvest_app_pose', 10)
        self.poseArrayTimer = self.create_timer(0.2, self.publish_pose_array)
        self.triggerUpdatePoint = self.create_service(Trigger, 'trigger_update_harvest_point', self.trigger_update_point)

        self.point = None
        self.harvestPoseArrayMsg = None
        self.auxPoseArrayAppMsg = None

        self.get_logger().info('Initialize Ready')

    def pose_array_callback(self, msg):
        poses = msg.poses
        num_poses = len(poses)
        if num_poses != 0:
            xyz_values = np.zeros((num_poses, 3))
            for i, pose in enumerate(poses):
                xyz_values[i, 0] = pose.position.x
                xyz_values[i, 1] = pose.position.y
                xyz_values[i, 2] = pose.position.z

            self.point = xyz_values

    def publish_pose_array(self):
        if self.harvestPoseArrayMsg is not None and self.auxPoseArrayAppMsg is not None:
            self.poseArrayPub.publish(self.harvestPoseArrayMsg)
            self.poseArrayAppPub.publish(self.auxPoseArrayAppMsg)
    
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
            auxPoint, normalPointIntoCrop = self.auxilary_pose_offset(harvestPoint, normalPointOut * -1, distanceOffset=0.1)

            harvestPoseArrayMsg = PoseArray()
            harvestPoseArrayMsg.header = Header()
            harvestPoseArrayMsg.header.stamp = self.get_clock().now().to_msg()
            harvestPoseArrayMsg.header.frame_id = 'camera_link'

            auxPoseArrayAppMsg = PoseArray()
            auxPoseArrayAppMsg.header = Header()
            auxPoseArrayAppMsg.header.stamp = self.get_clock().now().to_msg()
            auxPoseArrayAppMsg.header.frame_id = 'camera_link'

            for i in range(harvestPoint.shape[1]):
                xGoal = harvestPoint[0, i]
                yGoal = harvestPoint[1, i]
                zGoal = harvestPoint[2, i]
                rM = self.rotation_matrix_align_z_axis(normalPointIntoCrop[:, i])
                r = R.from_matrix(rM)
                rq = r.as_quat().astype(dtype=float)
                harvestPose = Pose()
                harvestPose.position.x = float(xGoal)
                harvestPose.position.y = float(yGoal)
                harvestPose.position.z = float(zGoal)
                harvestPose.orientation.x = rq[0]
                harvestPose.orientation.y = rq[1]
                harvestPose.orientation.z = rq[2]
                harvestPose.orientation.w = rq[3]
                harvestPoseArrayMsg.poses.append(harvestPose)

                xAux = auxPoint[0, i]
                yAux = auxPoint[1, i]
                zAux = auxPoint[2, i]
                auxPose = Pose()
                auxPose.position.x = float(xAux)
                auxPose.position.y = float(yAux)
                auxPose.position.z = float(zAux)
                auxPose.orientation.x = rq[0]
                auxPose.orientation.y = rq[1]
                auxPose.orientation.z = rq[2]
                auxPose.orientation.w = rq[3]
                auxPoseArrayAppMsg.poses.append(auxPose)

            self.harvestPoseArrayMsg = harvestPoseArrayMsg
            self.auxPoseArrayAppMsg = auxPoseArrayAppMsg
            response.success = True
            response.message = "New Pose Is Updated"
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
