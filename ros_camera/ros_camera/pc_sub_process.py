import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class PointCloudOrderedReadDataNode(Node):

    def __init__(self):
        super().__init__('pointcloud_info_node')
        self.pointcloudSub = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.point_cloud_pub, 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

    def publish_static_transform(self, point, pointName):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'camera_link'
        static_transform.child_frame_id = pointName
        x = point[0]
        y = point[1]
        z = point[2]
        static_transform.transform.translation.x = float(z)
        static_transform.transform.translation.y = float(-x)
        static_transform.transform.translation.z = float(-y)
        static_transform.transform.rotation.w = 1.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(static_transform)

    def point_cloud_pub(self, msg):
        pixel1_x = 490
        pixel1_y = 234
        pixel2_x = 340
        pixel2_y = 257
        pixel3_x = 261
        pixel3_y = 253

        index1 = (pixel1_y * msg.width) + pixel1_x
        index2 = (pixel2_y * msg.width) + pixel2_x
        index3 = (pixel3_y * msg.width) + pixel3_x

        point_cloud = self.point_cloud2_to_array(msg)
        point1 = point_cloud[index1]
        point2 = point_cloud[index2]
        point3 = point_cloud[index3]

        self.publish_static_transform(point1, "point1tf")
        self.publish_static_transform(point2, "point2tf")
        self.publish_static_transform(point3, "point3tf")
    
    def process_depth_image(self, msg):
        # h = msg.height
        # w = msg.width
        # f = msg.fields
        # ps = msg.point_step
        # rs = msg.row_step
        # d = msg.data
        # print(f"h {h}, w {w}, ps {ps}, rs {rs}, len(d) {len(d)}")
        # pixel_x = 320
        # pixel_y = 240
        # index = (pixel_y * msg.width) + pixel_x

        mask = np.zeros((480, 640))
        mask[100:200, 300:400] = 1
        rows, cols = np.nonzero(self.mask)
        point_cloud = self.point_cloud2_to_array(msg)

        pointFromMask = []
        for row, col in zip(rows, cols):
            index = (row * msg.width) + col
            xyzc = point_cloud[index]
            pointFromMask.append(xyzc)
    
    def point_cloud2_to_array(self, msg):
        # point cloud must be in ordered
        # point cloud have 4 element xyzc
        pointCloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :4]
        return pointCloud


def main(args=None):
    rclpy.init(args=args)
    pointcloudOrderedReadDataNode = PointCloudOrderedReadDataNode()
    rclpy.spin(pointcloudOrderedReadDataNode)
    pointcloudOrderedReadDataNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
