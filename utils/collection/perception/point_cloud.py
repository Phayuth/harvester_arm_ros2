import numpy as np
import rclpy
import std_msgs.msg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudNoneOrderedPublisher(Node):

    def __init__(self):
        super().__init__("point_cloud_publisher")
        self.pointCloudPub = self.create_publisher(PointCloud2, "point_cloud_topic", 10)
        self.pointCloudTimer = self.create_timer(1.0, self.publish_point_cloud)

    def publish_point_cloud(self):
        points = np.array([[0.1, 0.1, 0.0, 150, 10, 110], [0.2, 0.2, 0.0, 105, 255, 92], [0.3, 0.3, 0.0, 199, 214, 85]]).astype(np.float32)  # xyzRGB  # unordered, to publish the ordered, we must fill the blank element with data

        msg = PointCloud2()

        msg.header = std_msgs.msg.Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.height = points.shape[0]
        msg.width = 1  # points.shape[1]

        byteSizePerPoint = points[0, 0].itemsize
        numberOfInfoPerPoint = len(points[0])  # xyzrgb = 6

        msg.fields.append(PointField(name="x", offset=0 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="y", offset=1 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="z", offset=2 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="r", offset=3 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="g", offset=4 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name="b", offset=5 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))

        msg.is_bigendian = False
        msg.point_step = byteSizePerPoint * numberOfInfoPerPoint
        msg.row_step = byteSizePerPoint * numberOfInfoPerPoint * points.shape[1]
        msg.data = points.tobytes()

        msg.is_dense = False

        self.pointCloudPub.publish(msg)


class PointCloudOrderedReadDataNode(Node):

    def __init__(self):
        super().__init__("pointcloud_info_node")
        self.pointcloudSub = self.create_subscription(PointCloud2, "/camera/depth/color/points", self.point_cloud_pub, 10)

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

        pc = self.point_cloud2_to_array(msg)
        point1 = pc[index1]
        point2 = pc[index2]
        point3 = pc[index3]

    def point_cloud2_to_array(self, msg):
        # point cloud must be in ordered
        # point cloud have 4 element xyzc
        pointCloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)[:, :4]
        return pointCloud


def main_pub(args=None):
    rclpy.init(args=args)
    pointCloudNoneOrderedNode = PointCloudNoneOrderedPublisher()
    rclpy.spin(pointCloudNoneOrderedNode)
    pointCloudNoneOrderedNode.destroy_node()
    rclpy.shutdown()


def main_sub(args=None):
    rclpy.init(args=args)
    pointcloudOrderedReadDataNode = PointCloudOrderedReadDataNode()
    rclpy.spin(pointcloudOrderedReadDataNode)
    pointcloudOrderedReadDataNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main_pub()
    main_sub()

# https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
