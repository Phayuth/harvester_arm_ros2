import numpy as np
import rclpy
import std_msgs.msg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudNoneOrderedPublisher(Node):

    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.pointCloudPub = self.create_publisher(PointCloud2, 'point_cloud_topic', 10)
        self.pointCloudTimer = self.create_timer(1.0, self.publish_point_cloud)

    def publish_point_cloud(self):
        points = np.array([
            [0.1, 0.1, 0.0, 150, 10, 110],  # xyzRGB
            [0.2, 0.2, 0.0, 105, 255, 92],
            [0.3, 0.3, 0.0, 199, 214, 85]
        ]).astype(np.float32)  # unordered, to publish the ordered, we must fill the blank element with data

        msg = PointCloud2()

        msg.header = std_msgs.msg.Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.height = points.shape[0]
        msg.width = 1  # points.shape[1]

        byteSizePerPoint = points[0, 0].itemsize
        numberOfInfoPerPoint = len(points[0])  # xyzrgb = 6

        msg.fields.append(PointField(name='x', offset=0 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=1 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=2 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='r', offset=3 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='g', offset=4 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='b', offset=5 * byteSizePerPoint, datatype=PointField.FLOAT32, count=1))

        msg.is_bigendian = False
        msg.point_step = byteSizePerPoint * numberOfInfoPerPoint
        msg.row_step = byteSizePerPoint * numberOfInfoPerPoint * points.shape[1]
        msg.data = points.tobytes()

        msg.is_dense = False

        self.pointCloudPub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pointCloudNoneOrderedNode = PointCloudNoneOrderedPublisher()
    rclpy.spin(pointCloudNoneOrderedNode)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
