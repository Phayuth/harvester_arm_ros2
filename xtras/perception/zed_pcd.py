import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import ctypes
import struct
import sensor_msgs_py.point_cloud2 as pc2


class ReadPointCloud(Node):

    def __init__(self):
        super().__init__("pointcloud_info_node")
        self.pointcloudSub = self.create_subscription(PointCloud2, "/zed/zed_node/point_cloud/cloud_registered", self.run, 10)

    def run(self, msg):
        self.pointcloudSub.destroy()
        # xyz, rgb = self.method_sensor_py_with_rgb(msg)
        xyz, rgb = self.method_experimenting(msg)
        self.view_pcd(xyz, rgb)

        pcd = np.hstack((xyz, rgb))
        self.view_pcd(pcd[:,0:3], pcd[:,3:6])

    def method_sensor_py_no_rgb(self, msg):
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_numpy = np.array(list(gen), dtype=np.float32)
        return points_numpy

    def method_numpy_buffer(self, msg):
        # point cloud have 4 element xyzc
        pointCloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, msg.point_step // 4)
        nan_mask = np.isnan(pointCloud)
        nan_rows = np.any(nan_mask, axis=1)
        pcfiltered = pointCloud[~nan_rows]
        return pcfiltered

    def method_sensor_py_with_rgb(self, msg):
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = list(gen)
        xyz = []
        rgb = []
        for x in int_data:
            color = x[3]
            s = struct.pack(">f", color)
            i = struct.unpack(">l", s)[0]
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = pack & 0x000000FF
            # print(x[0], x[1], x[2], r, g, b)
            xyz.append([x[0], x[1], x[2]])
            rgb.append([r, g, b])
        return np.array(xyz), np.array(rgb)

    def method_experimenting(self, msg):
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = list(gen)

        # test
        data = np.array(int_data, dtype=np.float32)
        # color = data[:, 3][0]
        # print(f"> color: {color}")
        # s = struct.pack(">f", color)
        # print(f"> s: {s}")
        # i = struct.unpack(">l", s)
        # print(f"> i: {i}")
        # i = i[0]
        # print(f"> type(i): {type(i)}")
        # print(f"> i: {i}")
        # pack = ctypes.c_uint32(i).value
        # print(f"> pack: {pack}")

        # ccc = data[:, 3][0].tobytes()
        # print(f"> ccc: {ccc}")
        # integer_value = np.frombuffer(ccc, dtype=np.int32)
        # print(f"> integer_value: {integer_value}")
        # integer_value = np.frombuffer(ccc, dtype=np.int32)[0]
        # print(f"> integer_value: {integer_value}")
        # uint32_value = np.uint32(integer_value)
        # print(f"> uint32_value: {uint32_value}")

        colorfloat = data[:, 3].tobytes()
        uint32_val = np.frombuffer(colorfloat, dtype=np.uint32)
        r = (uint32_val & 0x00FF0000) >> 16
        g = (uint32_val & 0x0000FF00) >> 8
        b = uint32_val & 0x000000FF
        return data[:, 0:3], np.vstack((r, g, b)).T

    def view_pcd(self, xyz, rgb=None):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        if rgb is not None:
            rgb = rgb / 255
            pcd.colors = o3d.utility.Vector3dVector(rgb)
        pcd.remove_non_finite_points(remove_nan=True, remove_infinite=True)
        o3d.visualization.draw_geometries([pcd])


def main(args=None):
    rclpy.init(args=args)
    pd = ReadPointCloud()
    rclpy.spin_once(pd)
    pd.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
