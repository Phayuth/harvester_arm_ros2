import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from center_traker import Center
from camera_system import Camera, StereoProcess
import pathlib
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import json


class AllPointEstimation(Node):

    def __init__(self):
        super().__init__("image_publisher")
        # stereo
        path = pathlib.Path(__file__).parent.resolve()
        self.stereo = StereoProcess(10, 4, path, loadCalibrated=True)

        # vision
        self.classes = [40, 41]
        self.yoloWeightDir = "/home/yuth/ws_yuthdev/neural_network/datasave/neural_weight/yolov8x-seg.pt"
        self.model = YOLO(self.yoloWeightDir)
        self.centertracker = Center()

        # pub tf
        self.HstereoTobase = None
        self.tb = StaticTransformBroadcaster(self)
        self.cam_to_base_tf(str(path))

        # centroid pub
        self.centpub = self.create_publisher(PointStamped, "/p_cent_3d", 1)
        self.leftpub = self.create_publisher(PointStamped, "/p_left_3d", 1)
        self.rightpub = self.create_publisher(PointStamped, "/p_right_3d", 1)
        self.botpub = self.create_publisher(PointStamped, "/p_bot_3d", 1)
        self.toppub = self.create_publisher(PointStamped, "/p_top_3d", 1)
        self.pointmsg = PointStamped()
        self.pointmsg.header.frame_id = "base"

        # pub loop
        self.timer = self.create_timer(timer_period_sec=0.001, callback=self.timer_callback)

    def cam_to_base_tf(self, path):
        with open(path + "/camera-robot-pose.json", "r") as f:
            self.stereoTobasePose = json.load(f)

        with open(path + "/camera-robot-matrix.json", "r") as ff:
            self.HstereoTobase = json.load(ff)
            self.HstereoTobase = np.array(self.HstereoTobase)

        statictf = TransformStamped()
        statictf.header.stamp = self.get_clock().now().to_msg()
        statictf.header.frame_id = "base"
        statictf.child_frame_id = "stereo_optical"
        statictf.transform.translation.x = self.stereoTobasePose[0]
        statictf.transform.translation.y = self.stereoTobasePose[1]
        statictf.transform.translation.z = self.stereoTobasePose[2]
        statictf.transform.rotation.x = self.stereoTobasePose[3]
        statictf.transform.rotation.y = self.stereoTobasePose[4]
        statictf.transform.rotation.z = self.stereoTobasePose[5]
        statictf.transform.rotation.w = self.stereoTobasePose[6]
        self.tb.sendTransform(statictf)

    def draw_over_image(self, rgb_image, uv):
        draw_cx, draw_cy = uv
        if draw_cx != 0 and draw_cy != 0:
            cv2.circle(rgb_image, (draw_cx, draw_cy), 5, (255, 0, 255), -1)
            cv2.putText(rgb_image, text=f"{draw_cx}, {draw_cy}", org=(draw_cx, draw_cy), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, thickness=1, color=(255, 0, 255))

    def timer_callback(self):
        _, imgrraw = self.stereo.camright.read()
        _, imglraw = self.stereo.camleft.read()

        imglund, imgrund = self.stereo.undistort_image(imglraw, imgrraw)

        leftpoints, rightpoints = self.centertracker.trackedAllPoint(self.model, self.classes, imglund, imgrund)

        self.pointmsg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(leftpoints)):
            if (leftpoints[i] is not None) and (rightpoints[i] is not None):
                point3d = self.stereo.triangulate(np.array(leftpoints[i]), np.array(rightpoints[i]))
                point3d = self.HstereoTobase @ point3d
                self.pointmsg.point = Point(x=point3d[0], y=point3d[1], z=point3d[2])
                if i == 0:
                    self.centpub.publish(self.pointmsg)
                if i == 1:
                    self.leftpub.publish(self.pointmsg)
                if i == 2:
                    self.rightpub.publish(self.pointmsg)
                if i == 3:
                    self.toppub.publish(self.pointmsg)
                if i == 4:
                    self.botpub.publish(self.pointmsg)

        # visualize
        if True:
            cv2.imshow("right", imgrund)
            cv2.imshow("left", imglund)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.stereo.camright.release()
                self.stereo.camleft.release()
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    imagePublisherNode = AllPointEstimation()
    try:
        rclpy.spin(imagePublisherNode)
    except:
        pass
    finally:
        imagePublisherNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
