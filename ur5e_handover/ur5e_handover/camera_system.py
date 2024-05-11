import cv2
import numpy as np
import yaml
import pathlib
import sys
import matplotlib.pyplot as plt
import pytransform3d.camera as pc
import pytransform3d.transformations as pt

np.set_printoptions(suppress=True)
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")
from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt
from simplearuco import ARUCOBoardPose


class Camera:

    def __init__(self, id, infopath) -> None:
        self.capt = cv2.VideoCapture(id)
        self.info = self.load_camera_info(infopath)

    def load_camera_info(self, yamlPath):
        try:
            with open(yamlPath, "r") as f:
                p = yaml.load(f, yaml.FullLoader)
            height = p["image_height"]
            width = p["image_width"]
            distortion_model = p["distortion_model"]
            d = p["distortion_coefficients"]["data"]
            k = p["camera_matrix"]["data"]
            r = p["rectification_matrix"]["data"]
            p = p["projection_matrix"]["data"]

        except:
            "No calibration file is provide"

        info = {
            "height": height,
            "width": width,
            "distm": distortion_model,
            "d": np.array(d),
            "k": np.array(k).reshape(3, 3),
            "r": np.array(r).reshape(3, 3),
            "p": np.array(p).reshape(3, 4),
        }

        return info

    def save_camera_info(self, yamlPath):
        data = {
            "image_width": self.info["width"],
            "image_height": self.info["height"],
            "camera_name": "narrow_stereo",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": self.info["k"].flatten().tolist(),
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": 5,
                "data": self.info["d"].tolist(),
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": self.info["r"].flatten().tolist(),
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": self.info["p"].flatten().tolist(),
            },
        }

        with open(yamlPath, "w") as file:
            yaml.dump(data, file)

    def read(self):
        return self.capt.read()

    def release(self):
        self.capt.release()


class StereoProcess:

    def __init__(self, camleftid, camrightid, path, loadCalibrated=False) -> None:
        # load camera
        if loadCalibrated:
            leftfile = str(path) + "/streo_left.yaml"
            rightfile = str(path) + "/streo_right.yaml"
        else:
            leftfile = str(path) + "/left.yaml"
            rightfile = str(path) + "/right.yaml"
        self.camleft = Camera(camleftid, leftfile)
        self.camright = Camera(camrightid, rightfile)
        self.size = (self.camleft.info["width"], self.camleft.info["height"])

        if not loadCalibrated:
            # aruco asist
            self.aru = ARUCOBoardPose()

            # stereo type calibration
            # self.stereo_simple_baseline()
            self.stereo_complex_baseline()

            # save stereo calibration
            self.camleft.save_camera_info(str(path) + "/streo_left.yaml")
            self.camright.save_camera_info(str(path) + "/streo_right.yaml")

    def determine_camera_transform(self):
        self.HboardToCamRight = None
        self.HboardToCamLeft = None
        while True:
            _, imgrraw = self.camright.read()
            _, imglraw = self.camleft.read()
            if (res := self.aru.run(self.camright, imgrraw)) is not None:
                tvcr, Rr = res
                self.HboardToCamRight = rbt.conv_rotmat_and_t_to_h(Rr, tvcr.flatten())
            if (res := self.aru.run(self.camleft, imglraw)) is not None:
                tvcl, Rl = res
                self.HboardToCamLeft = rbt.conv_rotmat_and_t_to_h(Rl, tvcl.flatten())
            cv2.imshow("right", imgrraw)
            cv2.imshow("left", imglraw)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        cv2.destroyAllWindows()

        self.HCamRightToBoard = rbt.hinverse(self.HboardToCamRight)
        self.HCamLeftToBoard = rbt.hinverse(self.HboardToCamLeft)
        self.HCamRightToCamLeft = self.HboardToCamLeft @ self.HCamRightToBoard
        self.HCamLeftToCamRight = rbt.hinverse(self.HCamRightToCamLeft)

        # plot
        ax = pt.plot_transform(name="board")
        pt.plot_transform(ax, self.HCamRightToBoard, name="camera_right")
        pt.plot_transform(ax, self.HCamLeftToBoard, name="camera_left")
        plt.show()

    def stereo_simple_baseline(self, a=0):
        """
        Used when the camera baseline is near each other and relatively parallel view point which is more difficult to compute.
        """
        # relative camera pose
        self.determine_camera_transform()
        self.T = self.HCamLeftToCamRight[:3, 3].reshape(3, 1)
        self.R = self.HCamLeftToCamRight[:3, :3]

        # Determine recitied rotation matrix and projection matrix for each camera given known real world R and T
        self.camleft.info["r"], self.camright.info["r"], self.camleft.info["p"], self.camright.info["p"], Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
            self.camleft.info["k"],
            self.camleft.info["d"],
            self.camright.info["k"],
            self.camright.info["d"],
            self.size,
            self.R,
            self.T,
            None,
            None,
            None,
            None,
            None,
            alpha=a,
        )

        # Computes the undistortion and rectification transformation map
        self.leftmapx, self.leftmapy = cv2.initUndistortRectifyMap(
            self.camleft.info["k"],
            self.camleft.info["d"],
            self.camleft.info["r"],
            self.camleft.info["p"],
            self.size,
            cv2.CV_32FC1,
            None,
            None,
        )
        self.Rightmapx, self.Rightmapy = cv2.initUndistortRectifyMap(
            self.camright.info["k"],
            self.camright.info["d"],
            self.camright.info["r"],
            self.camright.info["p"],
            self.size,
            cv2.CV_32FC1,
            None,
            None,
        )

    def stereo_complex_baseline(self):
        """
        Used when the camera baseline is complex, large and with rotation
        """
        # relative camera pose
        self.determine_camera_transform()

        RT1 = np.concatenate([np.eye(3), [[0], [0], [0]]], axis=-1)  # RT matrix for C1 is identity.
        RT2 = self.HCamLeftToCamRight[0:3, :]  # RT matrix for C2 is the R and T obtained from stereo calibration.
        self.camleft.info["p"] = self.camleft.info["k"] @ RT1  # projection matrix for C1
        self.camright.info["p"] = self.camright.info["k"] @ RT2  # projection matrix for C2

    def remap(self, src, mapx, mapy):
        return cv2.remap(src, mapx, mapy, cv2.INTER_LINEAR)

    def triangulate(self, point1, point2):
        if point1.dtype != "float64":
            point1 = point1.astype(np.float64)

        if point2.dtype != "float64":
            point2 = point2.astype(np.float64)

        point3d = cv2.triangulatePoints(self.camleft.info["p"], self.camright.info["p"], point1.reshape(2, -1), point2.reshape(2, -1), None).flatten()
        point3d /= point3d[-1]
        return point3d

    def undistort_image(self, imglraw, imgrraw):
        imglund = cv2.undistort(imglraw, self.camleft.info["k"], self.camleft.info["d"])
        imgrund = cv2.undistort(imgrraw, self.camright.info["k"], self.camright.info["d"])
        # opt_cam_mat, valid_roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, img.shape[:2][::-1], 0)
        # ud_img = cv2.undistort(img, camera_matrix, dist_coefs, None, opt_cam_mat)
        return imglund, imgrund


if __name__ == "__main__":
    from ultralytics import YOLO
    from center_traker import Center

    def centroid_only():
        path = pathlib.Path(__file__).parent.resolve()
        stereo = StereoProcess(4, 10, path, loadCalibrated=False)
        # simaru = SimpleARUCO()

        classes = [40, 41]
        yoloWeightDir = "/home/yuth/ws_yuthdev/neural_network/datasave/neural_weight/yolov8x-seg.pt"
        model = YOLO(yoloWeightDir)

        center = Center()

        while True:
            _, imgrraw = stereo.camright.read()
            _, imglraw = stereo.camleft.read()

            # centerRight = simaru.aruco_pixels(imgrraw)
            # centerLeft = simaru.aruco_pixels(imglraw)
            imglund, imgrund = stereo.undistort_image(imglraw, imgrraw)

            mean_left, mean_right = center.trackedCenterShow(model, classes, imglund, imgrund)
            # mean_left = (432,452)
            # mean_right = (284,467)
            if (mean_left is not None) and (mean_right is not None):
                # only consider the first
                point3d = stereo.triangulate(np.array(mean_left), np.array(mean_right))
                print(f"> point3d: {point3d}")

            cv2.imshow("right", imgrund)
            cv2.imshow("left", imglund)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        stereo.camright.release()
        stereo.camleft.release()
        cv2.destroyAllWindows()

    # centroid_only()


    def allpoint():
        path = pathlib.Path(__file__).parent.resolve()
        stereo = StereoProcess(4, 10, path, loadCalibrated=False)
        # simaru = SimpleARUCO()

        classes = [40, 41]
        yoloWeightDir = "/home/yuth/ws_yuthdev/neural_network/datasave/neural_weight/yolov8x-seg.pt"
        model = YOLO(yoloWeightDir)

        center = Center()

        while True:
            _, imgrraw = stereo.camright.read()
            _, imglraw = stereo.camleft.read()

            # centerRight = simaru.aruco_pixels(imgrraw)
            # centerLeft = simaru.aruco_pixels(imglraw)
            imglund, imgrund = stereo.undistort_image(imglraw, imgrraw)

            leftpoints, rightpoints = center.trackedAllPoint(model, classes, imglund, imgrund)

            for i in range(len(leftpoints)):
                if (leftpoints[i] is not None) and (rightpoints[i] is not None):
                    point3d = stereo.triangulate(np.array(leftpoints[i]), np.array(rightpoints[i]))
                    print(f"> point3d: {point3d}")

            cv2.imshow("right", imgrund)
            cv2.imshow("left", imglund)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        stereo.camright.release()
        stereo.camleft.release()
        cv2.destroyAllWindows()

    allpoint()