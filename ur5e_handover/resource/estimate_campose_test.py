import cv2
import numpy as np
import sys

sys.path.append("/home/yuth/ws_yuthdev/neural_network")
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt


class Camera:

    def __init__(self, id) -> None:
        self.capt = cv2.VideoCapture(id)
        self.cameraMatrix = np.array([598.460339, 0.000000, 317.880979, 0.000000, 597.424060, 233.262422, 0.000000, 0.000000, 1.000000]).reshape(3, 3)
        self.distCoeffs = np.array([0.142729, -0.282139, -0.005699, -0.012027, 0.000000])

    # def releasecam(self):
    #     self.capt.release()

    # def readcam(self):
    #     _, imgraw = self.read()
    #     return imgraw


class ARUCOBoardPose:
    # https://docs.opencv.org/4.9.0/db/da9/tutorial_aruco_board_detection.html
    def __init__(self) -> None:
        # detection
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.board = cv2.aruco.GridBoard((5, 7), 0.0275, 0.006875, self.dictionary, None)
        self.detectorParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detectorParams)

    def run(self, camera: Camera, imageUndst):
        corners, ids, rej = self.detector.detectMarkers(imageUndst)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(imageUndst, corners, ids)  # aruco corner

            objPoints, imgPoints = self.board.matchImagePoints(corners, ids, None, None)

            retval, rvc, tvc = cv2.solvePnP(objPoints, imgPoints, camera.cameraMatrix, camera.distCoeffs, None, None, False)
            R, _ = cv2.Rodrigues(rvc)

            if objPoints is not None:
                cv2.drawFrameAxes(imageUndst, camera.cameraMatrix, camera.distCoeffs, rvc, tvc, 0.1, 3)

            return tvc, R


if __name__ == "__main__":
    np.set_printoptions(suppress=True)
    con = ARUCOBoardPose()

    camright = Camera(4)
    camleft = Camera(10)

    HboardToCamRight = None
    HboardToCamLeft = None
    while True:
        _, imgrraw = camright.capt.read()
        _, imglraw = camleft.capt.read()

        if (res := con.run(camright, imgrraw)) is not None:
            tvcr, Rr = res
            HboardToCamRight = rbt.conv_rotmat_and_t_to_h(Rr, tvcr.flatten())
        if (res := con.run(camleft, imglraw)) is not None:
            tvcl, Rl = res
            HboardToCamLeft = rbt.conv_rotmat_and_t_to_h(Rl, tvcl.flatten())
        cv2.imshow("right", imgrraw)
        cv2.imshow("left", imglraw)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    camright.capt.release()
    camleft.capt.release()
    cv2.destroyAllWindows()

    # plot
    HCamRightToBoard = rbt.hinverse(HboardToCamRight)
    HCamLeftToBoard = rbt.hinverse(HboardToCamLeft)
    HCamRightToCamLeft = HboardToCamLeft @ HCamRightToBoard

    import matplotlib.pyplot as plt
    import pytransform3d.camera as pc
    import pytransform3d.transformations as pt


    # cLsensorsize, cLK, cLR, cLP = load_param(pathLeft)
    # cRsensorsize, cRK, cRR, cRP = load_param(pathRight)

    virtual_image_distance = 1

    # origin
    ax = pt.plot_transform(name="board")
    pt.plot_transform(ax, HCamRightToBoard, name="camera_right")
    pt.plot_transform(ax, HCamLeftToBoard, name="camera_left")
    # pc.plot_camera(ax, cam2world=cam1, M=cLK, sensor_size=cLsensorsize, virtual_image_distance=virtual_image_distance)
    # pc.plot_camera(ax, cam2world=cam2, M=cRK, sensor_size=cRsensorsize, virtual_image_distance=virtual_image_distance)
    plt.show()
