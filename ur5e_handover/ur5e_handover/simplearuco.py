import cv2
import numpy as np


class SimpleARUCO:

    def __init__(self) -> None:
        self.markerLength = 0.120  # m
        self.detectorParams = cv2.aruco.DetectorParameters()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detectorParams)

    def aruco_pixels(self, imageRectified):
        corners, ids, rej = self.detector.detectMarkers(imageRectified)
        centerx = None
        centery = None
        if not ids is None:
            cv2.aruco.drawDetectedMarkers(imageRectified, corners, ids)  # aruco corner
            for i in range(len(ids)):
                centerx = (corners[i][0][0][0] + corners[i][0][2][0]) / 2
                centery = (corners[i][0][0][1] + corners[i][0][2][1]) / 2
                cv2.circle(imageRectified, (int(centerx), int(centery)), 15, (200, 2, 1), 3)

        return (centerx, centery)


class ARUCOBoardPose:
    # https://docs.opencv.org/4.9.0/db/da9/tutorial_aruco_board_detection.html
    def __init__(self) -> None:
        # detection
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.board = cv2.aruco.GridBoard((5, 7), 0.0275, 0.006875, self.dictionary, None)
        self.detectorParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detectorParams)

    def run(self, camera, imgraw):
        corners, ids, rej = self.detector.detectMarkers(imgraw)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(imgraw, corners, ids)  # aruco corner

            objPoints, imgPoints = self.board.matchImagePoints(corners, ids, None, None)

            retval, rvc, tvc = cv2.solvePnP(objPoints, imgPoints, camera.info["k"], camera.info["d"], None, None, False)
            R, _ = cv2.Rodrigues(rvc)

            if objPoints is not None:
                cv2.drawFrameAxes(imgraw, camera.info["k"], camera.info["d"], rvc, tvc, 0.1, 3)

            return tvc, R


class ARUCOGenerate:

    def __init__(self) -> None:
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        size = (5, 7)
        markerLength = 0.04
        markerSeparation = 0.01
        self.board = cv2.aruco.GridBoard(size, markerLength, markerSeparation, self.dictionary, None)
        image = self.board.generateImage(outSize=(400, 500), marginSize=10, borderBits=1)
        import matplotlib.pyplot as plt

        plt.imshow(image, cmap="gray")
        plt.show()


if __name__ == "__main__":
    from camera_system import Camera
    import pathlib

    path = pathlib.Path(__file__).parent.resolve()
    camleft = Camera(10, str(path) + "/left.yaml")

    board = ARUCOBoardPose()

    while True:
        _, imglraw = camleft.read()
        board.run(camleft, imglraw)
        cv2.imshow("img raw", imglraw)
        # cv2.imshow("img rect", imgrect)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    camleft.release()
    cv2.destroyAllWindows()

    # a = ARUCOGenerate()
