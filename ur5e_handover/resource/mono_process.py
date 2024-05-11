import cv2
import numpy as np
import yaml
import pathlib


class MonoProcess:

    def __init__(self) -> None:
        # load info
        path = pathlib.Path(__file__).parent.resolve()
        self.camInfo = self.load_camera_info(str(path) + "/mono_cal.yaml")
        self.P = np.zeros((3, 4), dtype=np.float64)
        self.R = np.eye(3, dtype=np.float64)
        self.size = (self.camInfo["width"], self.camInfo["height"])

        # compute joined undistort and rectified image
        self.set_alpha()

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

        camerainfo = {
            "height": height,
            "width": width,
            "distm": distortion_model,
            "d": np.array(d),
            "k": np.array(k).reshape(3, 3),
            "r": np.array(r).reshape(3, 3),
            "p": np.array(p).reshape(3, 4),
        }

        return camerainfo

    def set_alpha(self, a=0):
        ncm, _ = cv2.getOptimalNewCameraMatrix(self.camInfo["k"], self.camInfo["d"], self.size, a)
        for j in range(3):
            for i in range(3):
                self.P[j, i] = ncm[j, i]

        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.camInfo["k"], self.camInfo["d"], self.R, ncm, self.size, cv2.CV_32FC1)

    def remap(self, src):
        """
        Apply the post-calibration undistortion to the source image
        """
        return cv2.remap(src, self.mapx, self.mapy, cv2.INTER_LINEAR)


if __name__ == "__main__":
    con = MonoProcess()
    camright = cv2.VideoCapture(4)

    while True:
        _, imgraw = camright.read()

        imgrect = con.remap(imgraw)

        cv2.imshow("img raw", imgraw)
        cv2.imshow("img rect", imgrect)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    camright.release()
    cv2.destroyAllWindows()
