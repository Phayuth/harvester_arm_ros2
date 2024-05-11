import cv2
import numpy as np
import yaml
import pathlib

np.set_printoptions(suppress=True)


class StereoProcess:

    def __init__(self) -> None:
        # load info
        path = pathlib.Path(__file__).parent.resolve()
        self.camLeftInfo = self.load_camera_info(str(path) + "/stereo_left.yaml")
        self.camRightInfo = self.load_camera_info(str(path) + "/stereo_right.yaml")
        self.T = np.array([0.12706311302032947, 0.0029770309111915097, -0.005154077728906227]).reshape(3, 1)
        self.R = np.array([0.9971029786476852, -0.0019668709532512972, -0.07603802595127855, 0.0017952190287162575, 0.9999956840656479, -0.0023257340165657977, 0.07604227219483246, 0.002182491404366247, 0.9971022061808493]).reshape(3, 3)
        self.size = self.size

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
        # determine recitied rotation matrix and projection matrix for each camera given known real world R and T
        self.camLeftInfo["r"], self.camRightInfo["r"], self.camLeftInfo["p"], self.camRightInfo["p"], Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
            self.camLeftInfo["k"],
            self.camLeftInfo["d"],
            self.camRightInfo["k"],
            self.camRightInfo["d"],
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
            self.camLeftInfo["k"],
            self.camLeftInfo["d"],
            self.camLeftInfo["r"],
            self.camLeftInfo["p"],
            self.size,
            cv2.CV_32FC1,
            None,
            None,
        )
        self.Rightmapx, self.Rightmapy = cv2.initUndistortRectifyMap(
            self.camRightInfo["k"],
            self.camRightInfo["d"],
            self.camRightInfo["r"],
            self.camRightInfo["p"],
            self.size,
            cv2.CV_32FC1,
            None,
            None,
        )

    def remap(self, src, mapx, mapy):
        return cv2.remap(src, mapx, mapy, cv2.INTER_LINEAR)


if __name__ == "__main__":
    con = StereoProcess()
    # camright = cv2.VideoCapture(4)

    # while True:
    #     _, imgraw = camright.read()

    #     imgrect = con.remap(imgraw)

    #     cv2.imshow("img raw", imgraw)
    #     cv2.imshow("img rect", imgrect)
    #     if cv2.waitKey(1) & 0xFF == ord("q"):
    #         break
    # camright.release()
    # cv2.destroyAllWindows()

