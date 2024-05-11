import cv2
import numpy as np

# mono calibration


def do_calibration():
    reproj_err, intrinsics, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(opts, ipts, size, intrinsics_in, None, flags=calib_flags)
    # OpenCV returns more than 8 coefficients (the additional ones all zeros) when CALIB_RATIONAL_MODEL is set.
    # The extra ones include e.g. thin prism coefficients, which we are not interested in.
    if calib_flags & cv2.CALIB_RATIONAL_MODEL:
        distortion = dist_coeffs.flat[:8].reshape(-1, 1)  # rational polynomial
    else:
        distortion = dist_coeffs.flat[:5].reshape(-1, 1)  # plumb bob

    # R is identity matrix for monocular calibration
    R = np.eye(3, dtype=np.float64)
    P = np.zeros((3, 4), dtype=np.float64)

    set_alpha(0.0)


def set_alpha(a):
    """
    Set the alpha value for the calibrated camera solution.  The alpha
    value is a zoom, and ranges from 0 (zoomed in, all pixels in
    calibrated image are valid) to 1 (zoomed out, all pixels in
    original image are in calibrated image).
    """

    # NOTE: Prior to Electric, this code was broken such that we never actually saved the new
    # camera matrix. In effect, this enforced P = [K|0] for monocular cameras.
    # TODO: Verify that OpenCV #1199 gets applied (improved GetOptimalNewCameraMatrix)
    ncm, _ = cv2.getOptimalNewCameraMatrix(intrinsics, distortion, size, a)
    for j in range(3):
        for i in range(3):
            P[j, i] = ncm[j, i]
    mapx, mapy = cv2.initUndistortRectifyMap(intrinsics, distortion, R, ncm, size, cv2.CV_32FC1)


def remap(src):
    """
    :param src: source image
    :type src: :class:`cvMat`

    Apply the post-calibration undistortion to the source image
    """
    return cv2.remap(src, mapx, mapy, cv2.INTER_LINEAR)
