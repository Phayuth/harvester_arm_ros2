import cv2
import numpy as np


def do_calibration():
    # provide board calibration
    # give it to `stereocalibrate function` -> we get output:
    # left intrinsic k , distortion d
    # right intrinsic k, distortion d
    # R rotation, T translation, from camera 2 to camera 1, (real world value rad & mm or m depend on what size we give in during calibration)
    T = np.zeros((3, 1), dtype=np.float64)
    R = np.eye(3, dtype=np.float64)
    flags = cv2.CALIB_FIX_INTRINSIC

    retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(opts, lipts, ripts, l.intrinsics, l.distortion, r.intrinsics, r.distortion, size, R, T, criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1, 1e-5), flags=flags)

    set_alpha(0.0)


def set_alpha(a=0):
    # Set the alpha value for the calibrated camera solution. The
    # alpha value is a zoom, and ranges from 0 (zoomed in, all pixels
    # in calibrated image are valid) to 1 (zoomed out, all pixels in
    # original image are in calibrated image).

    # calculate new projection and transformation from a rectified image
    # we get output:
    # R1, R2 # rectified rotation matrix
    # P1, P2 # projection matrix
    # R, T is input
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(l.intrinsics, l.distortion, r.intrinsics, r.distortion, size, R, T, l.R, r.R, l.P, r.P, alpha=a)

    # Computes the undistortion and rectification transformation map.
    # we get output:
    # lmapx, lmapy, rmapx, rmapy
    cv2.initUndistortRectifyMap(l.intrinsics, l.distortion, l.R, l.P, size, cv2.CV_32FC1, l.mapx, l.mapy)
    cv2.initUndistortRectifyMap(r.intrinsics, r.distortion, r.R, r.P, size, cv2.CV_32FC1, r.mapx, r.mapy)


def remap(src):
    """
    :param src: source image
    :type src: :class:`cvMat`

    Apply the post-calibration undistortion to the source image
    """
    return cv2.remap(src, mapx, mapy, cv2.INTER_LINEAR)


if calibrated:
    # Show rectified images
    lremap = l.remap(lgray)
    rremap = r.remap(rgray)
    lrect = lremap
    rrect = rremap
    if x_scale != 1.0 or y_scale != 1.0:
        lrect = cv2.resize(lremap, (lscrib_mono.shape[1], lscrib_mono.shape[0]))
        rrect = cv2.resize(rremap, (rscrib_mono.shape[1], rscrib_mono.shape[0]))