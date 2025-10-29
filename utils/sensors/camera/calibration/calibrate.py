#  NOTE this script was adapted from Kenneth Jiang's OpenCV calibration script which can be found here:
#  https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
#  Minor changes were added to better fit my usecase of calibrating for ORB-SLAM3

import cv2
assert cv2.__version__[0] >= '3', 'The fisheye module requires opencv version >= 3.0.0' # (NOTE) changed the comparitor to support versions > 3 
import numpy as np
import os
import glob
CHECKERBOARD = (6,8) # (NOTE) modified the calibration board layout
subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
_img_shape = None
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('./2025-10-29/*.jpg')
for fname in images:
    img = cv2.imread(fname)
    if _img_shape == None:
        _img_shape = img.shape[:2]
    else:
        assert _img_shape == img.shape[:2], "All images must share the same size."
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
    # If found, add object points, image points (after refining them)
     # (NOTE) added lnes 30-50 here to view the images were the pattern is valid to make sure we're covering the full field-of-view
    img_disp = img.copy()
    if ret:
        # Draw the full chessboard corner pattern
        cv2.drawChessboardCorners(img_disp, CHECKERBOARD, corners, ret)
        status = f"ret=True, corners={len(corners)}"
        color = (0, 200, 0)  # green
    else:
        status = "ret=False (no pattern found)"
        color = (0, 0, 255)  # red

    # Put status text on the image
    cv2.putText(img_disp, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

    # Show image and wait for Enter
    cv2.imshow("Corners", img_disp)
    print("Press Enter to continue...")
    while True:
        k = cv2.waitKey(0) & 0xFF
        if k in (13, 10):  # Enter/Return
            break
    cv2.destroyWindow("Corners")
    if ret == True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
        imgpoints.append(corners)
N_OK = len(objpoints)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
rms, _, _, _, _ = \
    cv2.fisheye.calibrate(
        objpoints,
        imgpoints,
        gray.shape[::-1],
        K,
        D,
        rvecs,
        tvecs,
        calibration_flags,
        (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )
print("Found " + str(N_OK) + " valid images for calibration")
print("DIM=" + str(_img_shape[::-1]))
print("K=np.array(" + str(K.tolist()) + ")")
print("D=np.array(" + str(D.tolist()) + ")")