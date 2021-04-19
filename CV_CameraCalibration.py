import numpy as np
import cv2 as cv
import glob
from picamera.array import PiRGBArray
from picamera import PiCamera
import picamera
import time

# SETTINGS FOR WHAT YOU'RE TRYING TO DO IN THIS FILE
CAPTURE_CALIB_PHOTOS = False
NUM_PHOTOS = 20
CALIB_FROM_PHOTOS = False

# Camera capture width and height in pixels
WIDTH, HEIGHT = 640, 480

# Length of chessboard square
SQUARE_LEN = ((9 + (5.75 / 16)) / 8)


def capture_photos():
    # Set up camera
    camera = PiCamera()
    camera.resolution = (WIDTH, HEIGHT)
    with picamera.array.PiRGBArray(camera) as stream:
        for i in range(NUM_PHOTOS):
            num = str(i)
            # Capture img
            camera.capture(stream, format="bgr")
            img = stream.array
            # Show and save img
            cv.imshow("calibration_img_"+num, img)
            cv.imwrite("calibration_img_"+num+".png", img)
            cv.waitKey(0)
            cv.destroyWindow("calibration_img_"+num)

            stream.truncate(0)


def calibrate_from_photos():
    # Criteria for sub-pixel corner detection
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    # Object points in real world coordinates
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
    # Scale to measured length of chessboard square
    objp = objp * SQUARE_LEN

    objpoints = []
    imgpoints = []

    # Load images from directory
    images = glob.glob('*.png')

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, corners = cv.findChessboardCorners(gray, (9,6), None)

        if ret:
            # Sub pixel corner detection
            corners2 = cv.cornerSubPix(gray, corners, (3,3), (-1,-1), criteria)
            # Add img points and object points to array
            imgpoints.append(corners2)
            objpoints.append(objp)
            # Show detected chessboard corners
            cv.drawChessboardCorners(img, (9,6), corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(0)

    cv.destroyAllWindows()

    # Get camera calibration data
    ret, mtx, dst, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints,
                                                     gray.shape[::-1], None, None)

    print("ret: ", ret)
    print("mtx: ", mtx)
    print("dst: ", dst)
    # Save camera calibration data
    np.savez("CV_CameraCalibrationData.npz", k=mtx, dist=dst)


if __name__ == '__main__':
    if CAPTURE_CALIB_PHOTOS:
        capture_photos()

    if CALIB_FROM_PHOTOS:
        calibrate_from_photos()
