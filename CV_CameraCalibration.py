#!/usr/bin/env python
"""Camera Calibration Module for Demo-2. SEED Lab: Group 007.

REQUIREMENTS AND COMPATIBILITY:
Requires install of numpy, picamera, glob, and opencv.
Built for use with a Raspberry PiCamera.

PURPOSE:
This is a program for getting camera calibration data for a given camera.
It may be used to capture and save a number of calibration images, process
the calibration images to get accurate corner locations and data, and generate
the best fitting camera calibration matrices and data. This data is then saved
to an external file 'CV_CameraCalibrationData.npz' for use in other programs.

METHODS:
This program uses a pi camera array to capture user images holding a calibration
chessboardboard, and cv to display and save those images as they are captured.
Glob is used to pull up each of the captured photos for processing. Specified
sub-pixel criteria and specialized cv functions are used to get highly accurate
corner detection within the images. A cv camera calibration function is used with
this data in order to generate the most accurate camera calibration matrices and
data possible from this function. Finally, a numpy save function is used to save
the camera calibration matrix, and distortion coefficients to a file for use in
other programs. Global variables at the top of the file are used to select what
functions the user would like to perform from this program.

INSTRUCTIONS:
Set the global variable "CAPTURE_CALIB_PHOTOS" to "True" to capture photos of a
printed calibration chessboard in a variety of positions using the PiCamera. Set
the global variable "NUM_PHOTOS" equal to an integer to determine the number of
calibration photos to be captured. No less than ten photos should be used for any
calibration. Set the global variable "CALIB_FROM_PHOTOS" to "True" to calibrate
from the captured calibration photos within the directory. Please ensure all
photos in the directory are from the same calibration, as all '.png's within the
directory will be used by the program to generate the calibration data. This data
can then be imported to another program from the file
'CV_CameraCalibrationData.npz'.

OUTPUTS:
The camera calibration matrix and distortion coefficients are automatically
saved as 'k' and 'dist' respectively in the file 'CV_CameraCalibrationData.npz'.

REFERENCE:
Much of the camera calibration code is taken directly from, or heavily inspired by
the example in the online opencv docs at the following address:
https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html
Changes were made for easier use and integration, along with some custom settings.
"""

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
    # Initialize empty points arrays
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
