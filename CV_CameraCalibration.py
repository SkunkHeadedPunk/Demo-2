#!/usr/bin/env python
"""Camera Calibration Module for Demo-1. SEED Lab: Group 007.

REQUIREMENTS AND COMPATIBILITY:
Requires install of numpy, picamera, matplotlib, glob, and opencv.
Built for use with a Raspberry PiCamera.

PURPOSE:
This is an all encompassing program for getting camera calibration data for a
given camera. It may be used to generate a custom ChArUco calibration board,
capture and save a number of calibration images, process the calibration
images to get accurate corner locations and data, and generate the best
fitting camera calibration matrices and data.

METHODS:
This program uses built in CV Aruco functions to generate a desired ChArUco
board, and matplotlib to display it for saving and printing purposes. It uses
a pi camera array to capture user images holding the calibration board, and cv
to display and save those images as they are captured. Glob is used to pull up
each of the captured photos for processing. Specified sub-pixel criteria and
specialized cv functions are used to get highly accurate corner detection. A
cv ChArUco camera calibration function is used with this data, and an initial
camera matrix guess, in order to generate the most accurate camera calibration
matrices and data possible from this function. Finally, a numpy save function
is used to save the camera calibration matrix, and distortion coefficients to
a file for use in other programs. Global variables at the top of the file are
used to select what functions the user would like to use from this program.

INSTRUCTIONS:
Set the global variable "GENERATE_BOARD" to "True" in order to generate a
ChArUco board for calibration. Set the global variable "CAPTURE_CALIB_PHOTOS"
to "True" to capture photos of the printed ChArUco board in a variety of
positions using the PiCamera. Set the global variable "NUM_PHOTOS" equal to
an integer to determine the number of calibration photos captured. No less
than ten photos should be used for any calibration. Set the global variable
"CALIB_FROM_PHOTOS" to "True" to calibrate from the captured calibration
photos within the directory. Please ensure all photos in the directory are
from the same calibration, as they will all be used by the program to generate
the calibration data.

OUTPUTS:
The camera calibration matrix and distortion coefficients are automatically
saved as 'k' and 'dist' respectively in the file 'cam_matrices_charuco.npz'.

REFERENCE:
Much of this code is taken directly from, or heavily inspired by the online
example at the below address:
https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/sandbox/ludovic/aruco_calibration_rotation.html
Changes were made for easier use and integration, along with some custom
settings.
"""

__author__ = "Jack Woolery"
__email__ = "lwoolery@mines.edu"


import cv2 as cv
from cv2 import aruco
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import glob
from picamera import PiCamera
import picamera

# SETTINGS FOR WHAT YOU'RE TRYING TO DO IN THIS FILE
GENERATE_BOARD = False
CAPTURE_CALIB_PHOTOS = False
NUM_PHOTOS = 10
CALIB_FROM_PHOTOS = False

# Set directory for files
workdir = "data/"

# Import Aruco dictionary and desired ChArUco board
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(5, 7, 1.5, 1, aruco_dict)

# Set up camera
camera = PiCamera(resolution=(3264, 2464))

# Function to generate a ChArUco board to print out for calibration
def generate_ChArUco_board():
    imboard = board.draw(outSize=(2000, 2000))
    cv.imwrite("ChArUco1.tiff", imboard)
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")
    plt.show()

# Function to capture a number of photos of the ChArUco board
# Take photos of the board in varying positions and angles
def capture_photos():
    with picamera.array.PiRGBArray(camera) as stream:
        for i in range(NUM_PHOTOS):
            num = str(i)
            camera.capture(stream, format="bgr")
            img = stream.array
            scale = 0.5
            disp_img = cv.resize(img, (int(img.shape[1] * scale),
                                       int(img.shape[0] * scale)
                                       ))
            cv.imshow("img"+num, disp_img)
            cv.imwrite("CalibrationImg"+num+".png", img)
            cv.waitKey(0)
            cv.destroyWindow("img"+num)
            stream.truncate(0)

# Get Aruco corners, ids, and image shape
def read_charuco():
    images = glob.glob('*.png')
    allCorners = []
    allIds = []
    decimator = 0
    # Sub pixel corner detection criterion
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv.imread(im)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            # Sub pixel detection for improving accuracy
            for corner in corners:
                cv.cornerSubPix(gray, corner,
                                winSize = (20, 20),
                                zeroZone = (-1, -1),
                                criteria = criteria)
            res2 = cv.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator += 1
    imsize = gray.shape
    return allCorners, allIds, imsize

# Generate camera calibration matrices from detected data
def calibrate_camera(allCorners, allIds, imsize):
    print("CAMERA CALIBRATION")
    # Create initial camera matrix
    cameraMatrixInit = np.array([[ 2000.,    0., imsize[0]/2.],
                                 [    0., 2000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5, 1))
    flags = (cv.CALIB_USE_INTRINSIC_GUESS + cv.CALIB_RATIONAL_MODEL)
    # Get camera calibration matrices and data
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     preViewErrors) = cv.aruco.calibrateCameraCharucoExtended(
         charucoCorners=allCorners,
         charucoIds=allIds,
         board=board,
         imageSize=imsize,
         cameraMatrix=cameraMatrixInit,
         distCoeffs=distCoeffsInit,
         flags=flags,
         criteria=(cv.TERM_CRITERIA_EPS & cv.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


if __name__ == '__main__':
    if GENERATE_BOARD is True:
        generate_ChArUco_board()

    if CAPTURE_CALIB_PHOTOS is True:
        capture_photos()

    if CALIB_FROM_PHOTOS is True:
        allCorners, allIds, imsize = read_charuco()
        
        (ret, camera_matrix, distortion_coefficients0,
         rotation_vectors, translation_vectors) = calibrate_camera(allCorners, allIds, imsize)

        # Print the camera calibration matrices and vectors
        print("ret: ", ret)
        print("camera_matrix: ", camera_matrix)
        print("distortion_coefficients0: ", distortion_coefficients0)
        print("rotation_vectors: ", rotation_vectors)
        print("translation_vectors: ", translation_vectors)

        # Save the camera calibration matrix and distortion coefficients to file
        np.savez("cam_matrices_charuco.npz", k=camera_matrix, dist=distortion_coefficients0)
