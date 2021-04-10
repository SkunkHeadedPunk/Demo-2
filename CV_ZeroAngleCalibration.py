#!/usr/bin/env python
"""CV Zero Angle Calibration module for Demo-1. DEED Lab: Group 007.

REQUIREMENTS AND COMPATIBILITY:
Required install of numpy, picamera, time, math, and opencv.
Built for Raspberry Pi Camera Module V2, but should work with other 

PURPOSE:
This file gets zero angle calibration to compensate for the initial zero angle
of the camera if the camera is pointed slightly off the z-axis. It increases
accuracy for angle detection.

METHODS:
The Aruco marker should be positioned centered in front of the camera anywhere
within ten feet. The program generates the mesasured angle to the marker a
number of times using computer vision techniques, gets the average angle, and
outputs it to 'CV_ZeroAngle.npz'. This angle can then be used to zero out the
angles for the current camera position in 'ComputerVision.py', thus increasing
detected angle accuracy, and preventing error from minor camera tilt around its
y-axis.

INSTRUCTIONS:
Aim the camera down a straight line and center an Aruco marker on the same
line. You may select the number of calibration images using the global variable
'CALIBRATE_IMG_COUNT' depending on available calibration time, and desired
accuracy. A number between 5 and 15 is recommended. Simply run the program and
it will automatically save the zero calibration angle to 'CV_ZeroAngle.npz'. To
use this calibration angle with 'ComputerVision.py' after this program is run,
simply open 'ComputerVision.py' and set the global variable 'USE_CALIB_ANGLE'
to 'True'. This will automatically import the value from the file and use it
to calibrate future runs of the program. Make sure that 'USE_CALIB_ANGLE' is
set to 'False' if the camera has been moved since the last run of this program,
or run this program again to calibrate.

OUTPUTS:
The detected calibration zero angle from this program is automatically saved to
'CV_ZeroAngle.npz' in the project directory. That file is updated and
overwritten each time this program is run.
"""

__author__ = "Jack Woolery"
__email__ = "lwoolery@mines.edu"

from picamera.array import PiRGBArray
from picamera import PiCamera
import picamera
import time
import cv2 as cv
import numpy as np
import math

# FOR ZERO ANGLE CALIBRATION
CALIBRATE_ZERO_ANGLE = True
CALIBRATE_IMG_COUNT = 5


# Initialize camera
camera = PiCamera()

# Get the Aruco dictionary
arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)

# Measured Aruco marker length in inches
MARKER_LENGTH_IN = 3.8125

# Image capture dimensions
# Max res: 3280 x 2464
WIDTH = 3264
HEIGHT = 2464

# Scale for resizing images for display
SCALE = 0.25

# Load camera properties matrices from file
# This file is generated from the camera calibration
KD = np.load('cam_matrices_charuco.npz')
K = KD['k']
DIST_COEFFS = KD['dist']

### Parameters from camera specs sheet
##FOV = 62.2 # degrees
##Cx = WIDTH / 2
##Cy = HEIGHT / 2
##F = 3.04 * 10 ** -3
##S = 1.12 * 10 ** -6
##Fx = F / S / 2
##Fy = F / S / 2
### Set camera intrinsic matrix
##K = np.array([[Fx, 0, Cx],
##              [0, Fy, Cy],
##              [0, 0, 1]])
##DIST_COEFFS = 0


def detect_marker(img, calibrate, calib_angle):
    corners, ids, rejectedCorners = cv.aruco.detectMarkers(image=img,
                                                           dictionary=arucoDict,
                                                           cameraMatrix=K,
                                                           distCoeff=DIST_COEFFS
                                                           )                                   

    img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)

    # If an Aruco marker is detected
    if ids is not None:
        print("Marker detected")
        for tag in ids:
            cv.aruco.drawDetectedMarkers(image=img,
                                         corners=corners,
                                         ids=ids,
                                         borderColor=(0, 0, 255)
                                         )
                    
        distance, angle_rad, angle_deg = get_vals(corners)

##        # RESIZE AND DISPLAY THE IMAGE
##        disp_img = cv.resize(img_marked, (int(img_marked.shape[1] * SCALE),
##                             int(img_marked.shape[0] * SCALE))
##                             )
##        
##        cv.imshow("Stream", disp_img)
##        cv.waitKey(1)
##        cv.destroyWindow("Stream")

    # If an Aruco marker is not detected
    if ids is None:
        print("Marker not detected")
        # Return zeros
        angle_deg = 0
        angle_rad = 0
        distance = 0

##        # RESIZE AND DISPLAY THE IMAGE
##        disp_img = cv.resize(img, (int(img.shape[1] * SCALE),
##                                   int(img.shape[0] * SCALE)
##                                   ))
##
##        cv.imshow("Stream", disp_img)
##        cv.waitKey(1)
##        cv.destroyWindow("Stream")

    # Code for calibrating zero angle
    if calibrate is True:
        calib_angle.append(angle_rad)
        
    return distance, angle_deg, angle_rad, calib_angle, img


def get_vals(corners):
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
        corners,
        markerLength = MARKER_LENGTH_IN,
        cameraMatrix = K,
        distCoeffs = DIST_COEFFS
        )

    # Unpack translation vector
    # This vector contains x, y, z distances of tag from camera
    t_vec = tvecs[0][0]

    # Calculate distance using the root of the sum of the squares
    distance = math.sqrt(t_vec[0] ** 2 + t_vec[2] ** 2)
    print("distance: ", round(distance, 2), "inches")

    # Calculate angle using trigonometry with distance values
    angle_rad = np.arctan(t_vec[0] / t_vec[2])
    angle_rad = - angle_rad
    angle_deg = angle_rad * 180 / math.pi
    print("angle: ", round(angle_deg, 2), "degrees;     ",
          round(angle_rad, 2), "radians")

    
    return distance, angle_rad, angle_deg


if __name__ == '__main__':
    camera.resolution = (WIDTH, HEIGHT)

    # Set this true to calibrate the angle for zero degrees
    zero_angles = []
    sum_angles = 0
    count = 0

    with picamera.array.PiRGBArray(camera) as stream:
        while True:
            camera.capture(stream, format="bgr")
            img = stream.array

            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            distance, angle_deg, angle_rad, zero_angles, img_mark = detect_marker(gray_img,
                                     CALIBRATE_ZERO_ANGLE,
                                     zero_angles
                                     )
            
            # Truncate the stream to clear for next image capture
            stream.truncate(0)

            # Capture only ten images for angle calibration
            print("count: ", count)
            if CALIBRATE_ZERO_ANGLE is True and count == CALIBRATE_IMG_COUNT-1:
                print("zero_angles[] = ", zero_angles)
                for i in range(CALIBRATE_IMG_COUNT):
                    sum_angles = sum_angles + zero_angles[i]
                    print("sum_angles = ", sum_angles)
                avg_zero_angle = sum_angles / (CALIBRATE_IMG_COUNT)
                print("Calibration zero angle = ", avg_zero_angle)
                np.savez("CV_ZeroAngle.npz", zero_angle=avg_zero_angle)
                break;
            count = count + 1

    cv.destroyAllWindows()


