#!/usr/bin/env python
"""Computer Vision module for Demo-1. SEED Lab: Group 007.

REQUIREMENTS AND COMPATIBILITY:
Requires install of numpy, picamera, time, math, and opencv. 
Built for Raspberry Pi Camera Module v2, but works with other picameras.

This file uses essential camera calibration matrices from the
'CV_CameraCalibrationMatrices.npz' file created by 'CV_CameraCalibration.py',
and an optional value from the file 'CV_ZeroAngle.npz', created by
'CV_ZeroAngleCalibration' for calibrating the zero angle. The camera
calibration matrices need to be updated using the respective file for any new
camera, and the zero angle value needs to be updated any time you reposition
the camera.

PURPOSE AND METHODS:
This program uses computer vision techniques and opencv to capture a stream of
images and detect Aruco markers. Opencv tequniques are used to get the
translation vector to the Aruco marker. X, Y, Z values from the translation
vector are used to calculate distance to the marker, and also calculate the
angle from the camera to the marker using trigonometry and math functions.


INSTRUCTIONS:
Set the global variable 'USE_CALIB_ANGLE' to 'True' in order to use the zero
angle calibration. The zero angle calibration should be performed every time
you move the camera module. The calibration angle can be updated using the file
'CV_ZeroAngle.py'. Set 'USE_CALIB_ANGLE' to 'False' if the value has not been
updated.

Set 'DISP_IMGS' to 'True' in order to view the stream of images as they are
captured. The amount of time each image is visible can be adjusted by changing
the WAIT_KEY value. This number is in milliseconds. It also affects the actual
capture rate of the images in the stream, if set to 'True'.

This program runs continuously. To exit, make sure the selected window is the
running Python Shell, and press ctrl+C.


OUTPUTS:
The detected angles from the camera to the Aruco marker are given in degrees
and radians in the 'main' function as 'angle_deg' and 'angle_rad',
respectively.

The detected distances from the camera to the Aruco marker are given in the
'main' function as 'distance'.
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
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import smbus
import busio
import board

# I2C INITIALIZATION
bus = smbus.SMBus(1)
i2c = busio.I2C(board.SCL, board.SDA)

# LCD INITIALIZATION
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [100, 0, 100]

# FOR ZERO ANGLE CALIBRATION
USE_CALIB_ANGLE = False

CALIB_ANGLE_FILE = np.load('CV_ZeroAngle.npz')
CALIB_ANGLE = - CALIB_ANGLE_FILE['zero_angle']


# TO DISPLAY STREAM IMAGES
DISP_IMGS = True
# Amount of time to display images (ms)
WAIT_KEY = 5000
# Scale for resizing images for display
SCALE = 0.5

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

# Load camera properties matrices from file
# This file is generated from the camera calibration
KD = np.load('CV_CameraCalibrationMatrices.npz')
K = KD['k']
DIST_COEFFS = KD['dist']


def detect_marker(img):
    # Detect Aruco markers, corners, and IDs
    corners, ids, _ = cv.aruco.detectMarkers(image=img,
                                             dictionary=arucoDict,
                                             cameraMatrix=K,
                                             distCoeff=DIST_COEFFS
                                             )                                   
    # Convert image to color
    img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)

    # If an Aruco marker is detected
    if ids is not None:
        print("Marker detected")
        lcd.clear()
        lcd.message = "Marker detected"
        for tag in ids:
            cv.aruco.drawDetectedMarkers(image=img,
                                         corners=corners,
                                         ids=ids,
                                         borderColor=(0, 0, 255)
                                         )
                    
        distance, angle_rad, angle_deg = get_vals(corners)

    # If an Aruco marker is not detected
    if ids is None:
        print("Marker not detected")
        lcd.clear()
        lcd.message = "Marker not detected"
        # Return zeros
        angle_deg = 0
        angle_rad = 0
        distance = 0

    # Resize image for smaller display size
    disp_img = cv.resize(img, (int(img.shape[1] * SCALE),
                               int(img.shape[0] * SCALE)
                               ))

    return distance, angle_deg, angle_rad, disp_img


def get_vals(corners):
    # Get rotation and translation vectors for Aruco marker
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
    if USE_CALIB_ANGLE is True:
        angle_rad = angle_rad + CALIB_ANGLE
    angle_deg = angle_rad * 180 / math.pi
    print("angle: ", round(angle_deg, 2), "degrees;     ",
          round(angle_rad, 2), "radians")
    
    return distance, angle_rad, angle_deg


if __name__ == '__main__':
    camera.resolution = (WIDTH, HEIGHT)

    # Set up picamera array
    with picamera.array.PiRGBArray(camera) as stream:
        # Run capture stream continuously
        while True:
            camera.capture(stream, format="bgr")
            img = stream.array

            # Convert to grayscale for Aruco detection
            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            
            # Detect Aruco marker, and get detected angle and distance
            distance, angle_deg, angle_rad, disp_img = detect_marker(gray_img)
            lcd.message = '\n' + str(round(angle_deg, 2)) + " degrees"

            # Optional display stream images
            if DISP_IMGS is True:
                cv.imshow("Stream", disp_img)
                cv.waitKey(WAIT_KEY)
                cv.destroyWindow("Stream")
            
            # Truncate the stream to clear for next image capture
            stream.truncate(0)
            
