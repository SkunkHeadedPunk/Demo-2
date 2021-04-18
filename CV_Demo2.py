#!/usr/bin/env python
"""Computer Vision module for Demo-2. SEED Lab: Group 007.
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

__author__ = "Jack Woolery and Jeffrey Hostetter"
__email__ = "lwoolery@mines.edu, "

from picamera.array import PiRGBArray
from picamera import PiCamera
import picamera
import time
from time import time
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

# INITIALIZATION OF ARRAY TO SEND TO ARDUINO
dataToArduino = [0, 180, 0]

# LCD INITIALIZATION
#lcd_columns = 16
#lcd_rows = 2
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.clear()
#lcd.color = [100, 0, 100]

address = 0x04

# FOR ZERO ANGLE CALIBRATION
USE_CALIB_ANGLE = False

CALIB_ANGLE_FILE = np.load('CV_ZeroAngle.npz')
CALIB_ANGLE = - CALIB_ANGLE_FILE['zero_angle']


# TO DISPLAY STREAM IMAGES
DISP_STREAM_UNDETECTED = True
STREAM_UNDETECTED_WAITKEY = 1
DISP_STREAM_DETECTED = True
STREAM_DETECTED_WAITKEY = 1
# TO DISPLAY PRECISE IMAGE
DISP_PRECISE_IMG = True
PRECISE_IMG_WAITKEY = 5

# AMOUNT OF TIME TO DISPLAY STREAM IMAGES
WAIT_KEY = 1
# Set wait key to minimum for fastest stream when not displaying
##if DISP_STREAM == False:
##    WAIT_KEY = 1

## __________IMAGE SCALING__________ ##
# Image scale for precise detection
IMG_CAPTURE_SCALE = 1
# Scale for resizing images for display
DISP_SCALE = 1

# Get the Aruco dictionary
arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_100)

# Measured Aruco marker length in inches
MARKER_LENGTH_IN = 3.8125

# Image capture dimensions
# Max res: 3280 x 2464
# MAX WORKING DIMENSIONS
WIDTH, HEIGHT = 640, 480
width, height = str(WIDTH), str(HEIGHT)

# Load camera properties matrices from file
# This file is generated from the camera calibration
FACTOR = 1  # Scale factor for calibration - CV use only
factor = str(FACTOR)
KD = np.load('CV_ChessboardCalibrationMatrices_scale'+factor+'.npz')
K = KD['k']
DIST_COEFFS = KD['dist']

# Physical distances relative to robot
DIST_FRONT_AXLE = 0
DIST_CENTER_ROTATION = 8
DIST_FROM_MARKER = 8


# Gets runtime / FPS data
def get_timing(start_time):
    runtime = time() - start_time
    fps = 1 / runtime
    runtime = round(runtime, 3)
    fps = round(fps, 3)
##    print("Runtime: ", runtime, "seconds")
    print("FPS: ", fps)
    print("\n")
    return fps


# Detects marker and precise corners
def detect_marker(img):
    # Get new camera matrix
##    newCamMtx, roi = cv.getOptimalNewCameraMatrix(K,
##                                                  DIST_COEFFS,
##                                                  (WIDTH, HEIGHT),
##                                                  1,
##                                                  (WIDTH, HEIGHT)
##                                                  )
    newCamMtx, roi = cv.getOptimalNewCameraMatrix(cameraMatrix=K,
                                                  distCoeffs=DIST_COEFFS,
                                                  imageSize=(WIDTH, HEIGHT),
                                                  alpha=1,
                                                  newImgSize=(WIDTH, HEIGHT)
                                                  )                                                  
    # Create undistorted, corrected image
    corr_img = cv.undistort(img, K, DIST_COEFFS, None, newCamMtx)

    # Detect Aruco marker corners and IDs
    corners, ids, _ = cv.aruco.detectMarkers(image=corr_img,
                                             dictionary=arucoDict,
                                             cameraMatrix=newCamMtx,
                                             distCoeff=0
                                             )
    # If marker detected...
    if ids is not None:
        # Perform subpixel corner detection
        gray_img = cv.cvtColor(corr_img, cv.COLOR_BGR2GRAY)
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,
                    100,
                    0.0001
                    )
        for corner in corners:
            cv.cornerSubPix(image=gray_img,
                            corners=corner,
                            winSize=(3,3),
                            zeroZone=(-1,-1),
                            criteria=criteria
                            )
        # Frame detected marker
        img = cv.aruco.drawDetectedMarkers(corr_img, corners, ids)
        # Get distance and angle to marker
        distance, angle_rad, angle_deg = get_vals(corners, newCamMtx)

    # If marker not detected...
    if ids is None:
        print("Marker not detected")
##        lcd.clear()
##        lcd.message = "Marker not detected"
        
        # Return zeros
        angle_deg = 180
        angle_rad = math.pi
        distance = 0

    return distance, angle_deg, angle_rad, img


# Gets distance and angle to Aruco marker
def get_vals(corners, newCamMtx):
    # Get rotation and translation vectors with respect to marker
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
        corners,
        markerLength = MARKER_LENGTH_IN,
        cameraMatrix = newCamMtx,
        distCoeffs = 0
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

##    # Send angle and distance to Arduino
##    dataToArduino[1] = int(round(angle_deg))
##    dataToArduino[2] = int(round(distance))
##    writeBlock(dataToArduino)
    
    return distance, angle_rad, angle_deg


####### FUNCTION FOR WRITING ARRAY TO ARDUINO #######
def writeBlock(block):
    try:
        bus.write_i2c_block_data(address, 1, block)
        #lcd.message = "Sent: " + userString + "\
    except:
        print("%%%%%%%%%%I2C Error%%%%%%%%%%%")
        print("%%%%%%%%%%I2C Error%%%%%%%%%%%")
        print("%%%%%%%%%%I2C Error%%%%%%%%%%%")
        print("%%%%%%%%%%I2C Error%%%%%%%%%%%")
        print("%%%%%%%%%%I2C Error%%%%%%%%%%%")
        print("%%%%%%%%%%I2C Error%%%%%%%%%%%")
        print("%%%%%%%%%%I2C Error%%%%%%%%%%%")


####### FUNCTION FOR WRITING A NUMBER TO THE ARDUINO #######
def writeNumber(num):
    try:
        bus.write_byte_data(address, 0, num)
    except:
        print("I2C Error")
    return -1


####### FUNCTION FOR READING A NUMBER FROM THE ARDUINO #######
def readNumber():
    try:
        number = bus.read_byte(address)
        return number
    except:
        print("I2C Error")
        return 0


def state0(state, img):
    print("Running State 0")

##    # Convert to grayscale for Aruco detection
##    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                
    # Detect if Aruco marker is present
    corners, ids, _ = cv.aruco.detectMarkers(image=img,
                                             dictionary=arucoDict,
                                             cameraMatrix=K,
                                             distCoeff=DIST_COEFFS
                                            )
    # If marker not detected...
    if ids is None:
        print("Beacon not detected")
        
        # Optional stream display
        if DISP_STREAM_UNDETECTED is True:
            cv.imshow("Stream - undetected", img)
            cv.waitKey(STREAM_UNDETECTED_WAITKEY)

    # If marker detected...
    if ids is not None:
        print("----------BEACON DETECTED----------")
        # Change to next state
        state = 1

        # Optional stream detected display
        if DISP_STREAM_DETECTED is True:
            for tag in ids:
                cv.aruco.drawDetectedMarkers(image=img,
                                             corners=corners,
                                             ids=ids,
                                             borderColor=(0, 0, 255)
                                            )
                # Display image from time of initial state0 detection
                cv.imshow("Stream - DETECTED", img)
                cv.waitKey(STREAM_DETECTED_WAITKEY)
                
            try:
                cv.destroyWindow("Stream - undetected")
            except:
                print("")

    return state


def state1():
    print("Running State 1")

    # Set up picamera array
    with picamera.array.PiRGBArray(camera) as stream:
        # Capture one image
        camera.capture(stream, format="bgr")
        img = stream.array

##        # Convert to grayscale for Aruco detection
##        gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                
        # Detect Aruco marker, and get detected angle and distance
        distance, angle_deg, angle_rad, img = detect_marker(img)

        # Optional display stream images
        if DISP_PRECISE_IMG is True:
            # Display image
            cv.imshow("Precise Img", img)
            cv.waitKey(PRECISE_IMG_WAITKEY)
                
        # Truncate the stream to clear for next image capture
        stream.truncate(0)
        
        return distance, angle_deg, angle_rad
  
            
if __name__ == '__main__':
    # Initialize state
    state = 0
    start = time()
    fps_arr = []

    # Initialize camera
    camera = PiCamera()
    camera.resolution = (WIDTH, HEIGHT)

    while True:
        ### GET 'state' FROM ARDUINO ###
        print("Reading state from Arduino")
        state = readNumber()

        # State0: Rotate robot and search continuously for marker
        if state == 0:
            # Set up capture array for PiCamera
            camera.exposure_mode = 'sports'
            rawCapture = PiRGBArray(camera, size=(WIDTH, HEIGHT))

            # Default distance and angle
            distance = 0
            angle_deg = 180

            for frame in camera.capture_continuous(rawCapture,
                                                   format="bgr",
                                                   use_video_port=True
                                                   ):
                # Get start time of state0 iteration
                start_time = time()

                # Run state0 function on captured frame
                img = frame.array
                state_send = state0(state, img)

                rawCapture.truncate(0)

                ###### RETURN STATE TO ARDUINO ######
                if state_send == 1:
                    dataToArduino[0] = state_send
                    writeBlock(dataToArduino)
                    state = 10
                    break;

                # Get FPS info
                fps_arr.append(get_timing(start_time))

            # Get average FPS
            avg_fps_sum = 0
            for val in fps_arr:
                avg_fps_sum = avg_fps_sum + val

            if len(fps_arr) != 0:
                avg_fps = avg_fps_sum / len(fps_arr)
                print("\nAverage FPS: ", avg_fps)

        # State 1: Robot has stopped; capture still photo, send dist & angle
        if state == 1:
            camera.exposure_mode = 'auto'
            distance, angle_deg, angle_rad = state1()
            
            #### SENDS DISTANCE AND ANGLE TO ARDUINO ####
            print("Sending angle and distance")
            dataToArduino[1] = int(round(angle_deg + 31))
            dataToArduino[2] = int(round(distance))
            writeBlock(dataToArduino)       
            
            # End after state 1
            break;

        # Final state
        if state == 5: # FINALSTATE?
            cv.destroyAllWindows()

        # Holding state for RPi
        if state == 10:
            print("Waiting to hear from Arduino")
