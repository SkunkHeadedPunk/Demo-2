# Demo-2
Demo-2 for Group 007 in SEED Lab

__Purpose of Repository__
This repository contains code and Aruco markers for Demo-2 for SEED Lab. This code for both Arduino and 
Raspberry Pi programs a constructed robot to turn in place until a PiCamera detects an Aruco marker, then
approach the beacon within one foot, and drive in a circle around the beacon, travelling no further than two
feet away from it.



__File Organization__

ArucoMarker_7x7_id0.pdf: 7x7 Aruco marker image 

ArucoMarker_4x4_id0.pdf: 4x4 Aruco marker image

CV_CameraCalibration.py: Raspberry Pi program to get the camera calibration data. Includes image
	capture, image processing, calibration data generation, and storage of data to file.

CV_CameraCalibrationData.npz: Output file for 'CV_CameraCalibration.py'. Contains camera
	calibration data used as input for 'CV_Demo-2.py'.

CV_Demo2.py: Main Raspberry Pi computer vision module for detecting Aruco marker distance and angle
	from robot, and sending to Arduino.


