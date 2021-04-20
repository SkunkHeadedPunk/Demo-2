# Demo-2
Demo-2 for Group 007 in SEED Lab

__Purpose of Repository:__

This repository contains code and Aruco markers for Demo-2 for SEED Lab. This code for both Arduino and 
Raspberry Pi programs a constructed robot to turn in place until a PiCamera detects an Aruco marker, then
approach the beacon within one foot, and drive in a circle around the beacon, travelling no further than two
feet away from it.



__File Organization:__

ArucoMarker_7x7_id0.pdf: 7x7 Aruco marker image 


CV_CameraCalibration.py: Raspberry Pi program to get the camera calibration data. Includes image
	capture, image processing, calibration data generation, and storage of data to file.

CV_CameraCalibrationData.npz: Output file for 'CV_CameraCalibration.py'. Contains camera
	calibration data used as input for 'CV_Demo-2.py'.

CV_Demo2.py: Main Raspberry Pi computer vision module for detecting Aruco marker distance and angle
	from robot, and sending to Arduino.

Framework_4_12_21.ino: Arduino file outlining the finite state machine that the Arduino will be 	
	running, as well as the general structure of variables, communication with the Pi, and locations of control systems. 
	
Demo2ArduinoCommunication.ino: Arduino file outlining the framework for communication between the 
	Arduino and the Pi, from the side of the Arduino. This file also shows the order of the  
	data in the transmitted array. 

demo2_final_arduino.ino: Arduino file used in Demo 2 to accomplish the tasks outlined above and in the Demo 2 handout.
	The file contains instructions on how to accompish each portion of the demo as well as further details on its functionaliy. 
	For this file to function and run properly, it must be used in conjunction with a Raspberry Pi running CV_Demo2.py. 
