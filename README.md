# Demo-1 
SEED Lab Demo-1 - Group 007

__Purpose of Repository__
This repository contains two sets of programs relating to computer vision, and robot movement:

Computer Vision: The computer vision programs use a Raspberry Pi. The main computer vision file uses a picamera to detect an Aruco marker within a 10' x 10' space,
and determines the angle from the center line of the camera to the Aruco marker, in addition to the distance from the picamera to the Aruco marker. This program runs
and detects continuously. This portion also contains files to generate the camera calibration matrices for use in the main file, and optional zero angle calibration files
to improve detected angle from a given camera position.

Robot Movement:



__File Organization__
+dual-mc33926-motor-shield-master.zip: Arduino library containing built-in functions for driving and initializing the motor driver shield. 

Encoder-1.4.1.zip: Arduino library containing built-in Arduino functions for reading the encoders 

MovingForward.ino: Arduino program for moving the robot forward a set number of feet, in a straight line.

rotating.ino: Arduino program for rotating the robot a set number of degrees, rotating in place and then moving forwards 1 ft.  

finalDemo1Code.ino: Arduino program for completing the tasks of Demo 1. Hard code the desired distance and spin for each test, as well as if the robot is going in a straight line or doing the turning test

ComputerVision.py: Main Raspberry Pi program to detect an Aruoco marker, and it's angle and distance from the center of the raspberry pi camera. The measured angle and a marker detection message is then displayed on an LCD display.

CV_CameraCalibration.py: Raspberry Pi program to get the camera calibration matrices by generating a ChArUco board, capturing a number of images of it,
	and using computer vision techniques to detemine the optimal camera calibration matrices.

CV_CameraCalibrationMatrices.npz: Output file for 'CV_CameraCalibration.py'. Contains the camera calibration matrices used as an input for 'ComputerVision.py'.

CV_GetMeasuredVals.py: Raspberry Pi program to quickly and easliy calculate measured angles and distances for the Aruco marker from the camera for testing.
	Simply input the x and z distances to get the true distance and angle in order to compare to values generated from 'ComputerVision.py'.

CV_ZeroAngleCalibration.py: Raspberry Pi program to get the calibrated zero angle for a given position of the camera. See the header within the file for details.

CV_ZeroAngle.npz: Output file for 'CV_ZeroAngleCalibration.py'. Contains the calibrated zero angle for use as an input for 'ComputerVision.py'.
