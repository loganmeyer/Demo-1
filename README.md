# Demo-1 
dual-mc33926-motor-shield-master.zip: Arduino library containing built-in functions for driving and initializing the motor driver shield. 

Encoder-1.4.1.zip: Arduino library containing built-in Arduino functions for reading the encoders 

MovingForward.ino: Arduino program for moving the robot forward a set number of feet, in a straight line. ***UNFINISHED***

rotating.ino: Arduino program for rotating the robot a set number of degrees, rotating in place and then moving forwards 1 ft. ***UNFINISHED***

ComputerVision.py: Main Raspberry Pi program to detect an Aruoco marker, and it's angle and distance from the center of the raspberry pi camera.

CV_CameraCalibration.py: Raspberry Pi program to get the camera calibration matrices by generating a ChArUco board, capturing a number of images of it,
	and using computer vision techniques to detemine the optimal camera calibration matrices.

CV_CameraCalibrationMatrices.npz: Output file for 'CV_CameraCalibration.py'. Contains the camera calibration matrices used as an input for 'ComputerVision.py'.

CV_GetMeasuredVals.py: Raspberry Pi program to quickly and easliy calculate measured angles and distances for the Aruco marker from the camera for testing.
	Simply input the x and z distances to get the true distance and angle in order to compare to values generated from 'ComputerVision.py'.

CV_ZeroAngleCalibration.py: Raspberry Pi program to get the calibrated zero angle for a given position of the camera. See the header within the file for details.

CV_ZeroAngle.npz: Output file for 'CV_ZeroAngleCalibration.py'. Contains the calibrated zero angle for use as an input for 'ComputerVision.py'.
