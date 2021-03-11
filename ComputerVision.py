#!/usr/bin/env python
"""Mini-Project for SEED Lab: Group 007.

Requires install of numpy, picamera, time, and opencv. 
Built for Arducam OV5647.

This program uses computer vision techniques and opencv to capture a stream of
images, and performs resize, and convert to gray operations, before detecting
Aruco Markers and the quadrant at which they appear in.

See 'README.md' for more details and instructions.
If you make changes to this file, make sure to update the shared repo;
you may use the instructions included in "raspi_git_instructions.txt"
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
                    
        distance, angle_rad, angle_deg, img_marked, disp_img =
        get_vals(corners, img)

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
        calib_angle.append{angle_deg)
        
    return distance, angle_deg, angle_rad, calib_angle, img

def adjust_zero_angle(measured_angle):
    

    return zero_angle


def get_vals(corners, img):
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

    
    return distance, angle_rad, angle_deg, img, disp_img


if __name__ == '__main__':
    camera.resolution = (WIDTH, HEIGHT)

    # Set this true to calibrate the angle for zero degrees
    calibrate_zero_angle = True
    count = 0
    zero_angles = []

    with picamera.array.PiRGBArray(camera) as stream:
        while True:
            camera.capture(stream, format="bgr")
            img = stream.array

            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            distance,
            angle_deg,
            angle_rad,
            zero_angles,
            img_mark = detect_marker(gray_img,
                                     calibrate_zero_angle,
                                     zero_angles
                                     )
            
            # Truncate the stream to clear for next image capture
            stream.truncate(0)

            # Capture only ten images for angle calibration
            print("count: ", count)
            if calibrate_zero_angle is True and count == 9:
                for i in range 9:
                    sum_angles = sum_angles + zero_angles[i]
                avg_zero_angle = sum_angles / 10
                print("Calibration zero angle = ", avg_zero_angle)
                print("Set the global variable 'ZERO_ANGLE' equal to this value")
                break;
            count = count + 1

    cv.destroyAllWindows()

