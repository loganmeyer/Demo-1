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

camera = PiCamera()

MARKER_LENGTH_MM = 100
MARKER_LENGTH_IN = MARKER_LENGTH_MM / 25.4
MARKER_LENGTH_IN = 3.8125

# Max res: 3280 x 2464
WIDTH = 3264
HEIGHT = 2464

KD = np.load('cam_matrices_charuco.npz')
K = KD['k']
DIST_COEFFS = KD['dist']

# Parameters from camera specs sheet
FOV = 62.2 # degrees
Cx = WIDTH / 2
Cy = HEIGHT / 2
F = 3.04 * 10 ** -3
S = 1.12 * 10 ** -6
Fx = F / S / 2
Fy = F / S / 2
# Set camera intrinsic matrix
##K = np.array([[Fx, 0, Cx],
##              [0, Fy, Cy],
##              [0, 0, 1]])
##DIST_COEFFS = 0

# Center point of marker with respect to marker
P_M = np.array([MARKER_LENGTH_IN / 2, -MARKER_LENGTH_IN / 2, 0, 1])
   
   

arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)

def detect_marker(img):
    corners, ids, rejectedCorners = cv.aruco.detectMarkers(image=img, dictionary=arucoDict, cameraMatrix=K, distCoeff=DIST_COEFFS)                                   

    img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    
    if ids is not None:
        print("Marker detected")
        for tag in ids:
            cv.aruco.drawDetectedMarkers(image=img,
                                         corners=corners,
                                         ids=ids,
                                         borderColor=(0, 0, 255)
                                         )
                    
        distance, angle_rad, angle_deg, img_marked, disp_img = get_distance(corners, img)
        
##        cv.imshow("Stream", disp_img)
##        cv.waitKey(1)
##        cv.destroyWindow("Stream")

    if ids is None:
        print("Marker not detected")
        angle_deg = 0
        angle_rad = 0
        distance = 0
        
        scale = 0.25
        disp_img = cv.resize(img, (int(img.shape[1] * scale),
                                   int(img.shape[0] * scale)
                                   ))
        
##        cv.imshow("Stream", disp_img)
##        cv.waitKey(1)
##        cv.destroyWindow("Stream")
        
    return distance, angle_deg, angle_rad, img

def adjust_zero_angle():
    detected_angle_at_zero = -0.17
    zero_angle = - detected_angle_at_zero

    return zero_angle


def get_angle(x):
    rel_x = abs(Cx - x)
    angle_deg = (FOV / 2) * (rel_x / Cx)
    
    if Cx - x < 0:
        angle_deg = - angle_deg
        
    angle_rad = math.pi * angle_deg / 180

    print("angle: ", round(angle_deg, 2), "degrees;    ",
          round(angle_rad, 2), "radians")
    return angle_deg, angle_rad


def get_distance(corners, img):
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
        corners,
        markerLength = MARKER_LENGTH_IN,
        cameraMatrix = K,
        distCoeffs = DIST_COEFFS
        )

    # Unpack translation vector
    t_vec = tvecs[0][0]
    
    distance = math.sqrt(t_vec[0] ** 2 + t_vec[2] ** 2)
    print("distance: ", round(distance, 2), "inches")

    angle_rad = np.arctan(t_vec[0] / t_vec[2])
    angle_rad = - angle_rad
    angle_deg = angle_rad * 180 / math.pi
    print("angle: ", round(angle_deg, 2), "degrees;     ",
          round(angle_rad, 2), "radians")

    scale = 0.25
    disp_img = cv.resize(img, (int(img.shape[1] * scale),
                               int(img.shape[0] * scale)
                               )) 
    
    return distance, angle_rad, angle_deg, img, disp_img


if __name__ == '__main__':

    camera.resolution = (WIDTH, HEIGHT)
    #camera.framerate = 15

    with picamera.array.PiRGBArray(camera) as stream:
        while True:
            camera.capture(stream, format="bgr")
            img = stream.array

            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            distance, angle_deg, angle_rad, img_mark = detect_marker(gray_img)
            
            # Truncate the stream to clear for next image capture
            stream.truncate(0)

    cv.destroyAllWindows()

