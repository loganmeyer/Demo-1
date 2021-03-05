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

MARKER_LENGTH_MM = 200
MARKER_LENGTH_IN = MARKER_LENGTH_MM / 25.4
# 3280, 2464
WIDTH = 3136
HEIGHT = 2400

KD = np.load('cam_matrices.npz')
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
K = np.array([[Fx, 0, Cx],
              [0, Fy, Cy],
              [0, 0, 1]])
   
   

arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)

def detect_marker(img):
    angle = []
##    # Perform binarization to enhance ability to locate corners correctly
##    _, binary_img = cv.threshold(src=img, maxval=255,
##                                 type=cv.THRESH_OTSU, thresh=0)
##    
##    cv.imshow("thresh_otsu", binary_img)
##    cv.waitKey(0)

    #binary_img = cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH
    corners, ids, _ = cv.aruco.detectMarkers(image=img, dictionary=arucoDict)
    img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    
    if ids is not None:
        print("Marker detected")
        for tag in ids:
            cv.aruco.drawDetectedMarkers(image=img,
                                         corners=corners,
                                         ids=ids,
                                         borderColor=(0, 0, 255)
                                         )
            sum_x = 0
            sum_y = 0
            for i in range(4):
                sum_x = sum_x + corners[0][0][i, 0]
                sum_y = sum_y + corners[0][0][i, 1]
            ave_x = sum_x / 4
            ave_y = sum_y / 4
            #loc.append([ave_x, ave_y])
         
        distance, img_marked = get_distance(corners, img)
        angle = 0

    if ids is None:
        print("Marker not detected")
        angle = 0
        distance = 0
        cv.imshow("img no marker", img)
        cv.waitKey(0)
    return distance, angle, img

def get_distance(corners, img):
    print("k: ", K)
    print("dist: ", DIST_COEFFS)
    rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
        corners,
        markerLength = MARKER_LENGTH_IN,
        cameraMatrix = K,
        distCoeffs = DIST_COEFFS
        )
    print("rvecs: ", rvecs)
    print("tvecs: ", tvecs)
    # Unpack vectors
    rvec_m_c = rvecs[0]
    tm_c = tvecs[0]
    # Use function to draw axis on Aruco marker
    
    cv.aruco.drawAxis(image=img, cameraMatrix=K, distCoeffs=DIST_COEFFS,
                      rvec=rvec_m_c, tvec=tm_c, length=MARKER_LENGTH_IN)

    scale = 0.5
    disp_img = cv.resize(img, (int(img.shape[1] * scale),
                               int(img.shape[0] * scale)
                               ))

    cv.imshow("img with axis", disp_img)
    cv.waitKey(0)

    # Calculate distance using the root of the sum of the squares from the
    # translation vector
    distance = math.sqrt(tm_c[0][0] ** 2 + tm_c[0][2] ** 2)
    print("distance: ", distance, "inches")
    
    
    return distance, img


if __name__ == '__main__':
##    x = input("Enter the number of seconds you'd like to take photos for: ")
##    # Change user input to int to set time for capture stream
##    x = int(x)
##    t_minus_x_seconds = time.time() + x

##    camera.shutter_speed = camera.exposure_speed
##    camera.exposure_mode = 'off'
##    g = camera.awb_gains
##    camera.awb_mode = 'off'
##    camera.awb_gains = g
    camera.resolution = (WIDTH, HEIGHT)
    #camera.framerate = 15

    with picamera.array.PiRGBArray(camera) as stream:
        #while time.time() < t_minus_x_seconds:
            camera.capture(stream, format="bgr")
            img = stream.array
            print("img size: ", img.shape)

            gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            distance, angle, img_markers = detect_marker(gray_img)

##            cv.imshow("Markers Stream", img_markers)
##            cv.waitKey(2)
##            cv.destroyWindow("Markers, stream")
            # Truncate the stream to clear for next image capture
            stream.truncate(0)

    cv.destroyAllWindows()
