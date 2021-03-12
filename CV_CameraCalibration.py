import cv2 as cv
import PIL, os
from cv2 import aruco
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import glob
from picamera.array import PiRGBArray
from picamera import PiCamera
import picamera
import time

# SETTINGS FOR WHAT YOU'RE TRYING TO DO IN THIS FILE
GENERATE_BOARD = False
CAPTURE_CALIB_PHOTOS = False
CALIB_FROM_PHOTOS = False


workdir = "data/"

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
board = aruco.CharucoBoard_create(5, 7, 1.5, 1, aruco_dict)

camera = PiCamera(resolution=(3264, 2464))

def generate_ChArUco_board():
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    #board = aruco.CharucoBoard_create(5, 7, 1.5, 1, aruco_dict)
    imboard = board.draw(outSize=(2000,2000))
    cv.imwrite("ChArUco1.tiff", imboard)
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.imshow(imboard, cmap = mpl.cm.gray, interpolation = "nearest")
    ax.axis("off")
    plt.show()

def capture_photos():
    #choice = "0"
    #i = 0
    with picamera.array.PiRGBArray(camera) as stream:
        #while choice != "q":
        for i in range(10):
            num = str(i)

            camera.capture(stream, format="bgr")
            img = stream.array

            scale = 0.5
            disp_img = cv.resize(img, (int(img.shape[1] * scale),
                                       int(img.shape[0] * scale)
                                       ))
            cv.imshow("img"+num, disp_img)
            #choice = input("Enter 'q' to quit, and 'd' to delete photo: ")
            #if choice != "d":
            cv.imwrite("CalibrationImg"+num+".png", img)
            cv.waitKey(0)
            cv.destroyWindow("img"+num)

            stream.truncate(0)
            #i = i + 1
    
def read_chessboards():
    images = glob.glob('*.png')
    # 1.125, 0.75
    #board = aruco.CharucoBoard_create(3, 5, 1.5, 1, aruco_dict)

    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv.imread(im)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict)

        if len(corners) > 0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv.cornerSubPix(gray, corner,
                                winSize = (20, 20),
                                zeroZone = (-1, -1),
                                criteria = criteria)
            res2 = cv.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator += 1
    imsize = gray.shape
    return allCorners, allIds, imsize

def calibrate_camera(allCorners, allIds, imsize):
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[ 2000.,    0., imsize[0]/2.],
                                 [    0., 2000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

    distCoeffsInit = np.zeros((5, 1))
    flags = (cv.CALIB_USE_INTRINSIC_GUESS + cv.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     preViewErrors) = cv.aruco.calibrateCameraCharucoExtended(
         charucoCorners=allCorners,
         charucoIds=allIds,
         board=board,
         imageSize=imsize,
         cameraMatrix=cameraMatrixInit,
         distCoeffs=distCoeffsInit,
         flags=flags,
         criteria=(cv.TERM_CRITERIA_EPS & cv.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors

if __name__ == '__main__':
    if GENERATE_BOARD is True:
        generate_ChArUco_board()

    if CAPTURE_CALIB_PHOTOS is True:
        capture_photos()

    if CALIB_FROM_PHOTOS is True:
        allCorners, allIds, imsize = read_chessboards()
        
        ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors = calibrate_camera(allCorners, allIds, imsize)
        print("ret: ", ret)
        print("camera_matrix: ", camera_matrix)
        print("distortion_coefficients0: ", distortion_coefficients0)
        print("rotation_vectors: ", rotation_vectors)
        print("translation_vectors: ", translation_vectors)

        np.savez("cam_matrices_charuco.npz", k=camera_matrix, dist=distortion_coefficients0)
        
