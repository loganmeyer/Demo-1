import numpy as np
import cv2 as cv
import glob

if __name__ == '__main__':
    # Termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points
    objp = np.zeros((6*9, 3), np.float32)
    print("objp: ", objp)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
    print("objp: ", objp)
    objp = objp * 15 / 16
    print("fixed: ", objp)
    # Arrays to store object points and image points from all the images
    objpoints = []
    imgpoints = []
    
    images = glob.glob('*.png')

    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (9,6), None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            print("objpoints: ", objpoints)

            cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv.drawChessboardCorners(img, (9, 6), corners, ret)
            cv.imshow(fname, img)
            cv.waitKey(1000)
            cv.destroyWindow(fname)
        

    cv.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints,
                                                      imgpoints,
                                                      gray.shape[::-1],
                                                      None,
                                                      None
                                                      )

    print("ret: ", ret)
    print("mtx: ", mtx)
    print("dist: ", dist)
    print("rvecs: ", rvecs)
    print("tvecs: ", tvecs)

##    img = cv.imread('CalibrationImg8.png')
##    h, w = img.shape[:2]
##    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
##
##    dst = cv.undistort(img, mtx, dist, None, newcameramtx)
##
##    x, y, w, h = roi
##    dst = dst[y:y+h, x:x+w]
##    cv.imshow('calibresult', dst)
##    cv.waitKey(0)
##    cv.destroyWindow('calibresult')

    np.savez("cam_matrices.npz", k=mtx, dist=dist)
    
