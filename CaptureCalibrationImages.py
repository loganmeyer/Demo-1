from picamera.array import PiRGBArray
from picamera import PiCamera
import picamera
import cv2 as cv
import numpy as np
import time

if __name__ == '__main__':
    camera = PiCamera(resolution=(1920, 1088))
##    camera.shutter_speed = camera.exposure_speed
##    camera.exposure_mode = 'off'
##    g = camera.awb_gains
##    camera.awb_mode = 'off'
##    camera.awb_gains = g

    with picamera.array.PiRGBArray(camera) as stream:
        for i in range (10):
            num = str(i)
            
            camera.capture(stream, format="bgr")
            img = stream.array

            cv.imshow("img"+num, img)
            cv.imwrite("CalibrationImg"+num+".png", img)
            cv.waitKey(0)
            cv.destroyWindow("img"+num)

            stream.truncate(0)
