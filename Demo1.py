import picamera
from time import sleep
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import cv2.aruco as aruco
import numpy as np
from math import floor
from datetime import datetime as dt
import smbus
import busio
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
"""
DEBUG PARAMETERS
"""
DisplayPics = False;
SavePics = True;
"""
FUNCTIONS
"""

"""
INITIALIZATION
"""



camera = PiCamera();
rawCapture = PiRGBArray(camera);
camera.resolution = (864, 480);
camera.framerate = 30;
# Wait for the automatic gain control to settle
time.sleep(2);
# Now fix the values
camera.shutter_speed = camera.exposure_speed;
camera.exposure_mode = 'off';
g = camera.awb_gains;
camera.awb_mode = 'off';
camera.awb_gains = g;
counter = 0;
calibrateCount = 0;

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((7*5,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
"""
MAIN LOOP
"""
while(calibrateCount < 15):
    camera.capture(rawCapture, format='bgr');
    image = rawCapture.array;
    rawCapture.truncate(0);
    if(DisplayPics):
        cv2.imshow("Text",image);
        if(cv2.waitKey(1) > 0):
            break;
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7,5),None)
    if ret == True:
        objpoints.append(objp)
        cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)
        cv2.drawChessboardCorners(image, (7,5), corners,ret)
        cv2.imshow('img',image)
        cv2.waitKey(500)
        calibrateCount = calibrateCount + 1;
        print(">",calibrateCount);
    else:
        #cv2.imshow('img',image)
        #cv2.waitKey(500);
        pass;
camera.capture(rawCapture, format='bgr');
image = rawCapture.array;
gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);
print(len(objpoints[0]),len(imgpoints[0]));
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None);
#print(type(ret));
#print(type(mtx));
#print(type(dist));
#print(type(rvecs));
#print(type(tvecs));
#np.savetxt('ret.dat',np.array(ret),delimiter=',');
print("ret:",ret);
np.savetxt('mtx.dat',mtx,delimiter=',');
#print("mtx:",mtx);
np.savetxt('dist.dat',dist,delimiter=',');
#print("dist:",dist);
np.savetxt('rvecs.dat',np.array(rvecs),delimiter=',');
#print("rvecs:",rvecs);
np.savetxt('tvecs.dat',np.array(tvecs),delimiter=',');
#print("tvecs:",tvecs);
