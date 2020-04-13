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
from math import *

"""
DEBUG PARAMETERS
"""
DisplayPics = True;
SavePics = False;
LCD_Enabled = True;
maxSamples = 5;
#Number of distance samples before moving to circle
"""
STATE MACHINE FUNCS
"""
def state0(image):
    #LOOKING FOR MARKER
    global aruco_dict, parameters;
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters = parameters);
    if(ids is not None):
        return st_dict("Marker Found");
    else:
        return st_dict("Looking For Marker");
        
def state1(image):
    #MARKER FOUND
    global aruco_dict, parameters, newcameramtx, sampleDist, sampleAngle, finalDist, finalAngle;
    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, aruco_dict, parameters = parameters);
    if(len(sampleDist) > maxSamples):
        finalDist = sum(sampleDist) / len(sampleDist);
        finalAngle = sum(sampleAngle) / len(sampleAngle);
        return st_dict("Approaching");
    if(ids is not None):
        pose_rvecs, pose_tvecs, pose_obj_points = aruco.estimatePoseSingleMarkers(corners, 2.40157 , newcameramtx, None, None);
        for i in range(len(pose_tvecs)):
            thing = pose_tvecs[i];
            vec = thing[0];
            #Vector has components (x,z,y), sorta
            xComp = vec[0];
            xComp = xComp - (xComp / 12) - 1.5;
            yComp = vec[2];
            yComp = yComp - ((yComp - 12) / 12);
            #The Aruco distances seem to lose about an inch per foot, past the first foot, so this fixes that
            markerDist = sqrt( pow(xComp,2) + pow(yComp,2));
            strDist = '%.2f' % markerDist;
            markerAngle = atan2(yComp,xComp) * ((360.0)/(2.0*np.pi)) - 90;
            strAngle = '%.2f' % markerAngle;
            #print("Vector",i,"Is a distance away of ", markerDist,"cm and at an angle of", markerAngle, "radians");
            sampleAngle.append(markerAngle);
            sampleDist.append(markerDist);
            return st_dict("Marker Found");
    else:
        return st_dict("Looking For Marker");
        
def state2(image):
    #APPROACHING
    print("TODO: Approaching Function");
    return st_dict("Done");
def state3(image):
    #CIRCLING
    print("TODO: Circling Function");
    return st_dict("Done");
def state4(image):
    #FINAL APPROACH
    print("TODO: Final Approach Function");
    return st_dict("Done");
def state5(image):
    #DONE
    print("YEET");
    return st_dict("Done");
    
# create a dictionary to describe the states
st_dict = {
    state0 : "Looking For Marker",
    state1 : "Marker Found",
    state2 : "Approaching",
    state3 : "Circling",
    state4 : "Final Approach",
    state5 : "Done"
    }

# initalization of actual state machine

state = state0 # initial state as pointer to state0 function
inputStr = input("Type a string to start the state machine")

"""
INITIALIZATION
"""
state = st_dict("Looking For Marker");

#A cool global variable that I am using cause python doesn't do static vars outside of clases
sampleAngle = [];
sampleDist = [];
finalDist = 0.0;
finalAngle = 0.0;

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250);
parameters = aruco.DetectorParameters_create();

ret = 0.1160215884105962;
mtx = np.loadtxt('mtx.dat',delimiter=',');
dist = np.loadtxt('dist.dat',delimiter=',');
rvecs = np.loadtxt('rvecs.dat',delimiter=',');
tvecs = np.loadtxt('tvecs.dat',delimiter=',');

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

if(LCD_Enabled):
    lcd_columns = 16;
    lcd_rows = 2;
    i2c = busio.I2C(board.SCL, board.SDA);
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows);
    lcd.text_direction = lcd.LEFT_TO_RIGHT;
        
"""
MAIN LOOP
"""

#State machine runner, just executes states, and moves to the next state

#State machine executes on each image taken (frame), so we avoid de-allocating our camera initialization

#This is because de/re-initializing the camera is slow af, so we just want to set it up once to take a stream of photos
#This could be rewritten to allow for (slightly) faster execution when we no longer need the camera, but
#it doesn't seem neccesary right now.

for frame in camera.capture_continuous(rawCapture, format = 'bgr', use_video_port=True):
    #grab the image
    image = frame.array;
    #reset the capture array for new data
    rawCapture.truncate(0);
    #calculate undistortion matrix for our picture
    h,w = image.shape[:2];
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h));
    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    dst = cv2.remap(image,mapx,mapy,cv2.INTER_LINEAR)
    # undistort
    dst = cv2.undistort(image, mtx, dist, None, newcameramtx);
    #crop the image
    x,y,w,h = roi;
    dst = dst[y:y+h, x:x+w];
    image = dst;
    
    if(DisplayPics):
        cv2.imshow("Text",np.hstack([image,dst]));
    new_state = state(image)
    if state == st_dict("Done")
        break;
    state = new_state
#state machine is finished.