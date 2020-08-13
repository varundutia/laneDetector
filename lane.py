from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
from picamera import PiCamera
import time
import cv2
import numpy as np
import math
m11=18
m12=23
m21=24
m22=25
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(m11, GPIO.OUT)
GPIO.setup(m12, GPIO.OUT)
GPIO.setup(m21, GPIO.OUT)
GPIO.setup(m22, GPIO.OUT)
GPIO.output(m11 , 0)
GPIO.output(m12 , 0)
GPIO.output(m21, 0)
GPIO.output(m22, 0)
p=GPIO.PWM(m11,40)
q=GPIO.PWM(m12,40)
r=GPIO.PWM(m21,40)
s=GPIO.PWM(m22,40)
p.start(90)
q.start(90)
r.start(90)
s.start(90)

def left_side_forward():
    GPIO.output(m21 , 0)
    GPIO.output(m22 , 0)
    GPIO.output(m11 , 1)
    GPIO.output(m12 , 0)

def right_side_forward():
   GPIO.output(m21 , 0)
   GPIO.output(m22 , 1)
   GPIO.output(m11 , 0)
   GPIO.output(m12 , 0)

def forward():
   GPIO.output(m11 , 1)
   GPIO.output(m12 , 0)
   GPIO.output(m21 , 0)
   GPIO.output(m22 , 1)
theta=0
minLineLength = 5
maxLineGap = 10
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
   image = frame.array
   gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
   blurred = cv2.GaussianBlur(gray, (5, 5), 0)
   edged = cv2.Canny(blurred, 85, 85)
   lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
   if(lines is not None):
      for x in range(0, len(lines)):
         for x1,y1,x2,y2 in lines[x]:
            cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
            theta=theta+math.atan2((y2-y1),(x2-x1))
   threshold=6
   if(theta>threshold):
       left_side_forward()
       print("left")
   if(theta<-threshold):
       right_side_forward()
       print("right")
   if(abs(theta)<threshold):
      forward()
      print ("straight")
   theta=0
   cv2.imshow("Frame",image)
   key = cv2.waitKey(1) & 0xFF
   rawCapture.truncate(0)
   if key == ord("q"):
       break
