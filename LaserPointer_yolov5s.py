# required imports
from ctypes import pointer
import tkinter as tk
from tkinter import ttk
from xmlrpc.client import boolean
import serial
import socket
import struct
import time
import cv2
import torch
import easygui
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode

# Variables for Wifi connection:
IP = "192.168.4.1" # Default ROBOT IP (with BROBOT JJROBOTS_XX wifi)
PORT = 2222        # Default ROBOT port
IN_PORT = 2223     # Default ROBOT input port
LaserPointerSock = None

SERIAL_PORT = False  # True if you want to use serial port, False (default) to use Wifi conection
# If you use wifi connection (SERIAL_PORT=False) be sure to connect first your PC to your robor netork (JJROBOTS_xx)
#Password: 87654321
COM_PORT = 'COM12'   # You could check the Serial port number on Arduino
COM_BAUD = 115200
ser = None

# Sending command bytes to robot
def sendCommand(Header,p1,p2,p3=0,p4=0,p5=0,p6=0,p7=0,p8=0):
    base = bytearray(Header)  # message
    param1 = bytearray(struct.pack(">h",p1))
    param2 = bytearray(struct.pack(">h",p2))
    param3 = bytearray(struct.pack(">h",p3))
    param4 = bytearray(struct.pack(">h",p4))
    param5 = bytearray(struct.pack(">h",p5))
    param6 = bytearray(struct.pack(">h",p6))
    param7 = bytearray(struct.pack(">h",p7))
    param8 = bytearray(struct.pack(">h",p8))
    message = base+param1+param2+param3+param4+param5+param6+param7+param8
    if SERIAL_PORT:
      #Send message via serial USB Connection
      try:
        ser.write(message)
      except:
        print("Could not send message (serial port)!")
    else:
      #Send message via Wifi Connection
      try:
        LaserPointerSock.sendto(message,(IP,PORT))
      except:
        print("Could not send message (Wifi)!")

# Turn laserpoinet on/off
def sendLPointer(state):
  global LPointer
  try:
    LPointer = state
    if (LPointer):
      print("Laser on")
    else:
      print("Laser off")
    sendCommand(b'JJAM',int(a1*100),int(a2*100),0,0,0,LPointer)
    time.sleep(0.5)
  except:
    print("Error!")

# Send new coordinates to Robot
def sendAngles(new_a1,new_a2):
  global a1,a2
  try:
    a1 = new_a1
    a2 = new_a2
    sendCommand(b'JJAM',int(a1*100),int(a2*100),0,0,0,LPointer)
  except:
    print("Error!")


#################   MAIN   ###########################
# Initialize global variables
a1=0
a2=0
LPointer = False
searching = None
lastMoving = False
scalingFactor = 1       #scale image for object detection
trackingSpeed = 0.035   #constant factor for object tracking speed
direction = 1           #defines direction for object search. 1: right; -1: left
counter = 0
line_thickness=3
cam_port = 2            #change to cam port of webcam
cam = cv2.VideoCapture(cam_port)

# Load yolov5s model from local source. Change path to local directory 
model = torch.hub.load('C:/Users/tobia/Documents/HIWI/Bilderkennung/YOLO/yolov5', 'yolov5s', source='local')
names = model.names

# Connect to Robot
if SERIAL_PORT:
  try:
    ser = serial.Serial(COM_PORT, COM_BAUD, timeout=1)
    
  except:
    print("Could not connect to serial port...",COM_PORT)
else:
  # WIFI conection
  try:
    print("Opening socket...")
    LaserPointerSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    LaserPointerSock.sendto(b'JJAH0000000000000000',(IP,PORT))
    print("Connected to Laser Pointer via Wifi... ")
  except:
    print("! Could not connect to laser pointer!. Check you are connected to the robot Wifi network (JJROBOTS_xx)")

sendCommand(b'JJAS',15,0,60,0) # 50% speed, 70% accel
sendAngles(a1,a2) # go to (0,0)

# Loop
while True:
    #get keyboard input
    c = cv2.waitKey(1)
    if c == 27: # esc
        break
    if c == 105: # i
        input0 = easygui.enterbox("What are you looking for?")
        # if input equals None, stop searching and turn off laserpointer
        if input0 == 'None':
            searching = None
            sendLPointer(False)
        # if yolov5 model knows input, search for input
        elif input0 in names.values():
            searching = input0
            print("looking for a", searching)
            counter = 0
        else:
            print("I dont know what a", input0, 'is.')
    
    # get current camera frame
    ret, frame = cam.read()
    frame = cv2.resize(frame, None, fx=scalingFactor, fy=scalingFactor, interpolation=cv2.INTER_AREA)

    # if robot is not searching, react to keyboard control [wasd]
    if searching is None:
        if c == 97:
            sendAngles(a1-2,a2)
            lastMoving = True
        elif c == 119:
            sendAngles(a1,a2+2)
            lastMoving = True
        elif c == 100:
            sendAngles(a1+2,a2)
            lastMoving = True
        elif c == 115:
            sendAngles(a1,a2-2)
            lastMoving = True
        else:
            # render current frame if not mooving
            if lastMoving:
                lastMoving = False
            else:
                results = model(frame)
                frame = results.render()[0] 
        #show current frame
        cv2.imshow('Input',frame)

    # If robot is searching, find object and move laserpointer to object 
    else:
        results = model(frame)
        frame = results.render()[0]
        cv2.imshow('Input',frame)
        print('searching')
        found = False
        # Looking for required object in detected objects
        for i in range(results.pandas().xyxy[0].values.shape[0]):
            # If object is required object, turn on laserpointer and move
            if results.pandas().xyxy[0].values[i][6] == searching:
                found = True
                counter = 0
                if not LPointer:
                    sendLPointer(True)
                print('found')
                # Get y/x-distance to frame center
                x = (results.pandas().xyxy[0].values[i][2] + results.pandas().xyxy[0].values[i][0] - frame.shape[1])/2
                y = (results.pandas().xyxy[0].values[i][3] + results.pandas().xyxy[0].values[i][1] - frame.shape[0])/2
                sendAngles(a1+x*(1+abs(x)/(frame.shape[1]/2))*trackingSpeed/scalingFactor,a2-y*(1+abs(y)/(frame.shape[1]/2))*trackingSpeed/scalingFactor)
                print('x:', x)
                print('y:', y)
                break
        # If required object is not detected in 5 consequtive frames, move to search for object
        if not found:
            # If required object is not detected, turn off laserpointer
            if LPointer:
                    sendLPointer(False)
            # If required object is not detected in 5 consequtive frames, move to search for object
            if counter > 5:
                if a1>100:
                    direction = -1
                elif a1<-100:
                    direction = 1
                sendAngles(a1+2*direction,a2)
            else:
                counter = counter + 1
    
# Shut down routine
sendLPointer(False)
cam.release()
cv2.destroyAllWindows()
