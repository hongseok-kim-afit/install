#!/usr/bin/env python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
from __future__ import print_function
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import argparse
import imutils
import sys
import os
#import t265_print_psn as psn

from datetime import date
from Xlib.display import Display
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from math import tan, pi
from serial import Serial
import serial           #test without this import, may be used in the usb input
import pymavlink        #test without this import
from pymavlink import mavutil
import math as m
import time
from pymavlink.dialects.v20 import common as mavlink2

#######################################3
# set the variables
set_speed = 1200 #throttle min = 991, max = 2015, but 1100 does not move
center_dist_bound = 50 #100 #last tested #pixels from center, defines x-y position tolerance
desired_distance = 1.0 #distance from desired target in meters
distance_bound = 0.5 #meters from target, defines distance tolerance
smooth = 2 # how much making a smooth turn
left_turn=1200
turn_max = 300
turn_min = 100
right_turn=1800  
neutral = 1500 
vel_dic = range (0,360,20)

###########################################
# functions
###########################################3
## movement function 
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs

def set_rc_channel_pwm(turn, throttle):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
        https://www.ardusub.com/developers/pymavlink.html
        channel 1 = left turn 991, neu = 1453, right turn =1965
                min = 1200, max 1800
                2 = transmission, max = 2015,neu=1499
                3 = throttle min = 991, max = 2015    
    """
   

    # Mavlink 2 supports up to 8 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [turn, 2000, throttle, 0, 1000, 0, 0, 0]
    
    '''
    print("pwm",type(pwm))
    print("rc",type(rc_channel_values))
    print("compo",type(master.target_component))
    print("system",type(master.target_system))
    '''
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

#########################################3
#connecting to autopilot
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600) # baud?
master.wait_heartbeat()

# connect to pixahwak with dronkit
vehicle = connect("/dev/ttyUSB0", baud=57600)

#initializing mode variable as GUIDED. Needed to not throw aruco detection exception
mode = 'GUIDED'

##############################################
# camera setting
###############################################
## D435i setting
'''
in this section we will set up for depth camera D435i
https://www.intelrealsense.com/depth-camera-d435i/
camera info
Depth output resolution: Up to 1280 × 720
Depth Field of View (FOV): 87° × 58° 
Depth frame rate: Up to 90 fps
RGB frame resolution: 1920 × 1080 
RGB sensor FOV (H × V): 69° × 42°
RGB frame rate: 30 fps 
USB‑C* 3.1 Gen 1* 
tracking camera mode s: stack, o: overlay q: quit
https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/t265_stereo.py
https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.config.html#pyrealsense2.config.enable_device
'''
#setup depth camera
pipe = rs.pipeline()
config = rs.config()
config.enable_device('040322073813') #D435i 040322073813
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
profile = pipe.start(config)

frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
color_init = np.asanyarray(color_frame.get_data())        
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
lineType               = 2

#####################################
# camera show
try:
    # Set up an OpenCV window to visualize the results
    WINDOW_TITLE1 = 'depth camera'
    while True:
        ##################################################
        ########## depth camera

        # Store next frameset for later processing:
        frameset = pipe.wait_for_frames()
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()

        color = np.asanyarray(color_frame.get_data())
        res = color.copy()
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        ########################################
        ######## connect to ardupilot        
        master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,1,1)
        msg = master.recv_match(type = "HEARTBEAT", blocking = False)
        if msg:
            mode = vehicle.mode
        print(mode)

        #mode check for command loop
        #if True: #for the test without mode
        # if mode == 'GUIDED':
        l_b = np.array([24, 133, 48]) # set hsv min color
        u_b = np.array([39, 200, 181]) # set hsv max color

        color = cv2.bitwise_and(color, color) #capture any color
        # mask = cv2.inRange(hsv, l_b, u_b)  #set the range of capture color
        # color = cv2.bitwise_and(color, color, mask=mask)  # only capture from l_b to u_b ## green color capture

        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())

        
        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        # Update color and depth frames:
        aligned_depth_frame = frameset.get_depth_frame()
        colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

        ### motion detector
        d = cv2.absdiff(color_init, color)
        gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
        (c, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(color, c, -1, (0, 255, 0), 2)
        color_init = color

        #Get depth data array
        depth = np.asanyarray(aligned_depth_frame.get_data())
        #print(depth)
        #Depth array is transposed when pulled, found by Charlie and Jacob                   #???
        depth = np.transpose(depth)

        for contour in c:
            if cv2.contourArea(contour) < 1500:
                continue
            (Cx,Cy), radius = cv2.minEnclosingCircle(contour)
            (x, y, w, h) = cv2.boundingRect(contour)  # draw box
            bottomLeftCornerOfText = (x, y)

            # get the target center
            #Cx = (x + w) / 2.0
            #Cy = (y + h) / 2.0
            
            center = (int(Cx), int(Cy))
            radius = int(radius)
            # Crop depth data:
            depth = depth[x:x+w, y:y+h].astype(float)

            depth_crop = depth.copy()

            if depth_crop.size == 0:
                continue
            depth_res = depth_crop[depth_crop != 0]


            # Get data scale from the device and convert to meters
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            depth_res = depth_res * depth_scale

            if depth_res.size == 0:
                continue

            dist = min(depth_res)
            #print(dist)

            cv2.circle(res, center, radius,(0,255,0),2) # draw target circle
            # cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 3) # draw target rectangle 
            text = "Depth: " + str("{0:.2f}").format(dist)
            cv2.putText(res,
                        text,
                        bottomLeftCornerOfText,
                        font,
                        fontScale,
                        fontColor,
                        lineType)
            #initializing autopilot control variables
            frame_center = (640,360)
            fov = (45,32.5)
            desired_pixel = frame_center
            #yaw_depth1 = depth[frame_center[0]-100, frame_center[1]]
            #yaw_depth2 = depth[frame_center[0]+100, frame_center[1]]
            target_center = [Cx,Cy] 
            target_depth = dist
            print(target_center, ':target center')
            print(target_depth, ':target depth')
            
            cv2.imshow(WINDOW_TITLE1, res)
        
finally:
    pipe.stop()
    pipe2.stop()
#####################################
# main code
##################################
t=time.gmtime()
date = date.today()
current_time = time.strftime("%H:%M:%S", t)

#os.mkdir("~/Desktop/Logfiles/SENG550_Group2_test_"+str(date)+str(t))

print("Script Start: ", current_time)
## set up the duration and update rate
DURATION = 3 # how many times do you want?
rate_u = 1 # put the update Hz