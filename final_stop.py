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
import json

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
desired_distance = 2.0 #distance from desired target in meters
distance_bound = 0.5 #meters from target, defines distance tolerance
smooth = 2 # how much making a smooth turn
left_turn=1200
turn_max = 300
turn_min = 100
right_turn=1800  
neutral = 1500 
vel_dic = range (0,360,20)
i = 0
stop_speed = 1000
position = []

#########################################3
### connecting to autopilot
## connect to pixahwak with mavlink
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600) # baud?
master.wait_heartbeat()

## connect to pixahwak with dronkit
vehicle = connect("/dev/ttyUSB0", baud=57600)

#initializing mode variable as GUIDED. Needed to not throw aruco detection exception
mode = 'GUIDED'

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

##############################################
def relative_position (current_position, goal):
    '''
    current_position: WGS84 or Cartesian corrdinates (1.1 = 1.1m)
    with put in dictionary file
    goal is [0, 0, 0]

    '''
    del_x = goal[0] - current_position.x
    del_y = goal[1] - current_position.y
    del_z = goal[2] - current_position.z
    distance = np.sqrt(del_x*del_x + del_y*del_y + del_z*del_z)
    if del_x == 0:
        if del_y > 0:
            angle = m.pi/2
        elif del_y <0:
            angle = m.pi/2*3
        else:
            angle = 0
    else:
        theta = m.atan(abs(del_y/abs(del_x)))
        if del_x > 0:
            if del_y < 0:
                angle = 2 * m.pi - theta
            else :
                angle = theta
        else:
            if del_y > 0:
                angle = m.pi - theta
            else:
                angle = m.pi + theta
    return(angle, distance)
    #return(angle/m.pi*180, distance) for test

def relative_move (current_position, goal, heading):
    """
    current_position: WGS84 or Cartesian corrdinates (1.1 = 1.1m)
    with put in dictionary file
    goal is [0, 0, 0]
    haeding with radian units, 0 ~ 2*pi
    main is left turn,
    left turn 991, neutral = 1453, right turn =1965

    """
    rel_angle, rel_distance = relative_position(current_position, goal)
    if 0< heading - rel_angle <m.pi:
        #turn = right_turn
        turn = neutral + smooth_turn(rel_angle,heading) 
        set_rc_channel_pwm(turn, set_speed)
        return(rel_angle,heading,'right_turn')
    else:
        #turn = left_turn
        turn = neutral - smooth_turn(rel_angle,heading)
        set_rc_channel_pwm(turn, set_speed)
        return(rel_angle,heading,'left_turn')

def smooth_turn (rel_angle, heading):
    '''
    set the max and min neutral=1500, min = 1200(left), Max = 1800(right)
    left turn 991, neu = 1453, right turn =1965
    return 100 ~ 300
    '''
    if abs(heading-rel_angle) <= 30/180*m.pi:
        return (turn_min)
    elif abs(heading-rel_angle) <= 60/180*m.pi:
        return (turn_max-turn_min)
    elif abs(heading-rel_angle) < 300/180*m.pi:
        return (turn_max)
    elif abs(heading-rel_angle) < 330/180*m.pi:
        return (turn_max-turn_min)
    elif abs(heading-rel_angle) < 360/180*m.pi:
        return (turn_min)

###########################################
### camera setting
###############################################
## general setting
# Set up an OpenCV window to visualize the results
WINDOW_TITLE1 = 'depth camera'
WINDOW_TITLE2 = 'tracking camera'

# set up viedeo name
cv2.namedWindow(WINDOW_TITLE1, cv2.WINDOW_NORMAL)
cv2.namedWindow(WINDOW_TITLE2, cv2.WINDOW_NORMAL)

##############################################
## d435
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
# Configure depth and color streams
pipe = rs.pipeline()
config = rs.config()
config.enable_device('040322073813') #D435i 040322073813

# Get device product line for setting a supporting resolution
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# set up for depth draw
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
lineType               = 2

###################################
## T265 setting
'''
in this section we will set up for tracking camera T265
https://www.intelrealsense.com/tracking-camera-t265/
camera info
Two Fisheye l                enses with combined 163±5° FOV 
USB 2.0 and USB 3.1 supported for either pure pose data or a combination of pose and images.
https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.config.html#pyrealsense2.config.enable_device
'''

#configure fisheye 1(left), 2(right)
pipe2 = rs.pipeline()
config2 = rs.config()
config2.enable_device('119622110606') # T265 camera 119622110606


####################################
# Start streaming with requested config
profile = pipe.start(config)
prifile2 = pipe2.start(config2)


t=time.gmtime()
date = date.today()
current_time = time.strftime("%H:%M:%S", t)
print("Script Start: ", current_time)


####################################
# save images setup
# have to put in after the pipeline start
# for save date make a directory
time_path = './data/'+str(date)+str(t)

left_path = str(time_path)+'/tracking_camera/left'
right_path = str(time_path)+'/tracking_camera/right'
depth_path = str(time_path)+'/depth_camera/depth'
pointcloud_path = str(time_path)+'/depth_camera/pointcloud'

os.makedirs(time_path)
os.makedirs(left_path)
os.makedirs(right_path)
os.makedirs(depth_path)
os.makedirs(pointcloud_path)

#######################################################
# D435i get initial color frame
frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
color_init = np.asanyarray(color_frame.get_data())      

try:
    while True:
        '''
        # for mode print
        mode = vehicle.mode
        print(mode)
        '''
        ## T265
        # Wait for a coherent pair of frames
        frames2 = pipe2.wait_for_frames()
        
        # get the position data
        pose = frames2.get_pose_frame()
        color_frame2 = frames2.get_color_frame()

        config2.enable_stream(rs.stream.fisheye, 1)
        config2.enable_stream(rs.stream.fisheye, 2)
        
        # left and right camera setting for video
        left = frames2.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())

        right = frames2.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())


        
        # check pose is not empty
        if pose:
            # Print some of the pose data to the terminal
            pose_data = pose.get_pose_data()
            
            '''
            ##############################################
            ## test for position, velocity, accleration
            print("Frame #{}".format(pose.frame_number))
            print("Position: {}".format(pose_data.translation)) # pose_data.translation.x = position of x 
            print(pose_data.translation.x)
            print("Velocity: {}".format(pose_data.velocity)) # pose_data.velocity.x = velocity of x
            print(pose_data.velocity.x)
            print("Acceleration: {}".format(pose_data.acceleration)) # pose_data.acceleration.x = acceleration of x
                                    # \n enter line
            print(pose_data.acceleration.x)
                
            ############################
            ## get the yaw 
            # https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/t265_rpy.py
            w = pose_data.rotation.w
            x = -pose_data.rotation.z
            y = pose_data.rotation.x
            z = -pose_data.rotation.y
            yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z)
            print(yaw*180/m.pi) # change to degree # t265 left -0~-180, right 0~180
            print("heading: {}\n".format(vehicle.heading)) # degree 0~360
            ##############################################
            '''

        ## D435
        # Wait for a coherent pair of frames: depth and color
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color = np.asanyarray(color_frame.get_data())

        # for draw depth to color frame
        depth_data = color.copy()

        # set the wanting color for capture
        # hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV) # capturecolor
        # l_b = np.array([24, 133, 48]) # set hsv min color
        # u_b = np.array([39, 200, 181]) # set hsv max color
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

            cv2.circle(depth_data, center, radius,(0,255,0),2) # draw target circle
            # cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 3) # draw target rectangle 
            text = "Depth: " + str("{0:.2f}").format(dist)
            cv2.putText(depth_data,
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
            #################################################
            ## just for stop test
            if target_depth > desired_distance+distance_bound:
                set_rc_channel_pwm(neutral,set_speed)
            else:
                set_rc_channel_pwm(neutral,stop_speed)
            '''
            #################################################
            ### vehicle movement algorithm

            if mode == 'MANUAL':
                if target_depth > desired_distance+distance_bound:
                    set_rc_channel_pwm(neutral,set_speed)
                    
                    if target_center[0] < frame_center[0]-center_dist_bound:
                        # set_rc_channel_pwm(turn, throttle)
                        set_rc_channel_pwm(left_turn,set_speed)

                    elif target_center[0] > frame_center[0]+center_dist_bound:
                        # set_rc_channel_pwm(turn, throttle)
                        set_rc_channel_pwm(right_turn,set_speed)
                else:
                    # set_rc_channel_pwm(turn, throttle)
                    set_rc_channel_pwm(neutral,stop_speed)
            '''
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        '''
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))
        '''
        #color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)
        
        # Show images
        cv2.imshow(WINDOW_TITLE1, depth_data)
        cv2.imshow(WINDOW_TITLE2, left_data)

        # timestamp
        t=time.gmtime()
        date = date.today()
        current_time = time.strftime("%M:%S", t)

        #############################################3
        '''        
        ## save images
        cv2.imwrite('%s/%d.jpg' % (left_path, i), left_data)
        cv2.imwrite('%s/%d.jpg' % (right_path, i), right_data)
        cv2.imwrite('%s/%d.jpg' % (depth_path, i), depth_data)
        cv2.imwrite('%s/%d.jpg' % (pointcloud_path, i), depth_colormap)
        '''
        ###############################################################
        data = {
            'frame' : current_time,
            'position': pose_data.translation,
            'depth' : target_depth
            }
        #data.append(data)
        print(data)
        i = i+1
        cv2.waitKey(30)

finally:

    # Stop streaming
    pipe.stop()
    pipe2.stop()