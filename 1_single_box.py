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
#set_speed = 1250 #throttle min = 991, max = 2015, but 1100 does not move
set_speed = 1250 #throttle min = 991, max = 2015, but 1100 does not move
# Object avoidance variables
depth_pixel = [640, 480] # width(x)=640, height(y) = 480
head_offset = depth_pixel[0] / 2 # depth_pixel[0] / 2= 320
desired_distance = 1.5 #distance from desired action for object avoidance in meters
distance_bound = 0.5 # pad for avoidance
distance_limit = 0.3 #meters for goal stop distance
noaction_distance = 0.5 # sometimes depth camera catch the ground so ignore, pad for noaction, distance and range
pixel_bound = 50 #last tested #pixels from center, defines x-y position tolerance
fov = 87 * m.pi/180 # Depth Field of View (FOV): 87° × 58°, RGB sensor FOV (H × V): 69° × 42°

# movement variables
smooth = 2 # how much making a smooth turn
#left turn 991, neutral = 1453, right turn =1965
left_turn=1200
turn_max = 440
turn_min = 100
right_turn=1800  
neutral = 1453 
stop_speed = 1000
goal_dist = 0.3
ii= 0
iii = 0
i = 0
# save data variables
position = []
target_depth = []
result_data = []
# coordi_avoid, coordi_edge, coordi_center, pixal_center, pixal_edge,

# selected_route = input('select route, 1: straight, 2: triangle, 3: box')
# howmany = input('how many')
#################
#set the stpt for route
abb = 3
# # triangle
# st_goal='triangle'
# goal1=[-abb,0,abb * 1.732]
# goal2=[abb,0,abb * 1.732]
# goal3=[abb,0,0]
# goal4=[0,0,0]
# goal = [goal1,goal2,goal4,goal1,goal2,goal4] # z=forward, x=right

# box
bba=3
st_goal='s_box'
goal1=[0,0,abb]
goal2=[bba,0,abb]
goal3=[bba,0,0]
goal4=[0,0,0]
goal = [goal1,goal2,goal3,goal4] # z=forward, x=right
# goal = [goal1,goal2,goal3,goal4, goal1,goal2,goal3,goal4, goal1,goal2,goal3,goal4] # z=forward, x=right

# # straight stop
# st_goal='straight1Avoid_multi'
# goal1=[0,0,abb]
# goal2=[0,0,0]
# goal = [goal1,goal2,goal1,goal2,goal1,goal2] # z=forward, x=right

goal_stpt = goal1
current_goal = goal[ii]
tx, ty, tz, ox, oy, oz=0, 0, 0, 0, 0, 0
coordi_center = [0,0,0]
coordi_edge = [0,0,0]
pre_coordi_edge = [0,0,0]
pre_coordi_avoid = [0,0,0]
coordi_avoid = [0,0,0]
#########################################3
### connecting to autopilot
## connect to pixahwak with mavlink
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600) 
master.wait_heartbeat()

## connect to pixahwak with dronkit
vehicle = connect("/dev/ttyUSB0", baud=57600)

#initializing mode variable as GUIDED. Needed to not throw aruco detection exception
mode = 'GUIDED'

###########################################
### functions
###########################################
## message send function to Pixahwak
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(turn, throttle):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
        rc_channel_values = [turn, transmission, throttle, 0, 1000, 0, 0, 0]
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

#######################################################################33
## position and grab the target function
# coordinate change function
'''
# tracking coordinate change to depth coordinate
# t265 right: positive x, up: positive y, back: positive z
# d435, right: positive x, down: positive y, forward: positive z
# https://github.com/IntelRealSense/librealsense/tree/development/examples/tracking-and-depth
# Please note that the homogeneous transformation is expected to be provided as 
  row-major 3x4 matrix of the form H = [R, t] where R is the rotation matrix and 
  t the translation vector of the depth, i.e. infrared 1, frame, with respect to the T265 body frame  
'''
def depth_to_tracking (pixel_location, depth, pose_data, width):
    '''
    step1: get body position coordinate from the depth and pixel_location
    step2: get depth position coordinate using rotate matrix from body to depth frame
           theta, caculated heading(axis y), from the tracking camera is same to depth camera
    step3: tansport depth position to tracking position coordinate
    step4: move to current position
    '''
    position_current =[0, 0, 0]
    ## step 1
    # to get the x,z coordinate at tracking camera
    theta = (pixel_location[0]-width/2) / (width/2) * fov / 2

    position_body = np.array([[depth * m.sin(theta)],[0],[depth * m.cos(theta)]])

    ## step2
    # get the heading from coordinate
    w = pose_data.rotation.w
    x = -pose_data.rotation.z
    y = pose_data.rotation.x
    z = -pose_data.rotation.y
    yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z)
    # yaw -180~0(left)0~180(right) for change to 0~360
    if yaw <= 0.:
        yaw = yaw + 2*m.pi
    #print('vehicle heading 0~360', yaw*180/m.pi)
    heading = yaw #-m.pi  # the z axis of tracking camera is negative

    # rotate body frame to depth frame
    b_R_d = np.array([[m.cos(heading), 0, m.sin(heading)],
                               [0, 1, 0],
                               [-m.sin(heading), 0, m.cos(heading)]])
    position_depth = b_R_d.dot(position_body)

    ## step 3
    # matrix of frome tracking position to depth camera
    t_T_d = np.array([[0.999968402, -0.006753626, -0.004188075, -0.015890727],
                      [-0.006685408, -0.999848172, 0.016093893, 0.028273059],
                      [-0.004296131, -0.016065384, -0.999861654, -0.009375589]]) 

    # rotation matrix from tracking to depth camera
    t_R_d = np.array([[0.999968402, -0.006753626, -0.004188075],
                      [-0.006685408, -0.999848172, 0.016093893],
                      [-0.004296131, -0.016065384, -0.999861654]]) 
    
    # rotation matrix from depth to tracking camera
    d_R_t = np.linalg.inv(t_R_d)
    
    # get the transport array
    d_t_t = np.reshape([-t_T_d[:,3]],(3,1))
    #get the transport matrix
    d_T_t = np.hstack((d_R_t,d_t_t))

    position_depth = [position_depth[0], position_depth[1], position_depth[2],1]
    
    #if m.pi/2 < heading < m.pi*3 / 2:
    #    position_depth = [-position_depth[0], position_depth[1], -position_depth[2],1]
    
    # transport 
    position_tracking = d_T_t.dot(position_depth)

    ## step 4
    position_current[0] = position_tracking[0] + pose_data.translation.x # negative change axis
    position_current[1] = position_tracking[1] + pose_data.translation.y
    position_current[2] = position_tracking[2] + pose_data.translation.z # negative change axis
    '''
    print('vehicle_heading',heading*180/m.pi)
    print('location', pose_data.translation)
    print('depth',depth)
    print('LOS', theta*180/m.pi)
    print('step1_position_body',position_body)
    print('step2_position_depth',position_depth)
    print('step3_position_tracking',position_tracking)
    print('step4_target_position_current',position_current)
    print('vehicle_current_pose', pose_data.translation)
    '''
    return float(position_current[0]), float(position_current[1]), float(-position_current[2])

##############################################
def relative_position (pose_data, goal):
    '''
    current_position: WGS84 or Cartesian corrdinates (1.1 = 1.1m)
    with put in dictionary file
    the type of goal is [0, 0, 0]
    the type of current_position is [current_position.x, current_position.y, current_position.z]
    current_position form T265, x is foward,z: right is positive, left is negative
    result of angle: positive x axis is 0, 
    315 degree located dimension 4
    # 2*m.pi - angle
    # -angle : change to direction to clockwise
    # 2*m.pi  change to positive number
    # https://www.ros.org/reps/rep-0103.html # axis definition
    '''
    
    # heading from T265
    # https://github.com/np.array(IntelRealSense/librealsense/blob/master/wrappers/python/examples/t265_rpy.py
    w = pose_data.rotation.w
    x = -pose_data.rotation.z
    y = pose_data.rotation.x
    z = -pose_data.rotation.y
    yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z)
    # yaw -180~0(left)0~180(right) for change to 0~360
    if yaw <= 0.:
        yaw = yaw + 2*m.pi
    #print('yaw',yaw*180/m.pi)
    
    # relative angle and relative distance
    # https://github.com/IntelRealSense/librealsense/issues/5178#issuecomment-549795232
    # https://www.ros.org/reps/rep-0103.html
    current_position = pose_data.translation
    del_x = goal[0] - current_position.x
    del_y = goal[1] - current_position.y
    del_z = goal[2] + current_position.z
    #print('current position:',[current_position.x, current_position.y, current_position.z])
    #print('vector',[del_x,del_y, del_z]) 
    
    distance = np.sqrt(del_x*del_x + del_z*del_z)

    if del_z == 0:
        if del_x > 0:
            angle = m.pi/2
        elif del_x < 0:
            angle = m.pi*3/2
        else:
            angle = 0
    else:
        angle = m.atan2(del_x, del_z) # get the -pi ~ pi

        # theta change to 0~360
        if angle <= 0:
            angle = angle + 2*m.pi

    return angle, distance, yaw

def check_inroute(pose_data, current_goal, point):
    '''
    check the point is in route which is from current postion to point
    pad,noaction_distance = +-0.5m 
    '''
    # print('pose_data',pose_data.translation)
    # print('current_goal', current_goal)
    # print('point', point)
    # print('coordi_edge',coordi_edge)
    # eliminate out of x 
    if point[0] > max(pose_data.translation.x, current_goal[0]) + distance_limit or point[0] <  min(pose_data.translation.x, current_goal[0]) - distance_limit:
        # print('max',max(pose_data.translation.x, current_goal[0]) + noaction_distance)
        # print('min',min(pose_data.translation.x, current_goal[0]) - noaction_distance)
        print('elim x')
        return 'out_route'
    elif point[2] > max(-pose_data.translation.z, current_goal[2]) + distance_limit or point[2] <  min(-pose_data.translation.z, current_goal[2])- distance_limit:
        #print('current_z, goal', -pose_data.translation.z, current_goal[2])

        print('elim z')
        return 'out_route'
    else:
        #print('inroute', point)
        return 'in_route'
        
###########################################3
## movement function 
def object_avoid(pixel_edge, pixel_center, target_depth, width, pose_data):
    '''
    turn direction form the picture frame
    |Quick left|left       |right             |/center/|left              |right      |quick_right| # turn direction
    |width/2   |pixel_bound|offset|pixel_bound|/center/|pixel_bound|offset|pixel_bound|width/2    |  # if difference is
    head_offset = width / 2 = 320 
    pixel_bound = 50 
    '''
    difference_edge = frame_center[0] - pixel_edge[0]
    difference_center = frame_center[0] - pixel_center[0]
    
    if difference_center * difference_edge > 0: # locate same side(target_edge and target center)
        print('midturn')
        difference = difference_edge
        turn_rate =  turn_max - turn_min
    else: # pixel_center and edge located other
        difference = difference_center
        print('maxturn')
        turn_rate = turn_max

    if difference > 0: # target located left
        if 0 < difference <= head_offset - pixel_bound: # 0< <270
            print('difference',difference,'head_offset',head_offset)
            print('turn_right')
            set_rc_channel_pwm(neutral + turn_rate,set_speed)
            located = 'left'
        
        elif head_offset - pixel_bound < difference <= head_offset + pixel_bound:
            print('difference',difference,'head_offset',head_offset)
            print('left_reverse1')
            set_rc_channel_pwm(turn_left,set_speed) # left_turn reverse

        elif head_offset + pixel_bound < difference:
            print('difference',difference,'head_offset',head_offset)
            print('avoid_point_left')
            relative_avoid(pose_data, avoid_clear, goal, ii)
            
    else: # target located center or right
        if 0 < -difference <= head_offset - pixel_bound:
            print('difference',difference,'head_offset',head_offset)
            print('turn_left')
            located = 'right'
            set_rc_channel_pwm(neutral - turn_rate,set_speed)
        
        elif head_offset - pixel_bound < -difference <= head_offset + pixel_bound:
            print('difference',difference,'head_offset',head_offset)
            print('right_reverse1')
            set_rc_channel_pwm(turn_right,set_speed) # right_turn reverse

        elif head_offset + pixel_bound < -difference:
            print('difference',difference,'head_offset',head_offset)
            print('avoid_point_right')
            relative_avoid(pose_data, avoid_clear, goal, ii)

def relative_avoid(pose_data, avoid_point, goal, ii):
    '''
    when the vehicle passed the 'z' coordinate of the target
    the vehicle goes to next goal

    '''
    print('vehicle go to : ',avoid_point)
    rel_angle, rel_distance, yaw = relative_position(pose_data, avoid_point)
    turn_angle = rel_angle - yaw

    # for make turn_angle 0~360
    if turn_angle <= 0:
        turn_angle = turn_angle + 2*m.pi
    
    # avoid point located front current position
    if avoid_point[2] > pose_data.translation.z:
        if turn_angle == 0:#go to straight
            print('positive_straight')
            set_rc_channel_pwm(neutral, set_speed)
            
            # when reched avoid point
            if avoid_point[2] < pose_data.translation.z:
                print('avoid_clear')
                ii = route_move (pose_data, goal, ii)

        elif 0< turn_angle <= m.pi: #turn = right_turn           
            print('positive_right_turn')
            turn = neutral + smooth_turn(turn_angle) 
            set_rc_channel_pwm(turn, set_speed)

            # when reched avoid point
            if avoid_point[2] < pose_data.translation.z:
                print('avoid_clear1')
                ii = route_move (pose_data, goal, ii)
        
        else: #turn = left_turn
            turn = neutral - smooth_turn(turn_angle) 
            set_rc_channel_pwm(turn, set_speed)
            print('positive_left_turn')
            # when reched avoid point
            if avoid_point[2] < pose_data.translation.z:
                print('avoid_clear2')
                ii = route_move (pose_data, goal, ii)

    elif avoid_point[2] < pose_data.translation.z:
        if turn_angle == 0:#go to straight
            set_rc_channel_pwm(neutral, set_speed)
            print('negative_straight')
            
            # when reched avoid point
            if avoid_point[2] > pose_data.translation.z:
                print('avoid_clear3')
                ii = route_move (pose_data, goal, ii)

        elif 0< turn_angle <= m.pi: #turn = right_turn
            turn = neutral + smooth_turn(turn_angle) 
            set_rc_channel_pwm(turn, set_speed)
            print('negative_right')
            # when reched avoid point
            if avoid_point[2] > pose_data.translation.z:
                print('avoid_clear4')
                ii = route_move (pose_data, goal, ii)
        
        else: #turn = left_turn
            turn = neutral - smooth_turn(turn_angle) 
            set_rc_channel_pwm(turn, set_speed)
            print('negative_LEFT')
            # when reched avoid point
            if avoid_point[2] > pose_data.translation.z:
                print('avoid_clear5')
                ii = route_move (pose_data, goal, ii)

    else:
        print('both not')
        ii = route_move (pose_data, goal, ii)



def relative_move (pose_data, goal):
    """
    current_position: WGS84 or Cartesian corrdinates (1.1 = 1.1m)
    with put in dictionary file
    goal is [x, y, z]
    haeding with radian units, 0 ~ 2*pi, 360 is north
    main is left turn,
    left turn 991, neutral = 1453, right turn =1965
    goal_dist = 0.3

    """
    #print(pose_data.rotation.w)
    print('vehicle go to : ',goal)
    rel_angle, rel_distance, yaw = relative_position(pose_data, goal)
    turn_angle = rel_angle - yaw
    # for make turn_angle 0~360
    if turn_angle <=0:
        turn_angle = turn_angle + 2*m.pi
    #print('turn_angle', turn_angle*180/m.pi)
    
    #print('rel_distance',rel_distance)
    if rel_distance > goal_dist:
        if turn_angle == 0:
            set_rc_channel_pwm(neutral, set_speed)
            #print('rel_angle=0',turn_angle*180/m.pi)
            if rel_distance < goal_dist:
                #print('stop')
                set_rc_channel_pwm(neutral, stop_speed)

        elif 0< turn_angle <=m.pi:
            #turn = right_turn
            turn = neutral + smooth_turn(turn_angle) 
            #print('rel_angle<pi',turn_angle*180/m.pi)
            set_rc_channel_pwm(turn, set_speed)
            if rel_distance < goal_dist:
                #print('stop')
                set_rc_channel_pwm(neutral, stop_speed)

        else:
            #turn = left_turn
            turn = neutral - smooth_turn(turn_angle) 
            #print('else',turn_angle*180/m.pi)
            set_rc_channel_pwm(turn, set_speed)
            if rel_distance < goal_dist:
                #print('stop')
                set_rc_channel_pwm(neutral, stop_speed)

    else:
        set_rc_channel_pwm(neutral, stop_speed)


def route_move (pose_data, goal, ii):
    """
    current_position: WGS84 or Cartesian corrdinates (1.1 = 1.1m)
    with put in dictionary file
    goal is [x, y, z]
    haeding with radian units, 0 ~ 2*pi, 360 is north
    main is left turn,
    left turn 991, neutral = 1453, right turn =1965
    goal_dist = 0.5

    """
    #print(pose_data.rotation.w)
    #print('ii',ii)
    current_goal = goal[ii]
    #print('current goal', current_goal)
    home = len(goal)
    rel_angle, rel_distance, yaw = relative_position(pose_data, current_goal)
    turn_angle = rel_angle - yaw
    #print('rel_angle',rel_angle*180/m.pi)

    # for make turn_angle 0~360
    if turn_angle <=0:
        turn_angle = turn_angle + 2*m.pi
    #print('turn_angle', turn_angle*180/m.pi)
    #print('goal distance: ', goal_dist)
    #print('rel_distance',rel_distance)
    #print('goal distance_type: ', type(goal_dist))
    #print('rel_distance_type',type(rel_distance))
    if rel_distance > goal_dist:
        if turn_angle == 0:
            set_rc_channel_pwm(neutral, set_speed)
            #print('rel_angle=0',turn_angle*180/m.pi)
            if rel_distance < goal_dist:
                set_rc_channel_pwm(neutral, stop_speed)

        elif 0< turn_angle <=m.pi:
            #turn = right_turn
            turn = neutral + smooth_turn(turn_angle) 
            #print('rel_angle<pi',turn_angle*180/m.pi)
            set_rc_channel_pwm(turn, set_speed)
            if rel_distance < goal_dist:
                #print('stop')
                set_rc_channel_pwm(neutral, stop_speed)

        else:
            #turn = left_turn
            turn = neutral - smooth_turn(turn_angle) 
            #print('else',turn_angle*180/m.pi)
            set_rc_channel_pwm(turn, set_speed)
            if rel_distance < goal_dist:
                #print('stop')
                set_rc_channel_pwm(neutral, stop_speed)

    else:
        set_rc_channel_pwm(neutral, stop_speed)
        
        # when reached the goal change to next goal
        if ii != home:
            ii=ii+1
            print('reaching ',ii-1, 'go to', ii)
    return ii

def smooth_turn (rel_angle):

    '''
    set the max and min neutral=1453, min = 1200(left), Max = 1800(right)
    left turn 991, neu = 1453, right turn =1965
    return 100 ~ 300
    '''
    if rel_angle <= 30/180*m.pi:
        #print('min')
        return (turn_min)
    elif rel_angle <= 60/180*m.pi:
        #print('mid')
        return (turn_max-turn_min)
    elif rel_angle < 300/180*m.pi:
        return (turn_max)
        #print('max')
    elif rel_angle < 330/180*m.pi:
        #print('mid')
        return (turn_max-turn_min)
    elif rel_angle < 360/180*m.pi:
        #print('min')
        return (turn_min)

def simple_object_avoid(pixel_edge, pixel_center, pose_data, current_goal, avoid_clear, coordi_edge):
    '''
    turn direction form the picture frame
    |Quick left|left       |right             |/center/|left              |right      |quick_right| # turn direction
    |width/2   |pixel_bound|offset|pixel_bound|/center/|pixel_bound|offset|pixel_bound|width/2    |  # if difference is
    head_offset = width / 2 = 320 
    pixel_bound = 50 
    '''
    
    difference_edge = frame_center[0] - pixel_edge[0]
    difference_center = frame_center[0] - pixel_center[0]

    if difference_center * difference_edge > 0: # locate same side(target_edge and target center)
        difference = difference_edge
        turn_rate =  turn_max
    else: # pixel_center and edge located other
        difference = difference_center
        turn_rate = turn_max

    if difference > 0: # target located left
        if 0 < difference <= head_offset - pixel_bound: 
            print('difference',difference,'head_offset',head_offset)
            print('turn_right')
            set_rc_channel_pwm(neutral + turn_rate,set_speed)
            if check_inroute(pose_data, current_goal, coordi_edge) == 'in_route':
                print('avoid_right')
                print('avoid_edge coordi',avoid_clear)
                relative_avoid(pose_data, avoid_clear, goal, ii)
            
    else: # target located center or right
        if 0 < -difference <= head_offset - pixel_bound:
            print('difference',difference,'head_offset',head_offset)
            print('turn_left')
            located = 'right'
            set_rc_channel_pwm(neutral - turn_rate,set_speed)
            if check_inroute(pose_data, current_goal, coordi_edge) == 'in_route':
                print('avoid_left')
                print('avoid_edge coordi',avoid_clear)

                relative_avoid(pose_data, avoid_clear, goal, ii)
        
        
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
config.enable_device('040322073813') #D435i s/n 040322073813


####point cloud setting
pipeline_wrapper = rs.pipeline_wrapper(pipe)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()            

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

# Get device product line for setting a supporting resolution
config.enable_stream(rs.stream.depth, depth_pixel[0], depth_pixel[1], rs.format.z16, 30)
config.enable_stream(rs.stream.color, depth_pixel[0], depth_pixel[1], rs.format.bgr8, 30)

####################################
# start time samp
t=time.gmtime()
date = date.today()
current_time = time.strftime("%H:%M:%S", t)
print("Script Start: ", current_time)

####################################

# save images setup
# have to put in after the pipeline start
# for save date make a directory
time_path = './data/'+str(st_goal)+str(current_time)
left_path = str(time_path)+'/tracking_camera/left'
right_path = str(time_path)+'/tracking_camera/right'
depth_path = str(time_path)+'/depth_camera/depth'
pointcloud_path = str(time_path)+'/depth_camera/pointcloud'
pointcloudPly_path = str(time_path)+'/depth_camera/pointcloudply'

os.makedirs(time_path)
os.makedirs(left_path)
os.makedirs(right_path)
os.makedirs(depth_path)
os.makedirs(pointcloud_path)
os.makedirs(pointcloudPly_path)

# # save the video
# config.enable_record_to_file(str(time_path)+'/video.bag')

###########################################################


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



#######################################################
## D435i get initial color frame
frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
color_init = np.asanyarray(color_frame.get_data())      

## T265
# Wait for a coherent pair of frames
frames2 = pipe2.wait_for_frames()

# get the position data
pose = frames2.get_pose_frame()
pose_data = pose.get_pose_data()
pose_init = pose_data
#print(pose_init)

#### point cloud
### point cloud function
class AppState:

    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        # camera view setting
        #self.pitch, self.yaw = m.radians(-10), m.radians(-15)
        self.pitch, self.yaw = m.radians(-90), m.radians(-0) # top view
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)


state = AppState()
# Get stream profile and camera intrinsics
profile_stream = pipe.get_active_profile()
depth_profile = rs.video_stream_profile(profile_stream.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()
w, h = depth_intrinsics.width, depth_intrinsics.height

# Processing blocks
pc = rs.pointcloud()
decimate = rs.decimation_filter()
decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
colorizer = rs.colorizer()
############################################################

def mouse_cb(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDOWN:
        state.mouse_btns[0] = True

    if event == cv2.EVENT_LBUTTONUP:
        state.mouse_btns[0] = False

    if event == cv2.EVENT_RBUTTONDOWN:
        state.mouse_btns[1] = True

    if event == cv2.EVENT_RBUTTONUP:
        state.mouse_btns[1] = False

    if event == cv2.EVENT_MBUTTONDOWN:
        state.mouse_btns[2] = True

    if event == cv2.EVENT_MBUTTONUP:
        state.mouse_btns[2] = False

    if event == cv2.EVENT_MOUSEMOVE:

        h, w = out.shape[:2]
        dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]

        if state.mouse_btns[0]:
            state.yaw += float(dx) / w * 2
            state.pitch -= float(dy) / h * 2

        elif state.mouse_btns[1]:
            dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
            state.translation -= np.dot(state.rotation, dp)

        elif state.mouse_btns[2]:
            dz = m.sqrt(dx**2 + dy**2) * m.copysign(0.01, -dy)
            state.translation[2] += dz
            state.distance -= dz

    if event == cv2.EVENT_MOUSEWHEEL:
        dz = m.copysign(0.1, flags)
        state.translation[2] += dz
        state.distance -= dz

    state.prev_mouse = (x, y)


cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow(state.WIN_NAME, w, h)
cv2.setMouseCallback(state.WIN_NAME, mouse_cb)


def project(v):
    """project 3d vector array to 2d"""
    h, w = out.shape[:2]
    view_aspect = float(h)/w

    # ignore divide by zero for invalid depth
    with np.errstate(divide='ignore', invalid='ignore'):
        proj = v[:, :-1] / v[:, -1, np.newaxis] * \
            (w*view_aspect, h) + (w/2.0, h/2.0)

    # near clipping
    znear = 0.03
    proj[v[:, 2] < znear] = np.nan
    return proj


def view(v):
    """apply view transformation on vector array"""
    return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation


def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
    """draw a 3d line from pt1 to pt2"""
    p0 = project(pt1.reshape(-1, 3))[0]
    p1 = project(pt2.reshape(-1, 3))[0]
    if np.isnan(p0).any() or np.isnan(p1).any():
        return
    p0 = tuple(p0.astype(int))
    p1 = tuple(p1.astype(int))
    rect = (0, 0, out.shape[1], out.shape[0])
    inside, p0, p1 = cv2.clipLine(rect, p0, p1)
    if inside:
        cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
    """draw a grid on xz plane"""
    pos = np.array(pos)
    s = size / float(n)
    s2 = 0.5 * size
    for i in range(0, n+1):
        x = -s2 + i*s
        line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
               view(pos + np.dot((x, 0, s2), rotation)), color)
    for i in range(0, n+1):
        z = -s2 + i*s
        line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
               view(pos + np.dot((s2, 0, z), rotation)), color)


def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
    """draw 3d axes"""
    line3d(out, pos, pos +
           np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
    line3d(out, pos, pos +
           np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
    line3d(out, pos, pos +
           np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
    """draw camera's frustum"""
    orig = view([0, 0, 5])
    w, h = intrinsics.width, intrinsics.height

    for d in range(1, 6, 2):
        def get_point(x, y):
            p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
            line3d(out, orig, view(p), color)
            return p

        top_left = get_point(0, 0)
        top_right = get_point(w, 0)
        bottom_right = get_point(w, h)
        bottom_left = get_point(0, h)

        line3d(out, view(top_left), view(top_right), color)
        line3d(out, view(top_right), view(bottom_right), color)
        line3d(out, view(bottom_right), view(bottom_left), color)
        line3d(out, view(bottom_left), view(top_left), color)


def pointcloud(out, verts, texcoords, color, painter=True):
    """draw point cloud with optional painter's algorithm"""
    if painter:
        # Painter's algo, sort points from back to front

        # get reverse sorted indices by z (in view-space)
        # https://gist.github.com/stevenvo/e3dad127598842459b68
        v = view(verts)
        s = v[:, 2].argsort()[::-1]
        proj = project(v[s])
    else:
        proj = project(view(verts))

    if state.scale:
        proj *= 0.5**state.decimate

    h, w = out.shape[:2]

    # proj now contains 2d image coordinates
    j, i = proj.astype(np.uint32).T

    # create a mask to ignore out-of-bound indices
    im = (i >= 0) & (i < h)
    jm = (j >= 0) & (j < w)
    m = im & jm

    cw, ch = color.shape[:2][::-1]
    if painter:
        # sort texcoord with same indices as above
        # texcoords are [0..1] and relative to top-left pixel corner,
        # multiply by size and add 0.5 to center
        v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
    else:
        v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
    # clip texcoords to image
    np.clip(u, 0, ch-1, out=u)
    np.clip(v, 0, cw-1, out=v)

    # perform uv-mapping
    out[i[m], j[m]] = color[u[m], v[m]]


out = np.empty((h, w, 3), dtype=np.uint8)
initial_vector, initial_distance, initial_heading = relative_position(pose_init, goal[0])
############################################################################3
try:
    

    while True:
        # Skip 5 first frames to give the Auto-Exposure time to adjust
        for x in range(5):
            pipe.wait_for_frames()
        
        # print frame number
        print(iii)

        # time set
        t=time.gmtime()
        date = date.today()
        current_time = time.strftime("%H:%M:%S", t)

        # for vehicle activated mode print
        mode = vehicle.mode
        print(mode)
        
        ## T265 
        # update new frame
        frames2 = pipe2.wait_for_frames()
        config2.enable_stream(rs.stream.fisheye, 1)
        config2.enable_stream(rs.stream.fisheye, 2)
        
        # left and right camera setting for video
        left = frames2.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())
        right = frames2.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())
                
        # grab new position
        pose = frames2.get_pose_frame()
        # check pose is not empty
        if pose:
        # Print some of the pose data to the terminal
            pose_data = pose.get_pose_data()

        ## D435i
        # Wait for a coherent pair of frames: depth and color
        frames = pipe.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        
        # depth filter
        # https://github.com/IntelRealSense/librealsense/blob/jupyter/notebooks/depth_filters.ipynb
        # delare
        decimation = rs.decimation_filter()
        spatial = rs.spatial_filter()
        temporal = rs.temporal_filter()
        hole_filling = rs.hole_filling_filter()
        depth_to_disparity = rs.disparity_transform(True)
        disparity_to_depth = rs.disparity_transform(False)

        #filter processing
        #filtered_depth = decimation.process(depth_frame)
        filtered_depth = depth_to_disparity.process(depth_frame)
        filtered_depth = spatial.process(filtered_depth)
        filtered_depth = temporal.process(filtered_depth)
        filtered_depth = disparity_to_depth.process(filtered_depth)
        filtered_depth = hole_filling.process(filtered_depth)
        
        color_frame = frames.get_color_frame()
        height=color_frame.get_height() #height = 480
        width=color_frame.get_width() #width = 640
        pixel_degree = (width/2) * fov / 2
        height1=depth_frame.get_height()#height = 480
        width1=depth_frame.get_width()  #width = 640
        
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        ## point cloud
        if not state.paused:
            depth_frame_point = decimate.process(depth_frame)  
            # Grab new intrinsics (may be changed by decimation)
            depth_intrinsics = rs.video_stream_profile(
            depth_frame_point.profile).get_intrinsics()
            w, h = depth_intrinsics.width, depth_intrinsics.height
            depth_image_point = np.asanyarray(depth_frame_point.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_colormap_point = np.asanyarray(
            colorizer.colorize(depth_frame_point).get_data())

            if state.color:
                mapped_frame, color_source = color_frame, color_image
            else:
                mapped_frame, color_source = depth_frame_point, depth_colormap_point

            points = pc.calculate(depth_frame_point)
            pc.map_to(mapped_frame)

            # Pointcloud data to arrays
            v, t = points.get_vertices(), points.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
            texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
        # Render
        now = time.time()

        out.fill(0)

        grid(out, (0, 0.5, 1), size=1, n=10)
        frustum(out, depth_intrinsics)
        axes(out, view([0, 0, 0]), state.rotation, size=0.1, thickness=1)

        if not state.scale or out.shape[:2] == (h, w):
            pointcloud(out, verts, texcoords, color_source)
        else:
            tmp = np.zeros((h, w, 3), dtype=np.uint8)
            pointcloud(tmp, verts, texcoords, color_source)
            tmp = cv2.resize(
                tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
            np.putmask(out, tmp > 0, tmp)

        if any(state.mouse_btns):
            axes(out, view(state.pivot), state.rotation, thickness=4)

        dt = time.time() - now

        cv2.setWindowTitle(
            state.WIN_NAME, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
            (w, h, 1.0/dt, dt*1000, "PAUSED" if state.paused else ""))

        ##################################################################
        # for draw depth to color frame
        depth_data = color_image.copy()

        # set the wanting color for capture
        # hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV) # capturecolor
        # l_b = np.array([24, 133, 48]) # set hsv min color
        # u_b = np.array([39, 200, 181]) # set hsv max color
        color = cv2.bitwise_and(color_image, color_image) #capture any color
        # mask = cv2.inRange(hsv, l_b, u_b)  #set the range of capture color
        # color = cv2.bitwise_and(color, color, mask=mask)  # only capture from l_b to u_b ## green color capture
        
        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())
    
        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        # Update color and depth frames:
        #aligned_depth_frame = frameset.get_depth_frame()
        # colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
        
        ### motion detector
        d = cv2.absdiff(color_init, color) # get the difference with previous depth_image
        #gray = cv2.cvtColor(color_init, cv2.COLOR_BGR2GRAY) # change to gray_image 
        gray = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY) # change to gray_image 

        # blur: https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # thresh: https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_thresholding/py_thresholding.html
        #_, th = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        # adding otsu filter
        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        dilated = cv2.dilate(th, np.ones((3, 3), np.uint8), iterations=3)
        
        #https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html
        (c, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # 4 points
        #(c, _) = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # 734 points
        # cv2.drawContours(color, c, -1, (0, 255, 0), 2)
        color_init = color
        #Get depth data array
        #depth = np.asanyarray(aligned_depth_frame.get_data())
        #depth1 = np.transpose(depth)
        
        #print('depth',depth1[320,:])
        depth = np.asanyarray(filtered_depth.get_data()) # for filter on
        #depth = np.asanyarray(depth_frame.get_data()) # for filter off

        depth = np.transpose(depth)


        ################################################################################
        ## test for line
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        depth_meter = depth * depth_scale

        # grab the 320 line
        vertical_center = int(depth_pixel[1]/2)
        grab_line = depth_meter[:,vertical_center]
        dist2 = min(grab_line[grab_line!=0])
        
        #grab_line2 = depth_meter[:,vertical_center+20]
        # print('dist2',dist2)
        # print('new axis',grab_line)
        # grab the depth min 90percent 
        percent = 0.2
        min_percent = 1 - percent
        max_percent = 1 + percent
        depth_line = np.where((grab_line > min_percent*dist2) & (grab_line < max_percent*dist2))

        #depth_line2 = np.where((grab_line2<3.000) & (grab_line2>.500))
        if len(depth_line) != 0:

            # calculate depth min of the vertical center line
            dist1 = min(grab_line[grab_line!=0])
            
            # for the except the ground
            l_fov = dist1 * m.tan(29/180*m.pi)
            pixel_ground = int((depth_pixel[1]/2) + 0.22 / l_fov * (depth_pixel[1]/2))
            if pixel_ground > 479:
                pixel_ground = 479
            # print('pixel_ground', pixel_ground)
            ##############################################
            x1=640
            w1=0
            for y2 in range (vertical_center, pixel_ground):
                grab_line1 = depth_meter[:,y2]
                depth_line1 = np.where((grab_line1 > min_percent*dist2) & (grab_line1 < max_percent*dist2))
                x1 = min(x1, depth_line[0][0])
                w1 = max(w1, len(depth_line[0]))
            ##############################################
            y2=480
            h2=0
            for x2 in range (x1,x1+w1):
                grab_Vline1 = depth_meter[int(x2),0:pixel_ground]
                depth_Vline1 = np.where((grab_Vline1 > min_percent*dist2) & (grab_Vline1 < max_percent*dist2))
                #print('depth',grab_Vline1[depth_Vline1])
                if len(depth_Vline1[0]) != 0:
                    y2 = min(y2, depth_Vline1[0][0])
                    h2 = max(h2, len(depth_Vline1[0]))
            #grab_Vline = depth_meter[int(h2),0:pixel_ground]
            ############################################



            # grab_Vline = depth_meter[int(x1+w1/2),0:pixel_ground]
            # depth_Vline = np.where((grab_Vline > min_percent*dist2) & (grab_Vline < max_percent*dist2))
            # print('depth_Vline',depth_Vline)
            
            # y1 = depth_Vline[0][0]
            # h1 = len(depth_Vline[0])
            y1 = y2
            h1 = h2
            # print('depth_line1',depth_line1)
            # print('depth_line2',depth_line2)
            cv2.rectangle(depth_data, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 3)

            
            bottomLeftCornerOfText1 = (x1, y1+h1)
            
            text1 = "Depth: " + str("{0:.2f}").format(dist1)
            cv2.putText(depth_data,
                        text1,
                        bottomLeftCornerOfText1,
                        font,
                        fontScale,
                        fontColor,
                        lineType)
            
        ######################################################################################
        '''
        return procedure (un sharp #)
        #(x, y, w, h) = cv2.boundingRect(contour)  # draw box
        #cv2.rectangle(depth_data, (x, y), (x + w, y + h), (0, 255, 0), 3) # draw target rectangle 
        '''
        ##############################################################
        i = i+1 
        for contour in c:
            if cv2.contourArea(contour) < 1500:
                continue
            #(Cx,Cy), radius = cv2.minEnclosingCircle(contour) # draw circle
            (x, y, w, h) = cv2.boundingRect(contour)  # draw box
            bottomLeftCornerOfText = (x, y)

            # get the target center
            #Cx = (x + w) / 2.0
            #Cy = (y + ofh) / 2.0
            
            #center = (int(Cx), int(Cy))
            #radius = int(radius)
            # Crop depth data:
            depth = depth[x:x+w, y:y+h].astype(float)
            
            depth_crop = depth.copy()

            if depth_crop.size == 0:
                continue
            #print(depth_crop)
            depth_res = depth_crop[depth_crop != 0] #catch value when the depth_crop is not 0
            #print(depth_res)
            # Get data scale from the device and convert to meters
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            depth_res = depth_res * depth_scale
            #print('depth_res',depth_res)
            if depth_res.size == 0:
                continue
            
            dist = min(depth_res[depth_res!=0])
            #print(dist)

            #cv2.circle(depth_data, center, radius,(0,255,0),2) # draw target circle
            cv2.rectangle(depth_data, (x, y), (x + w, y + h), (0, 255, 0), 3) # draw target rectangle 
            text = "Depth: " + str("{0:.2f}").format(dist)
            cv2.putText(depth_data,
                        text,
                        bottomLeftCornerOfText,
                        font,
                        fontScale,
                        fontColor,
                        lineType)
            '''
            ## put Text left top
            cv2.putText(depth_data,text, [80, 30], font, fontScale, fontColor, lineType)
            '''
            #initializing autopilot control variables
            frame_center = (width/2,height/2)
            desired_pixel = frame_center

            pixel_center = [x1 +w1/2,y1+h1/2] 
            target_depth = dist1     
            pixel_left = [x1, y1]
            pixel_right = [x1 + w1, y1]
                
            # print(coordi_left,'coordi_left')
            # print(coordi_avoid_left, ':coordi_avoid_left')
            # print(coordi_right,'coordi_right')
            # print(coordi_avoid_right, ':coordi_avoid_right')
            
            
        ###########################################
        ########### main code
        ##################################
            
        if mode == 'MANUAL':
        # if mode == 'HOLD':
            #print(pose_data.translation)
            # goal = [0,0,3] # z is forward
            current_goal = goal[ii]

            #print('current_goal',current_goal)
            if pixel_center[0] > frame_center[0]:
                coordi_left = depth_to_tracking(pixel_left, target_depth, pose_data, width)
                _,_,yaw = relative_position(pose_data, coordi_left)
                
                # consider the length of vehicle(0.5m) and avoid 0.5m, pad=distance_limit= 0.3
                if m.pi/2. < yaw < m.pi *3/2.:
                    coordi_avoid_left = [coordi_left[0]-(0.5+distance_limit), coordi_left[1], coordi_left[2]-(0.2)]
                else:
                    coordi_avoid_left = [coordi_left[0]-(0.5+distance_limit), coordi_left[1], coordi_left[2]+(0.2)]

                pixel_edge = pixel_left
                coordi_avoid = coordi_avoid_left
                coordi_edge = coordi_left
                # print('pixel_edge',pixel_edge)
                # print('coordi_avoid',coordi_avoid)
                # print('coordi_edge',coordi_edge)
            else:
                coordi_right = depth_to_tracking(pixel_right, target_depth, pose_data, width)
                _,_,yaw = relative_position(pose_data, coordi_right)

                # consider the length of vehicle(0.5m) and avoid 0.5m
                if m.pi/2. < yaw < m.pi *3/2.:
                    coordi_avoid_right = [coordi_right[0]+(0.5+distance_limit), coordi_right[1], coordi_right[2]-(0.2)]
                else:
                    coordi_avoid_right = [coordi_right[0]+(0.5+distance_limit), coordi_right[1], coordi_right[2]+(0.2)]

                pixel_edge = pixel_right               
                coordi_avoid = coordi_avoid_right
                coordi_edge = coordi_right
                # print('pixel_edge',pixel_edge)
                # print('coordi_avoid',coordi_avoid)
                # print('coordi_avoid',coordi_edge)

            
            # get the target coordinate
            tx, ty, tz = depth_to_tracking (pixel_center, target_depth, pose_data, width)
            ox, oy, oz = depth_to_tracking (pixel_edge, target_depth, pose_data, width)
            #print('object',[tx, ty, tz])

            # object_edge, and pre_object_edge in route, avoid both
            if check_inroute(pose_data, current_goal, coordi_edge)=='in_route' and check_inroute(pose_data, current_goal, pre_coordi_edge)=='in_route':
                pre_angle, pre_dist, pre_yaw = relative_position(pose_data, pre_coordi_edge)

                if noaction_distance < pre_dist < target_depth < desired_distance + distance_bound:
                    print('pre_coordi=inroute avoid,both_in')
                    relative_avoid(pose_data, pre_coordi_avoid, goal, ii)

                elif noaction_distance < target_depth < pre_dist < desired_distance + distance_bound:
                    ox, oy, oz = depth_to_tracking (pixel_edge, target_depth, pose_data, width)
                    tx, ty, tz = depth_to_tracking (pixel_center, target_depth, pose_data, width)

                    # stored object coordinates for loosing
                    pre_coordi_edge = coordi_edge
                    pre_coordi_avoid = coordi_avoid 
                    pre_coordi_avoid_depth = target_depth
                    pre_target_center = [tx,ty,tz]

                    print('coordi_avoid', pre_coordi_avoid)
                    simple_object_avoid(pixel_edge, pixel_center, pose_data, current_goal, pre_coordi_avoid, coordi_edge)
                    
                    print('avoid','pixel_center',pixel_center,'pixel_edge', pixel_edge,'depth\n' ,target_depth,'position' ,pose_data.translation,'coordi_edge', coordi_edge)
                    print('target_codination',[tx,ty,tz])

                else:
                    ii = route_move (pose_data, goal, ii)

            # object_edge and pre_stored object_edge out of route 
            elif check_inroute(pose_data, current_goal, coordi_edge)=='out_route' and check_inroute(pose_data, current_goal, pre_coordi_edge)=='out_route':
                #print('current_position',pose_data.translation.x, pose_data.translation.y, pose_data.translation.z)
                #print('target coordinate',tx,ty,tz)
                print('both out route')
                ii = route_move (pose_data, goal, ii)                    
                [tx, ty, tz] = [0, 0, 0]
                [ox, oy, oz] = [0, 0, 0]
        
            # object_edge out route, pre_object_edge in route, avoid pre_target
            elif check_inroute(pose_data, current_goal, coordi_edge)=='out_route' and check_inroute(pose_data, current_goal, pre_coordi_edge)=='in_route':
                pre_angle, pre_dist, pre_yaw = relative_position(pose_data, pre_coordi_edge)
                [tx, ty, tz] = [0, 0, 0]
                [ox, oy, oz] = [0, 0, 0]
                if noaction_distance < pre_dist < desired_distance + distance_bound:
                    print('pre_coordi=inroute avoid,new_out, old_in')
                    relative_avoid(pose_data, pre_coordi_avoid, goal, ii)
                else:
                    ii = route_move (pose_data, goal, ii)
            



            # avoid new target
            else:    
                if target_depth <= 2.5:
                    print('in range')
                    print('traget_depth',target_depth)
                    # avoid new target
                    if noaction_distance < target_depth < desired_distance + distance_bound:
                        ox, oy, oz = depth_to_tracking (pixel_edge, target_depth, pose_data, width)
                        tx, ty, tz = depth_to_tracking (pixel_center, target_depth, pose_data, width)

                        # stored object coordinates for loosing
                        pre_coordi_edge = coordi_edge
                        pre_coordi_avoid = coordi_avoid 
                        pre_coordi_avoid_depth = target_depth
                        pre_target_center = [tx,ty,tz]

                        print('coordi_avoid', pre_coordi_avoid)
                        simple_object_avoid(pixel_edge, pixel_center, pose_data, current_goal, pre_coordi_avoid, coordi_edge)
                        
                        print('avoid','pixel_center',pixel_center,'pixel_edge', pixel_edge,'depth\n' ,target_depth,'position' ,pose_data.translation,'coordi_edge', coordi_edge)
                        print('target_codination',[tx,ty,tz])

                    else:
                        ii = route_move (pose_data, goal, ii)

                # target greater than 3m
                else :
                    print('target out?')
                    ii = route_move (pose_data, goal, ii)
            
            coordi_center = [tx, ty, tz]
            coordi_edge = [ox, oy, oz]
        
                    
        ######################################################
        
        
        ######################################################
        # Show images
        
        
        cv2.imshow(WINDOW_TITLE2, left_data)
        cv2.imshow(state.WIN_NAME, out)
        cv2.imshow(WINDOW_TITLE1, depth_data)


        #############################################3
        
        ## save images
        cv2.imwrite('%s/%d.jpg' % (left_path, i), left_data)
        cv2.imwrite('%s/%d.jpg' % (right_path, i), right_data)
        cv2.imwrite('%s/%d.jpg' % (depth_path, i), depth_data)
        cv2.imwrite('%s/%d.jpg' % (pointcloud_path, i), out)
        points.export_to_ply('%s/%d.ply'% (pointcloudPly_path, i), mapped_frame)
        
        ## save data
        small_data = {
            'time' : current_time,
            'position': [pose_data.translation.x, pose_data.translation.y, pose_data.translation.z],
            'Velocity':[pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z],
            'Acceleration': [pose_data.acceleration.x, pose_data.acceleration.y, pose_data.acceleration.z],
            #'depth' : target_depth,
            'vehicle_heading': vehicle.heading,
            'goal' : current_goal,
            'object' : [coordi_center, target_depth],
            'object_edge' : [coordi_edge, target_depth]
            }
        result_data.append(small_data)
        data_file = open(str(time_path)+'/result_data.json',"w")
        json.dump(result_data, data_file)
        data_file.close()

        ## save videos
        
        
        ###############################################################
        ################################################
        key = cv2.waitKey(1)
        iii = iii+1
        print('\n')


finally:

    # Stop streaming
    pipe.stop()
    pipe2.stop()