import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2 as rs
import argparse
import imutils
import sys
import os
from datetime import date
from Xlib.display import Display

from serial import Serial
import serial           #test without this import, may be used in the usb input
import pymavlink        #test without this import
from pymavlink import mavutil
import math
import time
from pymavlink.dialects.v20 import common as mavlink2

t=time.gmtime()
date = date.today()
current_time = time.strftime("%H:%M:%S", t)

#os.mkdir("~/Desktop/Logfiles/SENG550_Group2_test_"+str(date)+str(t))

print("Script Start: ", current_time)

##Window Size setup
screen = Display(':0').screen()                                        #???

#Variables to initialize:
i = 0
ii = 0
imageNum = 0
camera_initialize = 0
dist_aruco = 0
velocity_old = 0
#User inputs for control and tuning:
set_speed = 0.1
command_rate = 0
#target_orientation = 'vertical'     #comment one out
target_orientation = 'horizontal'  #comment one out
center_dist_bound = 50 #100 #last tested #pixels from center, defines x-y position tolerance
distance_bound = 0.2 #meters from target, defines distance tolerance
desired_distance = 3.0 #distance from desired target in meters

####### Setup for ArUCo detection
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_ARUCO_ORIGINAL",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())


# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

'''
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
'''

# verify that the supplied ArUCo tag exists and is supported by OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(
		args["type"]))
	sys.exit(0)

# load the ArUCo dictionary and grab the ArUCo parameters
#print("[INFO] detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()

#setup depth camera
pipe = rs.pipeline()
config = rs.config()
config.enable_device('101622074821') #D455 serial number 035722250373
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

'''
#setup tracking camera 
pipe2 = rs.pipeline()
config2 = rs.config()
config2.enable_device('119622110606') # D435i FPV camera 040322073813
config2.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config2.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
profile2 = pipe2.start(config2)
frameset2 = pipe2.wait_for_frames()
color_frame2 = frameset2.get_color_frame()
color_init2 = np.asanyarray(color_frame2.get_data())
font                   = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,500)
fontScale              = 1
fontColor              = (255,255,255)
lineType               = 2
'''

'''
#connecting to autopilot
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600) # baud?
master.wait_heartbeat()
'''

'''
#camera gimbal orientation function
if camera_initialize == 0:
	if target_orientation == 'vertical':
	    tilt = 4500
	    roll = 0
	    pan = 0
	    master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,1,tilt,roll,pan,0, 0, 0,mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
	    camera_initialize = 1
	elif target_orientation == 'horizontal':
	    tilt = -3000
	    roll = 0
	    pan = 0
	    master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,1,tilt,roll,pan,0, 0, 0,mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)
	    camera_initialize = 1 

'''
'''
#connecting to sitl
master = mavutil.mavlink_connection('127.0.0.1:14550')
master.wait_heartbeat()
print('connected to sitl')
'''

#initializing mode variable as GUIDED. Needed to not throw aruco detection exception
mode = 'GUIDED'  

'''
#change mode command 
#Note: as this may may lock out RC mode change, it will only be used for ground testing
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
'''

'''
#arm throttle command (not needed if already flying)
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
'''

#starting outer while loop
while True:
    #Requesting mode from autopilot
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
    color = np.asanyarray(color_frame.get_data())
    res = color.copy()
    aruco_res = res.copy()

#    frameset2 = pipe2.wait_for_frames() # pipe 2 for tracking camera
#    color_frame2 = frameset2.get_color_frame()
#    color2 = np.asanyarray(color_frame2.get_data())
#    res2 = color2.copy()
#    aruco_res2 = res2.copy()     

    master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,1,1)
    msg = master.recv_match(type = "HEARTBEAT", blocking = False)
    if msg:
	    mode = mavutil.mode_string_v10(msg)    
    #print(mode)

    #############################      
    #Aruco tag detection and processing
    #ARUCO tracking seems to track the largest ARUCO tag last for (cX,cY)
    #Store next frameset for later processing:

    color = cv2.bitwise_and(color, color)
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)
    # Update color and depth frames:
    aligned_depth_frame = frameset.get_depth_frame()
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
    #Get depth data array
    depth = np.asanyarray(aligned_depth_frame.get_data())
    #Depth array is transposed when pulled, found by Charlie and Jacob 
    depth = np.transpose(depth)
    #ArUCo Detection
    (corners, ids, rejected) = cv2.aruco.detectMarkers(aruco_res, arucoDict, parameters=arucoParams)
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
      # flatten the ArUco IDs list
      ids = ids.flatten()
      # loop over the detected ArUCo corners
      for (markerCorner, markerID) in zip(corners, ids):
          # extract the marker corners (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
          corners = markerCorner.reshape((4, 2))
          (topLeft, topRight, bottomRight, bottomLeft) = corners
          # convert each of the (x, y)-coordinate pairs to integers
          topRight = (int(topRight[0]), int(topRight[1]))
          bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
          bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
          topLeft = (int(topLeft[0]), int(topLeft[1]))
          # draw the bounding box of the ArUCo detection
          cv2.line(aruco_res, topLeft, topRight, (0, 255, 0), 2)
          cv2.line(aruco_res, topRight, bottomRight, (0, 255, 0), 2)
          cv2.line(aruco_res, bottomRight, bottomLeft, (0, 255, 0), 2)
          cv2.line(aruco_res, bottomLeft, topLeft, (0, 255, 0), 2)
          # compute and draw the center (x, y)-coordinates of the ArUco marker
          cX = int((topLeft[0] + bottomRight[0]) / 2.0)
          cY = int((topLeft[1] + bottomRight[1]) / 2.0)
          cv2.circle(aruco_res, (cX, cY), 4, (0, 0, 255), -1)
          cv2.circle(aruco_res, (640,400), 4, (0, 0, 255), -1)        
          #Get depth information inside aruco tag
          depth_aruco_x = [min(topLeft[0],bottomLeft[0],topRight[0],bottomRight[0]),max(topLeft[0],bottomLeft[0],topRight[0],bottomRight[0])]
          depth_aruco_y = [min(topLeft[1],bottomLeft[1],topRight[1],bottomRight[1]),max(topLeft[1],bottomLeft[1],topRight[1],bottomRight[1])]
          #depth pull on center point
          #depth_aruco_x = [cX,cX]
          #depth_aruco_y = [cY,cY]
          #fixed depth pull	        
          #depth_aruco_x = [cX-10,cX+10]
          #depth_aruco_y = [cY-10,cY+10]
          #print(depth_aruco_x,depth_aruco_y)
          depth_aruco = depth[depth_aruco_x,depth_aruco_y].astype(float)
          if depth_aruco.size == 0:
              continue

          depth_aruco2 = depth_aruco[depth_aruco != 0]
          depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
          depth_aruco2 = depth_aruco2 * depth_scale
          if depth_aruco2.size == 0:
            continue

          dist_aruco = min(depth_aruco2)
          #print(dist_aruco)
          cv2.rectangle(aruco_res, (depth_aruco_x[0],depth_aruco_y[0]), (depth_aruco_x[1],depth_aruco_y[1]), (0, 255, 0), 3)
          text = "Depth: " + str("{0:.2f}").format(dist_aruco)
          cv2.putText(aruco_res,
                  text,
                  (depth_aruco_x[1],depth_aruco_y[1]),
                  font,
                  fontScale,
                  fontColor,
                  lineType)
          # draw the ArUco marker ID on the frame
          cv2.putText(aruco_res, str(markerID),(topLeft[0], topLeft[1] - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
      #cv2.namedWindow('RBG', cv2.WINDOW_AUTOSIZE)
      #cv2.imshow('RBG', res)
      #cv2.namedWindow('Depth', cv2.WINDOW_AUTOSIZE)
      #cv2.imshow('Depth', colorized_depth)
  
      #cv2.namedWindow('ArUCo', cv2.WINDOW_AUTOSIZE)
      #cv2.imshow('ArUCo',aruco_res)
      #cv2.waitKey(1) #cv2.waitKey() vital line of code, if zero takes stills and moves on button press
      ###########################END of ARUCO   
     
      
      #initializing autopilot control variables
      frame_center = (640,400)
      fov = (45,32.5)
      desired_pixel = frame_center
      yaw_depth1 = depth[frame_center[0]-100, frame_center[1]]
      yaw_depth2 = depth[frame_center[0]+100, frame_center[1]]
      target_center = [cX,cY] #ARUCO tracking seems to track the largest ARUCO tag last for (cX,cY)
      target_depth = dist_aruco #depth at target location
      print(target_center, ':target center')
      print(target_depth, ':target depth')

      #setting a faster movement speed while target is farther away from center        
      movement_speed = set_speed
      if abs(target_center[0] - frame_center[0]) > 225:
          movement_speed=movement_speed*3
      elif abs(target_center[1] - frame_center[1]) > 225:
          movement_speed=movement_speed*3
      #print(movement_speed)
      #calculating target distance from center        
      #target_dist = [np.cos(abs(target_center[0] - frame_center[0])/frame_center[0]*fov[0])*target_depth,np.cos(abs(target_center[1] - frame_center[1])/frame_center[1]*fov[1])*target_depth]
      #print(np.cos(np.deg2rad(abs(target_center[0] - frame_center[0])/frame_center[0]*fov[0])))
      #print(target_dist)

      #mode check for command loop
      if mode == 'GUIDED':
        #movement algorithm
        if target_orientation == 'vertical':
                                  
            if target_center[0] < frame_center[0]-center_dist_bound:
                #print('frame move left')
                velocity_y = -1*movement_speed
            elif target_center[0] > frame_center[0]+center_dist_bound:
                #print('frame move right')
                velocity_y = 1*movement_speed
            else:
                velocity_y = 0
            
            if target_center[1] < frame_center[1]-center_dist_bound:
                #print('frame move up')
                velocity_z = -1*movement_speed
            elif target_center[1] > frame_center[1]+center_dist_bound:
                #print('frame move down')
                velocity_z = 1*movement_speed
            else:
                velocity_z = 0
                      
            if target_depth > desired_distance+distance_bound:
                #print('frame move toward target')
                velocity_x = 1*movement_speed
            elif target_depth < desired_distance-distance_bound:
                #print('frame move away from target')
                velocity_x = -1*movement_speed
            else:
                velocity_x = 0
                
            #need to test yaw_rate signs    
            if yaw_depth1 > yaw_depth2:
                yaw_rate = 1
            elif yaw_depth1 < yaw_depth2:
                yaw_rate = -1
            else:
                yaw_rate = 0
                
        elif target_orientation == 'horizontal':
            yaw_rate = 0
            if target_center[0] < frame_center[0]-center_dist_bound:
                #print('frame move left')
                velocity_y = -1*movement_speed
            elif target_center[0] > frame_center[0]+center_dist_bound:
                #print('frame move right')
                velocity_y = 1*movement_speed
            else:
                velocity_y = 0
            
            if target_center[1] < frame_center[1]-center_dist_bound:
                #print('frame move up')
                velocity_x = 1*movement_speed
            elif target_center[1] > frame_center[1]+center_dist_bound:
                #print('frame move down')
                velocity_x = -1*movement_speed
            else:
                velocity_x = 0
                      
            if target_depth > desired_distance+distance_bound:
                #print('frame move toward target')
                velocity_z = 1*movement_speed
            elif target_depth < desired_distance-distance_bound:
                #print('frame move away from target')
                velocity_z = -1*movement_speed
            else:
                velocity_z = 0
        else:
            print('orientation undefined')
            velocity_x = 0
            velocity_y = 0
            velocity_z = 0
        #bitmasks https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        #Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
        #Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
        #Use Pos+Vel  : 0b110111000000 / 0x0DC0 / 3520 (decimal)
        #supposedly all 0b0000000000000000
       
        t=time.gmtime()
        current_time = time.strftime("%H:%M:%S", t)
        print(velocity_x,velocity_y,velocity_z,':commanded velocity x,y,z',current_time)
        if target_orientation =='vertical':
                print(yaw_rate)        
        msg1 = master.mav.set_position_target_local_ned_encode(0, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 3527, 0, 0, 0, velocity_x, velocity_y, velocity_z, 0 ,0, 0, 0, yaw_rate)
        master.mav.send(msg1)
        
        i = i + 1
        if i == 10:
           msg_pos = master.recv_match(type = "LOCAL_POSITION_NED", blocking = True)
           msg2 = master.recv_match(type = "VFR_HUD", blocking = True)
           t=time.gmtime()
           current_time = time.strftime("%H:%M:%S", t)
           print("Timestamp: ", current_time)
           print(msg2)
           print("")
           print(msg_pos)
           print("")
           i = 0
        
        #time.sleep(command_rate)
        #print(i)
        #print('command sent')
      
      else:
        i = i + 1
        if i == 10:  
            t=time.gmtime()
            current_time = time.strftime("%H:%M:%S", t)
            print("No ArUCo detected",current_time)
            i = 0
    
    cv2.putText(aruco_res, str(time.strftime("%H:%M:%S", t)),(1150, 20),cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 4)
    cv2.namedWindow('ArUCo', cv2.WINDOW_NORMAL)
    cv2.imshow('ArUCo',aruco_res)
    cv2.namedWindow('FPV', cv2.WINDOW_NORMAL)
    cv2.imshow('FPV',aruco_res2)
    
    ii = ii+1
    if ii < 20000:
    	cv2.imwrite("flight_image"+str(ii)+".jpg",aruco_res)


    cv2.waitKey(1) #cv2.waitKey() vital line of code, if zero takes stills and moves on button press
    





