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
import math
import time
from pymavlink.dialects.v20 import common as mavlink2

#########################################3
#connecting to autopilot
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600) # baud?
master.wait_heartbeat()


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

#####################################################
## camera functions
"""
In this section, we will set up the functions that will translate the camera
intrinsics and extrinsics from librealsense into parameters that can be used
with OpenCV.
The T265 uses very wide angle lenses, so the distortion is modeled using a four
parameter distortion model known as Kanalla-Brandt. OpenCV supports this
distortion model in their "fisheye" module, more details can be found here:
https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
"""

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

# Set up a mutex to share data between threads 
from threading import Lock
frame_mutex = Lock()
frame_data = {"left"  : None,
              "right" : None,
              "timestamp_ms" : None,
              "position" : None
              }

"""
This callback is called on a separate thread, so we must use a mutex
to ensure that data is synchronized properly. We should also be
careful not to do much work on this thread to avoid data backing up in the
callback queue.
"""
def callback(frame):
    global frame_data
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        position = frame.get_pose_frame.get_pose_data.translation
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()
        frame_mutex.acquire()
        frame_data["left"] = left_data
        frame_data["right"] = right_data
        frame_data["timestamp_ms"] = ts
        frame_data["position"] = position
        frame_mutex.release()


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

#setup tracking camera 
pipe2 = rs.pipeline()
config2 = rs.config()
config2.enable_device('119622110606') # T265 camera 119622110606

# Start streaming with requested config
profile2 = pipe2.start(config2, callback)
# pipe3.start(config3)

#######################################3
# set the variables
set_speed = 1300 #throttle min = 991, max = 2015
center_dist_bound = 50 #100 #last tested #pixels from center, defines x-y position tolerance
desired_distance = 1.0 #distance from desired target in meters
distance_bound = 0.5 #meters from target, defines distance tolerance
left_turn=1300
right_turn=1800                                   

'''
#######################################3
# test for set_rc_channel_pwm
# https://www.ardusub.com/developers/pymavlink.html
# channel 1 = left turn 991, neu = 1453, right turn =1965
#         2 = transmission, max = 2015,neu=1499
#         3 = throttle min = 991, max = 2015


'''
#####################################
# camera show
try:

    ###########################################
    ## tracking camera
    
    # Set up an OpenCV window to visualize the results
    WINDOW_TITLE1 = 'depth camera'
    WINDOW_TITLE2 = 'tracking camera   mode:s,o q:quit'
    cv2.namedWindow(WINDOW_TITLE2, cv2.WINDOW_NORMAL)
    
    # Configure the OpenCV stereo algorithm. See
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    # description of the parameters
    window_size = 5
    min_disp = 0
    # must be divisible by 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                   numDisparities = num_disp,
                                   blockSize = 16,
                                   P1 = 8*3*window_size**2,
                                   P2 = 32*3*window_size**2,
                                   disp12MaxDiff = 1,
                                   uniquenessRatio = 10,
                                   speckleWindowSize = 100,
                                   speckleRange = 32)

    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe2.get_active_profile()
    streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
               "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
    intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}

    # Print information about both cameras
    print("Left camera:",  intrinsics["left"])
    print("Right camera:", intrinsics["right"])

    # Translate the intrinsics from librealsense into OpenCV
    K_left  = camera_matrix(intrinsics["left"])
    D_left  = fisheye_distortion(intrinsics["left"])
    K_right = camera_matrix(intrinsics["right"])
    D_right = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

    # Get the relative extrinsics between the left and right camera
    (R, T) = get_extrinsics(streams["left"], streams["right"])

    # We need to determine what focal length our undistorted images should have
    # in order to set up the camera matrices for initUndistortRectifyMap.  We
    # could use stereoRectify, but here we show how to derive these projection
    # matrices from the calibration and a desired height and field of view

    # We calculate the undistorted focal length:
    #
    #         h
    # -----------------
    #  \      |      /
    #    \    | f  /
    #     \   |   /
    #      \ fov /
    #        \|/
    stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
    stereo_height_px = 300          # 300x300 pixel stereo output
    stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

    # We set the left rotation to identity and the right rotation
    # the rotation between the cameras
    R_left = np.eye(3)
    R_right = R

    # The stereo algorithm needs max_disp extra pixels in order to produce valid
    # disparity on the desired output region. This changes the width, but the
    # center of projection should be on the center of the cropped image
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2

    # Construct the left and right projection matrices, the only difference is
    # that the right projection matrix should have a shift along the x axis of
    # baseline*focal_length
    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0,               0,         1, 0]])
    P_right = P_left.copy()
    P_right[0][3] = T[0]*stereo_focal_px

    # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
    # since we will crop the disparity later
    Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                  [0, 1,       0, -stereo_cy],
                  [0, 0,       0, stereo_focal_px],
                  [0, 0, -1/T[0], 0]])

    # Create an undistortion map for the left and right camera which applies the
    # rectification and undoes the camera distortion. This only has to be done
    # once
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left"  : (lm1, lm2),
                         "right" : (rm1, rm2)}

    mode_t265 = "stack"

    
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
        #mode = 'GUIDED'
        master.mav.request_data_stream_send(master.target_system, master.target_component,mavutil.mavlink.MAV_DATA_STREAM_ALL,1,1)
        msg = master.recv_match(type = "HEARTBEAT", blocking = False)
        if msg:
            mode = mavutil.mode_string_v10(msg)    
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

        ###############################################3
        ############ tracking camera vedio 
        # Check if the camera has acquired any frames
        frame_mutex.acquire()
        valid = frame_data["timestamp_ms"] is not None
        frame_mutex.release()

        ################################
        ######## get the position from tracking camera
        #pose3 = psn.data.translation
        #print(pose3)
        

        # If frames are ready to process
        if valid:
            # Hold the mutex only long enough to copy the stereo frames
            frame_mutex.acquire()
            frame_copy = {"left"  : frame_data["left"].copy(),
                          "right" : frame_data["right"].copy()}
            frame_mutex.release()

            # Undistort and crop the center of the frames
            center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                          map1 = undistort_rectify["left"][0],
                                          map2 = undistort_rectify["left"][1],
                                          interpolation = cv2.INTER_LINEAR),
                                  "right" : cv2.remap(src = frame_copy["right"],
                                          map1 = undistort_rectify["right"][0],
                                          map2 = undistort_rectify["right"][1],
                                          interpolation = cv2.INTER_LINEAR)}

            # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
            disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

            # re-crop just the valid part of the disparity
            disparity = disparity[:,max_disp:]

            # convert disparity to 0-255 and color it
            disp_vis = 255*(disparity - min_disp)/ num_disp
            disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
            color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)
            '''this section get the position information
            https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/t265_example.py
            '''

            '''this section change the video mode

            '''
            pose = frame_mutex.get_pose_frame()
            
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()

            print("Position: {}".format(data.translation)) # data.translation.x = position of x 
        
        #     if mode_t265 == "stack":
        #         cv2.imshow(WINDOW_TITLE2, color_image)
        #         #cv2.imshow(WINDOW_TITLE2, np.hstack((color_image, disp_color)))
        #         cv2.imshow(WINDOW_TITLE1, res)
        #     if mode_t265 == "overlay":
        #         ind = disparity >= min_disp
        #         color_image[ind, 0] = disp_color[ind, 0]
        #         color_image[ind, 1] = disp_color[ind, 1]
        #         color_image[ind, 2] = disp_color[ind, 2]
        #         # d435_res[ind, 0] = colorized_depth[ind, 0]
        #         # d435_res[ind, 1] = colorized_depth[ind, 1]
        #         # d435_res[ind, 2] = colorized_depth[ind, 2]
        #         cv2.imshow(WINDOW_TITLE2, color_image)
        #         cv2.imshow(WINDOW_TITLE1, colorized_depth)

        # key = cv2.waitKey(1)
        # if key == ord('s'): mode_t265 = "stack"
        # if key == ord('o'): mode_t265 = "overlay"
        # if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE2, cv2.WND_PROP_VISIBLE) < 1:
        #     break
        
    
        
            '''                
            #need to test yaw_rate signs    
            if yaw_depth1 > yaw_depth2:
                yaw_rate = 1
            elif yaw_depth1 < yaw_depth2:
                yaw_rate = -1
            else:
                yaw_rate = 0
            ''' 
#########################################
### draw the circle
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

            #################################################
            ### vehicle movement algorithm

            if mode == 'MANUAL':
                if target_depth > desired_distance+distance_bound:
                    set_rc_channel_pwm(1500,set_speed)
                    
                    if target_center[0] < frame_center[0]-center_dist_bound:
                        # set_rc_channel_pwm(turn, throttle)
                        set_rc_channel_pwm(left_turn,set_speed)

                    elif target_center[0] > frame_center[0]+center_dist_bound:
                        # set_rc_channel_pwm(turn, throttle)
                        set_rc_channel_pwm(right_turn,set_speed)
                else:
                    # set_rc_channel_pwm(turn, throttle)
                    set_rc_channel_pwm(1500,1000)
                        
                
                    
                



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



#########################################
### test for 'set_rc_channel_pwn   - success!!!
'''
https://www.ardusub.com/developers/pymavlink.html
channel 1 = left turn 991, neu = 1453, right turn =1965
        2 = transmission, max = 2015,neu=1499
        3 = throttle min = 991, max = 2015
'''
# Set some turn
#set_rc_channel_pwm(1, 2000) #left turn 991, neu = 1453, right turn =1965

'''
#########################################3
### read all parameter

# Request all parameters
master.mav.param_request_list_send(
    master.target_system, master.target_component
)
while True:
    time.sleep(0.01)
    try:
        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('name: %s\tvalue: %d' % (message['param_id'],
                                       message['param_value']))
    except Exception as error:
        print(error)
        sys.exit(0)
'''
'''
##########################################
# send command to vehicle on rate
for x in range(0,DURATION):    
    master.mav.send(msg)
    ##############################
    ## MODE CHECK
    msg2 = master.recv_match(type = "HEARTBEAT", blocking = False)
    if msg2:   
        mode = mavutil.mode_string_v10(msg2)    
    print(mode)
    #####################

    time.sleep(rate_u)
    
'''