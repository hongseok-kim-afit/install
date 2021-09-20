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
from math import tan, pi
from serial import Serial
import serial           #test without this import, may be used in the usb input
import pymavlink        #test without this import
from pymavlink import mavutil
import math
import time
from pymavlink.dialects.v20 import common as mavlink2





#connecting to autopilot
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600) # baud?
master.wait_heartbeat()
'''
#connecting to sitl
master = mavutil.mavlink_connection('127.0.0.1:14550')
master.wait_heartbeat()
print('connected to sitl')
'''


#initializing mode variable as GUIDED. Needed to not throw aruco detection exception
mode = 'GUIDED'  
RC2_TRIM = '2015'
master.mav.manual_control_send(
    master.target_system,
    500,
    -500,
    250,
    500,
    0)

# To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
# It's possible to check and configure this buttons in the Joystick menu of QGC
buttons = 1 + 1 << 3 + 1 << 7
master.mav.manual_control_send(
    master.target_system,
    0,
    0,
    2015, # 500 means neutral throttle
    0,
    buttons)



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
# RC1 = steering 991 = left, 1503 = neutral, 2000 = right
name: RC1_MIN	value: 991
name: RC1_TRIM	value: 1503
name: RC1_MAX	value: 2015
name: RC1_REVERSED	value: 0
name: RC1_DZ	value: 30
name: RC1_OPTION	value: 0
# RC2= shift 1000-2000
name: RC2_MIN	value: 991
name: RC2_TRIM	value: 1501
name: RC2_MAX	value: 2015
name: RC2_REVERSED	value: 0
name: RC2_DZ	value: 0
name: RC2_OPTION	value: 0
# RC3 = throttle
name: RC3_MIN	value: 991
name: RC3_TRIM	value: 991
name: RC3_MAX	value: 2015
name: RC3_REVERSED	value: 0
name: RC3_DZ	value: 30
name: RC3_OPTION	value: 0
name: RC4_MIN	value: 991
name: RC4_TRIM	value: 1503
name: RC4_MAX	value: 2015
'''