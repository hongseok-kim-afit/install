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



#connecting to autopilot
master = mavutil.mavlink_connection("/dev/ttyUSB0", baud=57600) # baud?
master.wait_heartbeat()


#initializing mode variable as GUIDED. Needed to not throw aruco detection exception
mode = 'GUIDED'
Hor_vel = '1'

###########################################3
## movement function 
# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs

def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(8)]
    rc_channel_values[channel_id - 1] = pwm
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

#####################################
## main code
##################################

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
set_rc_channel_pwm(1, 2000) #left turn 991, neu = 1453, right turn =1965

# Set transmission
set_rc_channel_pwm(2, 2000)

# Set some throttle
set_rc_channel_pwm(3, 1300)
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