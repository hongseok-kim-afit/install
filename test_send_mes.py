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


#initializing mode variable as GUIDED. Needed to not throw aruco detection exception
mode = 'GUIDED'
Hor_vel = '1'

######################
## vehicle move definition
#################################3

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration, rate_u):
    """
    Move vehicle in direction based on specified velocity vectors.
    https://dronekit-python.readthedocs.io/en/latest/automodule.html#dronekit.Vehicle.send_mavlink
    """
    msg = master.mav.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component # 
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        master.mav.send(msg)
        time.sleep(rate_u)


def change_mode_vel(ch_x,ch_y,ch_z, duration, rate_u):
    '''
    '''
    ch_x=int(ch_x)
    ch_y=int(ch_y)
    ch_z=int(ch_z)
    msg = master.mav.manual_control_send(
        master.target_system,
        ch_x,# x,y and r will be between [-1000 and 1000]
        ch_y,# x,y and r will be between [-1000 and 1000]
        ch_z, # 500 means neutral throttle
        1,
        0)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        master.mav.send(msg)
        time.sleep(rate_u)

#####################################
## main code
##################################

## set up the duration and update rate
DURATION = 1 # how many times do you want?
rate_u = 1 # put the update Hz


## Set up velocity mappings
# velocity_x > 0 => fly North
# velocity_x < 0 => fly South
# velocity_y > 0 => fly East
# velocity_y < 0 => fly West
# velocity_z < 0 => ascend
# velocity_z > 0 => descend
v_x= 3.0
v_y= 0.0  
v_z =0

#Fly south and up.

#send_ned_velocity(v_x,v_y,v_z,DURATION, rate_u)


#
ch_x= -1000
ch_y= 0.0
ch_z= -1000  # -1000 does not move
change_mode_vel(ch_x,ch_y,ch_z, DURATION, rate_u)



'''
# Request parameter
master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'SURFACE_DEPTH',
    -1
)
print(master.target_system)
## result = 1
# Print old parameter value
message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' %
      (message['param_id'], message['param_value']))
      #  result = 
      #  name: STAT_RUNTIME	value: 152956

      #(message['param_id'].decode("utf-8"), message['param_value']))
'''


'''
master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                           "QGC will read this".encode())
'''


'''
# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].
master.mav.manual_control_send(
    1,
    master.target_system,
    master.target_component,
    MAV_FRAME_GLOBAL,
    POSITION_TARGET_TYPEMASK_VZ_IGNORE,
    0,
    0,
    0,
    1,
    1,
    1,
    0,
    0,
    0,
    0,
    0)

# To active button 0 (first button), 3 (fourth button) and 7 (eighth button)
# It's possible to check and configure this buttons in the Joystick menu of QGC
buttons = 1 + 1 << 3 + 1 << 7
master.mav.manual_control_send(
    master.target_system,
    0, # x,y and r will be between [-1000 and 1000]
    0, # x,y and r will be between [-1000 and 1000]
    500, # 500 means neutral throttle
    0,
    buttons)
'''


'''
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
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


# Set some roll
set_rc_channel_pwm(3, 1600)
'''


'''
    <message id="33" name="GLOBAL_POSITION_INT">
      <description>The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
               is designed as scaled integer message since the resolution of float is not sufficient.</description>
      <field type="uint32_t" name="time_boot_ms" units="ms">Timestamp (time since system boot).</field>
      <field type="int32_t" name="lat" units="degE7">Latitude, expressed</field>
      <field type="int32_t" name="lon" units="degE7">Longitude, expressed</field>
      <field type="int32_t" name="alt" units="mm">Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.</field>
      <field type="int32_t" name="relative_alt" units="mm">Altitude above ground</field>
      <field type="int16_t" name="vx" units="cm/s">Ground X Speed (Latitude, positive north)</field>
      <field type="int16_t" name="vy" units="cm/s">Ground Y Speed (Longitude, positive east)</field>
      <field type="int16_t" name="vz" units="cm/s">Ground Z Speed (Altitude, positive down)</field>
      <field type="uint16_t" name="hdg" units="cdeg" invalid="UINT16_MAX">Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX</field>
    </message>
'''
'''
#########################################3
########
https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_INT
##############################################
Field Name	Type	Units	Values	Description
time_boot_ms	uint32_t	ms		Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
target_system	uint8_t			System ID
target_component	uint8_t			Component ID
coordinate_frame	uint8_t		MAV_FRAME	Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
type_mask	uint16_t		POSITION_TARGET_TYPEMASK	Bitmap to indicate which dimensions should be ignored by the vehicle.
lat_int	int32_t	degE7		X Position in WGS84 frame
lon_int	int32_t	degE7		Y Position in WGS84 frame
alt	float	m		Altitude (MSL, Relative to home, or AGL - depending on frame)
vx	float	m/s		X velocity in NED frame
vy	float	m/s		Y velocity in NED frame
vz	float	m/s		Z velocity in NED frame
afx	float	m/s/s		X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
afy	float	m/s/s		Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
afz	float	m/s/s		Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
yaw	float	rad		yaw setpoint
yaw_rate	float	rad/s		yaw rate setpoint
'''



'''
###################################################3
https://ardupilot.org/rover/docs/logmessages.html
#################################################33
ADSB: Automatic Dependant Serveillance - Broadcast detected vehicle information
TimeUS 	Time since system startup
ICAO_address 	Transponder address
Lat 	Vehicle latitude
Lng 	Vehicle longitude
Alt 	Vehicle altitude
Heading 	Vehicle heading
Hor_vel 	Vehicle horizontal velocity
Ver_vel 	Vehicle vertical velocity
Squark 	Transponder squawk code
AHR2: Backup AHRS data
TimeUS 	Time since system startup
Roll 	Estimated roll
Pitch 	Estimated pitch
Yaw 	Estimated yaw
Alt 	Estimated altitude
Lat 	Estimated latitude
Lng 	Estimated longitude
Q1 	Estimated attitude quaternion component 1
Q2 	Estimated attitude quaternion component 2
Q3 	Estimated attitude quaternion component 3
Q4 	Estimated attitude quaternion component 4

'''