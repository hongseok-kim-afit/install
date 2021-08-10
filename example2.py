#!/usr/bin/env python
# Detect right/left turn from Inertial Measurement Unit(IMU)

import rospy

import math

from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion, quaternion_from_euler


def calculate_robot_angle(orientation):

	quaternion = (

	    orientation.x,

	    orientation.y,

	    orientation.z,

	    orientation.w)

	euler = euler_from_quaternion(quaternion)

	roll = euler[0]

	pitch = euler[1]

	yaw = euler[2]

	return math.degrees(yaw)


prev_angle = 0

def callback(msg):	

	global prev_angle	    

	curr_angle = calculate_robot_angle(msg.orientation)

	if curr_angle > prev_angle :

		print "TURINING LEFT"

	else:

		print "TURINIGN RIGHT"

	prev_angle = curr_angle



rospy.init_node('imu_values')


#Put your Hamster number instead of agent<number>

#The agent number is printed on the Hamster cover 

sub = rospy.Subscriber('/agent14/imu', Imu, callback)



rospy.spin()