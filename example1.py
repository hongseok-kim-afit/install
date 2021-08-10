#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan


def is_free(ranges ,start_index, end_index ,min_distance):

	#sub array from angle to angle

	s_ranges = ranges[start_index:end_index]

	#filter the array 

	b_ranges = filter(lambda x: x <= min_distance, s_ranges)

	return len(b_ranges) == 0


 

def callback(msg):		    

	if is_free(msg.ranges,160,200,0.2): print "FREE"

	else: print "BLOCKED"

 

rospy.init_node('scan_values')


#Put your Hamster number instead of agent<number>

#The agent number is printed on the Hamster cover 

sub = rospy.Subscriber('/agent14/scan', LaserScan, callback)



rospy.spin()