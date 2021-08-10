#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

from ackermann_msgs.msg import AckermannDriveStamped



pub = rospy.Publisher('/agent14/ackermann_cmd',AckermannDriveStamped, queue_size=1)


def is_free(ranges ,start_index, end_index ,min_distance):

	#sub array from angle to angle

	s_ranges = ranges[start_index:end_index]

	#filter the array 

	b_ranges = filter(lambda x: x <= min_distance, s_ranges)

	return len(b_ranges) == 0


 

def callback(msg):

	global pub		    

	if is_free(msg.ranges,160,200,1):

	    ack_msg = AckermannDriveStamped()

	    ack_msg.header.stamp = rospy.Time.now()

	    ack_msg.header.frame_id = 'your_frame_here'

	    ack_msg.drive.steering_angle = 0

	    ack_msg.drive.speed = 1

	    pub.publish(ack_msg)

	else:

	    ack_msg = AckermannDriveStamped()

	    pub.publish(ack_msg)

	    

rospy.init_node('driving_hamster')


#Put your Hamster number instead of agent<number>

#The agent number is printed on the Hamster cover 

sub = rospy.Subscriber('/agent14/scan', LaserScan, callback)



rospy.spin()
