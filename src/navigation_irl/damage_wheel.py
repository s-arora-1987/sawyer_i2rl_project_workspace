#!/usr/bin/env python
# import roslib; roslib.load_manifest('navigation_irl')

import rospy
from geometry_msgs.msg import Twist

pub = None
wheel = [1.0, 1.0] 
WHEEL_SEPARATION = 0.26
MAX_WHEEL_SPEED = 500


def cmd_vel(msg):

	global pub
	global wheel
	global WHEEL_SEPARATION
	global MAX_WHEEL_SPEED

	ts  = msg.linear.x * 1000 # m -> mm
	tw  = msg.angular.z  * (WHEEL_SEPARATION / 2) * 1000
	# Prevent saturation at max wheel speed when a compound command is sent.
	if ts > 0:
		ts = min(ts,   MAX_WHEEL_SPEED - abs(tw))
	else:
		ts = max(ts, -(MAX_WHEEL_SPEED - abs(tw)))
	left = int(ts - tw) * wheel[0]
	right = int(ts + tw) * wheel[1]

	

	ts = (left + right) / 2
	tw = ts - left

	msg.linear.x = ts / 1000
	msg.angular.z = tw / (1000 * WHEEL_SEPARATION/2)
	
	pub.publish(msg)
	
if __name__ == "__main__":

	rospy.init_node('damage_wheel')
	
	wheel[0] = rospy.get_param("~left_wheel_quality", 1.0)
	wheel[1] = rospy.get_param("~right_wheel_quality", 1.0)
	
	cmd_vel_sub = rospy.Subscriber('damage_wheel_cmd_vel_in', Twist, cmd_vel)
	pub = rospy.Publisher('damage_wheel_cmd_vel_out', Twist)
	
	rospy.spin()
	
