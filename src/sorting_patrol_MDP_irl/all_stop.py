#!/usr/bin/env python
import roslib; roslib.load_manifest('navigation_irl')

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

pub = None
enabled = False
zeroTwist = None


def cmd_vel(msg):
	global pub
	global enabled
	
	if not enabled:
		pub.publish(msg)
		return
	
	global zeroTwist
	pub.publish(zeroTwist)
	

def activate(msg):
	global enabled
	global pub
	
	enabled = msg.data
	if (enabled):
		# send a zero twist to make sure the robot immediately stops
		global zeroTwist
		
		pub.publish(zeroTwist)

if __name__ == "__main__":

	rospy.init_node('all_stop')

	zeroTwist = Twist()
	zeroTwist.linear.x = 0
	zeroTwist.linear.y = 0
	zeroTwist.linear.z = 0		
	zeroTwist.angular.x = 0
	zeroTwist.angular.y = 0
	zeroTwist.angular.z = 0


	cmd_vel_sub = rospy.Subscriber('all_stop_cmd_vel_in', Twist, cmd_vel)

	activate_sub = rospy.Subscriber('all_stop_enabled', Bool, activate)

	pub = rospy.Publisher('all_stop_cmd_vel_out', Twist)
	
	rospy.spin()