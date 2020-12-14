#!/usr/bin/env python
# import roslib; roslib.load_manifest('patroller_ctrl')

import rospy
import tf
from nav_msgs.msg import Odometry

pub = None
base_pose = None

def handle_base_pose(req):
	global base_pose

	if base_pose is not None:
		return

	base_pose = [-req.pose.pose.position.y, req.pose.pose.position.x]

def handle_pose(req):
	global base_pose
	if base_pose is None:
		return
	br = tf.TransformBroadcaster()
	q = [req.pose.pose.orientation.x, req.pose.pose.orientation.y, req.pose.pose.orientation.z, req.pose.pose.orientation.w]
#	(x, y, z) = tf.transformations.euler_from_quaternion(q)
#	print(z)
#	q = tf.transformations.quaternion_from_euler(0,0, 3.14/2.0 + z)
#	br.sendTransform( (-req.pose.pose.position.y, req.pose.pose.position.x, 0), q , rospy.Time.now(), "/robot_0/odom", "/map" )
	br.sendTransform( (base_pose[0] + req.pose.pose.position.x, base_pose[1] + req.pose.pose.position.y, 0), q , rospy.Time.now(), "/robot_0/odom", "/map" )
	global pub
	
	req.pose.pose.position.x += base_pose[0]
	req.pose.pose.position.y += base_pose[1]

	pub.publish(req)

if __name__ == '__main__':
	rospy.init_node("turtlebot_ground_truth")
	rospy.Subscriber("odom", Odometry, handle_pose)
	rospy.Subscriber("base_pose_ground_truth", Odometry, handle_base_pose)
	pub = rospy.Publisher("odom_truth", Odometry)

	rospy.spin()
