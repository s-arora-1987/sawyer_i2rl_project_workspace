#!/usr/bin/env python
# import roslib; roslib.load_manifest('patroller_ctrl')

import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
import random
import math


tolerance = .1
waypoints = []
curwaypoint = 0
direction = 1
pub = None
sd = .1

firstSignal = True
lastpublish = -1

def handle_pose(req):
    global pub
    global waypoints
    global curwaypoint
    global direction
    global tolerance
    global sd
    global firstSignal
    global lastpublish

    if firstSignal:
	import time
	time.sleep(4)
	firstSignal = False

    # is the current position within tolerance from the current waypoint? if so, change the way point and
    # notify the move_base

    x = req.pose.pose.position.x
    y = req.pose.pose.position.y
    if math.sqrt( (waypoints[curwaypoint][0] - x)*(waypoints[curwaypoint][0] - x) + (waypoints[curwaypoint][1] - y)*(waypoints[curwaypoint][1] - y)) < tolerance + sd:
#	print (math.sqrt( (waypoints[curwaypoint][0] - x)*(waypoints[curwaypoint][0] - x) + (waypoints[curwaypoint][1] - y)*(waypoints[curwaypoint][1] - y)), x, y, waypoints[curwaypoint][0], waypoints[curwaypoint][1])
	curwaypoint += direction
	if curwaypoint < 0:
		direction = direction * -1
		curwaypoint = 1
	if curwaypoint >= len(waypoints):
		direction = direction * -1
		curwaypoint = len(waypoints) - 2

	returnval = PoseStamped()
	returnval.pose.position = Point(random.gauss(waypoints[curwaypoint][0], sd), random.gauss(waypoints[curwaypoint][1], sd), 0)
	returnval.pose.orientation = Quaternion()
	returnval.pose.orientation.w = 1

	returnval.header.frame_id = "/map"
	lastpublish = rospy.get_time()
	pub.publish(returnval)
    elif rospy.get_time() - lastpublish > 5:
	returnval = PoseStamped()
	returnval.pose.position = Point(random.gauss(waypoints[curwaypoint][0], sd), random.gauss(waypoints[curwaypoint][1], sd), 0)
	returnval.pose.orientation = Quaternion()
	returnval.pose.orientation.w = 1

	returnval.header.frame_id = "/map"
	lastpublish = rospy.get_time()
	pub.publish(returnval)
	

def init():
    global pub

    rospy.init_node('patroller_controller')
    
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, handle_pose)
    pub = rospy.Publisher("move_base_simple/goal", PoseStamped)

    rospy.spin()
    
if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser(description="Patroller Controller")
    parser.add_argument("waypoints")
    parser.add_argument("startx")
    parser.add_argument("starty")
    parser.add_argument("tolerance")
    parser.add_argument("sd")
    parser.add_argument("name")
    parser.add_argument("log")

    args = parser.parse_args()

    f = open(args.waypoints)

    tolerance = float(args.tolerance)
    sd = float(args.sd)

    for line in f:
	l = line.split(" ")

	if len(l) > 1:
		waypoints.append((float(l[0]), float(l[1])))
	
		if math.sqrt((float(args.startx) - float(l[0]))*(float(args.startx) - float(l[0])) +(float(args.starty) - float(l[1]))*(float(args.starty) - float(l[1]))) < tolerance:
			curwaypoint = len(waypoints) - 1
    
    direction = 1 if random.random() < .5 else -1

    init()
