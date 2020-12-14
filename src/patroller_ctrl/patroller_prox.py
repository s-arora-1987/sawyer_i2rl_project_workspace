#!/usr/bin/env python
# import roslib; roslib.load_manifest('patroller_ctrl')

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import math
import tf
import numpy

maxsightdistance = 6
pub = None
lastAttacker = None
lastPatrolUpdate = 0
map = None

lastPatroller = None

def handle_attacker(req):
	global lastAttacker

	lastAttacker = req.pose.pose.position
	
	checkForDetection()

def convertPositionToStateBoyd(pos,angles):
	topX = 25.55
	bottomX = 8.65
	leftY = 8.40
	rightY = 14.0

	x = 9 - round ( (pos[1] - bottomX) / ((topX - bottomX) / 9) )
	if (x < 0):
		x = 0
	if (x > 9):
		x = 9
	y = round ( (pos[0] - leftY) / ((rightY - leftY) / 2) )
	if (y < 0):
		y = 0
	if (y > 2):
		y = 2

	if angles[2] < math.pi / 4 or angles[2] > 7 * math.pi / 4:
		direction = 0
	elif angles[2] > 3 * math.pi / 4 and angles[2] < 5 * math.pi / 4:
		direction = 2
	elif  angles[2] <= 3 * math.pi / 4 and angles[2] >=  math.pi / 4:
		direction = 1
	else:
		direction = 3			
		
	return (int(x),int(y), direction)


def convertPositionToStateBoydRightPatroller(pos, angles):
	leftX = 52.77
	rightX = 34.85
	topY = 25.55
	bottomY = 13.29

	x = round ( (pos[0] - rightX) / ((leftX - rightX) / 8) )
	if (x < 0):
		x = 0
	if (x > 8):
		x = 8
	y = 5 - round ( (pos[1] - bottomY) / ((topY - bottomY) / 5) )
	if (y < 0):
		y = 0
	if (y > 5):
		y = 5
		
	if angles[2] < math.pi / 4 or angles[2] > 7 * math.pi / 4:
		direction = 0
	elif angles[2] > 3 * math.pi / 4 and angles[2] < 5 * math.pi / 4:
		direction = 2
	elif  angles[2] <= 3 * math.pi / 4 and angles[2] >=  math.pi / 4:
		direction = 1
	else:
		direction = 3

	return (y,x, direction)
	
	
def handle_patroller(req):
	global lastPatroller
	
	lastPatroller = req
	
	checkForDetection()
	
def checkForDetection():
	
	global lastPatroller
	global lastAttacker
	global lastPatrolUpdate
	global maxsightdistance
	global map

	if rospy.get_time() - lastPatrolUpdate > 1 and lastAttacker is not None and lastPatroller is not None:
		# check the distance first

		distance = math.sqrt( (lastPatroller.pose.pose.position.x - lastAttacker.x)*(lastPatroller.pose.pose.position.x - lastAttacker.x) + (lastPatroller.pose.pose.position.y - lastAttacker.y)*(lastPatroller.pose.pose.position.y - lastAttacker.y) )
		
		if distance <= maxsightdistance:
			# now check if the angle is right, the attacker must be within the patroller's field of view
			angle = math.atan2(lastAttacker.y - lastPatroller.pose.pose.position.y, lastAttacker.x - lastPatroller.pose.pose.position.x)

			newangle = tf.transformations.quaternion_from_euler(0,0,angle)

			q1 = numpy.array((lastPatroller.pose.pose.orientation.x, lastPatroller.pose.pose.orientation.y, lastPatroller.pose.pose.orientation.z, lastPatroller.pose.pose.orientation.w ))
			q = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(q1), newangle)
			
			x, y, z = tf.transformations.euler_from_quaternion(q)
			
			if abs(z) <= (57.0 / 2 * .017453):
				
				a = lastPatroller.pose.pose.orientation

				angles = tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])
				if (angles[2] < 0):
				    angles = (0,0, 2 * math.pi + angles[2])
				
				if map == "boydright":
					# we have seen the attacker
					# note, really need to do a raycast here to check for occlusion!
									
					(paty, patx, patdirection) = convertPositionToStateBoydRightPatroller((lastPatroller.pose.pose.position.x, lastPatroller.pose.pose.position.y), angles)
					(aty, atx, atdirection) = convertPositionToStateBoydRightPatroller((lastAttacker.x, lastAttacker.y), angles)

				else:
					(paty, patx, patdirection) = convertPositionToStateBoyd((lastPatroller.pose.pose.position.x, lastPatroller.pose.pose.position.y), angles)
					(aty, atx, atdirection) = convertPositionToStateBoyd((lastAttacker.x, lastAttacker.y), angles)

				# y long hallway, x small, 1 is down for attacker but up for patroller. 2 is right for attacker but left for patroller. 
				if (patdirection == 0):
					# looking right
					if paty != aty or atx < patx:
						return
				elif (patdirection == 1):
					#lookup up
					if patx != atx or aty > paty:
						return
				elif (patdirection == 2):
					#looking left
					if paty != aty or atx > patx:
						return
				else:
					#looking down
					if patx != atx or aty < paty:
						return
				''' (0, 1, 1) (0, 1, 1) same state! 
				(0, 2, 3up) (0, 0, 3dn)
				(0, 3, 3) (0, 0, 3)
				why will attacker look up when he needs to go down? de-localization

				ignore such scenarios. '''
				if map == "boyd2" and ((atx == 0 and aty == paty) or atdirection == 3):
					return
				''' other scenarios observed
				(0, 1, 1) (0, 3, 1)
				(0, 0, 1) (0, 2, 1)
				(0, 0, 1) (0, 3, 1)
				(0, 4, 1) (0, 7, 1)
				(0, 0, 2) (2, 0, 2)  
				'''	
				f = open("/tmp/detection_pos","w")
				f.write(" "+str((atx,aty,atdirection))+" "+str((patx, paty, patdirection))) #
				f.close()

				global pub
				pub.publish(String("spotattacker"))
				


				lastPatrolUpdate = rospy.get_time()



def init():
	global pub

	rospy.init_node('patroller_controller')
	
	global map
	map = rospy.get_param("~map")
	
	rospy.Subscriber("base_pose_ground_truth", Odometry, handle_patroller)
	rospy.Subscriber("/robot_2/base_pose_ground_truth", Odometry, handle_attacker)
	pub = rospy.Publisher("/study_status", String)
	rospy.spin()

init()
