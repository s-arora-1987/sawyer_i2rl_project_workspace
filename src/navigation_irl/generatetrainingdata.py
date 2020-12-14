# import roslib; roslib.load_manifest('navigation_irl')

import rospy
import rosbag
	
import sys
import patrol.model
import numpy as np
import random
import mdp.simulation
import mdp.solvers
#import mdp.agent
import util.classes
import patrol.solvers
import math
from patrol.model import *
from mdp.simulation import simulate
import tf
import os
import operator


home = os.environ['HOME']

def mean_and_variance(data):
	n	= 0
	sum1 = 0
	sum2 = 0

	for x in data:
		n	= n + 1
		sum1 = sum1 + x

	mean = sum1/n

	for x in data:
		sum2 = sum2 + (x - mean)*(x - mean)

	variance = sum2/(n - 1)
	return (mean, variance)



def genPolicyForMap(m):
		
								
		# Call external solver here instead of python based				
	args = ["boydsimple_t", ]
	import subprocess			

	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

	stdin = m + "\n"
	stdin += "1.0\n"
	
	(transitionfunc, stderr) = p.communicate(stdin)
	
	
	
	args = ["boydpatroller", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	
	stdin = m + "\n"

	stdin += transitionfunc
					
					
	f = open(home + "/patrolstudy/toupload/patrollerT.log", "w")
	f.write(stdin)
	f.close()
	
	(stdout, stderr) = p.communicate(stdin)
	
	# stdout now needs to be parsed into a hash of state => action, which is then sent to mapagent
	p = {}
	stateactions = stdout.split("\n")
	for stateaction in stateactions:
		temp = stateaction.split(" = ")
		if len(temp) < 2: continue
		state = temp[0]
		action = temp[1]
								
		
		state = state[1 : len(state) - 1]
		pieces = state.split(",")	
		
		ps = patrol.model.PatrolState(np.array([int(pieces[0]), int(pieces[1]), int(pieces[2])]))

		if action == "MoveForwardAction":
			a = patrol.model.PatrolActionMoveForward()
		elif action == "TurnLeftAction":
			a = patrol.model.PatrolActionTurnLeft()
		elif action == "TurnAroundAction":
			a = patrol.model.PatrolActionTurnAround()
		elif action == "TurnRightAction":
			a = patrol.model.PatrolActionTurnRight()
		else:
			a = patrol.model.PatrolActionStop()

		p[ps] = a

	from mdp.agent import MapAgent
	return MapAgent(p)

policies = {}
def getPolicy(state, mapToUse):
	global policies
	
	p = policies[mapToUse]
	
	action = p.actions(state).keys()[0]
	
	return action.__str__()

	if (action.__str__() == "PatrolActionMoveForward"):
		return 0
	if (action.__str__() == "PatrolActionTurnLeft"):
		return 1
	if (action.__str__() == "PatrolActionTurnRight"):
		return 2
	if (action.__str__() == "PatrolActionTurnAround"):
		return 3
	if (action.__str__() == "PatrolActionStop"):
		return 4
	
	
def extractStatistics(bag, topic, statisticsEntries, mapToUse, timeScale):
		
	deltaxs = []
	lastx = None
	deltays = []
	lasty = None
	deltathetas = []
	lasttheta = None
	firsttheta = 0
	
	currentAction = None
	
	curGoalTime = timeScale * 3

	for topic, msg, t in bag.read_messages(topics=[topic]):
		if t.to_sec() < timeScale * 2:
			continue
		
		q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))

		x, y, angle = tf.transformations.euler_from_quaternion(q)
		if (angle < 0):
			angle = 2 * math.pi + angle
		
		if t.to_sec() >= curGoalTime:
			# we're in a new timestep, calculate statistics and reset
			sumx = reduce(operator.add, deltaxs, 0)
			(meanx, variancex) = mean_and_variance(deltaxs)
			sumy = reduce(operator.add, deltays, 0)
			(meany, variancey) = mean_and_variance(deltays)
			sumtheta = reduce(operator.add, deltathetas, 0)
			(meantheta, variancetheta) = mean_and_variance(deltathetas)
			
			if (abs(sumx) + abs(sumy) + abs(sumtheta)) < .01:
				currentAction = "PatrolActionStop" # Stop action
				
			statisticsEntries.append( (meanx, variancex, sumx, meany, variancey, sumy, meantheta, variancetheta, sumtheta, currentAction) )
			
			deltaxs = []
			lastx = None
			deltays = []
			lasty = None
			deltathetas = []
			lasttheta = None

			currentAction = None
			curGoalTime += timeScale
		else:
			
			if (currentAction is None):

				discreteState = convertPositionToStatePatroller( (msg.pose.pose.position.x, msg.pose.pose.position.y, angle), mapToUse)
				
				currentAction = getPolicy(discreteState, mapToUse)
				
			
			
			if (lasttheta is not None):
				test = angle - lasttheta
				while (test < -math.pi): test += 2*math.pi;
				while (test > math.pi): test -= 2*math.pi;
				
				deltathetas.append(test)
			else:
				firsttheta = angle
				
			lasttheta = angle
						
			if (lastx is not None):
				x = math.cos(lasttheta) * (msg.pose.pose.position.x - lastx) + math.sin(lasttheta)* (msg.pose.pose.position.y - lasty) 
#				print("x", msg.pose.pose.position.x, lastx, x, firsttheta)
				deltaxs.append(x)
			
			lastx = msg.pose.pose.position.x

			if (lasty is not None):
				y = math.sin	(lasttheta) * (msg.pose.pose.position.x - lastx) + math.cos(lasttheta) * (msg.pose.pose.position.y - lasty) 
#				print("y", msg.pose.pose.position.y, lasty, y, firsttheta)				
				deltays.append(y)
			
			lasty = msg.pose.pose.position.y
				
	

def run(gotimesLog, patroller1Bag, patroller2Bag):   
	# read in and process bag and log files
	
	import pickle
	
	# need to know when the attacker left at, the patroller start states and start times
	
	# load the  patroller model if exists
	
	# look through each of the bags to generate a list of actual positions for each robot at each timestep of the mdp
	f = open(gotimesLog, "r")
	decisions = pickle.load(f)
	f.close()
		
	lastOne = decisions[len(decisions) - 1]
	mapToUse = lastOne[7]
	timeScale = lastOne[8]		

	global policies
	
	policies[mapToUse] = genPolicyForMap(mapToUse)

	statisticsEntries = []

	bag = rosbag.Bag(patroller1Bag)
	extractStatistics(bag, '/robot_0/base_pose_ground_truth', statisticsEntries, mapToUse, timeScale)
	bag.close()

	bag = rosbag.Bag(patroller2Bag)
	extractStatistics(bag, '/robot_1/base_pose_ground_truth', statisticsEntries, mapToUse, timeScale)
	bag.close()
	
				
	f = open("output.csv","a")
	for stats in statisticsEntries:
		for idx, s in enumerate(stats):
			if (idx < len(stats) - 1):
				f.write("{:0.15f}".format(s))
				f.write(", ")
			else:
				f.write(str(s))
		f.write("\n")
	f.close()	

	
if __name__ == '__main__':

	run (home + "/patrolstudy/toupload/gotimes.log", home + "/patrolstudy/toupload/robot0.bag", home + "/patrolstudy/toupload/robot1.bag")
