# import roslib; roslib.load_manifest('navigation_irl')

import rospy
import rosbag
import tf
import math
from patrol.model import *
import sys

ogmap = None

def parsePredictionLine(l):
	temp = l.split(" = ")
	if len(temp) < 2: return None
	
	prob = float(temp[1])
	state = temp[0]
	
	state = state[2 : len(state) - 1]
	pieces = state.split(",")	
	
	return (int(pieces[0]), int(pieces[1]), int(pieces[2][0 : len(pieces[2]) - 1]), int(pieces[3]), prob )
	
def printEmptySpace():
	sys.stdout.write("           ")

def printValueEmptySpace():
	sys.stdout.write("    ")

def printMap(predictions, firstTimesteps, patrollerPositions, attackerPositions, mapToUse, values):
	# the robot positions are assumed to extend 10 timesteps back in time from basetime

	minTimestep = min(firstTimesteps)
	minTimestepCapped = min(minTimestep, -9)
	
	global ogmap
	themap = ogmap.theMap()
	
	for timestep in range(len(predictions[0]) + 9 + minTimestep):
		
		currentTimestep = minTimestepCapped + timestep
		
		sys.stdout.write("Time: ")
		sys.stdout.write("{}".format(currentTimestep))
		
		print("   ******************************************")	
		for row in range(len(themap)):
			
			for col in range(len(themap[row])):
				sys.stdout.write("|")

				if themap[row][col] == 0:
					printEmptySpace()

					continue
				
				
				if (currentTimestep > -10):
					# is the attacker here

					if (currentTimestep + 10 >= len(attackerPositions)):
						attackerHere = False # we've run out of positions
					else:
						ap = attackerPositions[currentTimestep + 10]
						if (ap.location[0] == row and ap.location[1] == col):
							attackerHere = True
						else:
							attackerHere = False
					
					# is patroller 1 here
					if (currentTimestep + 10 >= len(patrollerPositions[0])):
						patroller1here = False # we've run out of positions
					else:
						ap = patrollerPositions[0][currentTimestep + 10][0]
						if (ap.location[0] == row and ap.location[1] == col):
							patroller1Here = True
						else:
							patroller1Here = False
				
					# is patroller 2 here
					if (currentTimestep + 10 >= len(patrollerPositions[1])):
						patroller2here = False # we've run out of positions
					else:
						ap = patrollerPositions[1][currentTimestep + 10][0]
						if (ap.location[0] == row and ap.location[1] == col):
							patroller2Here = True
						else:
							patroller2Here = False
				else:
					attackerHere = False
					patroller1Here = False
					patroller2Here = False
				
				probs = []
				for i in range(len(firstTimesteps)):
					
					if (currentTimestep < firstTimesteps[i]):
						probs.append(0.0)
						continue
					
					p = predictions[i][currentTimestep - firstTimesteps[i]]
					if ( (row, col) not in p):
						probs.append(0.0)
						continue
						
					probs.append(p[(row, col)])
				
				if (attackerHere):
					sys.stdout.write("+")
				else:
					sys.stdout.write(" ")
					
				sys.stdout.write("{0:.2f}".format(probs[0]))

				if (patroller1Here):
					sys.stdout.write("<")
				else:
					sys.stdout.write(" ")

				sys.stdout.write("{0:.2f}".format(probs[1]))
				if (patroller2Here):
					sys.stdout.write("<")
				else:
					sys.stdout.write(" ")
				
			sys.stdout.write("|  ")
				
			for col in range(len(themap[row])):
				sys.stdout.write("|")
				
				s = (row, col)
				
				if (s in values[currentTimestep]) and currentTimestep >= 0:
					sys.stdout.write("{0:.2f}".format(values[currentTimestep][s]))
				else:
					printValueEmptySpace()
			print("|")


def runVis(gotimesLog, attackerpolicyLog, attackerBag, patroller1Bag, patroller2Bag, predictionsLog):   
	# read in and process bag and log files
	
	import pickle
	
	# need to know when the attacker left at, the patroller start states and start times
	
	# load the  patroller model if exists
	
	# look through each of the bags to generate a list of actual positions for each robot at each timestep of the mdp
	f = open(gotimesLog, "r")
	decisions = pickle.load(f)
	f.close()
		
	lastOne = decisions[len(decisions) - 1]
	print(lastOne)
	goTime = lastOne[0]
	patrollerStates = lastOne[3]
	patrollerTimes = lastOne[4]
	bestTime = lastOne[5]
	mapToUse = lastOne[7]
	timeScale = lastOne[8]

	if goTime > 100000000000000:
		# attacker never left
		# need to figure out the t = 0 of the last prediction, but we can't!
		baseTime = lastOne[1]
	else:
		baseTime = goTime - (bestTime * timeScale)


	attackerPolicy = None
	values = None
	f = open(attackerpolicyLog, "r")
	l = pickle.load(f)
	attackerPolicy = l[0][lastOne[6]]
	values = l[1][lastOne[6]]
	f.close()

	global ogmap
	
	if mapToUse == "boyd2":
		mapparams = boyd2MapParams(False)
		ogmap = OGMap(*mapparams)			
	else:
		mapparams = boydrightMapParams(False)
		ogmap = OGMap(*mapparams)		
	
	attackerPositions = []
	curState = 0
	curGoalTime = baseTime - (10 * timeScale)
	bag = rosbag.Bag(attackerBag)
	for topic, msg, t in bag.read_messages(topics=['/robot_2/base_pose_ground_truth']):
		if t.to_sec() >= curGoalTime:
			q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))

			x, y, angle = tf.transformations.euler_from_quaternion(q)
			if (angle < 0):
				angle = 2 * math.pi + angle
			attackerPositions.append(ogmap.toState((msg.pose.pose.position.x, msg.pose.pose.position.y, angle), True))
			
			curState += 1
			curGoalTime += timeScale
	bag.close()

   
	patrollerPositions = [[],[]]
	i = 0
	curState = 0
	curGoalTime = baseTime - (10 * timeScale)
	bag = rosbag.Bag(patroller1Bag)

	for topic, msg, t in bag.read_messages(topics=['/robot_0/base_pose_ground_truth']):
		if t.to_sec() >= curGoalTime:
			q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))

			x, y, angle = tf.transformations.euler_from_quaternion(q)
			if (angle < 0):
				angle = 2 * math.pi + angle

			patrollerPositions[i].append((ogmap.toState((msg.pose.pose.position.x, msg.pose.pose.position.y, angle), True), angle * 57.2957795))
			
			curState += 1
			curGoalTime += timeScale
	bag.close()
	i = 1
	curState = 0
	curGoalTime = baseTime - (10 * timeScale)
	bag = rosbag.Bag(patroller2Bag)
	for topic, msg, t in bag.read_messages(topics=['/robot_1/base_pose_ground_truth']):
		if t.to_sec() >= curGoalTime:
			q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))

			x, y, angle = tf.transformations.euler_from_quaternion(q)
			if (angle < 0):
				angle = 2 * math.pi + angle
			
			patrollerPositions[i].append((ogmap.toState((msg.pose.pose.position.x, msg.pose.pose.position.y, angle), True), angle * 57.2957795))
			
			curState += 1
			curGoalTime += timeScale
	bag.close()
	
	# project the predictions from 4 dimensional to 3 (drop orientation)
	
	f = open(predictionsLog, "r")
	pred = f.read()
	f.close()
	
	predictions = [[], []]
	firstTimesteps = [sys.maxint, sys.maxint]
	curPatroller = 0
	lastTimestep = -sys.maxint - 1
	
	pred = pred.split("\n")
	for line in pred:
		a = parsePredictionLine(line)
		if a is None:
			# we have a patroller marker
			curPatroller += 1
			lastTimestep = -sys.maxint - 1
			continue
		
		
		curTimestep = a[3]
		
		if curTimestep > lastTimestep:
			predictions[curPatroller].append({})
			lastTimestep = curTimestep
			
		if (curTimestep < firstTimesteps[curPatroller]):
			firstTimesteps[curPatroller] = curTimestep
			
		state = (a[0], a[1])
		
		if state in predictions[curPatroller][len(predictions[curPatroller]) - 1]:
			predictions[curPatroller][len(predictions[curPatroller]) - 1][state] += a[4]
		else:
			predictions[curPatroller][len(predictions[curPatroller]) - 1][state] = a[4]
			
	avgvalues = [{} for i in range(len(predictions[0]))]
	
	for (entry, v) in values.iteritems():
		
		s = (entry.location[0], entry.location[1])
		
		if (s in avgvalues[entry.time]):
			if (abs(avgvalues[entry.time][s])) <= abs(v):
				avgvalues[entry.time][s] = v 
		else:
			avgvalues[entry.time][s] = v / 4.0
	
	
	printMap(predictions, firstTimesteps, patrollerPositions, attackerPositions, mapToUse, avgvalues)
