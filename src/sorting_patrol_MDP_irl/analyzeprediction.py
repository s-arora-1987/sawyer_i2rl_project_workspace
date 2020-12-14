import roslib; roslib.load_manifest('navigation_irl')

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

home = os.environ['HOME']

	

def run(gotimesLog, predictionLog, patroller1Bag, patroller2Bag):   
	# read in and process bag and log files
	predictionQuality = []
	
	try:
		import pickle
		
		# need to know when the attacker left at, the patroller start states and start times
		
		# load the  patroller model if exists
		
		# look through each of the bags to generate a list of actual positions for each robot at each timestep of the mdp
		f = open(gotimesLog, "r")
		decisions = pickle.load(f)
		f.close()
			
		lastOne = decisions[len(decisions) - 1]
		print(lastOne)
		print("Finished de-serializing data for last go times entry (goTime, get_time(), totalMaxValue, patrollerStartStates, patrollerTimes, totalBestTime, totalBestPolicy, mapToUse, getTimeConv() ) ")
		goTime = lastOne[0]
		patrollerStates = lastOne[3]
		patrollerTimes = lastOne[4]
		bestTime = lastOne[5]
		mapToUse = lastOne[7]
		timeScale = lastOne[8]		
		
		baseTime = goTime - (bestTime * timeScale)
	
		predictions = [{}, {}]
		predictionStart = [100000, 100000]
		f = open(predictionLog)
		p = f.read()
		f.close()
		
		i = 0
		pr = p.split("\n")
		for p in pr:
			if (len(p) < 2) or p == "ENDP":
				i += 1
				continue
			# get state as just a string, relative timestep, and probability of finding the robot there
			split1 = p.split(" = ")
			split2 = split1[0].split("], ")
			state = split2[0][1 : ] + "]"
			
			timestep = int(split2[1][ 0 : (len(split2[1]) -1 )])
			prob = float(split1[1])
			
			if (timestep < predictionStart[i]):
				predictionStart[i] = timestep
			
			if not state in predictions[i].keys():
				predictions[i][state] = [0] * 100
			
			predictions[i][state][timestep - predictionStart[i]] = prob
			
		print predictionStart
		
		patrollerPositions = [[],[]]
		i = 0
		curState = 0
		curGoalTime = baseTime + (predictionStart[i] * timeScale)
		bag = rosbag.Bag(patroller1Bag)
		
		if mapToUse == "boyd2":
			mapparams = boyd2MapParams(False)
			ogmap = OGMap(*mapparams)
		else:
			mapparams = boydrightMapParams(False)
			ogmap = OGMap(*mapparams)			
	
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
		curGoalTime = baseTime + (predictionStart[i] * timeScale)
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

	except:
		pass
	
	predictionQuality = []
	
	for (j, pp) in enumerate(patrollerPositions):
		for (i, pos) in enumerate(pp):
			state = "[" + str(pos[0].location[0]) + ", " + str(pos[0].location[1]) + ", " + str(pos[0].orientation) + "]"
	#		if (i >= len(predictions[state])):
	#			continue
			try:
				predictionQuality.append(predictions[j][state][i])
				print(str(state) + " " + str(i) )
			except:
				predictionQuality.append(0)
		print("")
		
	print (predictionQuality)
	print("printed predictionQuality")
	
	import operator
	
	f = open("/tmp/studyresults","a")
	if (len(predictionQuality) == 0):
		f.write(" 0 0")
	else:	
		f.write(" " + str(reduce(operator.add, predictionQuality, 1) / len(predictionQuality)) + " " + str(reduce(operator.mul, predictionQuality, 1)))
	f.close()	

	
if __name__ == '__main__':

	run (home + "/patrolstudy/toupload/gotimes.log", home + "/patrolstudy/toupload/prediction.log", home + "/patrolstudy/toupload/robot0.bag", home + "/patrolstudy/toupload/robot1.bag")
