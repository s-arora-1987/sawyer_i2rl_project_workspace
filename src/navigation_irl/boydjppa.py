# import roslib; roslib.load_manifest('patroller_ctrl')

import sys
import patrol.model
import numpy as np
import math
import random
import mdp.simulation
#import mdp.agent
import util.classes
import tf
import psycopg2

dbserver = "shutter"
server = "shutter"
#dbserver = "localhost"
#server = "kbogert.homelinux.net"


def grid_main(bestTime, timeScale, patrollerModels, patrollerStartStates, patrollerTimes, patrollerPositions, predictTime, reward, penalty, detectDistance, interactionlength, mapToUse):
	random.seed()
	np.random.seed()
	
	## Initialize constants

	p_fail = 0
	

	if (mapToUse == "boydright"):
		themap = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 0], 
				     [0, 0, 1, 1, 1, 1, 0, 1, 0],
				     [0, 0, 0, 0, 1, 1, 1, 1, 1],
				     [0, 0, 0, 0, 0, 0, 1, 1, 1],
				     [0, 0, 0, 0, 0, 0, 0, 1, 0],
				     [0, 0, 0, 0, 0, 0, 1, 1, 0]])    
	else:

		themap = np.array( [[1, 1, 1, 1, 1], 
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],				     
				     [1, 0, 0, 0, 0],				     
				     [1, 1, 1, 1, 1]])
	## Create Model
	model = patrol.model.PatrolModel(p_fail, None, themap)
	model.gamma = 0.95
#	reward.setModel(model)

	t_max = math.ceil( math.log(0.01)/math.log(model.gamma) )   
	print(t_max)
	## Create initial distribution
	initial = util.classes.NumMap()
	for s in model.S():
		initial[s] = 1.0
	initial = initial.normalize()

	## Define player
#	policy = mdp.agent.HumanAgent(model)
	policies = patrollerModels

	pmodel = model



	if (mapToUse == "boydright"):
		map = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1], 
					 [1, 0, 1, 1, 1, 1, 0, 1, 0],
					 [1, 0, 0, 0, 1, 1, 1, 1, 1],
					 [1, 0, 0, 0, 0, 0, 1, 1, 1],
					 [1, 1, 1, 1, 1, 0, 0, 1, 0],
					 [0, 0, 0, 0, 1, 1, 1, 1, 0]])
	else:
		map = np.array([[1, 1, 1, 1, 1], 
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],				     
				     [1, 0, 0, 0, 0],				     
				     [1, 1, 1, 1, 1]])
	## Create Model
	p_fail = .20
	goalState = patrol.model.AttackerState(np.array([4,2]), 1)
	model = patrol.model.AttackerModel(p_fail, map, predictTime, goalState)
	model.gamma = 0.95
	
	print(patrollerStartStates, patrollerTimes, predictTime, interactionlength)
	# read in the bag files, and using the time offset and timescale parameters generate a list of actual patroller and attacker positions at each mdp timestep
	trajectory = mdp.simulation.create_patroller_trajectory_positions(pmodel, policies, patrollerStartStates, patrollerTimes, predictTime, interactionlength, mapToUse)
	
	# now find the distance between each predicted position and the actual positions for each timestep
	totalDistance = 0
	totalTimestep = 0
	
	for (num, traj) in enumerate(trajectory):
		lastTimestep = 0
		for (timestep, prediction) in enumerate(traj):
			if timestep >= len(patrollerPositions[num]):
				break
			lastTimestep = timestep + 1
			actualPos = patrollerPositions[num][timestep]
			
			ang = math.atan2(math.sin(actualPos[2] - prediction[2]), math.cos(actualPos[2] - prediction[2]))

			dist = math.sqrt((actualPos[0] - prediction[0])*(actualPos[0] - prediction[0]) + (actualPos[1] - prediction[1])*(actualPos[1] - prediction[1]) + ang*ang )
			
#			print(actualPos, prediction)
			print(str(num) + "," + str(timestep) + "," + str(dist))
			totalDistance += dist
		totalTimestep += lastTimestep	
			
	print (totalDistance, totalTimestep)

	return (totalDistance, totalTimestep)
	
	
if __name__ == '__main__':
	
	# open connection to database
	# read in list of entries ready to process
	# in a loop:
	#	Download the bag and log files for the entry
	#   Run the analysis for entry's params

	import argparse
	parser = argparse.ArgumentParser(description="Visualizer")
	parser.add_argument("db")

	args = parser.parse_args()		
	
	conn = psycopg2.connect("host=" + dbserver + " dbname=patrol user=patrol password=patrolrunner")
	cur = conn.cursor()

	cur.execute("select id, attacker, penalty, detect, predict, obsstates, attackerdelay, pfail, interactionlength, map from "+args.db+" where successful is not null and attackerattemptat is not null and attackerattemptat > 0 and jppasum is null  and attacker != 'random'");
	rows = cur.fetchall()


	downloadDir = "/tmp/download"
	
	import os
	
	if not os.path.exists(downloadDir):
		os.makedirs(downloadDir)
	
	from subprocess import call
	call(["sudo", "mount", "-t", "ramfs", "-o", "size=20m", "ramfs", downloadDir])
	call(["sudo", "chown", "kbogert", downloadDir])
	

	import pickle
	
	for row in rows:
		try:
			# download log and bag files
	
			call(["scp", server + ":/home/patrol/bags/" + str(row[0]) + "-*", downloadDir + "/"])
			call(["scp", server + ":/home/patrol/logupload/" + str(row[0]) + "-*", downloadDir + "/"])
			
			# uncompress bag files
			call(["tar", "-xzf", downloadDir + "/" + str(row[0]) + "-bags.tar.gz", "-C", downloadDir, "--strip-components", "4"])
			
			gotimesLog = downloadDir + "/" + str(row[0]) + "-gotimes.log"
			predicttime = int(row[4])
			interactionlength = row[8]
			if row[1] == "maxentirl":
				interactionlength = 0

			policyLog = downloadDir + "/" + str(row[0]) + "-policy.log"
			patroller1Bag = downloadDir + "/robot0.bag"
			patroller2Bag = downloadDir + "/robot1.bag"
		
			
			# read in and process bag and log files
			
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
			
			if row[9] == "boydright":
				timeScale = 4.5
			else:
				timeScale = 4
			
			baseTime = goTime - (bestTime * timeScale)
			
			patrollerModels = None
			if not policyLog is None:
				f = open(policyLog, "r")
				patrollerModels = pickle.load(f)
				f.close()
		
			import rosbag
		
			
			patrollerPositions = [[],[]]
			i = 0
			curState = 0
			curGoalTime = baseTime  - ((0 if patrollerTimes[0] < 0 else patrollerTimes[0]) * timeScale) 
			bag = rosbag.Bag(patroller1Bag)
		
			for topic, msg, t in bag.read_messages(topics=['/robot_0/base_pose_ground_truth']):
				if t.to_sec() >= curGoalTime:
					q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))
					
					x, y, angle = tf.transformations.euler_from_quaternion(q)
					if (angle < 0):
						angle = 2 * math.pi + angle
		
					patrollerPositions[i].append((msg.pose.pose.position.x, msg.pose.pose.position.y, angle))
					
					curState += 1
					curGoalTime += timeScale
			bag.close()
			i = 1
			curState = 0
			curGoalTime = baseTime - ((0 if patrollerTimes[1] < 0 else patrollerTimes[1]) * timeScale) 
			bag = rosbag.Bag(patroller2Bag)
			for topic, msg, t in bag.read_messages(topics=['/robot_1/base_pose_ground_truth']):
				if t.to_sec() >= curGoalTime:
					q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))
					
					x, y, angle = tf.transformations.euler_from_quaternion(q)
					if (angle < 0):
						angle = 2 * math.pi + angle
					
					patrollerPositions[i].append((msg.pose.pose.position.x, msg.pose.pose.position.y, angle))
					
					curState += 1
					curGoalTime += timeScale
			bag.close()
		
			answer = grid_main(bestTime, timeScale, patrollerModels, patrollerStates, patrollerTimes, patrollerPositions, predicttime, 10, row[2], int(row[3]), interactionlength , row[9])
			
			# place the answer into the database
			cur.execute("update "+args.db+" set jppasum = %s, jppatimesteps = %s where id = %s", (answer[0], answer[1], row[0]))
			
		except:
#			import traceback
#			traceback.print_exc(file=sys.stdout)
#			exit()
			pass
		
		fileList = os.listdir(downloadDir)
		for fileName in fileList:
			os.remove(downloadDir+"/"+fileName)		
	
		
		
		# commit the database changes
		conn.commit()

        call(["sudo", "umount", downloadDir])

