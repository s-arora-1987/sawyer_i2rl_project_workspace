#!/usr/bin/env python
import roslib; roslib.load_manifest('navigation_irl')

import rospy
from std_msgs.msg import String, Int8MultiArray, Int64
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
import tf
import patrol.model
from patrol.model import *
import mdp.agent
import sys
import Queue
import subprocess
import multiprocessing
import random
import cPickle as pickle
import os
import numpy
import operator
from patrol.time import *
import time

from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from two_scara_collaboration.srv import *
from two_scara_collaboration.msg import cylinder_blocks_poses,StringArray
from robotiq_2f_gripper_control.srv import *

from sortingMDP.model import *
from sortingMDP.reward import *
from mdp.solvers import *

home = os.environ['HOME']
def get_home():
	global home
	return home

startat = 0

def get_time():
	global startat
	return rospy.get_time() - startat


# class for holding the MDP and utils
mdpWrapper = None

global BatchIRLflag, lastQ, min_IncIRL_InputLen, irlfinish_time, patrtraj_len, \
	min_BatchIRL_InputLen, desiredRecordingLen, useRecordedTraj, recordConvTraj, \
	learned_mu_E, sessionNumber, learned_weights, num_Trajsofar, \
	stopI2RLThresh, sessionStart, sessionFinish, lineFoundWeights, \
	lineFeatureExpec, lineLastBeta, lineLastZS, lineLastZcount, lastQ, \
	wt_data, print_once, useRecordedTraj, recordConvTraj, boydpolicy, analyzeAttack
	
#I2RL
BatchIRLflag=False
recordConvTraj=0
sessionNumber=1
num_Trajsofar=0
normedRelDiff=sys.maxint*1.0 # start with a high value to call update()
stopI2RLThresh=0.0095 #tried 0.0001, didn't go below 0.025 0.04 
sessionStart=False
sessionFinish=False
lineFoundWeights=""
lineFeatureExpec=""
lineFeatureExpecfull=""
lineLastBeta=""
lineLastZS=""
lineLastZcount=""
lastQ="" #final likelihood value achieved
print_once=0

# flags for choosing to use recorded trajectories, and to execute only learning or full attack 
useRecordedTraj = 0
recordConvTraj = 0
analyzeAttack = 0
boydpolicy = {}

def printTrajectories(trajs):
	outtraj = ""
	for patroller in trajs:
		for sap in patroller:
			if (sap is not None):
				outtraj += "["
				outtraj += str(int(sap[0].location[0]))
				outtraj += ", "
				outtraj += str(int(sap[0].location[1]))
				outtraj += ", "
				outtraj += str(int(sap[0].location[2]))
				outtraj += "]:"
				if sap[1].__class__.__name__ == "PatrolActionMoveForward":
					outtraj += "MoveForwardAction"
				elif sap[1].__class__.__name__ == "PatrolActionTurnLeft":
					outtraj += "TurnLeftAction"
				elif sap[1].__class__.__name__ == "PatrolActionTurnRight":
					outtraj += "TurnRightAction"
				elif sap[1].__class__.__name__ == "PatrolActionTurnAround":
					outtraj += "TurnAroundAction"
				else:
					outtraj += "StopAction"
					
				outtraj += ":"
				outtraj += "1"
				outtraj += ";"
			
			outtraj += "\n"
		outtraj += "ENDTRAJ\n"
	return outtraj
	
def printTrajectoriesStatesOnly(trajs):
	outtraj = ""
	for patroller in trajs:
		for sap in patroller:
			if (sap is not None):
				outtraj += "["
				outtraj += str(int(sap[0].location[0]))
				outtraj += ", "
				outtraj += str(int(sap[0].location[1]))
				outtraj += ", "
				outtraj += str(int(sap[0].location[2]))
				outtraj += "]"			
			outtraj += ";"
		outtraj += "\nENDTRAJ\n"
	return outtraj	

def parseTs(stdout):
	T = []
	t = []
	weights = []
	transitions = stdout.split("\n")
	counter = 0
	for transition in transitions:
		counter += 1		
		if transition == "ENDT":
			T.append(t)
			t = []
			continue
		temp = transition.split(":")
		if temp[0] == "WEIGHTS":
			weights.append(temp[1])
			continue
		if len(temp) < 4: continue
		state = temp[0]
		action = temp[1]
		state_prime = temp[2]
		prob = float(temp[3])


		state = state[1 : len(state) - 1]
		state_prime = state_prime[1 : len(state_prime) - 1]
		pieces = state.split(",")	

		ps = [int(pieces[0]), int(pieces[1]), int(pieces[2])]

		pieces = state_prime.split(",")	
		ps_prime = [int(pieces[0]), int(pieces[1]), int(pieces[2])]

		if action == "MoveForwardAction":
			a = 0
		elif action == "TurnLeftAction":
			a = 1
		elif action == "TurnRightAction":
			a = 2
		elif action == "TurnAroundAction":
			a = 3
		else:
			a = 4
			
		t.append( (ps, a, ps_prime, prob))


	if (len(t)) > 0:
		T.append(t)
		
	while (len(T) < 2):
		T.append(T[0])
	
	return (T, weights)
	
def printTs(T):
	outtraj = ""
	for t1 in T:
		for t in t1:
			outtraj += "["
			outtraj += str(t[0][0])
			outtraj += ", "
			outtraj += str(t[0][1])
			outtraj += ", "
			outtraj += str(t[0][2])
			outtraj += "]:"
			if t[1] == 0:
				outtraj += "MoveForwardAction"
			elif t[1] == 1:
				outtraj += "TurnLeftAction"
			elif t[1] == 2:
				outtraj += "TurnRightAction"
			elif t[1] == 3:
				outtraj += "TurnAroundAction"
			else:
				outtraj += "StopAction"
			outtraj += ":["
			outtraj += str(t[2][0])
			outtraj += ", "
			outtraj += str(t[2][1])
			outtraj += ", "
			outtraj += str(t[2][2])
			outtraj += "]:"
			outtraj += str(t[3])
			outtraj += "\n"
			
		outtraj += "ENDT\n"	
	return outtraj


def parsePolicies(stdout, equilibrium, lineFoundWeights, lineFeatureExpec, \
	learned_weights, num_Trajsofar, \
	BatchIRLflag, wt_data, normedRelDiff):

	if stdout is None:
		print("no stdout in parse policies")
	
	stateactions = stdout.split("\n")
	#print("\n parse Policies from contents:")
	#print(stateactions)
	counter = 0
	pmaps = []	
	p = {}
	for stateaction in stateactions:
		counter += 1		
		if stateaction == "ENDPOLICY":
			pmaps.append(p)
			p = {}
			if len(pmaps) == 2: # change this if we ever support more than two patrollers
				break		
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
		elif action == "TurnRightAction":
			a = patrol.model.PatrolActionTurnRight()
		elif action == "TurnAroundAction":
			a = patrol.model.PatrolActionTurnAround()
		else:
			a = patrol.model.PatrolActionStop()
			
		p[ps] = a

	if (len(pmaps) < 2):
		#print("(len(pmaps) < 2)") # no results from i2rl
		#print("stateactions:"+str(stateactions)+"\n \n")		
		returnval = [mdp.agent.MapAgent(p), mdp.agent.MapAgent(p)]
	else:
		#print("(len(pmaps) > 2)")
		returnval = [mdp.agent.MapAgent(pmaps[0]), mdp.agent.MapAgent(pmaps[1])]
		
	pat = 0
	if (equilibrium is None):
		# now parse the equilibria
		p = {}
		for stateaction in stateactions[counter :]:
			counter += 1
			if stateaction == "ENDE":
				returnval.append(p)
				p = {}
				pat += 1
				if pat == 2: # change this if we ever support more than two patrollers
					#print("at 2nd ende")
					#print(stateactions[counter :])
					break		
			temp = stateaction.split(" = ")
			if len(temp) < 2: continue
			action = temp[0]
			percent = temp[1]
	
	
			if action == "MoveForwardAction":
				a = patrol.model.PatrolActionMoveForward()
			elif action == "TurnLeftAction":
				a = patrol.model.PatrolActionTurnLeft()
			elif action == "TurnRightAction":
				a = patrol.model.PatrolActionTurnRight()
			elif action == "TurnAroundAction":
				a = patrol.model.PatrolActionTurnAround()
			else:
				a = patrol.model.PatrolActionStop()
	
			p[a] = float(percent)		
	
	else:
		global patroller
			
		if patroller == "ideal":
			p = {}
			p[patrol.model.PatrolActionMoveForward()] = 1.0
			returnval.append(p)
	
			p = {}
			p[patrol.model.PatrolActionMoveForward()] = 1.0
			returnval.append(p)
			
		else:
			p = {}
			if equilibrium[0] == "c":
				p[patrol.model.PatrolActionMoveForward()] = 1.0
			elif equilibrium[0] == "s":
				p[patrol.model.PatrolActionStop()] = 1.0
			else:
				p[patrol.model.PatrolActionTurnAround()] = 1.0
			returnval.append(p)
	
			p = {}
			if equilibrium[1] == "c":
				p[patrol.model.PatrolActionMoveForward()] = 1.0
			elif equilibrium[1] == "s":
				p[patrol.model.PatrolActionStop()] = 1.0
			else:
				p[patrol.model.PatrolActionTurnAround()] = 1.0
			returnval.append(p)
	
	#print("policies parsed")
	#print(returnval)
	
	#check if contents of session variables exist in results
	#if cond'n and sessionFinish are specific to i2rl 
	if len(stateactions[counter:])>0: #and BatchIRLflag==False:
		# this change is not reflected in updatewithalg 
		sessionFinish = True
		print("\n sessionFinish = True")#results after i2rl session at time: "+str(rospy.Time.now().to_sec()))
		#file = open("/home/saurabh/patrolstudy/i2rl_troubleshooting/I2RLOPread_rosctrl.txt","r")
		lineFoundWeights = stateactions[counter]
		counter += 1
		global reward_dim
		found_weights = [[0.0]*reward_dim,[0.0]*reward_dim]
		
		found_weights = [[float(x) for x in \
		lineFoundWeights.split("\n")[0]. \
		strip("[]").split("], [")[0].split(", ")], \
		[float(x) for x in \
		lineFoundWeights.split("\n")[0]. \
		strip("[]").split("], [")[1].split(", ")]]
		
		#print("weights used for computeRelDiff"+str(found_weights))
# 		(wt_data, normedRelDiff)=computeRelDiff(found_weights, wt_data, normedRelDiff)
		learned_weights = found_weights
		
		#print("lineFoundWeights:"+lineFoundWeights)
		lineFeatureExpec = stateactions[counter]
		counter += 1
		
		num_Trajsofar = int(stateactions[counter].split("\n")[0])
		counter += 1
		
		#print("num_Trajsofar:"+str(num_Trajsofar))
# 		global lastQ
		lastQ = stateactions[counter].split("\n")[0]
		counter += 1
		
		lineFeatureExpecfull = stateactions[counter]
		counter += 1
		
	elif len(stateactions[counter:])==0:
		lineFoundWeights = lineFoundWeights 
		lineFeatureExpec = lineFeatureExpec 
		num_Trajsofar = num_Trajsofar
		lastQ = ""
		lineFeatureExpecfull = ""
		sessionFinish = False
		print("\n no results from irl session")
	
	return (returnval, lineFoundWeights, lineFeatureExpec, \
		learned_weights, num_Trajsofar, \
		sessionFinish, wt_data, normedRelDiff, lastQ, lineFeatureExpecfull)


def irlsolve_recdStates(q, traj, add_delay, algorithm, pmodel, mapToUse, NE, visibleStatesNum, \
			lineFoundWeights, lineFeatureExpec, learned_weights, num_Trajsofar, \
			BatchIRLflag, wt_data, normedRelDiff, traj_begintime, trajfull, useRegions):
	
	#f = open("/tmp/timestamps","a")
	# print "\n contents"+str(traj)+"\n"
	# print("\n sizes of complete trajectories for session "+str(sessionNumber)+": "+\
	# 	str(len(traj[0]))+", "+str(len(traj[1])))
	#f.close()

	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "w")
	f.write("")
	f.close()
	f = open(get_home() + "/patrolstudy/toupload/traj_irl.log", "w")
	f.write("")
	f.close()
	
	global patrollersuccessrate
	global usesimplefeatures
	lasttrajchangeamount = 0
	policies = None
	
	if patrollersuccessrate >= 0:
		args = [ 'boydsimple_t', ]
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		outtraj = ""
		
		outtraj += mapToUse + "\n"
		outtraj += str(patrollersuccessrate)
	else:
		t2 = traj[:]

		outtraj = ""
	
		if patrollersuccessrate == -2:
			args = [ 'boyd_em_t', ]	
			outtraj += str(visibleStatesNum / 14.0)	
			outtraj += "\n"
		else:
			args = [ 'boyd_t', ]
			t2 = filter_traj_for_t_solver(t2[0], t2[1])
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
		outtraj += mapToUse + "\n"
		outtraj += str(usesimplefeatures) + "\n"
		outtraj += printTrajectories(t2)

	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "a")
	f.write(outtraj)
	f.write("ITERATE\n")
	f.close()

	(stdout, stderr) = p.communicate(outtraj)
	(T, weights) = parseTs(stdout)
	
	stdout = None
	stderr = None
	outtraj = None

	f = open(get_home() + "/patrolstudy/toupload/t_weights.log", "w")
	for weight in weights:
		f.write(weight)
		f.write("\n")
	f.close()
		
	args = ["boydirl", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	outtraj = ""
	outtraj += mapToUse + "\n"
	
	if add_delay:
		outtraj += "true\n"
	else:
		outtraj += "false\n"

	if NE == "cc":
		equilibriumCode = 0
	elif NE == "sc":
		equilibriumCode = 1
	elif NE == "cs":
		equilibriumCode = 2
	elif NE == "tc":
		equilibriumCode = 3
	elif NE == "ct":
		equilibriumCode = 4
		
	outtraj += algorithm
	outtraj += "\n"
	outtraj += str(equilibriumCode)	
	outtraj += "\n"
	global equilibriumKnown
	if equilibriumKnown:
		outtraj += "true\n"
	else:
		outtraj += "false\n"
	global interactionLength
	outtraj += str(interactionLength)	
	outtraj += "\n"
	outtraj += str(visibleStatesNum / 14.0)	
	outtraj += "\n"
	# print("visibleStatesNum in traj_irl.log: ", str(visibleStatesNum / 14.0))
	
	outtraj += printTs(T)
	outtraj += str(useRegions)+"\n"

	T = None
			
	outtraj += printTrajectories(traj)

	if num_Trajsofar == 0:
		print("appending weights mue and mbar ")
		for i in range(2):
			for j in range(reward_dim):
				learned_weights[i][j]=random.uniform(-.99,.99)
		
		wt_data=numpy.array([learned_weights])
		
		lineFoundWeights = str(learned_weights)+"\n"
		# create initial feature expectations
		for i in range(2):
			for j in range(reward_dim):
				learned_mu_E[i][j]=0.0
		
		lineFeatureExpec = str(learned_mu_E)+"\n"

	if not not lineFoundWeights and lineFoundWeights[-1] != '\n':
		lineFoundWeights = lineFoundWeights + "\n"
	if not not lineFeatureExpec and lineFeatureExpec[-1] != '\n':
		lineFeatureExpec = lineFeatureExpec + "\n"

	outtraj += lineFoundWeights+lineFeatureExpec+ str(num_Trajsofar)+"\n"
	outtraj += printTrajectories(trajfull)
		
	f = open(get_home() + "/patrolstudy/toupload/traj_irl.log", "a")
	f.write(outtraj)
	f.close()
	
	#datacollected()
	
	(stdout, stderr) = p.communicate(outtraj)
	
	(policies, lineFoundWeights, lineFeatureExpec, learned_weights, \
	num_Trajsofar, \
	sessionFinish, wt_data, normedRelDiff, lastQ, lineFeatureExpecfull)\
	= parsePolicies(stdout, None, lineFoundWeights, lineFeatureExpec, learned_weights, \
	num_Trajsofar, \
	BatchIRLflag, wt_data, normedRelDiff)
	
	stdout = None
	stderr = None
	outtraj = None

	if patrollersuccessrate >= 0:
		args = [ 'boydsimple_t', ]
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		outtraj = ""
		
		outtraj += mapToUse + "\n"
		outtraj += str(patrollersuccessrate)
		
	else:
		t2 = traj[:]

		outtraj = ""
	
		if patrollersuccessrate == -2:
			args = [ 'boyd_em_t', ]	
			outtraj += str(visibleStatesNum / 14.0)	
			outtraj += "\n"
		else:
			args = [ 'boyd_t', ]
			t2 = filter_traj_for_t_solver(t2[0], t2[1])

	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		
		outtraj += mapToUse + "\n"
		outtraj += str(usesimplefeatures) + "\n"
	
		outtraj += printTrajectories(t2)

	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "a")
	f.write(outtraj)
	f.write("ITERATE\n")
	f.close()

	(stdout, stderr) = p.communicate(outtraj)
	(T, weights) = parseTs(stdout)
	
	stdout = None
	stderr = None
	outtraj = None

	f = open(get_home() + "/patrolstudy/toupload/t_weights.log", "w")
	for weight in weights:
		f.write(weight)
		f.write("\n")
	f.close()
	
	policies.append(T)
	
	q.put([lineFoundWeights, lineFeatureExpec, learned_weights, \
		num_Trajsofar, sessionFinish, wt_data, \
		normedRelDiff, lastQ, lineFeatureExpecfull, policies])
	return


class WrapperIRL():

	def __init__(self, mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, detectableStatesNum, p_fail, add_delay, eq):
		self.goTime = sys.maxint
		self.policy = None
		self.patPolicies = None
		self.map = mapToUse
		self.isSolving = False
		self.predictTime = predictTime
		self.eq = eq

		self.reward = reward
		self.penalty = penalty
		self.detectDistance = detectDistance
		self.trajfull = [[], []]
		self.traj = [[], []]
		self.traj_states = [[], []]
		self.trajOffsets = [[] , [] ]
		self.recd_convertedstates = [[], []] 
		self.recd_convertedstatesfull = [[], []] 
		self.recd_statesCurrSession = [[], []] 
		self.recd_statesCurrSessionFull = [[], []] 
	
		self.finalQ = None
		self.maxTime = 0
		self.gotimes = []
		self.p_fail = p_fail
		self.add_delay = add_delay
		self.searchCount = 0
		self.lastSolveAttempt = -1

		global patrollerOGMap
		global attackerOGMap
		

		p_fail = self.p_fail

		self.observableStateLow = detectableStatesNum
		self.observableStateHigh = detectableStatesNum

		self.pmodel = patrol.model.PatrolModel(p_fail, None, patrollerOGMap.theMap())
		
		global reward_dim, found_muE, exact_mu_E, learned_mu_E, learned_weights, wt_data
		if self.map == "boyd2":
			reward_dim = 6
		if self.map == "largeGrid":
			reward_dim = 8
		if self.map == "boydright":
			reward_dim = 5
		if self.map == "boydright2":
			reward_dim = 7

		found_muE = [[0.0]*reward_dim,[0.0]*reward_dim] 
		exact_mu_E = [[0.0]*reward_dim,[0.0]*reward_dim] 
		learned_mu_E=[[0.0]*reward_dim,[0.0]*reward_dim] 
		learned_weights=[[0.0]*reward_dim,[0.0]*reward_dim] # needed for compute reldiff
		wt_data=numpy.empty([2, reward_dim], dtype=float)
	
		self.pmodel.gamma = 0.99

		## Create Model
		self.goalStates = [self.getStateFor(goalPos), ]
		self.goalState = self.goalStates[0]
		print ("self.goalState",self.goalState)

		model = patrol.model.AttackerModel(p_fail, attackerOGMap.theMap(), self.predictTime, self.goalStates[0])
		model.gamma = 0.99

		self.model = model
		self.startState = self.getStateFor(startPos)


	def getStateFor(self, position):
		# will have to set the time component of the state correctly based on the current rospy time relative to when the attacker mdp's policy starts
		global attackerOGMap
		
		s = attackerOGMap.toState(position, True)

		if get_time() >= self.goTime:
			s.time = fromRospyTime(get_time() - self.goTime) + self.mdpTimeOffset
			s.time = min(s.time, self.maxTime-1)
			return s
		return s

	def getPositionFor(self, mdpState):
		global attackerOGMap

		return attackerOGMap.toPos(mdpState)


	def latestPolicy(self):
		return self.policy

	def goNow(self):

		if self.finalQ is not None and not self.goTime < sys.maxint:

			# check if there's something in the queue, if so we've got a new goTime
			try:
				newStuff = self.finalQ.get(False)
				global startat, BatchIRLflag
				# if BatchIRLflag == False:
				# 	global startat, startat_attpolicyI2RL
				# 	startat = startat_attpolicyI2RL
	
				print("startat for goNow loop: ", startat)
				self.p.join()
				#print("\n Got the queue items at: " + str(get_time()))
				
				policies = newStuff[0]
				valuearr = newStuff[1]

				#print("Pickling policy at: " + str(get_time()))
				
				f = open(get_home() + "/patrolstudy/toupload/attackerpolicy.log", "w")
				pickle.dump((policies, valuearr), f)
				f.close()

				self.maxTime = newStuff[2]
				calcStart = newStuff[3]
				patrollerStartStates = newStuff[4]
				patrollerTimes = newStuff[5]

				print("Finding Value Per timestep: " + str(get_time()))

				totalMaxValue = -sys.maxint - 1
				totalBestTime = -1
				totalBestPolicy = 0

				for (idx, values) in enumerate(valuearr):
					maxValue = -sys.maxint - 1
					bestTime = -1
	
					attackerStartState = self.startState
					for i in range(fromRospyTime(get_time() - calcStart), self.predictTime):
						attackerStartState.time = i
						if attackerStartState in values.keys():
							#print "v[attStart]"+str((i, values[attackerStartState]))
							if values[attackerStartState] > maxValue:
								maxValue = values[attackerStartState]
								bestTime = i
					
					if maxValue > totalMaxValue:
						totalMaxValue = maxValue
						totalBestTime = bestTime
						totalBestPolicy = idx
	
				print("\n Start Search at ",fromRospyTime(get_time() - calcStart),\
					 "\n BestTime ", totalBestTime, " MaxValue ", totalMaxValue)

				global mapToUse
				
				if totalBestTime >= 0 and totalMaxValue > 0:
					goTime = toRospyTime(totalBestTime) + calcStart
					self.policy = policies[totalBestPolicy]
				
					self.goalState = self.goalStates[totalBestPolicy] 
				else:
					goTime = sys.maxint
					self.finalQ = None  # didn't get a go time, retry


				self.gotimes.append( (goTime, get_time(), totalMaxValue, patrollerStartStates, patrollerTimes, totalBestTime, totalBestPolicy, mapToUse, getTimeConv() ) )
				f = open(get_home() + "/patrolstudy/toupload/gotimes.log", "w")
				pickle.dump(self.gotimes, f)
				f.close()

				self.mdpTimeOffset = totalBestTime
				self.goTime = goTime
				
				print("\n Got GoTime of "  + str(self.goTime))
				
			except Queue.Empty:
				pass

		return get_time() >= self.goTime

	def patrollerModel(self):
		return self.pmodel

	def addPercept(self, patroller, state, time):
		self.traj[patroller].append((state, time))
		return

	def addStates(self):
		# add states after a regular interval
		global lastcalltimeaddstates
		current_time = rospy.Time.now().to_sec()
		if current_time - lastcalltimeaddstates >= getTimeConv():# 3* getTimeConv()
			for patroller in range(len(self.traj)):
				# if (not not convertObsToTrajectory(self.traj[patroller], glbltrajstarttime)):
				global glbltrajstarttime
				new_stateactions = convertObsToTrajectory(self.traj[patroller], glbltrajstarttime)
				# if not not new_stateactions:
				#print "patroller"+str(patroller)+" - new_stateactions:"+str(new_stateactions)
				for sa in new_stateactions: 
					self.traj_states[patroller].append(sa) 

				global patrtraj_len
				patrtraj_len[patroller] = len(self.traj_states[patroller])
				# print "length after update "+str(patrtraj_len[patroller])
				lastcalltimeaddstates = rospy.Time.now().to_sec()

			'''
			Instead of waiting for visibility of both patrollers,
			start recording None for non-visible one from the
			beginning of moment first one is visible.
			Sending none for both will be useless.
			'''
			if (len(self.traj_states[0]) > len(self.traj_states[1]) )\
					and (len(self.traj_states[1]) == 0):
					print "filling None states for 1"
					for sa in self.traj_states[0]:
						self.traj_states[1].append(None)
			if (len(self.traj_states[1]) > len(self.traj_states[0]) )\
					and (len(self.traj_states[0]) == 0):
					print "filling None states for 0"
					for sa in self.traj_states[1]:
						self.traj_states[0].append(None)

			# need to happen only after storing states for all the patrollers
			global glbltrajstarttime
			glbltrajstarttime = fromRospyTime(get_time())

		# if (not not convertObsToDiscreteStates(self.traj[patroller], glbltrajstarttime)):
		# 	# list of state objects
		# 	conv_states = convertObsToDiscreteStates(self.traj[patroller], glbltrajstarttime)
		# 	# print("patroller "+str(patroller)+" current conv_states "+str(conv_states))
		# 	global patrtraj_len
		# 	patrtraj_len[patroller] = len(conv_states)

		# print "min(patrtraj_len): "+str(min(patrtraj_len))
		return

	def addPerceptFull(self, patroller, state, time):
		global glbltrajstarttime
		pass
		# self.trajfull[patroller].append((state, time))
					
	def recordStateSequencetoFile(self):
		
		f = open(get_home() + "/patrolstudy/toupload/recd_convertedstates.log", "w")
		f.write("")
		f.close()

		# record trajectory directly in file
		print("desired length reached. recordStateSequence to file ") 
		outtraj = ""
		global patrtraj_len
		
		for patroller in range(0,len(patrtraj_len)): 
			global glbltrajstarttime
			if not not self.traj_states[patroller]:#convertObsToTrajectory(self.traj[patroller], glbltrajstarttime):
				conv_traj = self.traj_states[patroller] #convertObsToTrajectory(self.traj[patroller], glbltrajstarttime)
				for sap in conv_traj: 
					if (sap is not None): 
						s = sap[0].location[0:3]
						#print(s)
						#print(sap[1].__class__.__name__)
						outtraj += str(s[0])+","+str(s[1])+","+str(s[2])+","
						if sap[1].__class__.__name__ == "PatrolActionMoveForward":
							outtraj += "PatrolActionMoveForward"
						elif sap[1].__class__.__name__ == "PatrolActionTurnLeft":
							outtraj += "PatrolActionTurnLeft"
						elif sap[1].__class__.__name__ == "PatrolActionTurnRight":
							outtraj += "PatrolActionTurnRight"
						elif sap[1].__class__.__name__ == "PatrolActionTurnAround":
							outtraj += "PatrolActionTurnAround"
						else:
							outtraj += "PatrolActionStop"
					else:
						outtraj += "None"
					outtraj += "\n"
			outtraj += "ENDREC\n"
		
		f = open(get_home() + "/patrolstudy/toupload/recd_convertedstates.log", "w")
		f.write(outtraj)
		f.close()
		return

	def recordFullStateSequencetoFile(self):
		f = open(get_home() + "/patrolstudy/toupload/recd_convertedstates_full.log", "w")
		f.write("")
		f.close()

		# record traj directly in file
		print("desired length reached. recordFullStateSequence to file ") 
		outtraj = ""
		global patrtraj_len
		for patroller in range(0,len(patrtraj_len)): 
			global glbltrajstarttime
			if not not self.traj_states[patroller]:#convertObsToTrajectory(self.traj[patroller], glbltrajstarttime):
				conv_traj = self.traj_states[patroller] #convertObsToTrajectory(self.traj[patroller], glbltrajstarttime)
				for sap in conv_traj: 
					if (sap is not None): 
						s = sap[0].location[0:3]
						#print(s)
						#print(sap[1].__class__.__name__)
						outtraj += str(s[0])+","+str(s[1])+","+str(s[2])+","
						if sap[1].__class__.__name__ == "PatrolActionMoveForward":
							outtraj += "PatrolActionMoveForward"
						elif sap[1].__class__.__name__ == "PatrolActionTurnLeft":
							outtraj += "PatrolActionTurnLeft"
						elif sap[1].__class__.__name__ == "PatrolActionTurnRight":
							outtraj += "PatrolActionTurnRight"
						elif sap[1].__class__.__name__ == "PatrolActionTurnAround":
							outtraj += "PatrolActionTurnAround"
						else:
							outtraj += "PatrolActionStop"
					else:
						outtraj += "None"
					outtraj += "\n"
			outtraj += "ENDREC\n"
		f = open(get_home() + "/patrolstudy/toupload/recd_convertedstates_full.log", "w")
		f.write(outtraj)
		f.close()
		return
	
	def readStateSequencefromFile(self):
		import string 
		f = open(get_home() + "/patrolstudy/toupload/recd_convertedstates.log", "r")
		patroller = 0
# 		print_once = 1
		for line in f: 
			loc_act = string.split(line.strip(),",") 
			if loc_act[0] == "ENDREC":
				patroller = patroller +1
			elif loc_act[0] == "None":
				self.recd_convertedstates[patroller].append(None)
			else:
				(row, col, direction)=(int(loc_act[0]),int(loc_act[1]),int(loc_act[2]))
				# arr_loc = [float(loc_act[0]),float(loc_act[1]),float(loc_act[2])]
				# print("\n\nread location",(row, col, direction))
				state= patrollerOGMap.buildState((row, col, direction),False)
# 				if isvisible(state, self.observableStateLow):
				if loc_act[3] == "PatrolActionMoveForward":
					action = PatrolActionMoveForward()
				elif loc_act[3] == "PatrolActionTurnLeft":
					action = PatrolActionTurnLeft()
				elif loc_act[3] == "PatrolActionTurnRight":
					action = PatrolActionTurnRight()
				elif loc_act[3] == "PatrolActionStop":
					action = PatrolActionStop()
				self.recd_convertedstates[patroller].append((state,action))
				# print("\n\nappended ",(state,action))
				
				
# 					if print_once == 1:
# 						print("not None state read from file \n ",self.recd_convertedstates)
# 						print_once = 0
# 				else:
# 					self.recd_convertedstates[patroller].append(None)

		f.close()
		return
		#print("\n\nappended last state ",(state,action))
		#datacollected()
	
	def readFullStateSequencefromFile (self):
		import string 
		f = open(get_home() + "/patrolstudy/toupload/recd_convertedstates_full.log", "r")
		patroller = 0
# 		print_once = 1
		for line in f: 
			loc_act = string.split(line.strip(),",") 
			if loc_act[0] == "ENDREC":
				patroller = patroller +1
			elif loc_act[0] == "None":
				self.recd_convertedstatesfull[patroller].append(None)
			else:
				(row, col, direction)=(int(loc_act[0]),int(loc_act[1]),int(loc_act[2]))
				
				# arr_loc = [float(loc_act[0]),float(loc_act[1]),float(loc_act[2])]
				# print("\n\nread location",(row, col, direction))
				state= patrollerOGMap.buildState((row, col, direction),False)
				
				if loc_act[3] == "PatrolActionMoveForward":
					action = PatrolActionMoveForward()
				elif loc_act[3] == "PatrolActionTurnLeft":
					action = PatrolActionTurnLeft()
				elif loc_act[3] == "PatrolActionTurnRight":
					action = PatrolActionTurnRight()
				elif loc_act[3] == "PatrolActionStop":
					action = PatrolActionStop()
				self.recd_convertedstatesfull[patroller].append((state,action))
		
	
	def getStartState(self):
		return self.startState

	def getGoalState(self):
		return self.goalState

	def update(self):
		self.updateWithAlg("NG", False)
	
	def updateWithAlg(self, alg, patrollerdelay):
		
		global obstime, sessionStart, num_Trajsofar, print_once, \
		patroller0LastSeenAt, patroller1LastSeenAt, BatchIRLflag, \
		minGapBwDemos, lineFoundWeights, lineFeatureExpec, learned_weights, \
		num_Trajsofar, lineLastBeta, lineLastZS, lineLastZcount, sessionFinish, \
		patrtraj_len, glbltrajstarttime, startat_attpolicyI2RL
		
		if not self.isSolving and self.policy is None:

			global obstime, sessionStart, sessionNumber, normedRelDiff, \
			stopI2RLThresh, sessionFinish, num_Trajsofar, print_once, \
			patroller0LastSeenAt, patroller1LastSeenAt, BatchIRLflag, \
			minGapBwDemos, lastQ, min_IncIRL_InputLen,\
			patrtraj_len, min_BatchIRL_InputLen, startat_attpolicyI2RL, \
			glbltrajstarttime, useRegions

			if ((self.policy is None) and (max(len(a) for a in self.traj)\
			> self.predictTime)) and \
			(min(patrtraj_len) > min_BatchIRL_InputLen and\
			BatchIRLflag == True) or \
			(min(patrtraj_len) > min_IncIRL_InputLen and \
			(min_IncIRL_InputLen*sessionNumber <= min_BatchIRL_InputLen)  \
			and sessionFinish == True and BatchIRLflag == False):

				global sessionStart, num_Trajsofar, print_once, BatchIRLflag, \
				lineFoundWeights, lineFeatureExpec, learned_weights, sessionNumber, \
				num_Trajsofar, lineLastBeta, lineLastZS, lineLastZcount, sessionFinish, \
				patrtraj_len, startat, glbltrajstarttime, recording_mode
				min_BatchIRL_InputLen, startat_attpolicyI2RL, useRegions

				#print("sessionStart, sessionFinish, BatchIRLflag: "+str(sessionStart)\
				#	+str(sessionFinish)+str(BatchIRLflag))

				# print "patrtraj_len"+str(patrtraj_len)
				# print "min(patrtraj_len) > min_IncIRL_InputLen"+str((min(patrtraj_len) > min_IncIRL_InputLen))

				self.isSolving = True
				self.q = multiprocessing.Queue()
				# print("calling irlsolve")
				# current session started
				sessionStart = True
				# i2rl specific flag
				sessionFinish = False
				
				print("beginning for session")
				# print("new q for sessionNumber")
 				print(sessionNumber)

				if BatchIRLflag == True:
					current_time2 = rospy.Time.now().to_sec()
					f = open(get_home()+"/perception_times.txt", "a")
					f.write("\n"+"Perception time is:"+str(current_time2-start_forGoTime))
					f.close()
					print "Perception time is:"+str(current_time2-start_forGoTime)

				input_traj = [[], []]
				input_trajfull = [[], []]

				global patrtraj_len
				for patroller in range(len(self.traj_states)):
					self.trajfull[patroller] = self.traj_states[patroller][:]
					if BatchIRLflag == True:
						input_traj[patroller] = self.traj_states[patroller][0:min_BatchIRL_InputLen]
						self.traj_states[patroller] = self.traj_states[patroller][min_BatchIRL_InputLen:]
						input_trajfull[patroller] = self.trajfull[patroller][0:min_BatchIRL_InputLen]
						self.trajfull[patroller] = self.trajfull[patroller][min_BatchIRL_InputLen:]
					else:
						input_traj[patroller] = self.traj_states[patroller][:min_IncIRL_InputLen]
						self.traj_states[patroller] = self.traj_states[patroller][min_IncIRL_InputLen:]
						input_trajfull[patroller] = self.trajfull[patroller][:min_IncIRL_InputLen]
						self.trajfull[patroller] = self.trajfull[patroller][min_IncIRL_InputLen:]
					patrtraj_len[patroller] = len(self.traj_states[patroller])

				print "input trajectories:\n"+str(input_traj)+"\n"

				f = open("/tmp/timestamps","a")
				f.write("\n session "+str(sessionNumber))
				f.close()
				
				# if not not lineFoundWeights:
				# 	if lineFoundWeights[-1]!='\n':
				# 		lineFoundWeights=lineFoundWeights+"\n"
				# 		lineFeatureExpec=lineFeatureExpec+"\n"
				# 		lineLastBeta=lineLastBeta+"\n"
				# 		lineLastZS=lineLastZS+"\n"
				
				global use_em, wt_data, normedRelDiff
				if use_em:
					self.p = multiprocessing.Process(target=emirlsolve, args=(self.q, currentsessionpercepts, patrollerdelay, alg, self.pmodel, self.map, self.eq, self.observableStateLow) )
				else:
					# self.p = multiprocessing.Process(target=irlsolve, args=(self.q, currentsessionpercepts, patrollerdelay, alg, self.pmodel, self.map, self.eq, self.observableStateLow,\
					# lineFoundWeights, lineFeatureExpec, learned_weights, num_Trajsofar, lineLastBeta, lineLastZS, \
					# lineLastZcount, BatchIRLflag, wt_data, normedRelDiff, traj_begintime) )

					self.p = multiprocessing.Process(target=irlsolve_recdStates, args=(
					self.q, input_traj, patrollerdelay, alg, self.pmodel, self.map, self.eq, \
					self.observableStateLow, lineFoundWeights, lineFeatureExpec, \
					learned_weights, num_Trajsofar, BatchIRLflag, wt_data, \
					normedRelDiff, glbltrajstarttime, input_trajfull, useRegions))

				#print("multiprocessing.Process before self.p.start() ")
				self.p.start()

				# num_Trajsofar=num_Trajsofar+1 increment happened in boydirl
				

		elif (self.isSolving and sessionStart == True and \
			self.patPolicies is None):

			global sessionStart, num_Trajsofar, print_once, BatchIRLflag, \
				lineFoundWeights, lineFeatureExpec, learned_weights, \
				num_Trajsofar, sessionFinish, \
				wt_data, normedRelDiff, lastQ, \
				Q_wait_timestamp, lineFeatureExpecfull

			try:
				temp = self.q.get(False)
				self.p.join()

				[lineFoundWeights, lineFeatureExpec, learned_weights, num_Trajsofar, \
				 sessionFinish, wt_data, normedRelDiff, lastQ, lineFeatureExpecfull, newStuff] = temp

				current_time = rospy.Time.now().to_sec()
				# print("sessionFinish: "+str(sessionFinish)+" at time "+str(current_time))
				# print("lineFoundWeights: ",lineFoundWeights)

				if not not newStuff and (sessionFinish == True or BatchIRLflag == True):
					# CONVERGENCE
					if ((min_IncIRL_InputLen * sessionNumber >= min_BatchIRL_InputLen) or (BatchIRLflag == True)):  # (lastQ > 1.9) (normedRelDiff < stopI2RLThresh) or (BatchIRLflag == True):

						global BatchIRLflag, \
						lineFeatureExpec, lineFeatureExpecfull
						print("\n irl converged. Learning finished at " + str(irlfinish_time))
						print("lineFoundWeights " + lineFoundWeights+"\nlineFeatureExpec \n" + lineFeatureExpec)

						self.T = newStuff.pop()
						self.patPolicies = newStuff
						print( "\n keys of policy: \n" + str(self.patPolicies[0]._policy.keys()) + "\n")
						#
						return None

					# ELSE continue for next session of i2rl
					elif (BatchIRLflag == False and sessionFinish == True):

						global sessionStart, sessionNumber, glbltrajstarttime_attpolicyI2RL, \
						sessionFinish, num_Trajsofar, print_once, min_IncIRL_InputLen, \
						lineFeatureExpec, found_muE, lastile, currentile, lineFoundWeights

						lastile=currentile
						#compute ILE
						global mapToUse, patrollersuccessrate, lineFoundWeights
						f = open(get_home() + "/patrolstudy/toupload/data_ile.log", "w")
						f.write("")
						f.close()

						global patrollersuccessrate
						args = ['boydsimple_t', ]
						p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
											 stderr=subprocess.PIPE)
						outtraj = ""
						outtraj += mapToUse + "\n"
						outtraj += str(patrollersuccessrate)
						(transitionfunc, stderr) = p.communicate(outtraj)
						
						args = ['boydile', ]
						p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
											 stderr=subprocess.PIPE)
						outtraj = ""
						outtraj += mapToUse + "\n"
						outtraj += transitionfunc + "ENDT" + "\n"
						outtraj += lineFoundWeights + "\n"
						f = open(get_home() + "/patrolstudy/toupload/data_ile.log", "a")
						f.write(outtraj)
						f.close()
						(ile, stderr) = p.communicate(outtraj)
						iles=[]
						iles=ile.split('\n')
						currentile=(float(iles[0])+float(iles[1]))/2
						print("ile diff \n " + str(lastile-currentile))

						# print "reached if for # ELSE continue for next session"
						sessionStart = False
						self.isSolving = False
						num_Trajsofar += 1
						# sessionNumber should be updated only here
						sessionNumber += 1
						print_once = 0  # print once empty q for next session

				# NOT USED
				# else if session call didn't return as expected
				# call solver again without deleting (but enlarged) traj
				elif not not newStuff and (BatchIRLflag == False and sessionFinish == False):

					global print_once
					self.isSolving = False
					self.q = None
					print("\n need another call for session " + str(sessionNumber))
					print_once = 0

			except Queue.Empty:  # no item in queue, boydirl is not done
				global print_once
				if print_once == 0:
					pass
					#print("print_once queue self.q empty")
				print_once = 1
				pass

		if self.finalQ is None and self.patPolicies is not None and rospy.get_time() - self.lastSolveAttempt > getTimeConv():
			# compute attacker policy in new process and share results in new queue
			# print("\n compute attacker policy because self.patPolicies is not None")
			self.finalQ = multiprocessing.Queue()

			self.p = multiprocessing.Process(target=getattackerpolicy, args=(self.finalQ, self.patPolicies, self.traj, self.pmodel,\
															 self.goalState, self.reward, self.penalty, self.detectDistance,\
															  self.predictTime, self.add_delay, self.p_fail, self.map, self.T) )
				
			self.p.start()
			self.lastSolveAttempt = rospy.get_time()
	
	def updateWithAlgRecTraj(self,alg,patrollerdelay):
		
# 		print("inside updateWithAlgRecTraj")
		if not self.isSolving and self.policy is None:
			global sessionStart, num_Trajsofar, print_once, BatchIRLflag, \
			lineFoundWeights, lineFeatureExpec, learned_weights, \
			num_Trajsofar, sessionFinish, \
			patrtraj_len, glbltrajstarttime, sess_starttime, useRegions
			
			# how should input be computed? 
			# if min input for batchirl is not stored yet  
			# or if number of required sessions are not reached
			# for each patroller, if available data is larger than
			# desired min_BatchIRL_InputLen, copy only desired part
			# or copy all. Chop the store part from original list 
			
			if ( (self.policy is None and BatchIRLflag == True) or \
			(self.policy is None and BatchIRLflag == False and sessionStart == False) or \
			(self.policy is None and BatchIRLflag == False and sessionStart == True \
			and sessionFinish == False) ):
			
				print("inside updateWithAlgRecTraj first if")
			# all none commented bcz it will happen in case of high occlusion and low input size
# 				allnone = 1 #   
# 				for lst in self.recd_statesCurrSession:
# 					for x in lst:
# 						if x is not None:
# 							allnone = 0
				# needed for both			
				emptytraj = (not self.recd_statesCurrSession[0] \
							and not self.recd_statesCurrSession[1])
#				while allnone == 1 or emptytraj == 1:
				while emptytraj == 1:
					if (not self.recd_convertedstates[0] \
						and not self.recd_convertedstates[1]):
						print(" no further states left to be read  ")
						emptyinput()
						
					#print("can't send empty traj for multiprocessing.Process! ")
					if BatchIRLflag == True and \
					(max(len(x) for x in self.recd_convertedstates)>= min_BatchIRL_InputLen) \
					and (sessionStart == False):
						# if batch irl input is needed
						#print("\n BatchIRLflag == True create input ")
						for patroller in range(0,len(self.recd_convertedstates)):
							if len(self.recd_convertedstates[patroller]) >= min_BatchIRL_InputLen:
								print("\n len(self.recd_convertedstates[patroller]) >= min_BatchIRL_InputLen ")
#								print(str(len(self.recd_convertedstates[patroller])))
								self.recd_statesCurrSession[patroller] = self.recd_convertedstates[patroller][0:min_BatchIRL_InputLen]
								self.recd_convertedstates[patroller] = self.recd_convertedstates[patroller][min_BatchIRL_InputLen:]
								self.recd_statesCurrSessionFull[patroller] = self.recd_convertedstatesfull[patroller][0:min_BatchIRL_InputLen]
								self.recd_convertedstatesfull[patroller] = self.recd_convertedstatesfull[patroller][min_BatchIRL_InputLen:]
#								print(str(len(self.recd_convertedstates[patroller])))
							else:
#								print(str(len(self.recd_convertedstates[patroller])))
								self.recd_statesCurrSession[patroller] = self.recd_convertedstates[patroller][:]
								self.recd_convertedstates[patroller] = []
								self.recd_statesCurrSessionFull[patroller] = self.recd_convertedstatesfull[patroller][:]
								self.recd_convertedstatesfull[patroller] = []
#								print(str(len(self.recd_convertedstates[patroller])))
					elif BatchIRLflag == False and \
					(min_IncIRL_InputLen*sessionNumber <= min_BatchIRL_InputLen) and \
					 (sessionStart == False):
						# or more sessions are needed
						#print("\n BatchIRLflag == False create input ")
						for patroller in range(0,len(self.recd_convertedstates)):
							if len(self.recd_convertedstates[patroller]) >= min_IncIRL_InputLen:
								print("\n len(self.recd_convertedstates[patroller]) >= min_IncIRL_InputLen ")
#								print(str(len(self.recd_convertedstates[patroller])))
								self.recd_statesCurrSession[patroller] = self.recd_convertedstates[patroller][0:min_IncIRL_InputLen]
								self.recd_convertedstates[patroller] = self.recd_convertedstates[patroller][min_IncIRL_InputLen:]
								self.recd_statesCurrSessionFull[patroller] = self.recd_convertedstatesfull[patroller][0:min_BatchIRL_InputLen]
								self.recd_convertedstatesfull[patroller] = self.recd_convertedstatesfull[patroller][min_BatchIRL_InputLen:]
#								print(str(len(self.recd_convertedstates[patroller])))
							else:
#								print(str(len(self.recd_convertedstates[patroller])))
								self.recd_statesCurrSession[patroller] = self.recd_convertedstates[patroller][:]
								self.recd_convertedstates[patroller] = []
								self.recd_statesCurrSessionFull[patroller] = self.recd_convertedstatesfull[patroller][:]
								self.recd_convertedstatesfull[patroller] = []
#								print(str(len(self.recd_convertedstates[patroller])))
					else:
						print("insufficient states for batch. BatchIRLflag == True and \
					(max(len(x) for x in self.recd_convertedstates)< min_BatchIRL_InputLen) ")
# 					allnone = 1
# 					for lst in self.recd_statesCurrSession:
# 						for x in lst:
# 							if x is not None:
# 								allnone = 0
					emptytraj = (not self.recd_statesCurrSession[0] \
								and not self.recd_statesCurrSession[1])
				print("input size for current session "+str((max(len(x) for x in self.recd_statesCurrSession))))
				
				self.isSolving = True
				self.q = multiprocessing.Queue()
				sessionStart = True
				sessionFinish = False
				
				print("beginning for session ")
 				print(sessionNumber)
 				sess_starttime=rospy.Time.now().to_sec()
 				#print(" at time "+str(sess_starttime))
 				
				input = [[], []]
				input[0] = self.recd_statesCurrSession[0][:]
				input[1] = self.recd_statesCurrSession[1][:]
				
				trajFull = [[], []]
				trajFull[0] = self.recd_statesCurrSessionFull[0][:]
				trajFull[1] = self.recd_statesCurrSessionFull[1][:]
				
 				
				f = open("/tmp/studyresults2","a") 
				f.write("") 
				# f.write("\n session "+str(sessionNumber)+" starting at time "+str(sess_starttime))
				# f.write("\n size(demo) for session "+str(sessionNumber)+": "\
				#	+str(len(self.recd_statesCurrSession[0]))+", "+str(len(self.recd_statesCurrSession[1])) )
				f.close() 
				
				if not not lineFoundWeights:
					if lineFoundWeights[-1]!='\n': 
						lineFoundWeights=lineFoundWeights+"\n"
						lineFeatureExpec=lineFeatureExpec+"\n"
				
				global wt_data, normedRelDiff, useRegions
				traj_begintime = glbltrajstarttime
				self.p = multiprocessing.Process(target=irlsolve_recdStates, args=(self.q, input, patrollerdelay, alg, self.pmodel, self.map, self.eq, \
																		self.observableStateLow, lineFoundWeights, lineFeatureExpec, learned_weights, \
																		num_Trajsofar, BatchIRLflag, wt_data, normedRelDiff, traj_begintime, trajFull,\
																		useRegions) )
				self.p.start()
				
		elif (self.isSolving and sessionStart == True and\
			  self.patPolicies is None):
			
			global sessionStart, num_Trajsofar, print_once, BatchIRLflag, \
			lineFoundWeights, lineFeatureExpec, learned_weights, \
			num_Trajsofar, sessionFinish, \
			wt_data, normedRelDiff, lastQ, sess_starttime, sess_endtime, \
			Q_wait_timestamp, lineFeatureExpecfull
			
			try:
				temp=self.q.get(False)
				self.p.join() 
				
 				[lineFoundWeights, lineFeatureExpec, learned_weights, num_Trajsofar, \
				sessionFinish, wt_data, normedRelDiff, lastQ, lineFeatureExpecfull, newStuff]=temp
				
				current_time=rospy.Time.now().to_sec()
				#print("sessionFinish: "+str(sessionFinish)+" at time "+str(current_time))
				#print("lineFoundWeights: ",lineFoundWeights)
				
				if not not newStuff and (sessionFinish==True or BatchIRLflag==True):
					
					# CONVERGENCE
					if (min_IncIRL_InputLen*sessionNumber >= min_BatchIRL_InputLen) or (BatchIRLflag == True): #(lastQ > 1.9) (normedRelDiff < stopI2RLThresh) or (BatchIRLflag == True):
						
						global irlfinish_time, cum_learntime, sess_starttime, BatchIRLflag, \
						lineFeatureExpec, found_muE, lineFeatureExpecfull, exact_mu_E
						
						irlfinish_time = rospy.Time.now().to_sec()
						print("\n irl converged. Learning finished at "+str(irlfinish_time))
						print("lineFoundWeights "+lineFoundWeights)
 						cum_learntime = cum_learntime+irlfinish_time-sess_starttime

						self.T = newStuff.pop()
						self.patPolicies = newStuff
						# code computing LBA 
						# print("started opening correctboydpolicy")

						if mapToUse == "largeGridPatrol":
							correctboydpolicy = get_home() + "/patrolstudy/largeGridpolicy_mdppatrol"
						if mapToUse == "boydright":
							correctboydpolicy = get_home() + "/patrolstudy/boydright_policy"
						if mapToUse == "boydright2":
							correctboydpolicy = get_home() + "/patrolstudy/boydright2_policy"
						if mapToUse == "boyd2":
							correctboydpolicy = get_home() + "/patrolstudy/boydpolicy_mdppatrolcontrol"
						
 						# correctboydpolicy = get_home() + "/patrolstudy/onlyLBA_patpolicy.log"
						f = open(correctboydpolicy,"r")
						global boydpolicy
						for stateaction in f:
							temp=stateaction.strip().split(" = ")
							if len(temp) < 2: continue
							state = temp[0]
							action = temp[1]
							state = state[1 : len(state) - 1]
							pieces = state.split(",")
							ps = (int(pieces[0]), int(pieces[1]), int(pieces[2]) )
							boydpolicy[ps] = action
							
						f.close()
						
						
						policyaccuracy = []
						lba=0.0
						i=0
						global mapToUse
						if mapToUse == "boydright" or mapToUse == "boyd2" or mapToUse == "boydright2":
						
							for patroller in self.patPolicies:
								totalsuccess = 0
								totalstates = 0
								
								if (patroller.__class__.__name__ == "MapAgent"):
									f1 = open(get_home() + "/patrolstudy/output_tracking/unmatchedactionspat"+str(i)+"_"+str(min_BatchIRL_InputLen)+".log", "w")
									f2 = open(get_home() + "/patrolstudy/output_tracking/actionspat"+str(i)+"_"+str(min_BatchIRL_InputLen)+".log", "w")

									for state in self.pmodel.S():
										if state in patroller._policy:# check key existence 
											action = patroller.actions(state).keys()[0]
											if action.__class__.__name__ == "PatrolActionMoveForward":
												action = "MoveForwardAction"
											elif action.__class__.__name__ == "PatrolActionTurnLeft":
												action = "TurnLeftAction"
											elif action.__class__.__name__ == "PatrolActionTurnRight":
												action = "TurnRightAction"
											elif action.__class__.__name__ == "PatrolActionTurnAround":
												action = "TurnAroundAction"
											else:
												action = "StopAction"
											
										ps = (int(state.location[0]), int(state.location[1]), int(state.location[2]))
										if ps in boydpolicy.keys():# and ps[1] <= 2:
											
											totalstates += 1
											#print("a matching state for patroller "+str(i))
											#print("learned patpolicy action ", str(action))
											if state in patroller._policy:# check key existence 
												f2.write(str(ps)+"="+str(action)+"\n")
												if (boydpolicy[ps] == action):
													#print("found a matching action for patroller "+str(i))
													totalsuccess += 1
												else:
													f1.write(str(ps)+"="+str(action)+"\n")
									
									f1.close()
									f2.close()
								#print("totalstates, totalsuccess: "+str(totalstates)+", "+str(totalsuccess))
								if float(totalstates) == 0:
									break
								
								lba=float(totalsuccess) / float(totalstates)
								if mapToUse == "boyd2":
									lba=(50*lba)+5 #scaling magnitude and subtracting offset
								else:
									lba = (50 * lba)
								policyaccuracy.append(lba)
								
								print("LBA["+str(i)+"] = "+str(policyaccuracy[i]))
								i = i+1
						

						global mapToUse, patrollersuccessrate, lineFoundWeights
						f = open(get_home() + "/patrolstudy/toupload/data_ile.log", "w")
						f.write("")
						f.close()
					
						global patrollersuccessrate
						args = [ 'boydsimple_t', ]
					
						p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
					
						outtraj = ""
						
						outtraj += mapToUse + "\n"
						outtraj += str(patrollersuccessrate)
					
						(transitionfunc, stderr) = p.communicate(outtraj)
					
						args = [ 'boydile', ]
						p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
						outtraj = ""
						outtraj += mapToUse + "\n"
						outtraj += transitionfunc + "ENDT"+"\n"
						outtraj += lineFoundWeights + "\n"
					
						f = open(get_home() + "/patrolstudy/toupload/data_ile.log", "a")
						f.write(outtraj)
						f.close()

						(ile, stderr) = p.communicate(outtraj)
						print("ILE \n "+ile)
						
						global reward_dim

						temp_muE = [[0.0]*reward_dim,[0.0]*reward_dim]
	
						temp_muE = [[float(x) for x in \
						lineFeatureExpec.split("\n")[0]. \
						strip("[]").split("], [")[0].split(", ")], \
						[float(x) for x in \
						lineFeatureExpec.split("\n")[0]. \
						strip("[]").split("], [")[1].split(", ")]]
						#print("temp_muE  ",temp_muE)

						for i in range(0,2):
							for j in range(0,len(temp_muE[i])):
								found_muE[i][j] += temp_muE[i][j]
						
						
						temp_muE = [[0.0]*reward_dim,[0.0]*reward_dim]
	
						temp_muE = [[float(x) for x in \
						lineFeatureExpecfull.split("\n")[0]. \
						strip("[]").split("], [")[0].split(", ")], \
						[float(x) for x in \
						lineFeatureExpecfull.split("\n")[0]. \
						strip("[]").split("], [")[1].split(", ")]]
						#print("temp_muE  ",temp_muE)

						for i in range(0,2):
							for j in range(0,len(temp_muE[i])):
								exact_mu_E[i][j] += temp_muE[i][j]
						
						# MCMC Err = |exact-found|/|exact| Why is it going higher than 1 when muE is always positive? 
						temp1=numpy.array(found_muE[0])
						temp2=numpy.array(exact_mu_E[0])
						denom=numpy.linalg.norm(temp2,ord=1)
						if denom==0:

							MCMC_err1=2
							MCMC_err2=2
						else:
							MCMC_err1=numpy.linalg.norm(numpy.subtract(temp2,temp1),ord=1)/denom		
							temp1=numpy.array(found_muE[1])
							temp2=numpy.array(exact_mu_E[1])		
							MCMC_err2=numpy.linalg.norm(numpy.subtract(temp2,temp1),ord=1)/numpy.linalg.norm(temp2,ord=1)
						
						print("MCMC_err1",MCMC_err1,"MCMC_err2",MCMC_err2)
						
						global lastQ
						print(" cum_learntime",cum_learntime)
						
						#f.write("\n session "+str(sessionNumber)+" ends in time "+str(irlfinish_time-sess_starttime))
						#f.write("\n cum_learntime "+str(cum_learntime))
	 					#f.write("\n irl converged. Learning finished at ")
	 					#f.write(str(irlfinish_time))
						
						print "policyaccuracy:"+str(policyaccuracy)
# 						datacollected()
# 						rospy.signal_shutdown('')
						
						if BatchIRLflag == True or BatchIRLflag == False:
							print("writing to studyresults2 ")
							f = open("/tmp/studyresults2","a")
							f.write("\nLBA1:\n"+str(policyaccuracy[0])+\
								"\nLBA2:\n"+str(policyaccuracy[1]))
# 								"\nMCMC_err:\n"+str((MCMC_err1+MCMC_err2)*100/4))
							global reward_dim
							for i in range(0,2):
								for j in range(0,len(temp_muE[i])):
									f.write("\nfound_muE"+str(reward_dim*i+j)+":\n"+str(found_muE[i][j]))

							f.write("\nQ:\n"+str(lastQ)+\
								"\nCUMLEARNTIME:\n"+str(cum_learntime)+"\nILE:\n"+ile)

							f.close()

						return

					# ELSE continue for next session of i2rl
					elif (BatchIRLflag == False and sessionFinish == True): 
						global sessionStart, sessionNumber, \
						sessionFinish, num_Trajsofar, print_once, min_IncIRL_InputLen, lineFeatureExpec, found_muE 

						
 						sess_endtime=rospy.Time.now().to_sec()
 						global cum_learntime
 						cum_learntime = cum_learntime+sess_endtime-sess_starttime
 						
						#f = open("/tmp/studyresults2","a")
						#f.write("\n session "+str(sessionNumber)+" ends in time "+str(sess_endtime-sess_starttime))
						#f.write("\n cum_learntime "+str(cum_learntime))
						#f.close()
						
						# start collecting demonstration
						if (min_IncIRL_InputLen*sessionNumber < min_BatchIRL_InputLen) and BatchIRLflag==False:
							# new demo for next session
							print("(min_IncIRL_InputLen*sessionNumber < min_BatchIRL_InputLen) make input empty")
							self.recd_statesCurrSession = [[], []]
						current_time=rospy.Time.now().to_sec()

						# Read the currently computed feature expectations and accumulate
#						temp_muE = [[0.0]*reward_dim,[0.0]*reward_dim]
	
#						temp_muE = [[float(x) for x in \
#						lineFeatureExpec.split("\n")[0]. \
#						strip("[]").split("], [")[0].split(", ")], \
#						[float(x) for x in \
#						lineFeatureExpec.split("\n")[0]. \
#						strip("[]").split("], [")[1].split(", ")]]

#						print("temp_muE  ",temp_muE)
#						for i in range(0,2):
#							for j in range(0,len(temp_muE[i])):
#								found_muE[i][j] += temp_muE[i][j]

						
# 						#compute LBA
# 						self.T = newStuff.pop()
# 						policies = newStuff
# 						#print("started opening correctboydpolicy")
# 												
# 						correctboydpolicy = get_home() + "/patrolstudy/boydpolicy_mdppatrolcontrol_bkup"
# 						f = open(correctboydpolicy,"r")
# 						global boydpolicy
# 						for stateaction in f:
# 								temp=stateaction.strip().split(" = ")
# 								if len(temp) < 2: continue
# 								state = temp[0]
# 								action = temp[1]
# 								state = state[1 : len(state) - 1]
# 								pieces = state.split(",")
# 								ps = (int(pieces[0]), int(pieces[1]), int(pieces[2]) )
# 						
# 								boydpolicy[ps] = action
# 						f.close()
# 						
# 						# code computing LBA 
# 						policyaccuracy = 0
# 						global mapToUse
# 						if mapToUse == "boyd" or mapToUse == "boyd2":
# 							totalsuccess = 0
# 							totalstates = 0
# 		
# 							for patroller in policies:
# 								if (patroller.__class__.__name__ == "MapAgent"):
# 									for state in self.pmodel.S():
# 										if state in patroller._policy:# check key existence 
# 											action = patroller.actions(state).keys()[0]
# 											if action.__class__.__name__ == "PatrolActionMoveForward":
# 												action = "MoveForwardAction"
# 											elif action.__class__.__name__ == "PatrolActionTurnLeft":
# 												action = "TurnLeftAction"
# 											elif action.__class__.__name__ == "PatrolActionTurnRight":
# 												action = "TurnRightAction"
# 											elif action.__class__.__name__ == "PatrolActionTurnAround":
# 												action = "TurnAroundAction"
# 											else:
# 												action = "StopAction"
# 											
# 										ps = (int(state.location[0]), int(state.location[1]), int(state.location[2]))
# 										if ps in boydpolicy.keys():# and ps[1] <= 2:
# 											
# 											totalstates += 1
# 											#print("a matching state, boydpolicy[ps] ",str(boydpolicy[ps]))
# 											#print("learned patpolicy action ", str(action))
# 											if state in patroller._policy:# check key existence 
# 												if (boydpolicy[ps] == action):
# 													#print("found a matching action")
# 													totalsuccess += 1
# 							# print("totalstates, totalsuccess: "+str(totalstates)+", "+str(totalsuccess))
# 							policyaccuracy = float(totalsuccess) / float(totalstates)
# 
# 						
# 						global mapToUse, patrollersuccessrate, lineFoundWeights
# 						f = open(get_home() + "/patrolstudy/toupload/data_ile.log", "w")
# 						f.write("")
# 						f.close()
# 					
# 						global patrollersuccessrate
# 						args = [ 'boydsimple_t', ]
# 					
# 						p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
# 					
# 						outtraj = ""
# 						
# 						outtraj += mapToUse + "\n"
# 						outtraj += str(patrollersuccessrate)
# 					
# 						(transitionfunc, stderr) = p.communicate(outtraj)
# 					
# 						args = ["boydile", ]
# 						p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
# 						outtraj = ""
# 						outtraj += mapToUse + "\n"
# 						outtraj += transitionfunc + "ENDT"+"\n"
# 						outtraj += lineFoundWeights + "\n"
# 					
# 						f = open(get_home() + "/patrolstudy/toupload/data_ile.log", "a")
# 						f.write(outtraj)
# 						f.close()
# 
# 						(ile, stderr) = p.communicate(outtraj)
# 							
# 						print("\nLBA\n"+str(policyaccuracy)+"\nILE\n"+ile)
						
						# sessionFinish should stay True
						# flipped flags makes sure it does start next session
						sessionStart = False
						self.isSolving = False
						num_Trajsofar += 1
						# sessionNumber should be updated only here
						sessionNumber += 1
						print_once = 0 # print once empty q for next session
# 						
# 						if (min_IncIRL_InputLen*sessionNumber >= min_BatchIRL_InputLen):
# 							f = open("/tmp/studyresults2","a")
# 							#f.write("\nLBA:\n"+str(policyaccuracy)+"\nILE:\n"+ile)
# 		 					f.close()
# 							
						

				# else if session is yet to be finished
				# call solver again without deleting (but enlarged) traj
				elif not not newStuff and (BatchIRLflag==False and sessionFinish==False): 
					
					global print_once
					self.isSolving = False
					self.q = None
					print("\n need another call for session "+str(sessionNumber)) 
					global sess_starttime
	 				sess_starttime=rospy.Time.now().to_sec()
	 				#print(" at time "+str(sess_starttime))
					print_once = 0
						
			except Queue.Empty: # no item in queue, boydirl is not done
				global print_once 
				if print_once == 0:
					print("\n print_once queue self.q empty\n")
				print_once = 1
				pass		
					
		if self.finalQ is None and self.patPolicies is not None and rospy.get_time() - self.lastSolveAttempt > getTimeConv():
			# compute attacker policy in new process and share results in new queue
			print("\n compute attacker policy using policies learned from recorded trajectories")
			self.finalQ = multiprocessing.Queue()
			self.p = multiprocessing.Process(target=getattackerpolicy, args=(self.finalQ, self.patPolicies, self.traj, self.pmodel,\
																			 self.goalState, self.reward, self.penalty, self.detectDistance,\
																			  self.predictTime, self.add_delay, self.p_fail, self.map, self.T) )
				
			self.p.start()
			self.lastSolveAttempt = rospy.get_time()
		
	
	def getModel(self):
		return self.model

class WrapperIRLDelay(WrapperIRL):


	def update(self):
		self.updateWithAlg("NG", True)


class WrapperMaxEntIRL(WrapperIRL):


	def update(self):
#		self.updateWithAlg("MAXENT", False)
		global useRecordedTraj, recordConvTraj
		if useRecordedTraj == 1 and recordConvTraj == 0:
			self.updateWithAlgRecTraj("MAXENT", True)
		else:
			self.updateWithAlg("MAXENT", True)

class WrapperMaxEntIRLZExact(WrapperIRL):


	def update(self):
		global useRecordedTraj, recordConvTraj
		if useRecordedTraj == 1 and recordConvTraj == 0:
			self.updateWithAlgRecTraj("MAXENTZEXACT", True)
		else:
			self.updateWithAlg("MAXENTZEXACT", True)

class WrapperMaxEntIRLDelay(WrapperIRL):


	def update(self):
		self.updateWithAlg("MAXENT", True)


class WrapperLMEIRL(WrapperIRL):


	def update(self):
		global useRecordedTraj, recordConvTraj
		if useRecordedTraj == 1 and recordConvTraj == 0:
			self.updateWithAlgRecTraj("LME", True)
		else:
			self.updateWithAlg("LME", True)

class WrapperLMEI2RL(WrapperIRL):

	def update(self):
		global useRecordedTraj, recordConvTraj
		if useRecordedTraj == 1 and recordConvTraj == 0:
			self.updateWithAlgRecTraj("LME2", True)
		else:
			self.updateWithAlg("LME2", True)

class WrapperLMEIRLBLOCKEDGIBBS(WrapperIRL):

	def update(self):
		global useRecordedTraj, recordConvTraj
		if useRecordedTraj == 1 and recordConvTraj == 0:
			self.updateWithAlgRecTraj("LMEBLOCKEDGIBBS", True)
		else:
			self.updateWithAlg("LMEBLOCKEDGIBBS", True)

class WrapperLMEIRLBLOCKEDGIBBS2(WrapperIRL):

	def update(self):
		global useRecordedTraj, recordConvTraj
		if useRecordedTraj == 1 and recordConvTraj == 0:
			self.updateWithAlgRecTraj("LMEBLOCKEDGIBBS2", True)
		else:
			self.updateWithAlg("LMEBLOCKEDGIBBS2", True)

class WrapperLMEIRLBLOCKEDGIBBSTIMESTEP(WrapperIRL):

	def update(self):
		global useRecordedTraj, recordConvTraj
		if useRecordedTraj == 1 and recordConvTraj == 0:
			self.updateWithAlgRecTraj("LMEBLOCKEDGIBBSTIMESTEP", True)
		else:
			self.updateWithAlg("LMEBLOCKEDGIBBSTIMESTEP", True)

class WrapperLMEIRLBLOCKEDGIBBSTIMESTEPSA(WrapperIRL):

	def update(self):
		self.updateWithAlg("LMEBLOCKEDGIBBSSATIMESTEP", True)

class WrapperIdealIRL(WrapperIRL):

	def update(self):
		global obstime
	
		if not self.isSolving and self.policy is None:
					
#			print(count, get_time(), self.policy, max(len(a) for a in self.traj))
			if ((get_time() > obstime and self.policy is None)) and max(len(a) for a in self.traj) > self.predictTime:  # found enough states for both patrollers
				self.isSolving = True

				self.q = multiprocessing.Queue()

				self.p = multiprocessing.Process(target=idealirlsolve, args=(self.q,self.add_delay, self.map, self.traj, self.pmodel, self.observableStateLow, self.eq) )

				self.p.start()

		elif self.isSolving:
			try:
				newStuff = self.q.get(False)
				
				self.p.join()
				# got a new set of patroller policies, kickoff the final step
				self.T = newStuff.pop()
				self.patPolicies = newStuff
				
				# save the generated policy to the logs
				f = open(get_home() + "/patrolstudy/toupload/policy.log", "w")
				pickle.dump(self.patPolicies,f)
				f.close()
				
				self.q = multiprocessing.Queue()
				
			except Queue.Empty:
				pass		

		if self.finalQ is None and self.patPolicies is not None and rospy.get_time() - self.lastSolveAttempt > getTimeConv():
			self.finalQ = multiprocessing.Queue()

			self.p = multiprocessing.Process(target=getattackerpolicy, args=(self.finalQ, self.patPolicies, self.traj, self.pmodel, self.goalState, self.reward, self.penalty, self.detectDistance, self.predictTime, self.add_delay, self.p_fail, self.map, self.T) )
			self.p.start()
			self.lastSolveAttempt = rospy.get_time()

def print_traj_sortingMDP(conv_traj):
	outtraj = ""
	s = None
	act_str = None

	for sap in conv_traj: 
		if (sap is not None): 
			s = sap[0]
			outtraj += "["+str(s._onion_location)+","\
			+str(s._prediction)+","+\
			str(s._EE_location)+","+\
			str(s._listIDs_status)+"]:"

			if sap[1].__class__.__name__ == "InspectAfterPicking":
				act_str = "InspectAfterPicking"
				 
			elif sap[1].__class__.__name__ == "InspectWithoutPicking":
				act_str = "InspectWithoutPicking"
				 
			elif sap[1].__class__.__name__ == "Pick":
				act_str = "Pick"
				 
			elif sap[1].__class__.__name__ == "PlaceOnConveyor":
				act_str = "PlaceOnConveyor"
				 
			elif sap[1].__class__.__name__ == "PlaceInBin":
				act_str = "PlaceInBin"
				 
			elif sap[1].__class__.__name__ == "ClaimNewOnion":
				act_str = "ClaimNewOnion"
				 
			elif sap[1].__class__.__name__ == "ClaimNextInList":
				act_str = "ClaimNextInList"
				
			else:
				act_str = "ActionInvalid"

			outtraj += act_str

		else:
			outtraj += "None"
		outtraj += "\n"
	
	print outtraj

	return 

def parse_sorting_policy(buf):
	# stdout now needs to be parsed into a hash of state => action, which is then sent to mapagent
	p = {}
	stateactions = buf.split("\n")
	for stateaction in stateactions:
		temp = stateaction.split(" = ")
		if len(temp) < 2: continue
		state = temp[0]
		action = temp[1]
												
		state = state[1 : len(state) - 1]
		pieces = state.split(",")	
		
		ss = sortingState(int(pieces[0]), int(pieces[1]), int(pieces[2]), int(pieces[3]))

		if action == "InspectAfterPicking":
			act = InspectAfterPicking()
		elif action == "InspectWithoutPicking":
			act = InspectWithoutPicking()
		elif action == "Pick":
			act = Pick()
		elif action == "PlaceOnConveyor":
			act = PlaceOnConveyor()
		elif action == "PlaceInBin":
			act = PlaceInBin()
		elif action == "ClaimNewOnion":
			act = ClaimNewOnion()
		elif action == "ClaimNextInList":
			act = ClaimNextInList()
		else:
			print("Invalid input policy to parse_sorting_policy")
			exit(0)
		
		p[ss] = act

	from mdp.agent import MapAgent
	return MapAgent(p)

class WrapperSortingForwardRL():

	def __init__(self, p_fail=0.05, reward_type='sortingReward2'):

		self.p_fail = p_fail
		self.isSolving = False
		self.traj_states = []
		self.recd_convertedstates = [] 
		# self.sortingMDP = sortingModel(p_fail)
		self.sortingMDP = sortingModelbyPSuresh2(p_fail) 
		sortingReward = None
		initial = util.classes.NumMap()

		if reward_type == 'sortingReward2':
			sortingReward = sortingReward2(8) 
			params_pickinspect = [1,-1,-1,1,-0.2,1,-1.5,-1.0] 
			params = params_pickinspect
			params_rolling = [1,-0.5,-0.75,1.3,-0.2,0,1.88,-1.0] 
			params = params_rolling
			for s in self.sortingMDP.S():
				initial[s] = 1.0

		if reward_type == 'sortingReward7':
			sortingReward = sortingReward7(11) 
			params_pickinspect = [2, 1, 2, 1, 0.2, 1, 0, 4, 0, 0, 4] 
			params = params_pickinspect
			params_rolling = [0, 4, 0, 4, 0.2, 0, 8, 0, 8, 4, 0] 
			params = params_rolling
			s = sortingState(0,2,0,0)	
			initial[s] = 1.0
		
		norm_params = [float(i)/sum(np.absolute(params)) for i in params]
		sortingReward.params=norm_params
		self.sortingMDP.reward_function = sortingReward 
		self.sortingMDP.gamma = 0.998 
		initial = initial.normalize()

		args = [get_home() + "/catkin_ws/devel/bin/solveSortingMDP", ] 
		sp = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		stdin = str(norm_params)	
		(stdout, stderr) = sp.communicate(stdin)
		self.policy = parse_sorting_policy(stdout)						

		# self.policy = mdp.solvers.ValueIteration(0.55).solve(self.sortingMDP)
		# Python code gives different policy for same set of weights
		# As D code is the language for solver, it is better to read policy output for there
		# example mdppatrolctrl.py 

		conv_traj = mdp.simulation.simulate(self.sortingMDP,self.policy,initial,30)
		print("\nsimuated mdp traj ")
		print_traj_sortingMDP(conv_traj)

		# for s in self.sortingMDP.S():
	 # 		ol = None
	 # 		pr = None
	 # 		el = None

		# 	if s._onion_location == 0:
		# 		ol = 'Onconveyor'
		# 	elif s._onion_location == 1:
		# 		ol = 'Infront'
		# 	elif s._onion_location == 2:
		# 		ol = 'Inbin'
		# 	else:
		# 		ol = 'Picked/AtHomePose'
		# 	if s._prediction == 0:
		# 		pr = 'bad'
		# 	elif s._prediction == 1:
		# 		pr = 'good'
		# 	else:
		# 		pr = 'unknown'
		# 	if s._EE_location == 0:
		# 		el = 'Onconveyor'
		# 	elif s._EE_location == 1:
		# 		el = 'Infront'
		# 	elif s._EE_location == 2:
		# 		el = 'Inbin'
		# 	else:
		# 		el = 'Picked/AtHomePose'
		# 	print str((ol,pr,el)) 
		# 	print '\tpi({}) = {}'.format(s, self.policy._policy[s]) 

		# self.startState = sortingState(-1,-1,-1,-1) #(0,2,3) conv, unknown, home

		impl_GoHome() 
		try: 
			print "making location EE 0 by going to conveyor center" 
			resp = handle_move_sawyer_service("conveyorCenter",index_chosen_onion) 
			time.sleep(1.0)
		except rospy.ServiceException, e: 
			print "Service call failed: %s"%e 
			return False

		self.startState = sortingState(0,2,0,0) 
		self.currentState = self.startState 
		# self.currentAction = self.next_optimal_action() 

	def update_current_state(self): 

		global location_claimed_onion, prediction_chosen_onion, location_EE, ListStatus,\
		last_location_claimed_onion

		if last_location_claimed_onion != 0 and location_claimed_onion == 0 and (self.currentState != self.startState):
			# self.currentState = sortingState(4, prediction_chosen_onion, 0, ListStatus)				
			# with psuresh's cahnges in mdp
			self.currentState = sortingState(4, 2, 0, 2)				
		else:
			self.currentState = sortingState(location_claimed_onion, prediction_chosen_onion, location_EE, ListStatus)

		# check placed before changing last_location_claimed_onion
		if last_location_claimed_onion != location_claimed_onion:
			last_location_claimed_onion = location_claimed_onion
		
		return 

	def next_optimal_action(self):
		self.currentAction = self.policy._policy[self.currentState]
		return self.currentAction

	def update_trajectories(self, state, action):
		sa = (self.currentState, self.currentAction)
		self.traj_states.append(sa) 
		return

	def recordStateSequencetoFile(self):
		
		f = open(get_home() + "/patrolstudy/toupload/recd_convertedstates.log", "w")
		f.write("")
		f.close()

		# record trajectory directly in file
		print("desired length reached. recordStateSequence to file ") 
		outtraj = ""
		
		conv_traj = self.traj_states
		for sap in conv_traj: 
			if (sap is not None): 
				s = sap[0]
				#print(s)
				#print(sap[1].__class__.__name__)
				outtraj += str(s._onion_location)+","\
				+str(s._prediction)+","+\
				str(s._EE_location)+","+\
				str(s._listIDs_status)+","
				if sap[1].__class__.__name__ == "InspectAfterPicking":
					outtraj += "InspectAfterPicking"
				elif sap[1].__class__.__name__ == "InspectWithoutPicking":
					outtraj += "InspectWithoutPicking"
				elif sap[1].__class__.__name__ == "Pick":
					outtraj += "Pick"
				elif sap[1].__class__.__name__ == "PlaceOnConveyor":
					outtraj += "PlaceOnConveyor"
				elif sap[1].__class__.__name__ == "PlaceInBin":
					outtraj += "PlaceInBin"
				elif sap[1].__class__.__name__ == "ClaimNewOnion":
					outtraj += "ClaimNewOnion"
				elif sap[1].__class__.__name__ == "ClaimNextInList":
					outtraj += "ClaimNextInList"
				else:
					outtraj += "ActionInvalid"
			else:
				outtraj += "None"
			outtraj += "\n"
		outtraj += "ENDREC\n"
		
		print outtraj

		f = open(get_home() + "/patrolstudy/toupload/recd_convertedstates.log", "w")
		f.write(outtraj)
		f.close()
		return


def abortRun():
	global pub
	pub.publish(String("abortrun"))

def abortRun2():
	global pub
	pub.publish(String("abortrun2"))

def datacollected():
	global pub
	pub.publish(String("datacollected"))

def emptyinput():
	global pub
	pub.publish(String("emptyinput"))


amclmessageReceived = 0 
lastpublish = 0
waitUntilGoalState = False

def step():
	
	global state
	global goal
	global lastPositionBelief
	global mdpWrapper
	global cur_waypoint
	global percepts
	global perceptsUpdated
	global currentAttackerState     
	global amclmessageReceived
	global lastpublish
	global mapToUse


	if mdpWrapper is None:
		print("\n \n mdpWrapper is None \n ")
		return

	if not state == "w":
		return


	else:
		global irlfinish_time, useRecordedTraj, recordConvTraj, print_once

		# are we using recorded trajectories
		if useRecordedTraj == 0:

			current_time = rospy.Time.now().to_sec()
			# recording time for 70  states is approx. 150 s and waiting for GoTime
			# + compute policy + learning was 900s-1200 s
			# That time is reduced significantly.
			# With 70 input states, learning has reached saturation point
			# learning time is approx. 600 s. Time limit for computing policy
			# and go time is approx (900 - 150 - 600 =) 150 s to 450 s

			global start_forGoTime, timeoutThresh
			if (current_time - start_forGoTime > timeoutThresh):
				if mdpWrapper.patPolicies is not None:
					print("aborting because leanring didn't finish within threshold")
					abortRun2()
				else:
					print("aborting because time period from-learning-to-computinggotime crossed thresh secs")
					abortRun()
				return

# 			current_time = rospy.Time.now().to_sec()
# 			if (current_time - irlfinish_time > 200):
# 				print("aborting because time for computing go time crossed 100 secs")
# 				abortRun()
# 				return
# 			else:
			#print("\n current_time:"+str(current_time)+\
			#	" irlfinish_time:"+str(irlfinish_time))
			# same call for irl and computing attackerpolicy

			# perceive and store states
			if BatchIRLflag == True and min(patrtraj_len) <= min_BatchIRL_InputLen:
				mdpWrapper.addStates()
			if BatchIRLflag == False and sessionNumber * min_IncIRL_InputLen <= min_BatchIRL_InputLen:
				mdpWrapper.addStates()

			mdpWrapper.update()
			# should we go now?
			# check if we got a valid go time

			if mdpWrapper.goNow():
				global irlfinish_time
				current_time = rospy.Time.now().to_sec()
				f = open("/tmp/timestamps","a")
				f.write("time for computing go time:"+str(current_time - irlfinish_time))
				f.write("\n Computing attacker's val function finished at "+str(rospy.Time.now().to_sec()))
				f.close()

				global start_forGoTime
				f = open("/tmp/studyresults2", "a")
				f.write("\nTMRG:\n" + str(current_time - start_forGoTime))
				f.close()

				print("\n Computing attacker's val function finished at "+str(current_time - start_forGoTime))
				Attack()
	#			import std_srvs.srv
	#			try:
	#				resetservice = rospy.ServiceProxy('move_base_node/clear_costmaps', std_srvs.srv.Empty)
	#
	#				resetservice()
	#			except:
	#				# ros can't be counted on for anything, at least we tried.
	#				pass


				# get the current state (to base the plan on), then give the first goal message
				currentAttackerState = mdpWrapper.getStateFor(lastPositionBelief)
				action = mdpWrapper.latestPolicy().actions(currentAttackerState).keys()[0]
				newMdpState = action.apply(currentAttackerState)
				if not mdpWrapper.model.is_legal(newMdpState):
					newMdpState.location = currentAttackerState.location
					newMdpState.orientation = currentAttackerState.orientation
					if not mdpWrapper.model.is_legal(newMdpState):
						newMdpState.time = currentAttackerState.time

				newWayPoint = mdpWrapper.getPositionFor(newMdpState)
				print("cur_waypoint", newWayPoint, " curstate:", currentAttackerState, "newstate", newMdpState)
				currentAttackerState = newMdpState
				cur_waypoint = newWayPoint

				returnval = PoseStamped()
				returnval.pose.position = Point(cur_waypoint[0], cur_waypoint[1], 0)

				q = tf.transformations.quaternion_from_euler(0,0,cur_waypoint[2])
				returnval.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

				returnval.header.frame_id = "/map"
				goal.publish(returnval)
				lastpublish = get_time()
		else:
# 			print("\n recordConvTraj, (not mdpWrapper.recd_convertedstates[0]\
# 				and not mdpWrapper.recd_convertedstates[1]), patPolicies \n")
# 			print(recordConvTraj)
			emptytraj=(not mdpWrapper.recd_convertedstates[0] \
						and not mdpWrapper.recd_convertedstates[1])
# 			print(emptytraj)
# 			print(mdpWrapper.patPolicies)
			
			global recordConvTraj, desiredRecordingLen, print_once


			if recordConvTraj == 1 and (max(patrtraj_len) < desiredRecordingLen) :
				# perceive and store states
				mdpWrapper.addStates()

			if recordConvTraj == 1 and (max(patrtraj_len) >= desiredRecordingLen):
				# print("calling recordStateSequencetoFile()")
				
				mdpWrapper.recordStateSequencetoFile()
				mdpWrapper.recordFullStateSequencetoFile()
				print("finished recordStateSequencetoFile() recordFullStateSequencetoFile()")
				datacollected()
				
			elif recordConvTraj == 0 and emptytraj == True \
			and mdpWrapper.patPolicies is None:
				# reading recorded sequence
				
				print("\n calling readStateSequencefromFile() \n")
				mdpWrapper.readStateSequencefromFile()
				
				allnone = 1
				for lst in mdpWrapper.recd_convertedstates:
					for x in lst:
						if x is not None:
							allnone = 0
				emptytraj=(not mdpWrapper.recd_convertedstates[0] \
						and not mdpWrapper.recd_convertedstates[1])
				if (allnone == 1 or emptytraj==True):
					print(" no states exist in recorded trajs ")
					f = open("/tmp/studyresults2","a") 
					f.write("\nno states exist in recorded trajs ")
					f.close()
					datacollected()
					
				mdpWrapper.readFullStateSequencefromFile()
				print("\n finished readFullStateSequencefromFile() \n")
				
				allnone = 1
				for lst in mdpWrapper.recd_convertedstatesfull:
					for x in lst:
						if x is not None:
							allnone = 0
				emptytraj=(not mdpWrapper.recd_convertedstatesfull[0] \
						and not mdpWrapper.recd_convertedstatesfull[1])
				if (allnone == 1 or emptytraj==True):
					print(" no states exist in recorded Full trajs ")
					f = open("/tmp/studyresults2","a") 
					f.write("\nno states exist in recorded Full trajs ")
					f.close()
					datacollected()
				
			elif recordConvTraj == 0 and emptytraj == False \
			and mdpWrapper.patPolicies is None:
				# learning to computeLBA with recorded state sequences
# 				print("in step() calling update")
				mdpWrapper.update()
				
			elif recordConvTraj == 0 and mdpWrapper.patPolicies is not None:
				# Are analyzing only learning step or the complete attack
				global analyzeAttack
				if analyzeAttack==0:

					global lineFeatureExpec
					print("\nlineFeatureExpec:\n"+lineFeatureExpec)

					print(" learning using recorded traj converged in step() ")
					f = open("/tmp/studyresults2","a") 
					f.write("\nlearning using recorded traj converged in step() ")
					f.close()
					datacollected()
				else:
					current_time = rospy.Time.now().to_sec()
					# recording time for 70  states is approx. 150 s and waiting for GoTime 
					# + compute policy + learning was 900s-1200 s 
					# That time is reduced significantly. 
					# With 70 input states, learning has reached saturation point
					# learning time is approx. 600 s. Time limit for computing policy
					# and go time is approx (900 - 150 - 600 =) 150 s to 450 s
					 
					global start_forGoTime, timeoutThresh
					if (current_time - start_forGoTime > timeoutThresh):
						print("aborting because time period from-learning-to-computinggotime crossed thresh secs")
						abortRun()
						return
					# compute wrapper.finalQ with attacker policy
					mdpWrapper.update()
					# should we go now?
					# check if we got a valid go time
					
					if mdpWrapper.goNow():
						print "mdpWrapper.goNow()"
						global irlfinish_time
						current_time = rospy.Time.now().to_sec()
						f = open("/tmp/timestamps","a")
						f.write("time for computing go time:"+str(current_time - irlfinish_time))
						f.write("\n Computing attacker's val function finished at "+str(rospy.Time.now().to_sec()))
						f.close()
						print("\n Computing attacker's val function finished at "+str(rospy.Time.now().to_sec()))
						Attack()
			#			import std_srvs.srv
			#			try:
			#				resetservice = rospy.ServiceProxy('move_base_node/clear_costmaps', std_srvs.srv.Empty)
			#
			#				resetservice()
			#			except:
			#				# ros can't be counted on for anything, at least we tried.
			#				pass
			
						
						# get the current state (to base the plan on), then give the first goal message
						currentAttackerState = mdpWrapper.getStateFor(lastPositionBelief)
						action = mdpWrapper.latestPolicy().actions(currentAttackerState).keys()[0]   
						newMdpState = action.apply(currentAttackerState)
						if not mdpWrapper.model.is_legal(newMdpState):
							newMdpState.location = currentAttackerState.location
							newMdpState.orientation = currentAttackerState.orientation
							if not mdpWrapper.model.is_legal(newMdpState):
								newMdpState.time = currentAttackerState.time
			
						newWayPoint = mdpWrapper.getPositionFor(newMdpState)
						print("cur_waypoint", newWayPoint, " curstate:", currentAttackerState, "newstate", newMdpState)
						currentAttackerState = newMdpState
						cur_waypoint = newWayPoint
			
						returnval = PoseStamped()
						returnval.pose.position = Point(cur_waypoint[0], cur_waypoint[1], 0)
			
						q = tf.transformations.quaternion_from_euler(0,0,cur_waypoint[2])   
						returnval.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
			
						returnval.header.frame_id = "/map"
						goal.publish(returnval)
						lastpublish = get_time()
	
					return
			else:
				global patrtraj_len
				# print "max(patrtraj_len):"+str(max(patrtraj_len))
				if print_once == 1:
					print("None of conditions in step method satisfied")
					print_once = 0

# leftmostonion_position=None

# def callback_trackedOnion(point_msg):

# 	leftmostonion_position.x = point_msg.x
# 	leftmostonion_position.y = point_msg.y
# 	leftmostonion_position.z = point_msg.z
# 	return

# needed services
try: 
	attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
	attach_srv.wait_for_service()
except rospy.ServiceException, e: 
	print "creating handle failed /link_attacher_node/attach service : %s"%e 
try: 
	detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
	detach_srv.wait_for_service()
except rospy.ServiceException, e: 
	print "creating handle failed /link_attacher_node/attach service : %s"%e 

req_attach_detach = AttachRequest()

try: 
	handle_claim_track_service = rospy.ServiceProxy('claim_n_track', claim_track) 
	handle_claim_track_service.wait_for_service()
except rospy.ServiceException, e: 
	print "creating handle failed claim_n_track service : %s"%e 

try: 
	handle_pose_chosen_index = rospy.ServiceProxy('pose_chosen_index', pose_chosen_index) 
	handle_pose_chosen_index.wait_for_service()
except rospy.ServiceException, e: 
	print "creating handle failed pose_chosen_index service : %s"%e 


index_chosen_onion = -1
last_index_chosen_onion = -1
ground_truth_chosen_onion = 1
prediction_chosen_onion = 2
target_location_x = -100
target_location_y = -100
target_location_z = -100
location_claimed_onion = -1 
last_location_claimed_onion = -1
location_EE = -1 # home
ground_truths_all_onions = {}
ListIndices = [] 
ListPoses = [] # may be needed to enable making listIndices empty when objects disappear 
ListStatus = 0
current_cylinder_x = []
last_size_current_cylinder_x = None
list_pool = None

def callback_poses(cylinder_poses_msg):

	global location_claimed_onion, index_chosen_onion,\
	ConveyorWidthRANGE_LOWER_LIMIT,ObjectPoseZ_RANGE_UPPER_LIMIT,\
	ListIndices, ListPoses, ListStatus, current_cylinder_x

	current_cylinder_x = cylinder_poses_msg.x
	current_cylinder_y = cylinder_poses_msg.y
	current_cylinder_z = cylinder_poses_msg.z
	if index_chosen_onion < len(current_cylinder_x) and index_chosen_onion != -1:
		target_location_x = current_cylinder_x[index_chosen_onion]
		target_location_y = current_cylinder_y[index_chosen_onion]
		target_location_z = current_cylinder_z[index_chosen_onion]

		if target_location_x >= ConveyorWidthRANGE_LOWER_LIMIT and \
		target_location_z <= ObjectPoseZ_RANGE_UPPER_LIMIT:
			location_claimed_onion = 0 # on conveyor
		else:
			if (target_location_x < 0.5 and target_location_y < 0.2 and\
			target_location_z > 0.7 and target_location_z <= 0.99) or\
			(target_location_x >= ConveyorWidthRANGE_LOWER_LIMIT and \
			target_location_z > ObjectPoseZ_RANGE_UPPER_LIMIT and\
			target_location_z <= ObjectPoseZ_RANGE_UPPER_LIMIT+0.2):
				location_claimed_onion = 3 # at home or picked
			else:
				if target_location_x > 0.5 and target_location_x < 0.69 and \
				target_location_y < 0.2 and\
				target_location_z >= 1.3 and target_location_z <= 1.55:
					location_claimed_onion = 1 # in front
				else:
					if target_location_x < 0.16 and \
					target_location_y > 0.56 and target_location_y < 0.7 and \
					target_location_z > 0.77 and target_location_z < 0.82:
						location_claimed_onion = 2 # at bin
					else:
						location_claimed_onion = -1 # either deleted or still changing
		

		# print "target_location_x,target_location_y,target_location_z"+\
		# str((target_location_x,target_location_y,target_location_z))
		# print "location_claimed_onion"+\
		# str(location_claimed_onion)
	
	loc = location_claimed_onion
	if loc == 0:
		str_loc = "Conv"
	else:
		if loc == 1:
			str_loc = "InFront"
		else:
			if loc == 2:
				str_loc = "AtBin"
			else:
				if loc == 3:
					str_loc = "Home"
				else:
					str_loc = "Unknown"

	global last_index_chosen_onion, last_location_claimed_onion
	if index_chosen_onion != last_index_chosen_onion:
		last_index_chosen_onion = index_chosen_onion
		print "callback_poses - index_chosen_onion:" + str(index_chosen_onion)
		print str((target_location_x,target_location_y,target_location_z))
		print "location_claimed_onion:" + str(str_loc)

	# if objects are deleted and nothing is grasped, make list empty
	ind_location_x = None
	ind_location_y = None
	ind_location_z = None
	all_deleted = True 
	for ind in ListIndices: 
		if ind < len(current_cylinder_x) and ind != -1:
			ind_location_x = current_cylinder_x[ind]
			ind_location_y = current_cylinder_y[ind]
			ind_location_z = current_cylinder_z[ind]

			if ind_location_x != -100:
				all_deleted = False
				if (ind_location_x,ind_location_y,ind_location_z) not in ListPoses:
					ListPoses.append((ind_location_x,ind_location_y,ind_location_z))

		else:
		# if an object index is in list but not in current_cylinder_x, there is a synchronization issue
		# between pose publisher and spawner. restart.
			ListPoses = None
			ListIndices = None
			ListStatus = 2

	if all_deleted: 
		# ind_location_x == -100: 
		# if all objects (including grasped) have disappeared, make lists empty
		print "objects have disappeared, make lists empty"
		ListPoses = []
		ListIndices = []
		ListStatus = 0
		
	# else: # else add pose to list
	# 	if (ind_location_x,ind_location_y,ind_location_z) not in ListPoses:
	# 		ListPoses.append((ind_location_x,ind_location_y,ind_location_z))


	return 

def callback_modelname(color_indices_msg):
	global ground_truths_all_onions, current_cylinder_x,\
	last_size_current_cylinder_x
	# print "color_indices_msg.data:"+str(color_indices_msg.data)
	# update ground_truths for new batch

	if len(current_cylinder_x) != last_size_current_cylinder_x:
		ground_truths_all_onions = {}
		for ind in range(0, len(current_cylinder_x)):
			if current_cylinder_x[ind] != -100:
				# dictionary, not a list
				ground_truths_all_onions[ind] = color_indices_msg.data[ind]

		last_size_current_cylinder_x = len(current_cylinder_x)
		print "ground_truths_all_onions:"+str(ground_truths_all_onions)

	
	return

# def add_model_name_input_index(ind):
# 	global req_attach_detach, ground_truth_chosen_onion
# 	if (ind < len(ground_truth_chosen_onions)) and (ind != -1):
# 		ground_truth_chosen_onion = ground_truth_chosen_onions[ind]
# 		if (ground_truth_chosen_onion == 0):#clean
# 		  req_attach_detach.model_name_1 = "red_cylinder_" + str(ind);
# 		else:#defective
# 		  req_attach_detach.model_name_1 = "blue_cylinder_" + str(ind);    
# 		# print "attached model: "+str(req_attach_detach.model_name_1)
# 	return

def update_pose_chosen_index():
	global index_chosen_onion, target_location_x, target_location_y,\
	target_location_z, location_claimed_onion 
	global ConveyorWidthRANGE_LOWER_LIMIT, ObjectPoseZ_RANGE_UPPER_LIMIT 

	if index_chosen_onion != -1:

		res_pose = handle_pose_chosen_index(index_chosen_onion)
		if res_pose.success == True: 
			target_location_x = res_pose.position_x
			target_location_y = res_pose.position_y
			target_location_z = res_pose.position_z

			if target_location_x >= ConveyorWidthRANGE_LOWER_LIMIT and \
			target_location_z <= ObjectPoseZ_RANGE_UPPER_LIMIT:
				location_claimed_onion = 0 # on conveyor
			else:
				if (target_location_x < 0.5 and target_location_y < 0.2 and\
				target_location_z > 0.7 and target_location_z <= 0.99) or\
				(target_location_x >= ConveyorWidthRANGE_LOWER_LIMIT and \
				target_location_z > ObjectPoseZ_RANGE_UPPER_LIMIT and\
				target_location_z <= ObjectPoseZ_RANGE_UPPER_LIMIT+0.2):
					location_claimed_onion = 3 # at home or picked
				else:
					if target_location_x > 0.5 and target_location_x < 0.69 and \
					target_location_y < 0.2 and\
					target_location_z >= 1.3 and target_location_z <= 1.55:
						location_claimed_onion = 1 # in front
					else: 
						if target_location_x < 0.19 and \
						target_location_y > 0.56 and target_location_y < 0.7 and \
						target_location_z > 0.74 and target_location_z < 0.84: 
							location_claimed_onion = 2 # at bin
						else:
							print "x,y,z: "+str((target_location_x,target_location_y,target_location_z))
							location_claimed_onion = -1 # either deleted or still changing


	if index_chosen_onion == -1 or res_pose.success == False:
		print "update_pose_chosen_index - can't find pose of onion"
		return False

	return True


def callback_pool(pool_msg):
	global list_pool
	list_pool = pool_msg.data
	return

def highAccuracyPred(ind):
	global prediction_chosen_onion, ground_truths_all_onions
	alternative = None
	ground_truth_chosen_onion = None

	if ind not in ground_truths_all_onions.keys():
		print "input index - "+str(ind)
		print "ground_truths_all_onions:"+str(ground_truths_all_onions)
		print "chosen onion not in pool"
		return -1

	ground_truth_chosen_onion = ground_truths_all_onions[ind]

	if ind != -1:
		if ground_truth_chosen_onion==0: alternative = 1
		else: alternative = 0
		prediction_chosen_onion = np.random.choice([ground_truth_chosen_onion,alternative],1,p=[0.95,0.05])[0]

	return prediction_chosen_onion 

def lowAccuracyPred(ind):
	global prediction_chosen_onion, ground_truths_all_onions
	alternative = None
	ground_truth_chosen_onion = None
	if ind not in ground_truths_all_onions.keys():
		print "chosen onion not in pool"
		return -1
	
	ground_truth_chosen_onion = ground_truths_all_onions[ind]

	if ind != -1:
		if ground_truth_chosen_onion==0: 
			alternative=1
			prediction_chosen_onion = np.random.choice([ground_truth_chosen_onion,alternative],1,p=[0.75,1-0.75])[0] 
		else: 
			alternative=0
			prediction_chosen_onion = np.random.choice([ground_truth_chosen_onion,alternative],1,p=[0.85,1-0.85])[0] 
		print "gt, alt, pred"+str((ground_truth_chosen_onion,alternative,prediction_chosen_onion))

	return prediction_chosen_onion

def callback_location_claimed_onion(loc_onion_msg):
	global location_claimed_onion
	location_claimed_onion = loc_onion_msg.data
	print "updated location_claimed_onion:"+str(location_claimed_onion)
	return

def callback_location_EE(loc_EE_msg):
	global location_EE
	location_EE = loc_EE_msg.data
	print "updated location_EE:"+str(location_EE)
	return

last_argument = None

def execute_policy():
	global index_chosen_onion, WrapperObjectRL, last_index_chosen_onion,\
	location_claimed_onion, location_EE, last_argument
	result = None

	print("execut epolicy location_EE, ListStatus = ",(location_EE, ListStatus))

	print "last action executed. location_claimed_onion,location_EE:"\
	+str((location_claimed_onion,location_EE))

	global target_location_x
 	if index_chosen_onion != -1 and target_location_x != -100: 
 		print "update_pose_chosen_index"
 		update_pose_chosen_index() 
 		print "location_claimed_onion:"+str(location_claimed_onion)
 		
	 	if location_claimed_onion == -1:
			# onion can't be in bin because detach happens later, 
			print "onion can't be in bin because detach happens later, find why update_pose_chosen_index not finding location?"
			# print "location_claimed_onion == -1,index_chosen_onion = -1"
			# index_chosen_onion = -1
	 		return 
 	else:
 		# onion deleted? 
 		print "onion deleted - prediction_chosen_onion = 2,ListStatus = 2"
 		# global prediction_chosen_onion, ListStatus
 		# prediction_chosen_onion, ListStatus = 2, 2
 		impl_ClaimNewOnion()
		update_pose_chosen_index()
		if location_claimed_onion == -1:
			print "onion deleted - location not changing by claiming random onion" 
			return 
		else:
			print "onion deleted - location changed by claiming random onion" 

		global prediction_chosen_onion, ListStatus
 		prediction_chosen_onion = 2
 		
 		# ListStatus = 2
 		# psuresh mdp, both behaviors start over if onions deleted 
 		ListStatus = 0

	try:
		resp = handle_update_state_EE() 
		location_EE = resp.locations[1] 
		# if WrapperObjectRL.currentState ==  WrapperObjectRL.startState:
		# 	print ("current state is start state, adjusting ee location and ListStatus")
		# 	location_EE, ListStatus = 0, 0

		print "UPDATE_STATE srv. location_claimed_onion,location_EE:"\
		+str((location_claimed_onion,location_EE))
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 

	if location_claimed_onion == -1:
		return 

	global prediction_chosen_onion
	print "prediction_chosen_onion:"+str(prediction_chosen_onion)

	WrapperObjectRL.update_current_state()
	print "state:"+str(WrapperObjectRL.currentState)

	# this is only way to update the onion location before it gets in bin and gets deleted
	if last_argument == "PlaceInBin":
		res = detach_forBin()
		print "last_argument == PlaceInBin detach_forBin "+str(res)

	act = WrapperObjectRL.next_optimal_action()
	print "action:"+str(act)

	argument = act.__class__.__name__
	switcher = {
	"InspectAfterPicking": impl_InspectAfterPicking,
	"InspectWithoutPicking": impl_InspectWithoutPicking,
	"Pick": impl_Pick,
	"PlaceOnConveyor": impl_PlaceOnConveyor,
	"PlaceInBin": impl_PlaceInBin,
	"GoHome": impl_GoHome,
	"ClaimNewOnion": impl_ClaimNewOnion,
	"ClaimNextInList": impl_ClaimNextInList
	}
	# Get the function from switcher dictionary
	func = switcher.get(argument, lambda: "Invalid action")
	if argument not in switcher.keys():
		"Invalid action"
	
	result=func()

	while result == None:
		sleep(0.1)

	last_argument = argument

	return result

def impl_InspectAfterPicking():
	global handle_move_sawyer_service, index_chosen_onion
	print "InspectAfterPicking"
	try: 
		resp = handle_move_sawyer_service("home",index_chosen_onion) 
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False

	try: 
		# print "lookNrotate" 
		resp = handle_move_sawyer_service("lookNrotate",index_chosen_onion) 
		global prediction_chosen_onion
		prediction_chosen_onion = highAccuracyPred(index_chosen_onion) 
		print "highAccuracyPred(index_chosen_onion):"+str(prediction_chosen_onion) 

	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False

	# psuresh mdp 
	global ListStatus 
	ListStatus = 2 

	return True

def impl_InspectWithoutPicking():
	global handle_move_sawyer_service, index_chosen_onion, ListIndices
	print "InspectWithoutPicking"
	try: 
		resp = handle_move_sawyer_service("home",index_chosen_onion) 
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False

	try: 
		resp = handle_move_sawyer_service("roll",index_chosen_onion) 
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False

	# find bad onions
	global ground_truths_all_onions
	print "finding bad onions, ground_truths_all_onions:"+str(ground_truths_all_onions)
	print "before finding bad onions, ListIndices:"+str(ListIndices)
	for i in ground_truths_all_onions.keys(): 
		print "check ground_truths_all_onions for index "+str(i)
		if lowAccuracyPred(i) == 0:
			ListIndices.append(i)

	print "ListIndices:"+str(ListIndices)

	# if list is non-empty, make first index claimed onion 
	global ListPoses, ListStatus, prediction_chosen_onion
	if ListIndices == []:
		ListStatus = 0
		prediction_chosen_onion = 2
	else:
		ListStatus = 1
		index_chosen_onion = ListIndices[0] 
		ListIndices = ListIndices[1:] 
		ListPoses = ListPoses[1:] 
		prediction_chosen_onion = 0

	print "prediction_chosen_onion:"+str(prediction_chosen_onion)+" ListStatus:"+str(ListStatus)

	return True

attempt = 0

def impl_Pick():
	global handle_move_sawyer_service, index_chosen_onion, location_claimed_onion

	print "Pick"
	try: 
		print "hover" 
		# check if object out of range
		update_pose_chosen_index()
		if location_claimed_onion == -1:
			print "hover not possible"
	 		global index_chosen_onion
	 		print "index_chosen_onion = -1"
	 		index_chosen_onion = -1 # execute policy will change list status and location and prediction
			try: 
				# print "liftgripper" 
				resp = handle_move_sawyer_service("conveyorCenter",index_chosen_onion) 
				time.sleep(1.0)
			except rospy.ServiceException, e: 
				print "Service call failed: %s"%e 
				return False
			return False
		resp = handle_move_sawyer_service("hover",index_chosen_onion) 
		time.sleep(2.5)
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False

	if resp.success == False:
		global attempt
		attempt = 0
		global index_chosen_onion
		if attempt == 0:
			print "perturbing startBin loc"
			print "hover returned false attempt 0, give intermediate waypoint"
			try: 
				resp = handle_move_sawyer_service("perturbStartBin",index_chosen_onion) 
				time.sleep(2.0)
			except rospy.ServiceException, e: 
				print "Service call failed: %s"%e 
				return False 
			attempt =1
		else:
			# if attempt == 1:
			print "hover returned false attempt 1, goto home and back to bin" 
			handle_move_sawyer_service("home",index_chosen_onion)
			time.sleep(2.0)
			try: 
				resp = handle_move_sawyer_service("bin",index_chosen_onion) 
				time.sleep(2.0)
			except rospy.ServiceException, e: 
				print "Service call failed: %s"%e 
				return False 
			attempt =0

		return False
	else:
		try: 
			print "lowergripper" 
			# check if object out of range
			update_pose_chosen_index()
			if location_claimed_onion == -1:
				print "grasping not possible"
		 		global index_chosen_onion
		 		print "index_chosen_onion = -1"
		 		index_chosen_onion = -1 # execute policy will change list status and location and prediction
				try: 
					# print "liftgripper" 
					resp = handle_move_sawyer_service("conveyorCenter",index_chosen_onion) 
					time.sleep(1.0)
				except rospy.ServiceException, e: 
					print "Service call failed: %s"%e 
					return False
				return False

			resp = handle_move_sawyer_service("lowergripper",index_chosen_onion) 
			time.sleep(1.5)
		except rospy.ServiceException, e: 
			print "Service call failed: %s"%e 
			return False
		if resp.success == False:
			return False
		else:
			try: 
				print "attach object" 
				# object out of range
				if location_claimed_onion == -1:
			 		global index_chosen_onion
			 		print "attach object - index_chosen_onion = -1"
			 		index_chosen_onion = -1 # execute policy will change list status and location and prediction
					try: 
						# print "liftgripper" 
						resp = handle_move_sawyer_service("conveyorCenter",index_chosen_onion) 
						time.sleep(1.0)
					except rospy.ServiceException, e: 
						print "Service call failed: %s"%e 
						return False
					return False

				# add_model_name_input_index(index_chosen_onion)
				call_success = False
				resp = handle_move_sawyer_service("attach",index_chosen_onion) 
				time.sleep(1.0)
				call_success = resp.success
			except rospy.ServiceException, e: 					
				print "Service call failed: %s"%e 

			current_time = rospy.get_time()
			while (call_success == False) and (rospy.get_time()-current_time <= 8.0):
				try: 
					# object out of range
					if location_claimed_onion == -1:
				 		global index_chosen_onion
				 		print "attach object - index_chosen_onion = -1"
				 		index_chosen_onion = -1 # execute policy will change list status and location and prediction
						try: 
							# print "liftgripper" 
							resp = handle_move_sawyer_service("conveyorCenter",index_chosen_onion) 
							time.sleep(1.0)
						except rospy.ServiceException, e: 
							print "Service call failed: %s"%e 
							return False
						return False

					print "try again attach object" 
					resp = handle_move_sawyer_service("attach",index_chosen_onion) 
					time.sleep(1.0)
					call_success = resp.success
				except rospy.ServiceException, e: 					
					print "Service call failed: %s"%e 

			if call_success == False:
				print "couldn't attach in time limit"
				return False
			else:
				try: 
					print "liftgripper" 
					resp = handle_move_sawyer_service("liftgripper",index_chosen_onion) 
					time.sleep(1.5)
				except rospy.ServiceException, e: 
					print "Service call failed: %s"%e 
					return False

				if resp.success == False:
					print "liftgripper failed"
					notgrasped = detach_notgrasped() 
					print "notgrasped:"+str(notgrasped)
					return False
	
	# impl_GoHome()

	update_pose_chosen_index()
	print "impl_Pick. location_claimed_onion:"+str(location_claimed_onion)
 	if location_claimed_onion == -1:
 		global index_chosen_onion
 		index_chosen_onion = -1 # execute policy will change list status and location and prediction
		try: 
			# print "liftgripper" 
			resp = handle_move_sawyer_service("conveyorCenter",index_chosen_onion) 
			time.sleep(1.0)
		except rospy.ServiceException, e: 
			print "Service call failed: %s"%e 
			return False
		print "onion deleted"
		print "pick fails"
		return False

	notgrasped = detach_notgrasped() 
	print "notgrasped:"+str(notgrasped)
	global location_EE
	print "impl_Pick. location_EE:"+str(location_EE)

	return True

def impl_PlaceOnConveyor():
	global handle_move_sawyer_service, index_chosen_onion
	print "PlaceOnConveyor"
	try: 
		# print "conveyorCenter" 
		resp = handle_move_sawyer_service("conveyorCenter",index_chosen_onion) 
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False
	if resp.success == False:
		print "couldn't reach conveyorCenter"
		return False
	else:
		try: 
			# print "detach object" 
			call_success = False
			resp = handle_move_sawyer_service("detach",index_chosen_onion) 
			time.sleep(1.0)
			call_success = resp.success
		except rospy.ServiceException, e: 					
			print "Service call failed: %s"%e 

		current_time = rospy.get_time()
		while (call_success == False) and (rospy.get_time()-current_time <= 3.0):
			try: 
				resp = handle_move_sawyer_service("detach",index_chosen_onion) 
				time.sleep(1.0)
				call_success = resp.success
			except rospy.ServiceException, e: 					
				print "Service call failed: %s"%e 

		# if call_success == False:
		# 	print "couldn't detach in time limit"
		# 	return False

	return True

def impl_PlaceInBin():
	global handle_move_sawyer_service, index_chosen_onion
	print "PlaceInBin"
	try: 
		print "bin" 
		resp = handle_move_sawyer_service("bin",index_chosen_onion) 
		time.sleep(2.0)
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False

	if resp.success == False:
		print "couldn't go to bin"
		return False
	else:

		# for psuresh mdp
		global ListIndices, ListStatus, prediction_chosen_onion, ListPoses
		print "ListIndices:"+str(ListIndices)	
		# print "PlaceInBin pre processing ListStatus:"+str(ListStatus)	

		if ListStatus == 1:
			# roll behavior
			if len(ListIndices) == 0:
				ListStatus = 0
			else:
				ListStatus = 1

		prediction_chosen_onion = 2
		print "PlaceInBin post processing ListStatus:"+str(ListStatus)+" prediction_chosen_onion:"+str(prediction_chosen_onion)	

		return True
		# detach should happen after updating state
	

	return True

def detach_forBin():
	global index_chosen_onion
	try: 
		print "detach object" 
		call_success = False
		resp = handle_move_sawyer_service("detach",index_chosen_onion) 
		time.sleep(1.0)
		call_success = resp.success
	except rospy.ServiceException, e: 					
		print "Service call failed: %s"%e 

	current_time = rospy.get_time()
	while (call_success == False) and (rospy.get_time()-current_time <= 3.0):
		try: 
			resp = handle_move_sawyer_service("detach",index_chosen_onion) 
			time.sleep(1.0)
			call_success = resp.success
		except rospy.ServiceException, e: 					
			print "Service call failed: %s"%e 

	# if call_success == False:
	# 	print "couldn't detach in time limit"
	# 	return False
	return call_success

def impl_GoHome():
	global handle_move_sawyer_service, index_chosen_onion
	print "GoHome"
	try: 
		resp = handle_move_sawyer_service("home",index_chosen_onion) 
		return True
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False

def impl_ClaimNewOnion():
	global last_index_chosen_onion, index_chosen_onion
	try:
		last_index_chosen_onion = index_chosen_onion
		print_once = 0
		while (index_chosen_onion == last_index_chosen_onion or index_chosen_onion == -1):
			# repeat if claimed nothing or if claimed same object again
			if print_once == 0:
				print "repeat if claimed nothing or if claimed same object again"
				print_once = 1
			resp = handle_claim_track_service(1) 
			index_chosen_onion = resp.chosen_index
		print "resp.success:"+str(resp.success)+",resp.chosen_index:"+str(resp.chosen_index) 
		global prediction_chosen_onion
		prediction_chosen_onion = 2
		return True
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
		return False

def impl_ClaimNextInList(): 
	global ListIndices, index_chosen_onion, ListStatus, ListPoses, prediction_chosen_onion
	print "impl_ClaimNextInList"
	print("pre ListStatus:",ListStatus)

	if ListIndices == []: 
		ListPoses = []
		# ListStatus = 2
		# psuresh mdp
		ListStatus = 0
		impl_ClaimNewOnion() 
		update_pose_chosen_index() 
		if location_claimed_onion == -1:
			print "impl_ClaimNextInList - location not changing by claiming random onion" 
			return False
		else:
			print "impl_ClaimNextInList - location changed by claiming random onion" 
		return False
	else:
		index_chosen_onion = ListIndices[0]
		ListIndices = ListIndices[1:]
		ListPoses = ListPoses[1:]
		update_pose_chosen_index() 
		while location_claimed_onion == -1:
			print "impl_ClaimNextInList - onion fell down, claiming next one in list" 
			index_chosen_onion = ListIndices[0]
			ListIndices = ListIndices[1:]
			ListPoses = ListPoses[1:]
			update_pose_chosen_index() 

		ListStatus = 1
		# prediction_chosen_onion = 0
		# psuresh mdp
		prediction_chosen_onion = 2
		return True

def detach_notgrasped():
	try: 
		print "detach_notgrasped" 
		resp = handle_move_sawyer_service("detach_notgrasped",index_chosen_onion) 
		time.sleep(1.0)
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 
	return resp.success

# def impl_MoveAllInList():
# 	global ListIndices

# 	for i in ListIndices:
# 		res2 = impl_Pick()
# 		if res2 == True:
# 			res3 = impl_PlaceInBin()
# 	# empty list
# 	ListIndices = []
# 	return True	


if __name__ == "__main__":

	global BatchIRLflag, lastQ, min_IncIRL_InputLen, irlfinish_time, \
	patrtraj_len, min_BatchIRL_InputLen, startat, glbltrajstarttime, \
	cum_learntime, lineFeatureExpecfull, desiredRecordingLen, \
	useRecordedTraj, recordConvTraj, recordonce, print_once, analyzeAttack, \
	timeoutThresh, sessionStart, \
	sessionFinish, recordConvTraj, \
	startat, glbltrajstarttime, currentile, lastile, lastcalltimeaddstates,\
	index_chosen_onion, attach_srv, detach_srv, req_attach_detach, \
	handle_move_sawyer_service, handle_claim_track_service,\
	WrapperObjectRL

	rospy.init_node('sorting_onions')

	time.sleep(1)
	sessionStart = False
	sessionFinish = True
	lastcalltimeaddstates=0
	desiredRecordingLen = 20
	useRecordedTraj = 1
	recordConvTraj = 0
	cum_learntime = 0 
	min_IncIRL_InputLen= 5
	min_BatchIRL_InputLen= 20 #500 
	lastQ = "" #final likelihood value achieved
	irlfinish_time = sys.maxint
	patrtraj_len = [1,1]
	print_once = 1
	lineFeatureExpecfull = ""
	useRegions=0

	startat = rospy.get_time()
	glbltrajstarttime = fromRospyTime(get_time()) # get_time() needs startat

	currentile=100.0000
	lastile=100.0000

	policy = "irl";
	# rospy.get_param("~policy")
	# pfail = float(rospy.get_param("~pfail"))
	# usesimplefeatures = rospy.get_param("~usesimplefeatures") == "true"
	# BatchIRLflag = (int(rospy.get_param("~BatchIRLflag")))==1
	# min_BatchIRL_InputLen = int(rospy.get_param("~min_BatchIRL_InputLen"))
	# min_IncIRL_InputLen = int(rospy.get_param("~min_IncIRL_InputLen"))
	# desiredRecordingLen = int(rospy.get_param("~desiredRecordingLen"))
	# useRecordedTraj = int(rospy.get_param("~useRecordedTraj"))
	# recordConvTraj = int(rospy.get_param("~recordConvTraj")) 
	# analyzeAttack = int(rospy.get_param("~analyzeAttack")) 
	# timeoutThresh = int(rospy.get_param("~timeoutThresh"))
	# useRegions = int(rospy.get_param("~useRegions"))

	f = open("/tmp/timestamps","w") 
	f.write("\n time durations for current trial") 
	f.close() 

	# print("\nBatchIRLflag:\n"+str(BatchIRLflag)+"\nmin_BatchIRL_InputLen:\n"\
		# +str(min_BatchIRL_InputLen)+"\nmin_IncIRL_InputLen:\n"+str(min_IncIRL_InputLen)\
		# +"\nuseRecordedTraj:\n"+str(useRecordedTraj)+"\nrecordConvTraj:\n"+str(recordConvTraj)\
		# +"\ndesiredRecordingLen:\n"+str(desiredRecordingLen)+"\nanalyzeAttack:\n"\
		# +str(analyzeAttack)) 
	f = open("/tmp/studyresults2","w") 
	f.write("BatchIRLflag:\n"+str(BatchIRLflag))
	f.write("\n min_BatchIRL_InputLen:\n"+str(min_BatchIRL_InputLen))
	f.write("\n min_IncIRL_InputLen:\n"+str(min_IncIRL_InputLen))
	f.close()

	global ConveyorWidthRANGE_LOWER_LIMIT, ObjectPoseZ_RANGE_UPPER_LIMIT,\
	handle_update_state_EE, handle_move_sawyer_service

	ConveyorWidthRANGE_LOWER_LIMIT = rospy.get_param("/ConveyorWidthRANGE_LOWER_LIMIT")
	ObjectPoseZ_RANGE_UPPER_LIMIT = rospy.get_param("/ObjectPoseZ_RANGE_UPPER_LIMIT")

	# updating the locations of onion and EE
	# rospy.Subscriber("location_claimed_onion", Int64, callback_location_claimed_onion)
	# rospy.Subscriber("location_EE", Int64, callback_location_EE)

	try: 
		handle_update_state_EE = rospy.ServiceProxy('update_state', update_state) 
		handle_update_state_EE.wait_for_service()
	except rospy.ServiceException, e: 
		print "creating handle failed update_state service : %s"%e 
	try: 
		handle_move_sawyer_service = rospy.ServiceProxy('move_sawyer', move_robot) 
		handle_move_sawyer_service.wait_for_service()
	except rospy.ServiceException, e: 
		print "creating handle failed move_sawyer service : %s"%e 

	# service to claim and track leftmost onion	
	# rospy.wait_for_service('claim_n_track') 
	# # subscribe to position of leftmost onion
	# rospy.Subscriber("pose_claimedobject", Point, callback_trackedOnion)
	# rospy.wait_for_service('move_sawyer') 

	rospy.Subscriber("current_cylinder_blocks", Int8MultiArray, callback_modelname)
	rospy.Subscriber("cylinder_blocks_poses", cylinder_blocks_poses, callback_poses)
	rospy.Subscriber("SAWYER_cylinder_active_pool", Int8MultiArray, callback_pool)

	# testing retrieval of model_names
	req_attach_detach.link_name_1 = "base_link"
	req_attach_detach.model_name_2 = "sawyer"
	req_attach_detach.link_name_2 = "right_l6"

	resp = handle_move_sawyer_service("home",-1) 
	# global ground_truths_all_onions, ListIndices
	# while not ground_truths_all_onions:
	# 	pass

	# global ListStatus
	# res1 = impl_InspectWithoutPicking()
	# if res1 == True:
	# 	res2 = impl_Pick()
	# 	if res2 == True:
	# 		res3 = impl_PlaceInBin()
	# 		print ("ListStatus after PlaceInBin:",str(ListStatus))
	# 		if res3 == True:
	# 			res4 = impl_ClaimNextInList()
	# 			if ListStatus == 1 and res4 == True:
	# 				res2 = impl_Pick()
	# 				if res2 == True:
	# 					res3 = impl_PlaceInBin()
	# 					print ("ListStatus after PlaceInBin:",str(ListStatus))
	# 					if res3 == True:
	# 						res4 = impl_ClaimNextInList()
	# 						if ListStatus == 1 and res4 == True:
	# 							res2 = impl_Pick()
	# 							if res2 == True:
	# 								res3 = impl_PlaceInBin() 
	# 								print ("ListStatus after PlaceInBin:",str(ListStatus)) 

	# try:
	# 	resp = handle_update_state() 
	# 	location_claimed_onion = resp.locations[0] 
	# 	location_EE = resp.locations[1]
	# 	print "UPDATE_STATE srv. location_claimed_onion,location_EE:"\
	# 	+str((location_claimed_onion,location_EE))
	# except rospy.ServiceException, e: 
	# 	print "Service call failed: %s"%e 
	# 	return False
	# exit(0) 

	# print "test handle_pose_chosen_index" 
	# res_pose = handle_pose_chosen_index(0) 
	# print str((res_pose.position_x,res_pose.position_y,res_pose.position_z)) 

	# impl_ClaimNewOnion()
	print "WrapperObjectRL"
	# WrapperObjectRL = WrapperSortingForwardRL(p_fail=0.05,reward_type='sortingReward2')
	WrapperObjectRL = WrapperSortingForwardRL(p_fail=0.05,reward_type='sortingReward7')

	r = rospy.Rate(20) # 10hz 30	
	global location_claimed_onion, location_EE, ground_truths_all_onions, ListIndices, \
	ListStatus, index_chosen_onion, target_location_x
	notgrasped = True

	while not rospy.is_shutdown():
		# implement MDP policy
		# res of previous will update the state, so need not check that
		execute_policy() 
		if len(WrapperObjectRL.traj_states) > desiredRecordingLen: 
			WrapperObjectRL.recordStateSequencetoFile() 
			exit(0) 

		r.sleep()

'''
		# Trial for fixing implementation issues in sequential execution of actions
		res3 = False
		res1 = impl_InspectWithoutPicking()
		print "res1:"+str(res1)
		print "ListIndices:"+str(ListIndices)
		print "ListStatus:"+str(ListStatus)
		print "index_chosen_onion:"+str(index_chosen_onion)
		res2 = False
		while ListStatus != 2:

		 	if index_chosen_onion != -1:
		 		print "update_pose_chosen_index"
		 		update_pose_chosen_index()
		 		print "target_location_x:"+str(target_location_x)
		 		print "location_claimed_onion:"+str(location_claimed_onion)

			if res1 == True and index_chosen_onion != -1:

				print "location_claimed_onion"+str(location_claimed_onion)
				res1 = False
				###### PICK action
				### did it disappear at time of picking? If yes, then discontinue
				while not (res2 == True and notgrasped == False): 
					if location_claimed_onion == -1:
						break
					res2 = impl_Pick() 
					# DONt implement detach_notgrasped if PICK doesn't return true
					if res2 == True: 
						notgrasped = detach_notgrasped() 
						print "notgrasped:"+str(notgrasped)
					print "impl_Pick():"+str(res2)

				# picking finished 
			if res2 == True:
				res2 = False 	
				res3 = impl_PlaceInBin()
				while res3 == False:
					res3 = impl_PlaceInBin()
				# resetting is necessary
				notgrasped = True

			if ListStatus != 2 and res3 == True:
				res3 = False
				res1 = impl_ClaimNextInList()
				print "index_chosen_onion:"+str(index_chosen_onion)
				print "res1:"+str(res1)
				print "ListIndices:"+str(ListIndices)
				print "ListStatus:"+str(ListStatus)
			else:
				print "res1:"+str(res1)
				print "ListIndices:"+str(ListIndices)
				print "ListStatus:"+str(ListStatus)
				break


'''

		# try:
		# 	resp = handle_update_state_EE() 
		# 	location_EE = resp.locations[1]
		# 	print "UPDATE_STATE srv. location_claimed_onion,location_EE:"\
		# 	+str((location_claimed_onion,location_EE))
		# except rospy.ServiceException, e: 
		# 	print "Service call failed: %s"%e 
		# execute_policy()

'''
	inspect_single = 1
	call_success = True
	try:
		print "called to reach home"
		resp = handle_move_sawyer_service("home",0) 
		print "resp.success:"+str(resp.success) 
		try:
			last_index_chosen_onion = index_chosen_onion
			while (index_chosen_onion == last_index_chosen_onion):
				resp = handle_claim_track_service(inspect_single) 
				print "resp.success:"+str(resp.success)+",resp.chosen_index:"+str(resp.chosen_index) 
				index_chosen_onion = resp.chosen_index

			if (index_chosen_onion != -1):
				try: 
					print "hover" 
					resp = handle_move_sawyer_service("hover",index_chosen_onion) 
				except rospy.ServiceException, e: 
					print "Service call failed: %s"%e 
				try: 
					print "lowergripper" 
					resp = handle_move_sawyer_service("lowergripper",index_chosen_onion) 
				except rospy.ServiceException, e: 
					print "Service call failed: %s"%e 
				try: 
					print "attach object" 
					resp = handle_move_sawyer_service("attach",index_chosen_onion) 
					time.sleep(1.0)
					call_success = resp.success
					# attach_srv.call(req_attach_detach)
					print "call_success: "+str(call_success)
				except rospy.ServiceException, e: 					
					print "Service call failed: %s"%e 
				while (call_success == False):
					try: 
						print "attach object" 
						resp = handle_move_sawyer_service("attach",index_chosen_onion) 
						time.sleep(1.0)
						call_success = resp.success
						# attach_srv.call(req_attach_detach)
						print "call_success: "+str(call_success)
					except rospy.ServiceException, e: 					
						print "Service call failed: %s"%e 

				if (call_success == True):
					try: 
						print "liftgripper" 
						resp = handle_move_sawyer_service("liftgripper",index_chosen_onion) 
						time.sleep(1.0)
					except rospy.ServiceException, e: 
						print "Service call failed: %s"%e 

					print "called to reach home"
					resp = handle_move_sawyer_service("home",index_chosen_onion) 

					try: 
						print "lookNrotate" 
						resp = handle_move_sawyer_service("lookNrotate",index_chosen_onion) 
					except rospy.ServiceException, e: 
						print "Service call failed: %s"%e 

					try: 
						print "conveyorCenter" 
						resp = handle_move_sawyer_service("conveyorCenter",index_chosen_onion) 
					except rospy.ServiceException, e: 
						print "Service call failed: %s"%e 
					# try: 
					# 	print "bin" 
					# 	resp = handle_move_sawyer_service("bin",index_chosen_onion) 
					# except rospy.ServiceException, e: 
					# 	print "Service call failed: %s"%e 
					try: 
						print "detach object" 
						resp = handle_move_sawyer_service("detach",index_chosen_onion) 
						call_success = resp.success
						# detach_srv.call(req_attach_detach)
						print "call_success: "+str(call_success)
					except rospy.ServiceException, e: 
						print "Service call failed: %s"%e 
						
		except rospy.ServiceException, e: 
			print "Service call failed: %s"%e 
	except rospy.ServiceException, e: 
		print "Service call failed: %s"%e 

'''