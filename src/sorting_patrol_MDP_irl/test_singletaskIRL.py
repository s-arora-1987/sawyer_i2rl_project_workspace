#!/usr/bin/env python
import sys
import Queue
import subprocess
import multiprocessing
import random
import cPickle as pickle
import os
import operator
import time
import numpy as np
import util.classes
from patrol.model import boyd2MapParams, OGMap, PatrolModel 

from mdp.solvers import *
import mdp.agent
from mdp.simulation import *
# from ros_ctrl import printTs, printTrajectories, parsePolicies

home = os.environ['HOME']
def get_home():
	global home
	return home

##############################################################
##############################################################


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
		#print("\n sessionFinish = True")#results after i2rl session at time: "+str(rospy.Time.now().to_sec()))
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



##############################################################
###############################################################




if __name__ == "__main__": 


	# D code uses 1.0 success rate of transitions
	p_fail = 0.0
	m = "boyd2"
	mapparams = boyd2MapParams(False)
	ogmap = OGMap(*mapparams)

	## Create Model 
	model = PatrolModel(p_fail, None, ogmap.theMap())
	model.gamma = 0.9

	# Call external solver here instead of Python based 
	args = ["boydsimple_t", ]
	import subprocess			

	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
	stdin = m + "\n"
	stdin += "1.0\n"
	(transitionfunc, stderr) = p.communicate(stdin)
	# print(" outputof call to boydsimple_t ")
	#print(transitionfunc)

	p.stdin.close()
	p.stdout.close()
	p.stderr.close()

	args = ["boydpatroller", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
	
	stdin = m + "\n"
	useRegions = 0
	stdin += str(useRegions)+"\n"
	stdin += transitionfunc
	(stdout, stderr) = p.communicate(stdin)
	p.stdin.close()
	p.stdout.close()
	p.stderr.close()

	#print(" lenght of output of call to boydpatroller ",len(stdout))
	# print(stdout)
	pol = {}

	stateactions = stdout.split("\n")
	for stateaction in stateactions:
		temp = stateaction.split(" = ")
		# print("temp ", temp)
		if len(temp) < 2: 
			continue
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
 
		pol[ps] = a
	
	from mdp.agent import MapAgent
	policy = MapAgent(pol)
	
	initial = util.classes.NumMap()
	for s in model.S():
		if s.location[1] <= 5:
			initial[s] = 1.0			
	initial = initial.normalize()

	n_samples = 2
	t_max = 110
	traj = [[], []]
	#print( "demonstration")
	for i in range(n_samples): 
		# traj_list = simulate(model, policy, initial, t_max)
		traj_list = sample_traj(model, t_max, initial, policy) 
		traj[i] = traj_list
		# for (s,a,s_p) in traj_list:
			# print((s,a))
		#print("\n")

	# print(printTrajectories(traj))
	# exit(0)

	outtraj = None
	args = ["boydirl", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)				
	outtraj = ""
	outtraj += "boyd2" + "\n"
	add_delay = False 
	if add_delay:
		outtraj += "true\n"
	else:
		outtraj += "false\n"

	NE = "cs" 
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
	
	algorithm = "MAXENTZAPPROX" 
	outtraj += algorithm+"\n"
	outtraj += str(equilibriumCode)	+"\n"
	
	equilibriumKnown = False
	if equilibriumKnown:
		outtraj += "true\n"
	else:
		outtraj += "false\n"

	interactionLength, visibleStatesNum = 3, 1.0
	outtraj += str(interactionLength)+"\n"
	outtraj += str(visibleStatesNum / 14.0)	+"\n"

	(T, weights) = parseTs(transitionfunc)
	outtraj += printTs(T)

	useRegions = 0
	outtraj += str(useRegions)+"\n"

	outtraj += printTrajectories(traj)
	
	reward_dim = 6
	num_Trajsofar = 0
	learned_mu_E=[[0.0]*reward_dim,[0.0]*reward_dim] 
	learned_weights=[[0.0]*reward_dim,[0.0]*reward_dim] # needed for compute reldiff
	wt_data=numpy.empty([2, reward_dim], dtype=float)

	if num_Trajsofar == 0:
		
		for i in range(2):
			for j in range(reward_dim):
				learned_weights[i][j] = random.uniform(0.0,.99)
		
		wt_data = numpy.array([learned_weights])
		
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
	outtraj += printTrajectories(traj)

	#print("input to boydirl \n",printTrajectories(traj))
	# print(outtraj)

	(stdout, stderr) = p.communicate(outtraj)

	# print("output of boydirl ")
	# print(stdout)

	p.stdin.close()
	p.stdout.close()
	p.stderr.close()

	BatchIRLflag = True
	normedRelDiff = sys.maxint*1.0 

	(policies, lineFoundWeights, lineFeatureExpec, learned_weights, \
	num_Trajsofar, \
	sessionFinish, wt_data, normedRelDiff, lastQ, lineFeatureExpecfull)\
	= parsePolicies(stdout, None, lineFoundWeights, lineFeatureExpec, learned_weights, \
	num_Trajsofar, \
	BatchIRLflag, wt_data, normedRelDiff)

	policies = policies[0:2]
	print("number of policies learned ",len(policies))

	n_samples = 2 
	t_max = 80
	for i in range(len(policies)): 
		policy = policies[i] 
		print("trajs from policy learned for ",i)
		print("\n")
		for j in range(n_samples): 
			traj_list = sample_traj(model, t_max, initial, policy) 
			for (s,a,s_p) in traj_list:
				print((s,a))
			print("\n")

	exit(0)


