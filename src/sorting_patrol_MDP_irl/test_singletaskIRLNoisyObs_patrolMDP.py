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
import patrol
from test_singletaskIRL import printTs

from mdp.solvers import *
import mdp.agent
from mdp.simulation import *
import re

home = os.environ['HOME']
def get_home():
	global home
	return home

##############################################################
##############################################################


def parseTs(stdout):

	t = []
	weights = []
	transitions = stdout.split("\n")
	counter = 0
	for transition in transitions:
		counter += 1		
		if transition == "ENDT":
			break
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
	
	return (t, weights)

dummy_states = []
dict_stateEnum = {}
dict_actEnum = {}
f_st_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/traj_states.log", "w")
f_st_BIRLcode.write("")
f_st_BIRLcode.close()
f_ac_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/traj_actions.log", "w")
f_ac_BIRLcode.write("")
f_ac_BIRLcode.close()

f_st_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/traj_states.log", "a")
f_ac_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/traj_actions.log", "a")

def printTrajectoriesWPredScores(trajs,range_scores):
	outtraj = ""
	for patroller in trajs:

		for sap in patroller: 
			if (sap is not None): 
				s = sap[0]
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
					
				outtraj += ":"+str(random.uniform(range_scores[0],range_scores[1]))+";\n"
			
		outtraj += "ENDTRAJ\n"

	return outtraj

def enumerateForBIRLsortingModel1(trajs):

	patroller = trajs[0]
	for sap in patroller: 
		if (sap is not None): 
			s = sap[0]

			if sap[1].__class__.__name__ == "PatrolActionMoveForward":
				test_act = patrol.model.PatrolActionMoveForward()
			elif sap[1].__class__.__name__ == "PatrolActionTurnLeft":
				test_act = patrol.model.PatrolActionTurnLeft()
			elif sap[1].__class__.__name__ == "PatrolActionTurnRight":
				test_act = patrol.model.PatrolActionTurnRight()
			elif sap[1].__class__.__name__ == "PatrolActionTurnAround":
				test_act = patrol.model.PatrolActionTurnAround()
			else:
				test_act = patrol.model.PatrolActionStop()

			# adding data for BIRL MLIRL
			inds = dict_stateEnum.keys()[dict_stateEnum.values().index(s)]
			f_st_BIRLcode.write(str(inds)+",")
			inda = dict_actEnum.keys()[dict_actEnum.values().index(test_act)]
			f_ac_BIRLcode.write(str(inda)+",")

		else:
			print("can't enumerate bcz sap is none ")

		f_st_BIRLcode.write("\n")
		f_ac_BIRLcode.write("\n")

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
				
		p[ps] = act
		# print("parsed ss {} a {}".format(ss,act))

	from mdp.agent import MapAgent
	return MapAgent(p)

def parsePolicies(stdout, lineFoundWeights, lineFeatureExpec, \
	learned_weights, num_Trajsofar, BatchIRLflag):

	if stdout is None:
		print("no stdout in parse policies")
	
	stateactions = stdout.split("\n")
	#print("\n parse Policies from contents:")
	#print(stateactions)
	counter = 0
	p = {}
	for stateaction in stateactions:
		counter += 1		
		if stateaction == "ENDPOLICY":
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
				
		p[ps] = act

	returnval = [mdp.agent.MapAgent(p)]

	sessionFinish = True
	if len(stateactions[counter:])>0 and BatchIRLflag==False:
		# this change is not reflected in updatewithalg 

		sessionFinish = True
		# print("\n sessionFinish = True")#results after i2rl session at time: "+str(rospy.Time.now().to_sec()))
		# file = open("/home/saurabh/patrolstudy/i2rl_troubleshooting/I2RLOPread_rosctrl.txt","r")
		lineFoundWeights = stateactions[counter]
		counter += 1
		global reward_dim

		# print(lineFoundWeights[1:-1].split(", "))
		stripped_weights = lineFoundWeights[1:-1].split(", ")

		learned_weights = [float(x) for x in stripped_weights]
		
		# print("lineFoundWeights:"+lineFoundWeights) 
		lineFeatureExpec = stateactions[counter]
		counter += 1
		
		num_Trajsofar = int(stateactions[counter].split("\n")[0])
		counter += 1
		
	elif len(stateactions[counter:])==0:
		lineFoundWeights = lineFoundWeights 
		lineFeatureExpec = lineFeatureExpec 
		num_Trajsofar = num_Trajsofar
		sessionFinish = False
		print("\n no results from i2rl session")
	
	return (returnval, lineFoundWeights, lineFeatureExpec, \
		learned_weights, num_Trajsofar, sessionFinish)

def computeLBA(fileTruePolicy,model,mapAgentLrndPolicy):
	# read and compare policies using dictionaries 
	f = open(fileTruePolicy,"r")
	truePol = {}
	for stateaction in f:
		temp = stateaction.strip().split(" = ")
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
				
		
		truePol[ps] = a
	
	# print("number of keys for truePolicy ", len(truePol))
	# print("number of keys in leaerned policy ",len(mapAgentLrndPolicy._policy))
	# print("number of states in model ",len(model.S()))

	f.close()

	totalsuccess = 0
	totalstates = 0
	if (mapAgentLrndPolicy.__class__.__name__ == "MapAgent"):
		for s in model.S():
			if s in mapAgentLrndPolicy._policy:# check key existence 
				# print("number of actions in current state in learned policy",len(mapAgentLrndPolicy.actions(state).keys()))
				action = mapAgentLrndPolicy.actions(s).keys()[0]
				# action_name = action.__class__.__name__
				# print("action_name ",action_name)
				ss2 = (int(s._onion_location),int(s._prediction),\
					int(s._EE_location),int(s._listIDs_status))

				if ss2 in truePol.keys():
					totalstates += 1
					if (truePol[ss2] == action):
						# print("found a matching action")
						totalsuccess += 1
					# else:
					# 	print("for state {},  action {} neq action {} ".format(ss2,action,truePol[ss2]))

	print("totalstates, totalsuccess: "+str(totalstates)+", "+str(totalsuccess))
	if float(totalstates) == 0: 
		print("Error: states in two policies are different")
		return 0
	lba=float(totalsuccess) / float(totalstates)

	return lba

def saveDataForBaseline():

	#############################################################
	# BIRL input data for checking if problem is method
	#############################################################

	sortingMDP = model
	for s in sortingMDP.S():
		dummy_states.append(s)
	dummy_states.append(sortingState(-1,-1,-1,-1))

	ind = 0
	for s in dummy_states:
		ind = ind +1
		dict_stateEnum[ind] = s
	print("dict_stateEnum \n",dict_stateEnum)

	acts = [InspectAfterPicking(),PlaceOnConveyor(),PlaceInBin(),\
	Pick(),ClaimNewOnion(),InspectWithoutPicking(),ClaimNextInList()] 
	ind = 0
	for a in acts:
		ind = ind +1
		dict_actEnum[ind] = a

	# record first trajectory in data for single task BIRL
	enumerateForBIRLsortingModel1(traj)

	f_st_BIRLcode.close()
	f_ac_BIRLcode.close()

	f_TM_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/transition_matrix.txt", "w")
	f_TM_BIRLcode.write("")
	f_TM_BIRLcode.close()
	tuple_res = sortingMDP.generate_matrix(dict_stateEnum,dict_actEnum)
	dict_tr = tuple_res[0]
	f_TM_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/transition_matrix.txt", "a")
	for ind1 in range(1,len(dict_actEnum)+1):
		acArray2d = np.empty((len(dict_stateEnum),len(dict_stateEnum)))

		for ind2 in range(1,len(dict_stateEnum)+1):
			for ind3 in range(1,len(dict_stateEnum)+1):
				acArray2d[ind3-1][ind2-1] = dict_tr[ind1][ind3][ind2]

		for ind3 in range(1,len(dict_stateEnum)+1):
			for ind2 in range(1,len(dict_stateEnum)+1):
				f_TM_BIRLcode.write(str(acArray2d[ind3-1][ind2-1])+",")
			f_TM_BIRLcode.write("\n")
		f_TM_BIRLcode.write("\n")

	f_TM_BIRLcode.close()

	f_Phis_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/features_matrix.txt", "w")
	f_Phis_BIRLcode.write("")
	f_Phis_BIRLcode.close()
	f_Phis_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/features_matrix.txt", "a")
	for inda in range(1,len(dict_actEnum)+1):
		a = dict_actEnum[inda] 
		for inds in range(1,len(dict_stateEnum)+1):
			s = dict_stateEnum[inds]
			arraysPhis = sortingReward.features(s,a)
			for indk in range(1,len(arraysPhis)+1):
				f_Phis_BIRLcode.write(str(arraysPhis[indk-1])+",")
			f_Phis_BIRLcode.write("\n")
		f_Phis_BIRLcode.write("\n")
	f_Phis_BIRLcode.close() 

	wts_experts_array = np.empty((sortingReward._dim,len(np.unique(true_assignments))))
	j = 0
	for wt_ind in np.unique(true_assignments):
		for i in range(0,wts_experts_array.shape[0]):
			wts_experts_array[i][j] = List_TrueWeights[wt_ind][i]
		j += 1

	f_wts_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/weights_experts.log", "w")
	f_wts_BIRLcode.write("") 
	f_wts_BIRLcode.close() 
	f_wts_BIRLcode = open(get_home() + "/catkin_ws/src/BIRL_MLIRL_data/weights_experts.log", "a")
	for i in range(0,wts_experts_array.shape[0]):
		for e in range(0,wts_experts_array.shape[1]):
			f_wts_BIRLcode.write(str(wts_experts_array[i][e])+",")
		f_wts_BIRLcode.write("\n")
	f_wts_BIRLcode.close()

	#############################################################
	#############################################################

##############################################################
###############################################################


if __name__ == "__main__": 


	# D code for single task IRL uses 0.95 success rate of transitions
	p_fail = 0.0
	m = "boyd2"
	mapparams = boyd2MapParams(False)
	ogmap = OGMap(*mapparams)

	## Create Model 
	model = PatrolModel(p_fail, None, ogmap.theMap())
	model.gamma = 0.99


	#############################################################
	# Needed for synchornizing BIRL input data
	#############################################################

	reward_dim = 6
	List_TrueWeights = []
	params = [1, 0, 0, 0, 0.75, 0]
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	List_TrueWeights.append(norm_params)
	
	#############################################################
	# demonstration had two runs with one trajectory for each run 
	true_assignments = [0]

	params = List_TrueWeights[true_assignments[0]]
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]

	# for multiple starting state
	initial = util.classes.NumMap() 
	count = 0
	for s in model.S(): 
		if s.location[1] <= 5:
			initial[s] = 1.0			
			count+=1
	print("number of initial states ", count)
	initial = initial.normalize()

	#############################################################
	
	#############################################################


	# Call external solver here instead of Python based 
	args = [get_home() +"/catkin_ws/devel/bin/"+"boydsimple_t", ]
	import subprocess			

	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
	stdin = m + "\n"
	stdin += "1.0\n"
	# print("input to boydsimple_t ",stdin)
	(transitionfunc, stderr) = p.communicate(stdin)
	# print(" outputof call to boydsimple_t ")
	# print(transitionfunc)

	p.stdin.close()
	p.stdout.close()
	p.stderr.close()

	args = [get_home() +"/catkin_ws/devel/bin/"+"boydpatroller", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
	
	stdin = m + "\n"
	useRegions = 0
	stdin += str(useRegions)+"\n"
	stdin += transitionfunc
	# print("input to boydpatroller ",stdin)
	(stdout, stderr) = p.communicate(stdin)
	# print("output to boydpatroller\n ",stdout)
	print("\n\n")
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
	
	n_samples = 1

	# for each of two runs of irl, t_max will be divided into length_subtrajectory long trajs
	# length_subtrajectory = 10
	# length_subtrajectory = 15
	length_subtrajectory = 40
	# length_subtrajectory = 50
	# length_subtrajectory = 80

	# t_max = length_subtrajectory*1
	t_max = length_subtrajectory*3
	sample_learnedPolicy = 0

	#for I2RL
	num_sessions = 1
	num_Trajsofar = 0
	learned_mu_E=[0.0]*reward_dim
	learned_weights=[0.0]*reward_dim

	# parameters for solver
	restart_attempts = 3
	moving_window_length_muE = 3

	# threshold for convergence of gibbs sampling for robust-irl 
	# by Shervin 
	# 0.025 not giving expected trend
	conv_threshold_gibbs = 0.015
	# conv_threshold_gibbs = 0.01

	# which kind of sampling method is being used?
	use_ImpSampling = 0
	if (use_ImpSampling == 1):
		# imp sampling, for prediciton score 0.85-0.99  and only PIP
		conv_threshold_stddev_diff_moving_wdw = 0.0005 
	else:
		# gibbs sampling, for prediciton score 0.85-0.99  and only PIP
		# which value shows LBA monotonically decreasing with
		# confidence? 
		# 0.01 No trend 
		# 0.005 No trend
		# However, changing conv_threshold_gibbs made a difference
		conv_threshold_stddev_diff_moving_wdw = 0.005 


	# ranges of noise in observations 
	range_pred_scores1 = [1.0,1.0]
	range_pred_scores2 = [0.90,0.99]
	range_pred_scores3 = [0.80,0.90]
	range_pred_scores4 = [0.70,0.80]
	range_pred_scores5 = [0.60,0.70]

	ranges_pred_scores = [range_pred_scores1, range_pred_scores2, range_pred_scores3, range_pred_scores4, range_pred_scores5]

	print("writing result of calls to noisyObsRobustSamplingMeirl to file catkin_ws/src/navigation_irl/noisyObsRobustSamplingMeirl_LBA_data.csv") 
	# output LBA to file
	f_input_IRL = open(get_home() +'/catkin_ws/src/navigation_irl/noisyObsRobustSamplingMeirl_LBA_data.csv', "w")
	f_input_IRL.write("")
	f_input_IRL.close()
	f_rec = open(get_home()+'/catkin_ws/src/navigation_irl/noisyObsRobustSamplingMeirl_LBA_data.csv','a') 
	csvstring = "\n" 

	for range_sc in ranges_pred_scores: 
		for sess in range(num_sessions):

			traj = []
			print( "demonstration") 
			for i in range(n_samples): 
				# traj_list = simulate(model, policy, initial, t_max) 
				traj_list = sample_traj(model, t_max, initial, policy) 
				traj.append(traj_list) 
				# for (s,a,s_p) in traj_list:
					# print((s,a))
				# print("\n")

			outtraj = None
			args = [get_home() +"/catkin_ws/devel/bin/"+ "noisyObsRobustSamplingMeirlPatrol", ]
			p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)				
			outtraj = ""
			outtraj += "boyd2" + "\n"

			T =[]
			(T, weights) = parseTs(transitionfunc)
			outtraj += printTs([T])

			# algorithm = "MAXENTZAPPROX" 
			algorithm = "MAXENTZAPPROXNOISYOBS" 
			outtraj += algorithm+"\n"

			# add prediction scores
			outtraj += printTrajectoriesWPredScores(traj,range_sc)
			# print(printTrajectoriesWPredScores(traj,range_sc))

			# specific to sorting mdp 
			outtraj += str(norm_params)+"\n"
			outtraj += str(length_subtrajectory)+"\n"
			outtraj += str(conv_threshold_stddev_diff_moving_wdw)+"\n"\
			+str(restart_attempts)+"\n"+str(moving_window_length_muE)+"\n"\
			+str(use_ImpSampling)+"\n"+str(conv_threshold_gibbs)+"\n"


			if num_Trajsofar == 0:
				
				for j in range(reward_dim):
					learned_weights[j] = random.uniform(0.0,.99)
				
				lineFoundWeights = str(learned_weights) #+"\n"
				# create initial feature expectations
				for j in range(reward_dim):
					learned_mu_E[j]=0.0
				
				lineFeatureExpec = str(learned_mu_E) #+"\n"

			if not not lineFoundWeights and lineFoundWeights[-1] != '\n':
				lineFoundWeights = lineFoundWeights + "\n" 

			if not not lineFeatureExpec and lineFeatureExpec[-1] != '\n': 
				lineFeatureExpec = lineFeatureExpec + "\n" 

			outtraj += lineFoundWeights+lineFeatureExpec+ str(num_Trajsofar)+"\n"  

			# input data to file
			f_input_IRL = open(get_home() + "/catkin_ws/src/navigation_irl/data_singleTaskIRLNoisyObs_patrolling.log", "w")
			f_input_IRL.write("")
			f_input_IRL.close()
			f_input_IRL = open(get_home() + "/catkin_ws/src/navigation_irl/data_singleTaskIRLNoisyObs_patrolling.log", "a")
			f_input_IRL.write(outtraj)
			f_input_IRL.close()

			# print(outtraj)
			# exit(0)

			(stdout, stderr) = p.communicate(outtraj)

			print("output of meirl ") 
			# print(stdout)

			# exit(0)
			
			print("session {} finished".format(sess))
			p.stdin.close()
			p.stdout.close()
			p.stderr.close()

			# print("parsing policies ")
			emphasizedOutput = re.findall('BEGPARSING\n(.[\s\S]+?)ENDPARSING', stdout)[0] 
			# print(emphasizedOutput)

			BatchIRLflag = False 
			normedRelDiff = 0 
			(policies, lineFoundWeights, lineFeatureExpec, learned_weights, \
			num_Trajsofar, sessionFinish) \
			= parsePolicies(emphasizedOutput, lineFoundWeights, lineFeatureExpec, learned_weights, \
			num_Trajsofar, BatchIRLflag) 

			num_Trajsofar += t_max/length_subtrajectory
			# print("num_Trajsofar, learned_weights ",(num_Trajsofar, learned_weights))

		# LBA should be read after last session 
		# print("re.findall('LBA(.[\s\S]+?)ENDLBA', stdout) ",re.findall('LBA(.[\s\S]+?)ENDLBA', stdout))
		LBA = re.findall('LBA(.[\s\S]+?)ENDLBA', stdout)[0]
		print("LBA:",LBA) 

		hatphi_Diff_wrt_wonoise = re.findall('DIFF1(.[\s\S]+?)ENDDIFF1', stdout)[0]
		print("hatphi_Diff_wrt_wonoise:",hatphi_Diff_wrt_wonoise) 

		hatphi_Diff_wrt_scores1 = re.findall('DIFF2(.[\s\S]+?)ENDDIFF2', stdout)[0]
		print("hatphi_Diff_wrt_scores1:",hatphi_Diff_wrt_scores1) 

		################################ Simulating learned policy #################################

		policies = policies[0:2]
		# print("number of policies learned ",len(policies))

		# exit(0)
		if sample_learnedPolicy == 1:		
			n_samples = 4 
			t_max = 10
			for i in range(len(policies)): 
				policy = policies[i] 
				print("trajs from learned policy number ",i)
				print("\n")
				for j in range(n_samples): 
					traj_list = sample_traj(model, t_max, initial, policy) 
					for (s,a,s_p) in traj_list:
						print((s,a))
					print("\n")

		csvstring += str(hatphi_Diff_wrt_wonoise)+"," +str(hatphi_Diff_wrt_scores1)+","+str(LBA)+","

	f_rec.write(csvstring)
	f_rec.close()
