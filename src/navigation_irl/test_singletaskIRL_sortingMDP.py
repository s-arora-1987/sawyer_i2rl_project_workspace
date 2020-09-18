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
# from patrol.model import boyd2MapParams, OGMap, PatrolModel 
from sortingMDP.model import sortingModel,InspectAfterPicking,\
PlaceOnConveyor,PlaceInBin,Pick,ClaimNewOnion,InspectWithoutPicking,\
ClaimNextInList,sortingState 
from sortingMDP.model import sortingModel2,\
PlaceInBinClaimNextInList,sortingModelbyPSuresh,\
sortingModelbyPSuresh2,sortingModelbyPSuresh3,sortingModelbyPSuresh4

from sortingMDP.reward import sortingReward2,\
sortingReward3,sortingReward4,sortingReward5,\
sortingReward6,sortingReward7

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

dummy_states = []
dict_stateEnum = {}
dict_actEnum = {}
f_st_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/traj_states.log", "w")
f_st_BIRLcode.write("")
f_st_BIRLcode.close()
f_ac_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/traj_actions.log", "w")
f_ac_BIRLcode.write("")
f_ac_BIRLcode.close()

f_st_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/traj_states.log", "a")
f_ac_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/traj_actions.log", "a")

def printTrajectories(trajs):
	outtraj = ""
	for patroller in trajs:

		for sap in patroller: 
			if (sap is not None): 
				s = sap[0]
				outtraj += "[ "+str(s._onion_location)+", "\
				+str(s._prediction)+", "+\
				str(s._EE_location)+", "+\
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

				elif sap[1].__class__.__name__ == "PlaceInBinClaimNextInList":
					act_str = "PlaceInBinClaimNextInList"

				else:
					act_str = "ActionInvalid"

				outtraj += act_str

			else:
				outtraj += "None"

			outtraj += ":1;\n"

		outtraj += "ENDTRAJ\n"

	return outtraj

def enumerateForBIRLsortingModel1(trajs):

	patroller = trajs[0]
	for sap in patroller: 
		if (sap is not None): 
			s = sap[0]

			if sap[1].__class__.__name__ == "InspectAfterPicking":
				test_act = InspectAfterPicking()
				 
			elif sap[1].__class__.__name__ == "InspectWithoutPicking":
				test_act = InspectWithoutPicking()
				 
			elif sap[1].__class__.__name__ == "Pick":
				test_act = Pick()
				 
			elif sap[1].__class__.__name__ == "PlaceOnConveyor":
				test_act = PlaceOnConveyor()
				 
			elif sap[1].__class__.__name__ == "PlaceInBin":
				test_act = PlaceInBin()
				 
			elif sap[1].__class__.__name__ == "ClaimNewOnion":
				test_act = ClaimNewOnion()
				 
			elif sap[1].__class__.__name__ == "ClaimNextInList":
				test_act = ClaimNextInList()

			else:
				print("can't enumerate ",sap[1])

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
		elif action == "PlaceInBinClaimNextInList":
			act = PlaceInBinClaimNextInList()
		else:
			print("Invalid input policy to parse_sorting_policy")
			exit(0)
		
		p[ss] = act
		# print("parsed ss {} a {}".format(ss,act))

	from mdp.agent import MapAgent
	return MapAgent(p)

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
		ss = sortingState(int(pieces[0]), int(pieces[1]), int(pieces[2]), int(pieces[3]))
		print((state,pieces,ss))

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
		elif action == "PlaceInBinClaimNextInList":
			act = PlaceInBinClaimNextInList()
		else:
			print("Invalid input policy to parse_sorting_policy")
			exit(0)
			
		p[ss] = act

	if (len(pmaps) < 2):
		returnval = [mdp.agent.MapAgent(p), mdp.agent.MapAgent(p)]
	else:
		returnval = [mdp.agent.MapAgent(pmaps[0]), mdp.agent.MapAgent(pmaps[1])]

		
	if len(stateactions[counter:])>0: #and BatchIRLflag==False:
		# this change is not reflected in updatewithalg 
		sessionFinish = True
		#print("\n sessionFinish = True")#results after i2rl session at time: "+str(rospy.Time.now().to_sec()))
		#file = open("/home/saurabh/patrolstudy/i2rl_troubleshooting/I2RLOPread_rosctrl.txt","r")
		lineFoundWeights = stateactions[counter]
		counter += 1
		global reward_dim
		found_weights = [[0.0]*reward_dim,[0.0]*reward_dim]

		print(lineFoundWeights)

		found_weights = [[float(x) for x in \
		lineFoundWeights.split("\n")[0]. \
		strip("[]").split("], [")[0].split(", ")], \
		[float(x) for x in \
		lineFoundWeights.split("\n")[0]. \
		strip("[]").split("], [")[1].split(", ")]]
		
		learned_weights = found_weights
		
		#print("lineFoundWeights:"+lineFoundWeights)
		lineFeatureExpec = stateactions[counter]
		counter += 1
		
		num_Trajsofar = int(stateactions[counter].split("\n")[0])
		counter += 1
		
		#print("num_Trajsofar:"+str(num_Trajsofar))

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


	# D code for single task IRL uses 0.95 success rate of transitions
	p_fail = 0.05
	m = "sorting"
	# model = sortingModel(p_fail) 
	# model = sortingModel2(p_fail) 
	# model = sortingModelbyPSuresh(p_fail)
	model = sortingModelbyPSuresh2(p_fail)
	# model = sortingModelbyPSuresh3(p_fail)
	# model = sortingModelbyPSuresh4(p_fail)

	# print(sortingModelbyPSuresh._p_fail)
	model.gamma = 0.99
	# sortingReward = sortingReward2(8) 
	# sortingReward = sortingReward3(10) 
	# sortingReward = sortingReward4(10) 
	# sortingReward = sortingReward5(8) 
	# sortingReward = sortingReward6(11) 
	sortingReward = sortingReward7(11) 

	reward_dim = sortingReward._dim
	print("reward_dim ",reward_dim)

	model.reward_function = sortingReward 

	params_manualTuning_rolling_reward3 = [0.15, -0.08, -0.11, 0.3, -0.3, -0.15, 0.6, -0.15, 0.6, -0.2]
	params_manualTuning_rolling_reward4 = [0.0, 0.6, 0.0, 0.95, 0.8, 0.0, 0.9, 0.15, 0.9, 0.4]

	params_manualTuning_pickinspectplace_reward3 = [ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12, 0.0, -0.2]
	params_manualTuning_pickinspectplace_reward4 = [ 0.10, 0.0, 0.0, 0.22, 0.12, 0.44, 0.0, 0.12, 0.0, 0.2]
	'''
	reward 4
	// good placed on belt
	// not placing bad on belt
	// not placing good in bin
	// bad placed in bin
	// not staying still
	// classify after picking
	// create the list 
	// not picking a placed one
	// classify without picking
	// not placing uninspected in bin

	'''
	# params_manualTuning_pickinspectplace_reward5 =[1,-1,-1,1,-0.2,1,0,1]
	params_rolling_reward5 =[0,4,0,4,0.2,0,8,0]
	params_pickinspectplace_reward5 =[2,1,2,1,0.2,1,0,4]
	params_rolling_reward6 =[0,4,0,4,0.2,0,8,0,8,4,0]
	params_pickinspectplace_reward6 =[2,1,2,1,0.2,1,0,4,0,0,4]
	params_staystill_reward6 = [ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  0.0, 0.0, 0.0] 

	#############################################################
	# Needed for synchornizing BIRL input data
	#############################################################

	List_TrueWeights = []
	# index 0 for pick-inspect-place 
	params = params_pickinspectplace_reward6
	# for psuresh4 model
	# params_pickinspectplace_reward7_psurmodel4 = [0,7,0,9,0.2,1,0,4,0,0,4]
	# params = params_pickinspectplace_reward7_psurmodel4
	# params = params_pip_reward7_psurmodel4
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	List_TrueWeights.append(norm_params)
	# index 1 for stay-still 
	params = params_staystill_reward6
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	List_TrueWeights.append(norm_params)
	# index 2 for roll-pick-place 
	params = params_rolling_reward6
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	List_TrueWeights.append(norm_params)
	# index 3 for placeallonconvyor (specific to psuresh4 mdp model)
	params_placeallonconvyor_reward7_psurmodel4 = [2,0,2,0,0.2,1,0,4,0,0,4]
	params = params_placeallonconvyor_reward7_psurmodel4
	# index 3 for placeallinbin (specific to psuresh4 mdp model)
	# params_placeallinbin_reward7_psurmodel4 = [-5,1,0,1,0.2,1,0,4,0,0,4]
	# params = params_placeallinbin_reward7_psurmodel4
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	List_TrueWeights.append(norm_params)

	#############################################################
	# demonstration had two runs with one trajectory for each run 
	# true_assignments = [0,1,2]
	true_assignments = [0,1,2,3]

	# pick-inspect-place
	# params = List_TrueWeights[true_assignments[0]]
	# roll-pick-place
	# params = List_TrueWeights[true_assignments[0]]

	params = List_TrueWeights[true_assignments[2]]
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]

	initial = util.classes.NumMap()
	# ALWAYS START FROm  0,2,0,2
	# pick-inspect-place
	# s = sortingState(0,2,0,2)	
	# roll-pick-place
	s = sortingState(0,2,0,0)	
	initial[s] = 1.0
	initial = initial.normalize()

	#############################################################
	
	#############################################################

	# norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	args = [get_home() + "/catkin_ws/devel/bin/solveSortingMDP", ]
	p = subprocess.Popen(args, stdin	=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdin = str(norm_params) 
	print("input to solveSortingMDP ",stdin)
	(stdout, stderr) = p.communicate(stdin) 
	# print("output to solveSortingMDP ",stdout)
	policy = parse_sorting_policy(stdout) 
	p.stdin.close()
	p.stdout.close()
	p.stderr.close()
	
	n_samples = 2

	# for each of two runs of irl, t_max will be divided into length_subtrajectory long trajs
	t_max = 200
	# t_max = 300
	# t_max = 400
	# t_max = 2500
	length_subtrajectory = 50
	traj = [[], []]

	print( "demonstration")
	for i in range(n_samples): 
		# traj_list = simulate(model, policy, initial, t_max)
		traj_list = sample_traj(model, t_max, initial, policy) 
		traj[i] = traj_list
		# for (s,a,s_p) in traj_list:
			# print((s,a))
		#print("\n")

	print(printTrajectories(traj))

	# exit(0)

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

	f_TM_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/transition_matrix.txt", "w")
	f_TM_BIRLcode.write("")
	f_TM_BIRLcode.close()
	tuple_res = sortingMDP.generate_matrix(dict_stateEnum,dict_actEnum)
	dict_tr = tuple_res[0]
	f_TM_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/transition_matrix.txt", "a")
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

	f_Phis_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/features_matrix.txt", "w")
	f_Phis_BIRLcode.write("")
	f_Phis_BIRLcode.close()
	f_Phis_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/features_matrix.txt", "a")
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

	f_wts_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/weights_experts.log", "w")
	f_wts_BIRLcode.write("") 
	f_wts_BIRLcode.close() 
	f_wts_BIRLcode = open(get_home() + "/BIRL_MLIRL_data/weights_experts.log", "a")
	for i in range(0,wts_experts_array.shape[0]):
		for e in range(0,wts_experts_array.shape[1]):
			f_wts_BIRLcode.write(str(wts_experts_array[i][e])+",")
		f_wts_BIRLcode.write("\n")
	f_wts_BIRLcode.close()

	#############################################################
	#############################################################

	outtraj = None
	args = ["boydirl", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)				
	outtraj = ""
	outtraj += "sorting" + "\n"

	add_delay = False 
	if add_delay:
		outtraj += "true\n"
	else:
		outtraj += "false\n"

	algorithm = "MAXENTZAPPROX" 
	outtraj += algorithm+"\n"

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
	outtraj += str(equilibriumCode)	+"\n"
	equilibriumKnown = False
	if equilibriumKnown:
		outtraj += "true\n"
	else:
		outtraj += "false\n"

	interactionLength, visibleStatesNum = 3, 1.0
	outtraj += str(interactionLength)+"\n"
	outtraj += str(visibleStatesNum / 14.0)	+"\n"
	outtraj += "ENDT\n"	
	outtraj += "ENDT\n"	
	useRegions = 0
	outtraj += str(useRegions)+"\n"

	outtraj += printTrajectories(traj)
	
	num_Trajsofar = 0
	learned_mu_E=[[0.0]*reward_dim,[0.0]*reward_dim] 
	learned_weights=[[0.0]*reward_dim,[0.0]*reward_dim] # needed for compute reldiff
	wt_data=numpy.empty([2, reward_dim], dtype=float)
	if num_Trajsofar == 0:
		
		for i in range(2):
			for j in range(reward_dim):
				learned_weights[i][j] = random.uniform(-.99,.99)
		
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
	# outtraj += lineFoundWeights+lineFeatureExpec+ str(num_Trajsofar)+"\n"  

	# specific to sorting mdp 
	outtraj += str(norm_params)+"\n"
	outtraj += str(length_subtrajectory)+"\n"
	# second time needed
	outtraj += printTrajectories(traj)

	print("input to boydirl \n")
	f_input_IRL = open(get_home() + "/catkin_ws/src/navigation_irl/data_Dcode_BoydIRL_sorting.log", "w")
	f_input_IRL.write("")
	f_input_IRL.close()
	f_input_IRL = open(get_home() + "/catkin_ws/src/navigation_irl/data_Dcode_BoydIRL_sorting.log", "a")
	f_input_IRL.write(outtraj)
	f_input_IRL.close()

	# print(outtraj)
	# exit(0)

	(stdout, stderr) = p.communicate(outtraj)

	print("output of boydirl ")
	print(stdout)

	p.stdin.close()
	p.stdout.close()
	p.stderr.close()

	BatchIRLflag = True
	normedRelDiff = sys.maxint*1.0 

	import re
	stringPols = re.findall('BEGPARSING\n(.[\s\S]+?)ENDPARSING', stdout)[0]

	(policies, lineFoundWeights, lineFeatureExpec, learned_weights, \
	num_Trajsofar, \
	sessionFinish, wt_data, normedRelDiff, lastQ, lineFeatureExpecfull)\
	= parsePolicies(stringPols, None, lineFoundWeights, lineFeatureExpec, learned_weights, \
	num_Trajsofar, \
	BatchIRLflag, wt_data, normedRelDiff)

	policies = policies[0:2]
	print("number of policies learned ",len(policies))

	# exit(0)
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


