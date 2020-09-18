# -*- coding: utf-8 -*- 
from model import sortingModel,sortingModel2,\
InspectAfterPicking,PlaceOnConveyor,PlaceInBin,\
Pick,ClaimNewOnion,InspectWithoutPicking,\
ClaimNextInList,sortingState,sortingModelbyPSuresh,\
sortingModelbyPSuresh2,sortingModelbyPSuresh3 
from reward import sortingReward1 
from reward import sortingReward2,sortingReward4,\
sortingReward6,sortingReward7
import mdp.agent
import util.classes
import mdp.simulation
import numpy as np
import os
import subprocess

home = os.environ['HOME']
def get_home():
	global home
	return home

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


def parseNwrite_sorting_policy(fileObj,buf):
	# stdout now needs to be parsed into a hash of state => action, which is then sent to mapagent
	fileObj.write(buf)
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
		elif action == "Pickpip":
			act = Pickpip()
		elif action == "PlaceInBinpip":
			act = PlaceInBinpip()
		else:
			print("Invalid input policy to parse_sorting_policy")
			exit(0) # Pickpip,PlaceInBinpip
		
		p[ss] = act

	from mdp.agent import MapAgent
	return MapAgent(p)

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
		elif action == "Pickpip":
			act = Pickpip()
		elif action == "PlaceInBinpip":
			act = PlaceInBinpip()
		else:
			print("Invalid input policy to parse_sorting_policy")
			exit(0) # Pickpip,PlaceInBinpip
		
		p[ss] = act

	from mdp.agent import MapAgent
	return MapAgent(p)

def formatWrite_simulated_trajectories(inputBuf,num_trajs,sortingMDP,opt_policy,initial,traj_size):

	for i in range(0,num_trajs):

		outtraj = ""
		conv_traj = mdp.simulation.simulate(sortingMDP,opt_policy,initial,traj_size) 

		s = None
		act_str = None

		for sap in conv_traj: 
			if (sap is not None): 
				s = sap[0]
				outtraj += "[ "+str(s._onion_location)+", "\
				+str(s._prediction)+", "+\
				str(s._EE_location)+", "+\
				str(s._listIDs_status)+"]:"

				if sap[1].__class__.__name__ == "InspectAfterPicking":
					act_str = "InspectAfterPicking"
					test_act = InspectAfterPicking()
					 
				elif sap[1].__class__.__name__ == "InspectWithoutPicking":
					act_str = "InspectWithoutPicking"
					test_act = InspectWithoutPicking()
					 
				elif sap[1].__class__.__name__ == "Pick":
					act_str = "Pick"
					test_act = Pick()
					 
				elif sap[1].__class__.__name__ == "PlaceOnConveyor":
					act_str = "PlaceOnConveyor"
					test_act = PlaceOnConveyor()
					 
				elif sap[1].__class__.__name__ == "PlaceInBin":
					act_str = "PlaceInBin"
					test_act = PlaceInBin()
					 
				elif sap[1].__class__.__name__ == "ClaimNewOnion":
					act_str = "ClaimNewOnion"
					test_act = ClaimNewOnion()
					 
				elif sap[1].__class__.__name__ == "ClaimNextInList":
					act_str = "ClaimNextInList"
					test_act = ClaimNextInList()
					
				elif sap[1].__class__.__name__ == "Pickpip":
					act_str = "Pickpip"
					test_act = Pickpip()
					 
				elif sap[1].__class__.__name__ == "PlaceInBinpip":
					act_str = "PlaceInBinpip"
					test_act = PlaceInBinpip()

				else:
					act_str = "ActionInvalid"

				outtraj += act_str

				# adding data for BIRL MLIRL
				inds = dict_stateEnum.keys()[dict_stateEnum.values().index(s)]
				f_st_BIRLcode.write(str(inds)+",")
				inda = dict_actEnum.keys()[dict_actEnum.values().index(test_act)]
				f_ac_BIRLcode.write(str(inda)+",")

			else:
				outtraj += "None"
			outtraj += ":1;\n"
		
		outtraj += "ENDTRAJ\n"

		inputBuf += outtraj
		f_st_BIRLcode.write("\n")
		f_ac_BIRLcode.write("\n")

	return inputBuf

def formatPrint_simulated_trajectories(num_trajs,sortingMDP,opt_policy,initial):

	for i in range(0,num_trajs):

		outtraj = ""
		conv_traj = mdp.simulation.simulate(sortingMDP,opt_policy,initial,60) 

		s = None
		act_str = None

		for sap in conv_traj: 
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
					
				elif sap[1].__class__.__name__ == "Pickpip":
					act_str = "Pickpip"
					test_act = Pickpip()
					 
				elif sap[1].__class__.__name__ == "PlaceInBinpip":
					act_str = "PlaceInBinpip"
					test_act = PlaceInBinpip()

				else:
					act_str = "ActionInvalid"
					act_hash = "Null"

				outtraj += act_str

			else:
				outtraj += "None"
			outtraj += ":1;\n"
		
		outtraj += "ENDTRAJ"
		print outtraj

	return

import re
def parseStore_learnedPolicies(buf,List_learnedPolicies):
	# find the strings between \nBEGPOLICY\n and \nENDPOLICY\n
	# parse policy 
	# append to list

	list_stringPols = re.findall('BEGPOLICY\n(.[\s\S]+?)ENDPOLICY', buf)
	print("found "+str(len(list_stringPols))+" policies in output")

	for str_pol in list_stringPols:
		List_learnedPolicies.append(parse_sorting_policy(str_pol))
	
	print("parsed and stored the learned policies")
	return List_learnedPolicies

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
		
		ss = (int(pieces[0]), int(pieces[1]), int(pieces[2]), int(pieces[3]))

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
		elif action == "Pickpip":
			act = Pickpip()
		elif action == "PlaceInBinpip":
			act = PlaceInBinpip()
		else:
			print("Invalid input policy to parse_sorting_policy")
			exit(0)
		
		truePol[ss] = act
	
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

if __name__ == "__main__": 

	#  D code uses 0.05
	p_fail = 0.05
	# sortingMDP = sortingModel(p_fail) 
	# sortingMDP = sortingModel2(p_fail) 
	# sortingMDP = sortingModelbyPSuresh(p_fail) 
	sortingMDP = sortingModelbyPSuresh2(p_fail) 
	# sortingMDP = sortingModelbyPSuresh3(p_fail) 
	
	for state in sortingMDP.S():
		# print 'State:'+str(state)
		s=state
 		ol = None
 		pr = None
 		el = None
 		ls = None

		if s._onion_location == 0:
			ol = 'Onconveyor'
		elif s._onion_location == 1:
			ol = 'Infront'
		elif s._onion_location == 2:
			ol = 'Inbin'
		elif s._onion_location == 4:
			ol = 'Placed'
		else:
			ol = 'Picked/AtHomePose'

		if s._prediction == 0:
			pr = 'bad'
		elif s._prediction == 1:
			pr = 'good'
		else:
			pr = 'unknown'

		if s._EE_location == 0:
			el = 'Onconveyor'
		elif s._EE_location == 1:
			el = 'Infront'
		elif s._EE_location == 2:
			el = 'Inbin'
		else:
			el = 'Picked/AtHomePose'

		if s._listIDs_status == 0:
			ls = 'Empty'
		elif s._listIDs_status == 1:
			ls = 'NotEmpty' 
		else:
			ls = 'Unavailable' 

		# print str((ol,pr,el,ls)) 
		if len(sortingMDP.A(state))==0: 
			print "\n\nempty actions for state"+str(state)
			print str((ol,pr,el,ls)) +" \n\n"
		for act in sortingMDP.A(state):
			# print 'Action: ' + str(act)+',Reward:' + str(sortingMDP.R(state,act))
			pass

	# for enumeration
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

	List_TrueWeights = []
	List_Policies = []

	# sortingReward = sortingReward2(8) 
	# sortingReward = sortingReward4(10) 
	# sortingReward = sortingReward6(11) 
	sortingReward = sortingReward7(11) 

	sortingMDP.reward_function = sortingReward 
	sortingMDP.gamma = 0.99
	

	initial = util.classes.NumMap()
	# ALWAYS START FROm  0,2,0,2
	# s = sortingState(0,2,0,2)	
	# needed for suresh's mdp
	s = sortingState(0,2,0,0)	
	initial[s] = 1.0
	initial = initial.normalize()

	initial2 = util.classes.NumMap()
	s = sortingState(0,2,0,0)	
	initial2[s] = 1.0
	s = sortingState(0,2,0,2)	
	initial2[s] = 1.0
	# for s in sortingMDP.S():
	# 	if s._listIDs_status == 2:
	# 		initial2[s] = 1.0
	initial2 = initial2.normalize()

	# // double [] params_manualTuning_pickinspectplace = [ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12]; 
	# // double [] params_manualTuning_rolling = [0.15082956259426847, -0.075414781297134234, -0.11312217194570136, 
	# // 0.19607843137254902, -0.030165912518853699, 0.0, 0.28355957767722473, -0.15082956259426847]; 
	# // double [] params_neg_pickinpectplace = [ 0.0, 0.10, 0.22, 0.0, -0.12, 0.44, 0.0, -0.12]; 

	################################## 
	############### COMMENTED PART USED FOR TUNING VI TO GET GOOD TRUE POLICIES ####################
	################################## 

	f_pickinspectplace = get_home() + "/catkin_ws/src/navigation_irl/sortingMDP/policy_pickinspectplace.log"
	f = open(f_pickinspectplace, "w")
	# print("pickinspectplace trajs")
	params_manualTuning_pickinspectplace = [ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12]
	params_manualTuning_pickinspectplace_reward4 = [ 0.10, 0.0, 0.0, 0.22, 0.12, 0.44, 0.0, 0.12, 0.0, 0.2]
	params_pickinspectplace_reward6 =[2,1,2,1,0.2,1,0,4,0,0,4]

	params = params_pickinspectplace_reward6 

	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	# needed once to set dim of reward for simulating trajectories
	sortingReward.params = norm_params
	List_TrueWeights.append(norm_params)

	args = [get_home() + "/catkin_ws/devel/bin/solveSortingMDP", ]
	sp = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdin = str(norm_params)
	print("input for pip solving mdp ",stdin)	
	(stdout, stderr) = sp.communicate(stdin)
	opt_policy = parse_sorting_policy(stdout)		
	opt_policy = parseNwrite_sorting_policy(f,stdout)		
	f.close()
	# f = open(f_pickinspectplace, "r")
	# opt_policy = parse_sorting_policy(f.read())
	# f.close()
	List_Policies.append(opt_policy)
	sp.stdin.close()
	sp.stdout.close()
	sp.stderr.close()

	# num_trajs = 2
	# formatPrint_simulated_trajectories(num_trajs,sortingMDP,opt_policy1,initial)

	##################################

	# staying still behavior makes it claim, because claim action has high chance of not changing state
	f_stayStill = get_home() + "/catkin_ws/src/navigation_irl/sortingMDP/policy_stayStill.log"
	f = open(f_stayStill, "w")
	params_stayStill = [ 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] 
	# params_cyclePickPlace = [ 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] 
	params_stayStill_reward4 = [ 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,  0.0, 0.0] 
	params_stayStill_reward6 = [ 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,  0.0, 0.0, 0.0] 

	params = params_stayStill_reward6 

	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	List_TrueWeights.append(norm_params)

	args = [get_home() + "/catkin_ws/devel/bin/solveSortingMDP", ]
	sp = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdin = str(norm_params)	
	(stdout, stderr) = sp.communicate(stdin) 
	opt_policy = parse_sorting_policy(stdout) 
	opt_policy = parseNwrite_sorting_policy(f,stdout)		
	f.close()
	# f = open(f_stayStill, "r")
	# opt_policy = parse_sorting_policy(f.read())
	# f.close()
	List_Policies.append(opt_policy)
	sp.stdin.close()
	sp.stdout.close()
	sp.stderr.close()

	# formatPrint_simulated_trajectories(num_trajs,sortingMDP,opt_policy3,initial)

	##################################

	# print("rollpickplace trajs")
	f_rolling = get_home() + "/catkin_ws/src/navigation_irl/sortingMDP/policy_rolling.log"
	f = open(f_rolling, "w")
	params_manualTuning_rolling = [0.15, -0.07, -0.11, 0.2, -0.03, 0.0, 0.45, -0.15]
	params_manualTuning_rolling_reward4 = [0.0, 0.6, 0.0, 0.95, 0.8, 0.0, 0.9, 0.15, 0.9, 0.4]
	params_rolling_reward6 = [0, 4, 0, 4, 0.2, 0, 8, 0, 8, 4, 0] 

	params = params_rolling_reward6
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	# [0.13509862199405565, -0.067549310997027823, -0.10132396649554175, 0.17562820859227235, 
	# -0.027019724398811135, 0.0, 0.3582815455282356, -0.13509862199405565]
	List_TrueWeights.append(norm_params)

	# print("norm_params rollpickplace ", norm_params)

	args = [get_home() + "/catkin_ws/devel/bin/solveSortingMDP", ]
	sp = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdin = str(norm_params)	
	(stdout, stderr) = sp.communicate(stdin)
	opt_policy = parse_sorting_policy(stdout)		
	opt_policy = parseNwrite_sorting_policy(f,stdout)		
	f.close()
	# f = open(f_rolling, "r")
	# opt_policy = parse_sorting_policy(f.read())
	# f.close()
	List_Policies.append(opt_policy)
	sp.stdin.close()
	sp.stdout.close()
	sp.stderr.close()

	# formatPrint_simulated_trajectories(num_trajs,sortingMDP,opt_policy2,initial)

	##################################

	# print("pick place pick trajs")
	f_pickPlacePick = get_home() + "/catkin_ws/src/navigation_irl/sortingMDP/policy_pickPlacePick.log"
	# f = open(f_pickPlacePick, "w")

	# incentivize placing features and incentivize picking-placed feature
	params_manualTuning_pickPlacePick = [0.5, 0.5, 0, 0, 0, 0, 0, 1]
	params = params_manualTuning_pickPlacePick
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	# List_TrueWeights.append(norm_params)

	# args = [get_home() + "/catkin_ws/devel/bin/solveSortingMDP", ]
	# sp = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	# stdin = str(norm_params)
	# (stdout, stderr) = sp.communicate(stdin) 
	# opt_policy = parse_sorting_policy(stdout) 
	# formatPrint_simulated_trajectories(5,sortingMDP,opt_policy,initial) 
	# opt_policy = parseNwrite_sorting_policy(f,stdout)	
	# f.close()

	f = open(f_pickPlacePick, "r") 
	opt_policy = parse_sorting_policy(f.read()) 
	f.close() 
	# List_Policies.append(opt_policy)

	print("snity check: how similar are  p-i-p and p-i-p to each other? ") 
	print(computeLBA(f_pickinspectplace,sortingMDP,List_Policies[0]))
	print("how similar are  p-i-p and r-p-p to each other? ") 
	print(computeLBA(f_pickinspectplace,sortingMDP,List_Policies[2]))
	print("how similar are  p-i-p and stayStill to each other? ") 
	print(computeLBA(f_pickinspectplace,sortingMDP,List_Policies[1]))

	# exit(0)

	###########################################
	###########################################
	
	input_dpmMEIRL = "sorting\n"
	input_dpmMEIRL += "DPMMEIRL\n"
	# max_d, maximum clusters
	input_dpmMEIRL += "4\n" # "3\n"
	# maximum number of iterations
	input_dpmMEIRL += "2\n" # "2\n" # likelihood stayed same after 4  iterations. 7\n" 
	# descent threshold
	input_dpmMEIRL += "0.000005\n" # "0.00001\n" # "0.00005\n" 
	# descent_duration_thresh_secs
	input_dpmMEIRL += "90\n" # "240\n" # "180\n"
	# gradient_descent_step_size
	input_dpmMEIRL += "0.00001\n" # "0.0001\n" # 
	# restart attempts for descent 
	input_dpmMEIRL += "3\n" 
	# value iteration thresohld
	input_dpmMEIRL += "0.2\n" 
	# vi_duration_thresh_secs
	input_dpmMEIRL += "45\n" 
	# Ephi thresohld
	input_dpmMEIRL += "0.1\n" # 

	fixed_part_stdin = input_dpmMEIRL

	true_assignments4 = [0, 0, \
						 2, 2]  

	# true_assignments16 = [ 0, 0, 0, 0, 0, 0, 0, 0,\
						 # 2, 2, 2, 2, 2, 2, 2, 2]  

	true_assignments64 = [0]*32+[2]*32   

	true_assignments128 = [0]*64+[2]*64   

	true_assignments512 = [0]*256+[2]*256  

	# case with 3 clusters 
	# couldn't generate 3rd behavior 
	true_assignments3clus6 = [0]*2+[1]*2+[2]*2
	true_assignments3clus96 = [0]*32+[1]*32+[2]*32

	traj_size = 50

	print("writing result of call to dpmMEIRL to file Downloads/MTIRLdata_recbypython.txt") 
	f_rec = open(get_home()+'/Downloads/MTIRLdata_recbypython.txt','a') 

	csvstring = ""

	list_datainput = [true_assignments4,
					true_assignments64,
					true_assignments128,
					true_assignments512]
	# list_datainput = [true_assignments3clus6]
	# list_datainput = [true_assignments4]

	for current_data in list_datainput:

		input_dpmMEIRL = fixed_part_stdin
		true_assignments = current_data 
		for i in range(0,len(true_assignments)): 
			ind = true_assignments[i] 
			# input_dpmMEIRL = formatWrite_simulated_trajectories(input_dpmMEIRL,1,sortingMDP,\
			# 	List_Policies[ind],initialsPsuresh[ind],traj_size) 
			input_dpmMEIRL = formatWrite_simulated_trajectories(input_dpmMEIRL,1,sortingMDP,\
				List_Policies[ind],initial,traj_size) 

		input_dpmMEIRL += "ENDDEMO\n" 
		# print(input_dpmMEIRL)

		input_dpmMEIRL += str(true_assignments)+"\n" 

		# IMPORTANT 
		# As the indices used for computing EVD are indicecs of original list of trueWeights, all the trueWeights should be given as inputs.
		input_dpmMEIRL += str(len(List_TrueWeights))+"\n"
		for i in range(0, len(List_TrueWeights)):

			input_dpmMEIRL += str(List_TrueWeights[i])+"\n"
		
		f = open(get_home() + "/catkin_ws/src/navigation_irl/data_MEMTIRL.log", "w")
		# f = open(get_home() + "/catkin_ws/src/navigation_irl/sortingMDP/MTIRL_data_test.log", "w")
		# UNCOMMENT
		f.write(input_dpmMEIRL)
		f.close()
		
		# print("calling dpmMEIRL") 

		# exit(0) 

		args = [get_home() + "/catkin_ws/devel/bin/dpmMEIRL", ]
		sp = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
		stdin = input_dpmMEIRL
		# UNCOMMENT
		(stdout, stderr) = sp.communicate(stdin)
		sp.stdin.close()
		sp.stdout.close()
		sp.stderr.close()

		print(stdout)
		# exit(0)

		# for current input data size, find the output values
		# append the values to string which can later be added to 
		# external csv file

		list_stringEVD2_4 = re.findall('avg_EVD2_4\n(.[\s\S]+?)endavg_EVD2_4', stdout)
		print("found list_stringEVD2_4 ",list_stringEVD2_4)
		list_stringEVD2_16 = re.findall('avg_EVD2_16\n(.[\s\S]+?)endavg_EVD2_16', stdout)
		print("found list_stringEVD2_16 ",list_stringEVD2_16)
		list_stringEVD2_64 = re.findall('avg_EVD2_64\n(.[\s\S]+?)endavg_EVD2_64', stdout)
		print("found list_stringEVD2_64 ",list_stringEVD2_64)
		list_stringEVD2_128 = re.findall('avg_EVD2_128\n(.[\s\S]+?)endavg_EVD2_128', stdout)
		print("found list_stringEVD2_128 ",list_stringEVD2_128)
		list_stringEVD2_512 = re.findall('avg_EVD2_512\n(.[\s\S]+?)endavg_EVD2_512', stdout)
		print("found list_stringEVD2_512 ",list_stringEVD2_512)

		list_stringEVD3_4 = re.findall('avg_EVD3_4\n(.[\s\S]+?)endavg_EVD3_4', stdout)
		print("found list_stringEVD3_4 ",list_stringEVD3_4)
		list_stringEVD3_16 = re.findall('avg_EVD3_16\n(.[\s\S]+?)endavg_EVD3_16', stdout)
		print("found list_stringEVD3_16 ",list_stringEVD3_16)
		list_stringEVD3_64 = re.findall('avg_EVD3_64\n(.[\s\S]+?)endavg_EVD3_64', stdout)
		print("found list_stringEVD3_64 ",list_stringEVD3_64)
		list_stringEVD3_128 = re.findall('avg_EVD3_128\n(.[\s\S]+?)endavg_EVD3_128', stdout)
		print("found list_stringEVD3_128 ",list_stringEVD3_128)
		list_stringEVD3_512 = re.findall('avg_EVD3_512\n(.[\s\S]+?)endavg_EVD3_512', stdout)
		print("found list_stringEVD3_512 ",list_stringEVD3_512)

		list_stringfinalavgEVD_4 = re.findall('finalavgEVD_4\n(.[\s\S]+?)endfinalavgEVD_4', stdout)
		print("found list_stringfinalavgEVD_4 ",list_stringfinalavgEVD_4)
		list_stringfinalavgEVD_16 = re.findall('finalavgEVD_16\n(.[\s\S]+?)endfinalavgEVD_16', stdout)
		print("found list_stringfinalavgEVD_16 ",list_stringfinalavgEVD_16)
		list_stringfinalavgEVD_64 = re.findall('finalavgEVD_64\n(.[\s\S]+?)endfinalavgEVD_64', stdout)
		print("found list_stringfinalavgEVD_64 ",list_stringfinalavgEVD_64)
		list_stringfinalavgEVD_128 = re.findall('finalavgEVD_128\n(.[\s\S]+?)endfinalavgEVD_128', stdout)
		print("found list_stringfinalavgEVD_128 ",list_stringfinalavgEVD_128)
		list_stringfinalavgEVD_512 = re.findall('finalavgEVD_512\n(.[\s\S]+?)endfinalavgEVD_512', stdout)
		print("found list_stringfinalavgEVD_512 ",list_stringfinalavgEVD_512)

		list_stringassignmentAccuracy_4 = re.findall('assignmentAccuracy_4\n(.[\s\S]+?)endassignmentAccuracy_4', stdout)
		print("found list_stringassignmentAccuracy_4 ",list_stringassignmentAccuracy_4)
		list_stringassignmentAccuracy_16 = re.findall('assignmentAccuracy_16\n(.[\s\S]+?)endassignmentAccuracy_16', stdout)
		print("found list_stringassignmentAccuracy_16 ",list_stringassignmentAccuracy_16)
		list_stringassignmentAccuracy_64 = re.findall('assignmentAccuracy_64\n(.[\s\S]+?)endassignmentAccuracy_64', stdout)
		print("found list_stringassignmentAccuracy_64 ",list_stringassignmentAccuracy_64)
		list_stringassignmentAccuracy_128 = re.findall('assignmentAccuracy_128\n(.[\s\S]+?)endassignmentAccuracy_128', stdout)
		print("found list_stringassignmentAccuracy_128 ",list_stringassignmentAccuracy_128)
		list_stringassignmentAccuracy_512 = re.findall('assignmentAccuracy_512\n(.[\s\S]+?)endassignmentAccuracy_512', stdout)
		print("found list_stringassignmentAccuracy_512 ",list_stringassignmentAccuracy_512)

		# append depending on case
		# if case had 3 iterations, moving further is not needed
		data_found = False
		if len(list_stringEVD3_4) != 0:
			csvstring += str(list_stringEVD3_4[0])+","+\
			str(list_stringfinalavgEVD_4[0])+","+\
			str(list_stringassignmentAccuracy_4[0])+","
			data_found = True
		elif len(list_stringEVD3_16) != 0:
			csvstring += str(list_stringEVD3_16[0])+","+\
			str(list_stringfinalavgEVD_16[0])+","+\
			str(list_stringassignmentAccuracy_16[0])+","
			data_found = True
		elif len(list_stringEVD3_64) != 0:
			csvstring += str(list_stringEVD3_64[0])+","+\
			str(list_stringfinalavgEVD_64[0])+","+\
			str(list_stringassignmentAccuracy_64[0])+","
			data_found = True
		elif len(list_stringEVD3_128) != 0:
			csvstring += str(list_stringEVD3_128[0])+","+\
			str(list_stringfinalavgEVD_128[0])+","+\
			str(list_stringassignmentAccuracy_128[0])+","
			data_found = True
		elif len(list_stringEVD3_512) != 0:
			csvstring += str(list_stringEVD3_512[0])+","+\
			str(list_stringfinalavgEVD_512[0])+","+\
			str(list_stringassignmentAccuracy_512[0])+","
			data_found = True
		else:
			pass

		if data_found == False:
			if len(list_stringEVD2_4) != 0:
				csvstring += str(list_stringEVD2_4[0])+","+\
				str(list_stringfinalavgEVD_4[0])+","+\
				str(list_stringassignmentAccuracy_4[0])+","
			elif len(list_stringEVD2_16) != 0:
				csvstring += str(list_stringEVD2_16[0])+","+\
				str(list_stringfinalavgEVD_16[0])+","+\
				str(list_stringassignmentAccuracy_16[0])+","
			elif len(list_stringEVD2_64) != 0:
				csvstring += str(list_stringEVD2_64[0])+","+\
				str(list_stringfinalavgEVD_64[0])+","+\
				str(list_stringassignmentAccuracy_64[0])+","
			elif len(list_stringEVD2_128) != 0:
				csvstring += str(list_stringEVD2_128[0])+","+\
				str(list_stringfinalavgEVD_128[0])+","+\
				str(list_stringassignmentAccuracy_128[0])+","
			elif len(list_stringEVD2_512) != 0:
				csvstring += str(list_stringEVD2_512[0])+","+\
				str(list_stringfinalavgEVD_512[0])+","+\
				str(list_stringassignmentAccuracy_512[0])+","
			else:
				pass


	f_rec.write(csvstring+"\n")
	f_rec.close()

	exit(0)

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
	
	print(sortingReward._dim)	
	print(np.unique(true_assignments))
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

	exit(0)

	List_learnedPolicies = []
	# UNCOMMENT
	List_learnedPolicies = parseStore_learnedPolicies(stdout,List_learnedPolicies)

	print("cluster id, lba1, lba2, lba3, lba4")
	# List_filesTruePols = [f_pickinspectplace,f_stayStill,f_rolling,f_pickPlacePick] 
	List_filesTruePols = [f_pickinspectplace,f_stayStill,f_rolling,f_stayStill] 
	list_for_avgLBA = []
	mean_offset = 0.12
	q = 0 
	for learnedPol in List_learnedPolicies: 
		lba1 = computeLBA(List_filesTruePols[0],sortingMDP,learnedPol)
		lba2 = computeLBA(List_filesTruePols[1],sortingMDP,learnedPol)
		lba3 = computeLBA(List_filesTruePols[2],sortingMDP,learnedPol)
		lba4 = computeLBA(List_filesTruePols[3],sortingMDP,learnedPol)
		print((str(q),str(lba1),str(lba2),str(lba3),str(lba4)))
		list_for_avgLBA += [lba1,lba2,lba3,lba4]
		# print("lbas for cluster "+str(q)+":"+str(list_for_avgLBA))
		q += 1 

	# assign lcusters based on lba if trajectory matchign is reflected in lba
