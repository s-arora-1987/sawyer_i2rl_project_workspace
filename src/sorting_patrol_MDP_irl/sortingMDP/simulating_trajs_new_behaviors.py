# -*- coding: utf-8 -*- 
from model import sortingModel,InspectAfterPicking,PlaceOnConveyor,PlaceInBin, \
Pick,ClaimNewOnion,InspectWithoutPicking,ClaimNextInList,sortingState 
from reward import sortingReward1
from reward import sortingReward2
from reward import sortingReward3
import mdp.solvers
import util.classes
import mdp.simulation
import numpy as np
import os

home = os.environ['HOME'] 

def get_home():
	global home
	return home

if __name__ == "__main__": 

	sortingMDP = sortingModel() 
	list_rewards = []

	# generating negative behavior needs a wayt oavoid pick place pick cycle and sortingreward2 does not have that
	sortingReward = sortingReward3(9) 
	dim_wts = 9
	wts_experts_array = np.empty((dim_wts,2))

	# needed last one high to avoid cycles 
	params_negpickinspect = [ 0, 0.10, 0.22, 0, -0.12, 0.44, 0.0, 0.0, -0.25] 
	params = params_negpickinspect

	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	print "norm_params3:"+str(norm_params)

	for i in range(0,dim_wts):
		wts_experts_array[i][0] = norm_params[i]

	list_rewards.append(norm_params)
	sortingReward.params=norm_params
	sortingMDP.reward_function = sortingReward 
	sortingMDP.gamma = 0.998 

	initial = util.classes.NumMap()
	for s in sortingMDP.S():
		# we don't want rolling list because it is not needed for first two behaviors
		if s._listIDs_status == 2:
			initial[s] = 1.0
	
	initial = initial.normalize()

	opt_policy1 = mdp.solvers.ValueIteration(0.55).solve(sortingMDP)
	print"created opt_policy"

	# print "reward weights:"+str(sortingReward.params)
	# print "L1 norm weights:"+str(np.linalg.norm(sortingReward.params, ord=1))
	# print mdp.simulation.simulate(sortingMDP,opt_policy,initial,20)

	params_staystill = [ 0, 0.0, 0.0, 0, 1, 0.0, 0.0, 0.0, 0.0] 
	params = params_staystill

	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	print "norm_params2:"+str(norm_params)

	for i in range(0,dim_wts):
		wts_experts_array[i][1] = norm_params[i]

	list_rewards.append(norm_params)
	sortingReward.params=norm_params

	opt_policy2 = mdp.solvers.ValueIteration(0.55).solve(sortingMDP)

	pol_ind = [1,2,2,2,2,1,1,1,2,1,1,1,2,2,2,2,2,2,1,2,2,2,2,1,1,2,2,2,2,2]

	f = open(get_home() + "/catkin_ws/src/sorting_patrol_MDP_irl/sortingMDP/MTIRL_data_test.log", "w")
	f.write("")
	f.close()
	f = open(get_home() + "/catkin_ws/src/sorting_patrol_MDP_irl/sortingMDP/MTIRL_data_test.log", "a")
	f.write("DPMMEIRL\n")
	f.close()
	f = open(get_home() + "/catkin_ws/src/sorting_patrol_MDP_irl/sortingMDP/MTIRL_data_test.log", "a")

	for j in range(0,len(pol_ind)):

		outtraj = ""
		if pol_ind[j] == 1:
			print "opt_policy1"
			conv_traj = mdp.simulation.simulate(sortingMDP,opt_policy1,initial,30)
		else:
			print "opt_policy2, passing"
			# continue  
			conv_traj = mdp.simulation.simulate(sortingMDP,opt_policy2,initial,30)

		# conv_traj = mdp.simulation.simulate_givenstart(sortingMDP,opt_policy,s,30)

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
					
				else:
					act_str = "ActionInvalid"
					act_hash = "Null"

				outtraj += act_str

			else:
				outtraj += "None"
			outtraj += ":1;\n"

		outtraj += "ENDTRAJ\n"
		print outtraj
		
		f.write(outtraj)

	# f = open(get_home() + "/catkin_ws/src/sorting_patrol_MDP_irl/sortingMDP/MTIRL_data_test.log", "a")
	f.write("ENDDEMO\n")
	f.write(str(pol_ind[i])+"\n")
	f.close()

