# -*- coding: utf-8 -*-
from model import sortingModel,InspectAfterPicking,PlaceOnConveyor,PlaceInBin,\
Pick,ClaimNewOnion,InspectWithoutPicking,ClaimNextInList,sortingState 
from reward import sortingReward1
from reward import sortingReward2
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

	# sortingReward = sortingReward1(14) 
	sortingReward = sortingReward2(8) 
	dim_wts = 8
	wts_experts_array = np.empty((dim_wts,2))

	# params_pickinspect = [1,-1,-1,1,0,-0.2,1,0,0,-1.5,0,0,0,-1.0] 
	params_pickinspect = [1,-1,-1,1,-0.2,1,-1.5,-1.0] 

	params_pickinspect_tuningfromFE = [ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12] 
	params_DPMBIRL1 = [0.1699966423, -0.5516277664, -0.6421129529, 0.9331813738,
			-0.6127648891, 0.4726631827, -0.7684372134, -0.297890299]
	params_EMMLIRL1 = [0.4415719702, -0.4459898567, -0.4551824526, 0.2175143501, 
			-0.4039548401, 0.2734986896, 0.7888035685, -0.5319394064]			

	params = params_EMMLIRL1
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	print "norm_params1:"+str(norm_params)

	for i in range(0,dim_wts):
		wts_experts_array[i][0] = norm_params[i]

	list_rewards.append(norm_params)
	sortingReward.params=norm_params
	sortingMDP.reward_function = sortingReward 
	sortingMDP.gamma = 0.998 

	## Print out world information
	# print sortingMDP.info()
	# print sortingReward.info()
	# print 'States:' + str( [str(state) for state in sortingMDP.S()] )

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

	initial = util.classes.NumMap()
	for s in sortingMDP.S():
		# if s._onion_location==0 and s._prediction ==2 and s._listIDs_status ==2:
		# if s._onion_location==0 and s._prediction ==2 and s._EE_location == 3 and s._listIDs_status ==2:
		# 	initial[s] = 1.0
		if s._listIDs_status == 2 or s._prediction ==1:
			initial[s] = 1.0
		# initial[s] = 1.0
	
	initial = initial.normalize()
	# print"created initial"
	opt_policy1 = mdp.solvers.ValueIteration(0.55).solve(sortingMDP)
	# opt_policy = mdp.solvers.ValueIteration2(0.55).solve(sortingMDP)

	# print "number of states in policy keys - "+str(len(opt_policy._policy.keys()))
	# print "number of states - "+str(len(sortingMDP.S()))

 	# for (s, a) in opt_policy._policy.iteritems():
	# print str( [str((s,a)) for (s, a) in opt_policy._policy.iteritems()])
	# print 'Policy: '

	for s in sortingMDP.S():
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
		# print '\t pi({}) = {}'.format(s, opt_policy._policy[s])

	# print "reward weights:"+str(sortingReward.params)
	# print "L1 norm weights:"+str(np.linalg.norm(sortingReward.params, ord=1))
	# print mdp.simulation.simulate(sortingMDP,opt_policy,initial,20)

	# for s in sortingMDP.S():
	for i in range(0,5):

		outtraj = ""
		conv_traj = mdp.simulation.simulate(sortingMDP,opt_policy1,initial,30) 
		# conv_traj = mdp.simulation.simulate_givenstart(sortingMDP,opt_policy,s,10)

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
				# f_ac.write(act_str+",")


			else:
				outtraj += "None"
			outtraj += ":1;\n"
		# outtraj += "ENDTRAJ\n"
		# f_st.write("\n")
		# f_ac.write("\n")
		
		print outtraj
		# f.write(outtraj)

	outtraj = "ENDTRAJ\n"
	# f.write(outtraj)

	params_rolling = [1,-0.5,-0.75,1.3,0,-0.2,0,0,0,1.88,0,0,0,-1.0] 

	params_rolling_reward2 = [1,-0.5,-0.75,1.3,-0.2,0,1.88,-1.0] 
	params_DPMBIRL2 = [0.3284201432, -0.6143731731, -0.2706698797, 0.6186866198,
				-0.7352166586, 0.2227114516, 0.9721774592, -0.2248665648]
	params_EMMLIRL2 = [0.4411406227, -0.8878774817, -0.9889039716, 0.9121858826,
				-0.9884187832, 0.5033895729, -0.6637235171, -1]

	params = params_EMMLIRL2
	# params_rolling_prashant = [1,-0.5,1.3,-0.75,-0.2,0,1.88,-1.0] 
	# params = params_rolling_prashant

	# params_irloutput_rolling = [6.8854e-09, 1.87884e-09, 3.5755e-09, 0.194948, 2.67527e-34, 0.0286509, 0.776401, 2.0002e-10]
	# params = params_irloutput_rolling

	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	print "norm_params2:"+str(norm_params)

	for i in range(0,dim_wts):
		wts_experts_array[i][1] = norm_params[i]

	list_rewards.append(norm_params)
	sortingReward.params=norm_params

	initial2 = util.classes.NumMap()
	for s in sortingMDP.S():
		if s._listIDs_status == 2:
			initial2[s] = 1.0

	initial2 = initial2.normalize()

	opt_policy2 = mdp.solvers.ValueIteration(0.55).solve(sortingMDP)

	for s in sortingMDP.S():
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
		# print '\t pi({}) = {}'.format(s, opt_policy._policy[s])

	# for s in sortingMDP.S():
	for j in range(0,5):

		outtraj = ""
		conv_traj = mdp.simulation.simulate(sortingMDP,opt_policy2,initial,30)

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
				# f_ac.write(act_str+",")

			else:
				outtraj += "None"
			outtraj += ":1;\n"
		outtraj += "ENDTRAJ\n"
		
		print outtraj

