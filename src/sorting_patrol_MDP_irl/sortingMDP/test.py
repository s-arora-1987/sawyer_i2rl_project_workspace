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
	params_DPMBIRL1 = [0.8158771449, -0.7188695585, -0.587897962, 0.8681293273, 
				-0.4845075111, 0.5119944004, -0.8684166196, -0.8715143739]

	params = params_pickinspect
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

	# for enumeration
	dummy_states = []
	for s in sortingMDP.S():
		dummy_states.append(s)
	dummy_states.append(sortingState(-1,-1,-1,-1))

	dict_stateEnum = {}
	ind = 0
	for s in dummy_states:
		ind = ind +1
		dict_stateEnum[ind] = s

	acts = [InspectAfterPicking(),PlaceOnConveyor(),PlaceInBin(),\
	Pick(),ClaimNewOnion(),InspectWithoutPicking(),ClaimNextInList()] 
	dict_actEnum = {}
	ind = 0
	for a in acts:
		ind = ind +1
		dict_actEnum[ind] = a

	# test_act = InspectAfterPicking()
	# act_hash = str(test_act.__hash__())
	# print "InspectAfterPicking() hash "+str(test_ac.__hash__())

	f = open(get_home() + "/patrolstudy/toupload/MTIRL_data_test.log", "w")
	f.write("")
	f.close()
	f = open(get_home() + "/patrolstudy/toupload/MTIRL_data_test.log", "a")
	f.write("DPMMEIRL\n")
	f.close()

	f_st = open(get_home() + "/patrolstudy/toupload/traj_states.log", "w")
	f_st.write("")
	f_st.close()
	f_ac = open(get_home() + "/patrolstudy/toupload/traj_actions.log", "w")
	f_ac.write("")
	f_ac.close()

	f = open(get_home() + "/patrolstudy/toupload/MTIRL_data_test.log", "a")
	f_st = open(get_home() + "/patrolstudy/toupload/traj_states.log", "a")
	f_ac = open(get_home() + "/patrolstudy/toupload/traj_actions.log", "a")


	# for s in sortingMDP.S():
	for i in range(0,10):

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

				inds = dict_stateEnum.keys()[dict_stateEnum.values().index(s)]
				# f_st.write(str(inds)+",")
				# f_st.write("["+str(s._onion_location)+"\t"\
				# +str(s._prediction)+"\t"+\
				# str(s._EE_location)+"\t"+\
				# str(s._listIDs_status)+"],")

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
				inda = dict_actEnum.keys()[dict_actEnum.values().index(test_act)]
				# f_ac.write(str(inda)+",")


			else:
				outtraj += "None"
			outtraj += ":1;\n"
		# outtraj += "ENDTRAJ\n"
		# f_st.write("\n")
		# f_ac.write("\n")
		
		# print outtraj
		# f.write(outtraj)

	outtraj = "ENDTRAJ\n"
	# f.write(outtraj)

	f.close()

	f_st.close()
	f_ac.close()

	params_rolling = [1,-0.5,-0.75,1.3,0,-0.2,0,0,0,1.88,0,0,0,-1.0] 

	params_rolling = [1,-0.5,-0.75,1.3,-0.2,0,1.88,-1.0] 
	params_DPMBIRL2 = [0.1508106881, -0.7955924241, -0.5515305082, 0.6063774534,
			-0.6112858122, 0.1987572494, 0.9614906833, -0.2792128831]
			
	params = params_rolling
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
		# if s._onion_location==0 and s._prediction ==2 and s._listIDs_status ==2:
		# if s._onion_location==2 and s._prediction ==2 and s._EE_location == 1 and s._listIDs_status ==2:
		# 	initial2[s] = 1.0
		# if s._listIDs_status == 1:
		# 	initial2[s] = 1.0
		if s._listIDs_status == 2:
			initial2[s] = 1.0
		# initial2[s] = 1.0

	initial2 = initial2.normalize()

	policy2Dict = {}
	fopt_policy2 = open(get_home() + "/patrolstudy/toupload/expected_policy2.log", "r")
	import string 
	for line in fopt_policy2: 
		pi_stact = string.split(line.strip(),":") 
		# print pi_stact
		if pi_stact[0] == "pi":
			# print "line with st act"
			str_state = pi_stact[1]
			str_state_lst = string.split(str_state.strip(),",") 
			# print "str_state_lst:"+str(str_state_lst)
			# print str((int(str_state_lst[0][1]),int(str_state_lst[1][1]),\
			# 	int(str_state_lst[2][1]),int(str_state_lst[3][1])))
			state = sortingState(int(str_state_lst[0][1]),int(str_state_lst[1][1]),\
				int(str_state_lst[2][1]),int(str_state_lst[3][1]))
			# print state

			str_act = pi_stact[2]

			if str_act == "Pick":
				action = Pick()
			elif str_act == "ClaimNextInList":
				action = ClaimNextInList()
			elif str_act == "ClaimNewOnion":
				action = ClaimNewOnion()
			elif str_act == "PlaceInBin":
				action = PlaceInBin()
			elif str_act == "PlaceOnConveyor":
				action = PlaceOnConveyor()
			elif str_act == "InspectAfterPicking":
				action = InspectAfterPicking()
			elif str_act == "InspectWithoutPicking":
				action = InspectWithoutPicking()

			# print action
			policy2Dict[state] = action

	fopt_policy2.close()
	# print str(policy2Dict)
	# exit(0)

	# opt_policy2 = mdp.agent.MapAgent(policy2Dict)
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

	pol_ind = [1,2,2,2,2,1,1,1,2,1,1,1,2,2,2,2,2,2,1,2,2,2,2,1,1,2,2,2,2,2]
	pol_ind = [1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2]
	pol_ind = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2]
	pol_ind = []
	pol_ind.append([1,1,2,2])
	pol_ind.append([1,1,1,1,2,2,2,2])
	# pol_ind[1] = [1,1,1,1,2,2,2,2]
	pol_ind.append([1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2])
	# pol_ind[2] = [1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2]
	pol_ind.append([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2])
	# pol_ind[3] = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2]
	pol_ind.append([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,\
			   2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2])
	# pol_ind[4] = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,\
	# 		   2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2]
	pol_ind.append([1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,\
			   2,2,2,2,2,2,2,2])

	for i in range(0,6):

		f = open(get_home() + "/patrolstudy/toupload/MTIRL_data_test"+str(i)+".log", "w")
		f.write("")
		f.close()
		f = open(get_home() + "/patrolstudy/toupload/MTIRL_data_test"+str(i)+".log", "a")
		f.write("DPMMEIRL\n")
		f.close()

		f_st = open(get_home() + "/patrolstudy/toupload/traj_states"+str(i)+".log", "w")
		f_st.write("")
		f_st.close()
		f_ac = open(get_home() + "/patrolstudy/toupload/traj_actions"+str(i)+".log", "w")
		f_ac.write("")
		f_ac.close()
		
		f = open(get_home() + "/patrolstudy/toupload/MTIRL_data_test"+str(i)+".log", "a")
		f_st = open(get_home() + "/patrolstudy/toupload/traj_states"+str(i)+".log", "a")
		f_ac = open(get_home() + "/patrolstudy/toupload/traj_actions"+str(i)+".log", "a")

		# for s in sortingMDP.S():
		for j in range(0,len(pol_ind[i])):

			outtraj = ""
			if pol_ind[i][j] == 1:
				conv_traj = mdp.simulation.simulate(sortingMDP,opt_policy1,initial,30)
			else:
				conv_traj = mdp.simulation.simulate(sortingMDP,opt_policy2,initial2,30)

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

					inds = dict_stateEnum.keys()[dict_stateEnum.values().index(s)]
					f_st.write(str(inds)+",")
					# f_st.write("["+str(s._onion_location)+"\t"\
					# +str(s._prediction)+"\t"+\
					# str(s._EE_location)+"\t"+\
					# str(s._listIDs_status)+"],")

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
					inda = dict_actEnum.keys()[dict_actEnum.values().index(test_act)]
					f_ac.write(str(inda)+",")

				else:
					outtraj += "None"
				outtraj += ":1;\n"
			outtraj += "ENDTRAJ\n"
			f_st.write("\n")
			f_ac.write("\n")
			
			print outtraj
			
			f.write(outtraj)

		# outtraj = "ENDTRAJ\n"
		# f.write(outtraj)
		# f.close()

		f_st.close()
		f_ac.close()

		# f = open(get_home() + "/patrolstudy/toupload/MTIRL_data_test.log", "a")
		f.write("ENDDEMO\n")
		f.write(str(pol_ind[i])+"\n")
		f.close()



	f = open(get_home() + "/patrolstudy/toupload/MTIRL_data_test.log", "a")
	# f.write(str(list_rewards)+"\n")
	f.close()

	f_TM = open(get_home() + "/patrolstudy/toupload/transition_matrix.txt", "w")
	f_TM.write("")
	f_TM.close()

	tuple_res = sortingMDP.generate_matrix(dict_stateEnum,dict_actEnum)
	dict_tr = tuple_res[0]
	# print dict_tr
	f_TM = open(get_home() + "/patrolstudy/toupload/transition_matrix.txt", "a")

	# f_TM.write("order of visiting states in first dimension\n")
	# f_TM.write(str(tuple_res[2])+"\n")
	# f_TM.write("order of visiting actions in second dimension\n")
	# f_TM.write(str(tuple_res[3])+"\n")
	# f_TM.write("order of visiting next states in third dimension\n")
	# f_TM.write(str(tuple_res[4])+"\n")

	# tuple_res = (0,0,5,5)
	# dict_stateEnum = tuple_res[3]
	# dict_actEnum = tuple_res[2]

	for ind1 in range(1,len(dict_actEnum)+1):
		acArray2d = np.empty((len(dict_stateEnum),len(dict_stateEnum)))
		# print "acArray2d:"+str(acArray2d)
		for ind2 in range(1,len(dict_stateEnum)+1):
			#s
			for ind3 in range(1,len(dict_stateEnum)+1):
				#s'
				acArray2d[ind3-1][ind2-1] = dict_tr[ind1][ind3][ind2]

			# print "acArray2d:"+str(acArray2d)
			# exit(0)

		for ind3 in range(1,len(dict_stateEnum)+1):
			for ind2 in range(1,len(dict_stateEnum)+1):
				f_TM.write(str(acArray2d[ind3-1][ind2-1])+",")
			f_TM.write("\n")
		f_TM.write("\n")

	f_TM.close()

	f_Phis = open(get_home() + "/patrolstudy/toupload/features_matrix.txt", "w")
	f_Phis.write("")
	f_Phis.close()
	f_Phis = open(get_home() + "/patrolstudy/toupload/features_matrix.txt", "a")
    #for each a, create |S|xK matrix of feature values
	# acArray2d = np.empty((len(dict_stateEnum),sortingReward2.dim()))
	for inda in range(1,len(dict_actEnum)+1):
		a = dict_actEnum[inda]
		for inds in range(1,len(dict_stateEnum)+1):
			s = dict_stateEnum[inds]
			arraysPhis = sortingReward2.features(s,a)
			for indk in range(1,len(arraysPhis)+1):
				f_Phis.write(str(arraysPhis[indk-1])+",")
				# acArray2d[inds-1][indk-1] = arraysPhis[indk-1]
			f_Phis.write("\n")
		f_Phis.write("\n")
	f_Phis.close()          	
	
	f_wts = open(get_home() + "/patrolstudy/toupload/weights_experts.log", "w")
	f_wts.write("")
	f_wts.close()

	f_wts = open(get_home() + "/patrolstudy/toupload/weights_experts.log", "a")
	for i in range(0,wts_experts_array.shape[0]):
		for e in range(0,wts_experts_array.shape[1]):
			f_wts.write(str(wts_experts_array[i][e])+",")
		f_wts.write("\n")
	f_wts.close()

	# print sortingMDP.generate_matrix()[1]


	# exit(0)


'''
onion_location: 
on the conveyor, or 0
in front of eye, or 1
in bin or 2
at home after begin picked or 3 (in superficial inspection, onion is picked and placed)
prediction:
1 good
0 bad
2 unknown before inspection
EE_location:
conv
inFront
bin
at home
''' 
