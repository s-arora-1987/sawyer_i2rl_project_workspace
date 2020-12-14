import mdp.agent
import util.classes
from mdp.simulation import *
import numpy as np
import os
import subprocess
import mdp.solvers
from patrol.model import *
import patrol.reward
import numpy as np
home = os.environ['HOME']
def get_home():
	global home
	return home


def parse_patrolling_policy(buf):
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
		
		ps = patrol.model.PatrolState(np.array([int(pieces[0]), int(pieces[1]), int(pieces[2])] ) ) 
 
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
 
		p[ps] = a 

	from mdp.agent import MapAgent

	return MapAgent(p) 


def formatWrite_simulated_Patrolling_trajectories(inputBuf,num_trajs,model,policy,initial):

	for i in range(0,num_trajs):

		outtraj = ""
		traj_list = sample_traj(model, 120, initial, policy) 
		for sasp in traj_list: 
			if (sasp is not None): 
				(s,a,s_p) = sasp
				s = s.location[0:3]
				
				outtraj += "["+str(s[0])+","+str(s[1])+","+str(s[2])+"]:"

				if a.__class__.__name__ == "PatrolActionMoveForward":
					outtraj += "MoveForwardAction"
				elif a.__class__.__name__ == "PatrolActionTurnLeft":
					outtraj += "TurnLeftAction"
				elif a.__class__.__name__ == "PatrolActionTurnRight":
					outtraj += "TurnRightAction"
				elif a.__class__.__name__ == "PatrolActionTurnAround":
					outtraj += "TurnAroundAction"
				else:
					outtraj += "StopAction"

			else:
				outtraj += "None"
			outtraj += ":1;\n"
		
		outtraj += "ENDTRAJ\n"

		inputBuf += outtraj

	return inputBuf

if __name__ == "__main__": 

	#  D code uses 1.0 success rate of transitions
	p_fail = 0.05
	m = "boyd2"
	mapparams = boyd2MapParams(False)
	ogmap = OGMap(*mapparams)
	## Create Model
	model = patrol.model.PatrolModel(p_fail, None, ogmap.theMap())
	model.gamma = 0.9
	# Call external solver here instead of Python based
	args = ["boydsimple_t", ]
	import subprocess			

	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

	stdin = m + "\n"
	stdin += "1.0\n"
	
	(transitionfunc, stderr) = p.communicate(stdin)

	
	args = ["solveboydpatroller", ]

	initial2 = util.classes.NumMap()
	for s in model.S():
		initial2[s] = 1.0
	initial2 = initial2.normalize()

	List_TrueWeights = []
	List_Policies = []

	params_manualTuning_patrollingBiggerHallway = [1, 0, 0, 0, 0.0, 0]
	params_manualTuning_rotatingNearEnds = [0, 0, 0, 0, 0, 1]
	params_manualTuning_rotatingNearEnds1 = [0, 0, 0, 0, 1, 0]
	params_manualTuning_rotatingNearEnds2 = [0, 0, 0, 0, 0, 1]

	params = params_manualTuning_patrollingBiggerHallway
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	stdin = m + "\n"
	stdin += transitionfunc + "ENDT"+"\n"
	stdin += str(norm_params) + "\n"
	List_TrueWeights.append(norm_params)
	print("stdin ",stdin)
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	(stdout, stderr) = p.communicate(stdin)
	# print("\n computed patroller's policy", stdout)	
	opt_policy = parse_patrolling_policy(stdout)	
	List_Policies.append(opt_policy)
	p.stdin.close()
	p.stdout.close()
	p.stderr.close()

	params = params_manualTuning_rotatingNearEnds
	norm_params = [float(i)/sum(np.absolute(params)) for i in params]
	stdin = m + "\n"
	stdin += transitionfunc + "ENDT"+"\n"
	stdin += str(norm_params) + "\n"
	List_TrueWeights.append(norm_params)
	print("stdin ",stdin)
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	(stdout, stderr) = p.communicate(stdin)
	# print("\n computed patroller's policy", stdout)	
	opt_policy = parse_patrolling_policy(stdout)	
	List_Policies.append(opt_policy)
	p.stdin.close()
	p.stdout.close()
	p.stderr.close()

	input_dpmMEIRL = "boyd2\n"
	input_dpmMEIRL += "DPMMEIRL\n"
	# max_d, maximum clusters
	input_dpmMEIRL += "3\n"
	# maximum number of iterations
	input_dpmMEIRL += "3\n" # likelihood stayed same after 4  iterations. 7\n" 
	# descent threshold
	input_dpmMEIRL += "0.0001\n" # "0.000005\n"
	# value iteration thresohld
	input_dpmMEIRL += "0.1\n" 
	
	true_assignments = [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
	true_assignments = [ 1, 1, 0, 0] 
	true_assignments = [ 1, 1, 1, 1, 1, 0, 0, 0, 0, 0] 
	true_assignments = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
	true_assignments = [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1] 

	for i in range(0,len(true_assignments)): 
		ind = true_assignments[i]
		input_dpmMEIRL = formatWrite_simulated_Patrolling_trajectories(input_dpmMEIRL,1,model,List_Policies[ind],initial2) 

	input_dpmMEIRL += "ENDDEMO\n" 
	# print(input_dpmMEIRL) 
	input_dpmMEIRL += transitionfunc + "ENDT"+"\n"

	input_dpmMEIRL += str(true_assignments)+"\n"

	input_dpmMEIRL += str(len(List_TrueWeights))+"\n"
	for i in range(0, len(List_TrueWeights)):
		input_dpmMEIRL += str(List_TrueWeights[i])+"\n"

	# f = open(get_home() + "/catkin_ws/src/sorting_patrol_MDP_irl/MEMTIRL_boyd2Patrol_dataTest.log", "w")
	f = open(get_home() + "/Downloads/MEMTIRL_boyd2Patrol_dataTest2.log", "w")
	f.write(input_dpmMEIRL)
	f.close()
