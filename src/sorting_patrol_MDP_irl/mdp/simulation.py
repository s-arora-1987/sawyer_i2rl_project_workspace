'''
Created on Mar 31, 2011

@author: duckworthd
'''
import util.functions
import sys
import patrol.model

def simulate(model, agent, initial, t_max):
    '''
    Simulate an MDP for t_max timesteps or until the
    a terminal state is reached.  Returns a list
        [ (s_0, a_0, r_0), (s_1, a_1, r_1), ...]
    '''
    
    s = util.functions.sample(initial)
    result = []
    t = 0
    while t < t_max and not model.is_terminal(s): 
        a = agent.sample(s)
        s_p = util.functions.sample( model.T(s,a) )
        r = model.R(s,a)
        
        result.append( (s,a,r) )
        s = s_p
        t += 1
    if model.is_terminal(s):
        a = agent.sample(s)
       	r = model.R(s,a)

        result.append( (s,a,r) )
        
    return result

def simulate_givenstart(model, agent, s, t_max):
    '''
    Simulate an MDP for t_max timesteps or until the
    a terminal state is reached.  Returns a list
        [ (s_0, a_0, r_0), (s_1, a_1, r_1), ...]
    '''
    
    # s = util.functions.sample(initial)
    result = []
    t = 0
    while t < t_max and not model.is_terminal(s): 
        a = agent.sample(s)
        s_p = util.functions.sample( model.T(s,a) )
        r = model.R(s,a)
        
        result.append( (s,a,r) )
        s = s_p
        t += 1
    if model.is_terminal(s):
        a = agent.sample(s)
       	r = model.R(s,a)

        result.append( (s,a,r) )
        
    return result


def project(model, agent, initial, t_max):
	'''
	Does a full state space simulation, allowing
	each state at each timestep to have < 1.0
	probability of occupancy
	'''
	# will return an array of dicts
	
	
	# build initial state distribution
	result = [{}, ]
	for s in model.S():
		if s in initial.keys():
			result[0][s] = initial[s];
		else:
			result[0][s] = 0;
	
	
	t = 1
	
	while t < t_max:
		dist = {}
		for s in model.S():
			if result[t - 1][s] == 0:
				continue
			
			for (a, pr_a) in agent.actions(s):
				for (s_prime, pr_s_prime) in model.T(s, a):
					if not s_prime in dist:
						dist[s_prime] = pr_a * pr_s_prime * result[t - 1][s];
					else:
						dist[s_prime] += pr_a * pr_s_prime * result[t - 1][s];
		
		for s in model.S():
			if not s in dist.keys:
				dist[s] = 0
				
		result.append(dist)
		t += 1
		
	return result		
	

def sample_model(model, n_samples, distr, agent):
    '''
    sample states (s,a,r,s') where s sampled from distribution
    returns
        [(s_0,a_0,r_0,s_p_0), (s_1,a_1,r_1,s_p_1),...]
    '''
    result = []
    for i in range(n_samples):
        s = util.functions.sample(distr)
        a = agent.sample(s)
        r = model.R(s,a)
        s_p = util.functions.sample(model.T(s,a))
        result.append( (s,a,r,s_p) )
    return result
   
def sample_traj(model, t_max, initial, agent):
    '''
    sample trajectory without reward values
    '''
    
    atTerminal = False
    result = [] 
    t = 0
    s = util.functions.sample(initial) 
    while t < t_max and not atTerminal:
        a = agent.sample(s) 
        # print("(s,a) ",(s,a))
        s_p = util.functions.sample(model.T(s,a)) 
        # print("(model.T(s,a),s_p) ",(model.T(s,a),s_p))
        result.append( (s,a,s_p) ) 
        if model.is_terminal(s_p):
            atTerminal = True
        t += 1
        s = s_p
    
    return result

def multi_simulate(model, policies, initials, t_max, interactionlength):
	# policies = [ policy1, policy2, equilibrium1, equilibrium2 ]
	
	result = []
	Ss = []	
	for initial in initials:				
		
		Ss.append(util.functions.sample(initial))
		result.append([])
	t = 0
	atTerminal = False
	
	interactionCooldown = [-1 for i in range(len(policies))]
	
	while t < t_max and not atTerminal:
		actions = []
		for i in initials:
			actions.append(None)
			
		if not policies[2] == None and not policies[3] == None and interactionCooldown[0] < 0 and interactionCooldown[1] < 0:
			for (i, s) in enumerate(Ss):
				for (j, s2) in enumerate(Ss):
					if not i == j:
						if s2.conflicts(s) or (t > 0 and (result[i][t - 1][0].conflicts(s2) or s.conflicts(result[j][t - 1][0])) ):
							interactionCooldown[0] = interactionlength
							interactionCooldown[1] = interactionlength
		
		for (i, a) in enumerate(actions):
			if interactionCooldown[i] <= 0:
				actions[i] = policies[i].sample(Ss[i])
			elif interactionCooldown[i] > 1:
				actions[i] = patrol.model.PatrolActionStop()
			else:
				actions[i] = util.functions.sample(policies[2 + i])
				if actions[i].__class__.__name__ == "PatrolActionMoveForward":
					actions[i] = policies[i].sample(Ss[i])

			interactionCooldown[i] = interactionCooldown[i] - 1
					
		for (i, a) in enumerate(actions):
#			r = model.R(Ss[i],actions[i])
		
			result[i].append( (Ss[i],actions[i]) )
			Ss[i] = util.functions.sample( model.T(Ss[i],actions[i]) )
			if model.is_terminal(Ss[i]):
				atTerminal = True
				
				
		t += 1
		
	if atTerminal:
		for (i, s) in Ss:
			a = policies[i].sample(s)
			r = model.R(s,a)
		
			result[i].append( (s,a,r) )
		
	return result
				
				
				
				
def create_trajectories(pmodel, policies, patrollerStartStates, patrollerTimes, predictTime, interactionLength):
	# policies = [ policy1, policy2, equilibrium1, equilibrium2 ]
	# patrollerStartStates = [state, state]
	# patrollerTimes = [time, time]
	# predictTime = int
	

	min = sys.maxint

	for s in patrollerTimes:
		if (s < min):
			min = s
	

	sim_times = []
	for s in patrollerTimes:
		sim_times.append(s - min)
	

	startStates = []
	for s in patrollerStartStates:
		startStates.append(s)

# individually simulate first in order to get equal start times which is needed for multi-simulate

	for (i, startState) in enumerate(patrollerStartStates):
		initial = {}
		for s in pmodel.S():
			initial[s] = 0.0
		initial[startState] = 1.0
		
		for sar in simulate(pmodel, policies[i], initial, sim_times[i]):
			startStates[i] = sar[0]
		
	initials = []
	for (i, s) in enumerate(startStates):
		initial = {}
		for s in pmodel.S():
			initial[s] = 0.0
		initial[startStates[i]] = 1.0
		
		initials.append(initial)

	temp = multi_simulate(pmodel, policies, initials, min + predictTime, interactionLength)

	result = []
	
	for (num, sarArr) in enumerate(temp):
		traj = []
		for (i, sar) in enumerate(sarArr):
			if (i >= min):
				traj.append( (sar[0], 1.0) )
		result.append(traj)
		
	return result;


def create_patroller_trajectory_positions(pmodel, policies, patrollerStartStates, patrollerTimes, predictTime, interactionLength, mapToUse):
	# policies = [ policy1, policy2, equilibrium1, equilibrium2 ]
	# patrollerStartStates = [state, state]
	# patrollerTimes = [time, time]
	# predictTime = int
	

	min = sys.maxint

	for s in patrollerTimes:
		if (s < min):
			min = s
	

	sim_times = []
	for s in patrollerTimes:
		sim_times.append(s - min)
	
	
	result = []

	startStates = []
	for s in patrollerStartStates:
		startStates.append(s)
		result.append([])

# individually simulate first in order to get equal start times which is needed for multi-simulate

	for (i, startState) in enumerate(patrollerStartStates):
		initial = {}
		for s in pmodel.S():
			initial[s] = 0.0
		initial[startState] = 1.0
		
		for sar in simulate(pmodel, policies[i], initial, sim_times[i]):
			startStates[i] = sar[0]
			result[i].append(patrol.model.convertStateToPositionPatroller(sar[0], mapToUse))
		
	initials = []
	for (i, s) in enumerate(startStates):
		initial = {}
		for s in pmodel.S():
			initial[s] = 0.0
		initial[startStates[i]] = 1.0
		
		initials.append(initial)

	temp = multi_simulate(pmodel, policies, initials, min + predictTime, interactionLength)

	
	for (num, sarArr) in enumerate(temp):
		for (i, sar) in enumerate(sarArr):
			result[num].append(patrol.model.convertStateToPositionPatroller(sar[0], mapToUse))
		
	return result;