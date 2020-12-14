# -*- coding: utf-8 -*-

import patrol.model
import patrol.reward
import patrol.solvers
import numpy as np
import random
import mdp.simulation
import mdp.solvers
#import mdp.agent
import util.classes
import math

def grid_main():
    random.seed()
    np.random.seed()
    
    ## Initialize constants

    p_fail = 0.05
    longHallway = 10
    shortSides = 4
    patrolAreaSize = longHallway + shortSides + shortSides
    observableStateLow = 7
    observableStateHigh = 8
    
    
    # calculate farness for each node in the patrolled area
    farness = np.zeros(patrolAreaSize)
    for i in range(patrolAreaSize):
        sum = 0
        for j in range(patrolAreaSize):
            sum += abs(i - j)
    
        farness[i] = sum
    
    ## Create reward function
    reward = patrol.reward.PatrolReward(patrolAreaSize, farness, observableStateLow, observableStateHigh)
    reward_weights = np.zeros( reward.dim )
    reward_weights[0] = .2
    reward_weights[1] = .35
    reward_weights[2] = .45
    reward_weights[3] = 0
    reward_weights[4] = 0
    
    
    reward.params = reward_weights
    
    
    ## Create Model
    model = patrol.model.PatrolModel(p_fail, longHallway, shortSides)
    model.reward_function = reward
    model.gamma = 0.99

    t_max = math.ceil( math.log(0.01)/math.log(model.gamma) )   
    print(t_max)
    ## Create initial distribution
    initial = util.classes.NumMap()
    for s in model.S():
        initial[s] = 1.0
    initial = initial.normalize()
    
    ## Define feature function (approximate methods only)
#    feature_function = mdp.etc.StateActionFeatureFunction(model)
    feature_function = mdp.etc.StateFeatureFunction(model)
#    feature_function = gridworld.etc.GWLocationFF(model)

    ## Define player
#    policy = mdp.agent.HumanAgent(model)
    opt_policy = mdp.solvers.ValueIteration(50).solve(model)
    
#    policy = mdp.solvers.QValueIterator(100).solve(model)
#    policy = mdp.solvers.LSPI(40,100, feature_f=feature_function).solve(model)
    policy = mdp.solvers.LSTD(50, 100, feature_f=feature_function).solve(model)
#    policy = mdp.solvers.PolicyIterator(20, mdp.solvers.IteratingPolicyEvaluator(100)).solve(model)
#    policy = mdp.solvers.PolicyIterator(20, mdp.solvers.SamplingPolicyEvaluator(100,50)).solve(model)
#    policy = mdp.solvers.PolicyIterator(20, mdp.solvers.ExactPolicyEvaluator()).solve(model)
    
    ## Print out world information
    print model.info()
    print reward.info()
    print 'States: ' + str( [str(state) for state in model.S()] )
    print 'Action: ' + str( [str(action) for action in model.A()] )
    print 'Policy: '
    for s in model.S():
        print '\tpi({}) = {}'.format(s, opt_policy.actions(s))
    
    ## Estimate policy quality
    print 'Sample run:'
    for (s,a,r) in mdp.simulation.simulate(model, policy, initial, 20):
        print '%s, %s, %f' % (s,a,r)
#    print 'Average Score: %f' % (evaluate_policy(model, initial, policy, t_max),)
    mdp.etc.policy_report(opt_policy, policy, mdp.solvers.ExactPolicyEvaluator(), model, initial)
        
    #Calculate feature expectations of pi^(0) = mu^(0)
    samples = []
     

    centerObs = util.classes.NumMap()
    for s in model.S():
        centerObs[s] = 0
        if (s.location[0] == (observableStateLow + observableStateHigh) / 2):
            centerObs[s] = 1
    centerObs = centerObs.normalize() 
    
    for i in range(1):
        j = 0
        hist = []
        lastSeenAt = 0
        lastSeenDir = 0
        for (s,a,r) in mdp.simulation.simulate(model,opt_policy, initial, 50):
            if (s.location[0] < observableStateLow):
                hist.append((patrol.model.PatrolState(np.array([-2,lastSeenDir])),patrol.model.PatrolActionForward()))
            elif (s.location[0] > observableStateHigh):                
                hist.append((patrol.model.PatrolState(np.array([patrolAreaSize,lastSeenDir])),patrol.model.PatrolActionForward()))            
            else:
                lastSeenDir = s.location[1]
                if (j - lastSeenAt > 1):
                    count = (j - lastSeenAt) / 2
                    for k in range(len(hist) - count, len(hist)):
                        hist[k][0].location[1] = lastSeenDir
                
                lastSeenAt = j
                hist.append((s,a))
            j += 1
        samples.append(hist)

    
    ## Do IRL
#    irl = mdp.solvers.IRLExactSolver(20, mdp.solvers.ValueIteration(40))
#    (est_policy, w_est) = irl.solve(model, initial, opt_policy) 

#    irl = mdp.solvers.IRLApprximateSolver(50, mdp.solvers.ValueIteration(100), 1)
#    irl = mdp.solvers.IRLApprximateSolver(20, mdp.solvers.ValueIteration(50), 500)
#    irl = mdp.solvers.IRLApprximateSolver(20,mdp.solvers.PolicyIteration(20, mdp.solvers.IteratingPolicyEvaluator(20)), 500)

    q_value = patrol.solvers.QValueIteration(20)
    irl = patrol.solvers.IRLBayesianSolver(1200, mdp.solvers.PolicyIteration(20, mdp.solvers.IteratingPolicyEvaluator(20)), q_value)
    
    irlprior = patrol.solvers.PatrolPrior(samples, observableStateLow, observableStateHigh, patrolAreaSize )
    
#    model.reward_function = rewardirl
    model._p_fail = float(0)
    
    (est_policy) = irl.solve(model, samples, irlprior, 0.05, patrolAreaSize ) 
    
    model._p_fail = float(p_fail)
    
    ## Estimate estimated policy quality
    model.reward_function = reward
    model.reward_function.params = reward_weights
    mdp.etc.policy_report(opt_policy, est_policy, mdp.solvers.ExactPolicyEvaluator(), model, centerObs)
    
    for s in model.S():
        print 's = %s, pi*(s) = %s, pi_E(s) = %s' % ( s, opt_policy.actions(s), est_policy.actions(s) )
    print 'pi* and pi_E disagree on {} of {} states'.format( len([ s for s in model.S() if 
                                                            opt_policy.actions(s) != est_policy.actions(s) ]),
                                                            len(model.S()) )

    print 'Sample run:'
    for (s,a,r) in mdp.simulation.simulate(model, est_policy, centerObs, 50):
        print '%s, %s, %f' % (s,a,r)
        
if __name__ == '__main__':
    grid_main()