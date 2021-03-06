import gridworld.model
import gridworld.reward
import gridworld.etc
import numpy as np
import random
import mdp.simulation
import mdp.solvers
#import mdp.agent
import util.classes
import math
import pickle
import os


def grid_main():
    random.seed()
    np.random.seed()
    
    ## Initialize constants
    map = np.array( [[1, 1, 1, 1, 1], 
                     [1, 0, 0, 1, 1],
                     [1, 0, 1, 1, 1],
                     [1, 1, 1, 0, 0]])
    map = np.ones((20,20))
    box_size = np.array( [4,4] )
    p_fail = 0.3
    
    ## Create reward function
    reward = gridworld.reward.GWBoxReward(box_size, map)
#    reward_weights = np.random.rand( reward.dim ) - 0.5*np.ones( reward.dim )
#    reward_weights[-4:] = 0.1*reward_weights[-4:]

    reward_weights = np.random.rand( reward.dim )
    for (i, v) in enumerate(reward_weights):
        reward_weights[i] = random.random() if v < .1 else 0
    
    reward_weights /= reward_weights.sum()
    
    reward.params = reward_weights
    
    ## Create Model
    model = gridworld.model.GWModel(p_fail, map)
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
    
    if os.path.exists("savedpolicy"):
        thefile = open("savedpolicy", "r")
        opt_policy = pickle.load(thefile)
        thefile.close()
    else:
        opt_policy = mdp.solvers.ValueIteration(50).solve(model)
        thefile = open("savedpolicy", "w")
        pickle.dump(opt_policy, thefile)
        thefile.close()
    
    
#    policy = mdp.solvers.QValueIteration(100).solve(model)
#    policy = mdp.solvers.LSPI(40,100, feature_f=feature_function).solve(model)
#    policy = mdp.solvers.LSTD(50, 100, feature_f=feature_function).solve(model)
#    policy = mdp.solvers.PolicyIteration(20, mdp.solvers.IteratingPolicyEvaluator(100)).solve(model)
#    policy = mdp.solvers.PolicyIteration(20, mdp.solvers.SamplingPolicyEvaluator(100,50)).solve(model)
#    policy = mdp.solvers.PolicyIteration(20, mdp.solvers.ExactPolicyEvaluator()).solve(model)
 
    policy = opt_policy   
    ## Print out world information
    print model.info()
    print reward.info()
    print 'States: ' + str( [str(state) for state in model.S()] )
    print 'Action: ' + str( [str(action) for action in model.A()] )
    print 'Policy: '
    for s in model.S():
        print '\tpi({}) = {}'.format(s, policy.actions(s))
    
    ## Estimate policy quality
    print 'Sample run:'
    for (s,a,r) in mdp.simulation.simulate(model, policy, initial, 8):
        print '%s, %s, %f' % (s,a,r)
#    print 'Average Score: %f' % (evaluate_policy(model, initial, policy, t_max),)
    mdp.etc.policy_report(opt_policy, policy, mdp.solvers.ExactPolicyEvaluator(), model, initial)
    
    ## Do IRL
#    irl = mdp.solvers.IRLExactSolver(20, mdp.solvers.ValueIteration(40))
#    (est_policy, w_est) = irl.solve(model, initial, opt_policy) 

#    irl = mdp.solvers.IRLApprximateSolver(50, mdp.solvers.ValueIteration(100), 1)
#    irl = mdp.solvers.IRLApprximateSolver(30,mdp.solvers.PolicyIteration(20, mdp.solvers.IteratingPolicyEvaluator(20)), 50)
    irl = mdp.solvers.IRLApprximateSolver(30,mdp.solvers.LSTD(50, 100, feature_f=feature_function), 50)
        
    #Calculate feature expectations of pi^(0) = mu^(0)
    samples = []
 
    for i in range(1000):
        hist = []
        for (s,a,r) in mdp.simulation.simulate(model,opt_policy, initial, t_max):
            hist.append((s,a))
        samples.append(hist)
    
    (est_policy, w_est) = irl.solve(model, initial, samples ) 
    
    ## Estimate estimated policy quality
    model.reward_function.params = reward_weights
    mdp.etc.policy_report(opt_policy, est_policy, mdp.solvers.ExactPolicyEvaluator(), model, initial)
    
    for s in model.S():
        print 's = %s, pi*(s) = %s, pi_E(s) = %s' % ( s, policy.actions(s), est_policy.actions(s) )
    print 'pi* and pi_E disagree on {} of {} states'.format( len([ s for s in model.S() if 
                                                            policy.actions(s) != est_policy.actions(s) ]),
                                                            len(model.S()) )

if __name__ == '__main__':
    grid_main()