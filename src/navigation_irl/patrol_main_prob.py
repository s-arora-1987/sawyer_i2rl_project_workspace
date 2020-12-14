import patrol.model
import patrol.reward
import numpy as np
import random
import mdp.simulation
import mdp.solvers
#import mdp.agent
import util.classes
import patrol.solvers
import math
from patrol.model import *

def grid_main():
    random.seed()
    np.random.seed()
    
    ## Initialize constants

    p_fail = 0.05
    longHallway = 10
    shortSides = 2
    patrolAreaSize = longHallway + shortSides + shortSides
    observableStateLow = patrolAreaSize / 2 - 1
    observableStateHigh = patrolAreaSize / 2
    
    
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

    reward_weights[0] = 1
    reward_weights[1] = 0
    reward_weights[2] = 0
        
    
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
    opt_policy = mdp.solvers.ValueIteration(.1).solve(model)
    
#    policy = mdp.solvers.QValueIterator(100).solve(model)
#    policy = mdp.solvers.LSPI(40,100, feature_f=feature_function).solve(model)
#    policy = mdp.solvers.LSTD(50, 100, feature_f=feature_function).solve(model)
#    policy = mdp.solvers.PolicyIteration(20, mdp.solvers.IteratingPolicyEvaluator2(.1)).solve(model)
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
#    print 'Sample run:'
#    for (s,a,r) in mdp.simulation.simulate(model, policy, initial, 20):
#        print '%s, %s, %f' % (s,a,r)
#    print 'Average Score: %f' % (evaluate_policy(model, initial, policy, t_max),)
#    mdp.etc.policy_report(opt_policy, policy, mdp.solvers.ExactPolicyEvaluator(), model, initial)
    
    ## Do IRL
#    irl = mdp.solvers.IRLExactSolver(20, mdp.solvers.ValueIteration(40))
#    (est_policy, w_est) = irl.solve(model, initial, opt_policy) 

#    irl = mdp.solvers.IRLApprximateSolver(50, mdp.solvers.ValueIteration(100), 1)
#    irl = mdp.solvers.IRLApprximateSolver(20, mdp.solvers.ValueIteration(50), 500)
#    irl = patrol.solvers.IRLApprximateSolver(30,mdp.solvers.PolicyIteration(50, mdp.solvers.IteratingPolicyEvaluator2(.1)), 200)
    irl = patrol.solvers.IRLApprximateSolver(30,mdp.solvers.ValueIteration(.5), 200)
        
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
        for (s,a,r) in mdp.simulation.simulate(model,opt_policy, initial, 120):
            if (s.location[0] < observableStateLow):
                pass
#                p = patrol.model.PatrolState(np.array([-2,lastSeenDir]))
#                p.placeholder = True
#                p.negObsCount = j - lastSeenAt
#                hist.append([(p,patrol.model.PatrolActionForward(), 1.0), ])
            elif (s.location[0] > observableStateHigh):                
                pass
#                p = patrol.model.PatrolState(np.array([patrolAreaSize,lastSeenDir]))
#                p.placeholder = True
#                p.negObsCount = j - lastSeenAt
#                hist.append([(p,patrol.model.PatrolActionForward(), 1.0),])
            else:
                if (j - lastSeenAt > 1 and lastSeenAt > 0):
#                    print("Last:", len(hist) - 1, hist[len(hist) - 1])
                    # append j - lastSeenAt timesteps of possible states
                    for k in range(j - lastSeenAt + 1):
                        allStates = []
                        for sap in hist[len(hist) - 1]:
                            s2 = sap[0]
                            prob = sap[2]
                            
                            for a2 in model.A():
                                for (newS, newP) in model.T(s2, a2).items():
                                    if not (a2.__class__.__name__ == "PatrolActionTurnAround" and sap[1].__class__.__name__ == "PatrolActionTurnAround"):
                                        if not (newS.location[0] >= observableStateLow and newS.location[0] <= observableStateHigh):
                                            addState = True
                                            for (l) in allStates:
                                                if l[0] == newS and l[1] == a2:
                                                    allStates.remove(l)
                                                    allStates.append( (newS, a2, (prob * newP) + l[2]) )
                                                    addState = False
                                                    break
                                            if addState:
                                                allStates.append( (newS,a2, prob * newP) )
                                
                        hist.append(allStates)

                    hist.append([(s,a,1.0), ])
#                    print("Observed:", len(hist) - 1, hist[len(hist) - 1] )
                    
                    for k in range(len(hist) - 2, (len(hist) - 2) - ((j - lastSeenAt + 1)), -1):
                        curhist = hist[k]
                        
                        nexthist = []
                        for sap in curhist:
                            s2 = sap[0]
                            a2 = sap[1]
                            prob = sap[2]

                            foundChild = False
                            
                            for (newS, newP) in model.T(s2, a2).items():
                                for (sap2) in hist[k+1]:
                                    if sap2[0] == newS:
                                        foundChild = True
                                        break
                            
                            if foundChild:
                                nexthist.append(sap)
      
                        hist[k] = nexthist
                            
                    for k in range(len(hist) - 2, (len(hist) - 2) - (j - lastSeenAt + 1), -1):
                        curhist = hist[k]
                        sumP = 0
                        for sap in curhist:
                            sumP += sap[2]
                            
                        nexthist = []
                        for sap in curhist:
                            if sap[2] / sumP > .001:
                                nexthist.append( (sap[0], sap[1], sap[2] / sumP) )
                        hist[k] = nexthist
                        
#                    for k in range( (len(hist) - 2) - (j - lastSeenAt + 1), len(hist) ):
#                        print(k, hist[k])
                        
                    lastSeenAt = j
                else:
                    lastSeenAt = j
                    hist.append([(s,a,1.0), ])
            j += 1
        samples.append(hist)

    
#    rewardirl = patrol.reward.PatrolRewardIRL2(patrolAreaSize, observableStateLow, observableStateHigh, samples)
    rewardirl = patrol.reward.PatrolReward(patrolAreaSize, farness, observableStateLow, observableStateHigh)
    rewardirl_weights = np.zeros( rewardirl.dim )
    rewardirl.params = rewardirl_weights
    
    model.reward_function = rewardirl
#    model._p_fail = float(0)
    
    (est_policy, w_est) = irl.solve(model, centerObs, samples ) 
#    (est_policy, w_est) = irl.solve(model, initial, samples ) 
    
    model._p_fail = float(p_fail)
    print(w_est)
    
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
    
    exit()
    
    map = np.array( [[1, 0, 0, 0, 0, 0, 0, 0, 0, 1], 
                     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]])

    ## Create transfer reward function
#    reward = patrol.reward.AttackerRewardPatrollerTraj(AttackerState(np.array([2,2]), 0), 10, -100, samples, (AttackerState(np.array([2,5]), 0), ), (PatrolState(np.array([7,0])),) , (0,), 3)
    reward = patrol.reward.AttackerRewardPatrollerPolicy(AttackerState(np.array([2,2]), 0), 10, -100, model, (est_policy, ), (AttackerState(np.array([2,5]), 0), ), (PatrolState(np.array([7,0])),) , (0,), 3, 30)
    reward_weights = np.zeros( reward.dim )
    reward_weights[0] = 1
        
    
    reward.params = reward_weights
    
    
    ## Create Model
    model = patrol.model.AttackerModel(p_fail, map, 30)
    model.reward_function = reward
    model.gamma = 0.99

    attacker_policy = mdp.solvers.ValueIteration(50).solve(model)
    
    print model.info()
    print reward.info()
    print 'States: ' + str( [str(state) for state in model.S()] )
    print 'Action: ' + str( [str(action) for action in model.A()] )
    print 'Policy: '
    for s in model.S():
        print '\tpi({}) = {}'.format(s, attacker_policy.actions(s))    
    

    startPos = util.classes.NumMap()
    for s in model.S():
        if s.location[0] == 2 and s.location[1] == 5 and s.time == 0:
            startPos[s] = 1
        else:
            startPos[s] = 0
    
    startPos = startPos.normalize()
    for (s,a,r) in mdp.simulation.simulate(model, attacker_policy, startPos, 15):
        print '%s, %s, %f' % (s,a,r)
    
if __name__ == '__main__':
    grid_main()