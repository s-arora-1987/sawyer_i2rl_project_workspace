#!/usr/bin/env python
import roslib; roslib.load_manifest('navigation_irl')

import numpy as np
import rospy
from navigation_irl.srv import *
from navigation_irl.msg import *
from std_msgs.msg import String
import patrol.model
import patrol.reward
import random
import mdp.simulation
import mdp.solvers
import util.classes

mdpId = None
resetPub = None
perceptPub = None
calcPub = None

states = []
actions = []

def stateToId(state):
    global states
    return states.index(state)
    
def idToState(i):
    global states
    return states[i]

def actionToId(a):
    global actions
    return actions.index(a)
    
def idToAction(i):
    global actions
    return actions[i]

def initService():
    global mdpId
    try:
        mdpinit = rospy.ServiceProxy('irlinit', init)
        resp1 = mdpinit(initRequest("patrol"))
        mdpId = resp1.mdpId
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    

def initNode():
    global resetPub
    global perceptPub
    global calcPub
    resetPub = rospy.Publisher('reset', String)
    perceptPub = rospy.Publisher('percept', percept)
    calcPub = rospy.Publisher('calcPolicy', String)
    
    rospy.init_node('irltest')    

def stupidPythonIdiots():
    global resetPub
    global perceptPub
    global calcPub
    global mdpId
    global states
    global actions

    print(mdpId)
    rospy.wait_for_service('irlsimulate')
    
    initService()
    initNode()

    print(mdpId)
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
    model.gamma = 0.999
    
    states = model.S()
    actions = model.A()

    ## Create initial distribution
    initial = util.classes.NumMap()
    for s in model.S():
        initial[s] = 1.0
    initial = initial.normalize()
    
    ## Define feature function (approximate methods only)
#    feature_function = mdp.etc.StateActionFeatureFunction(model)
#    feature_function = mdp.etc.StateFeatureFunction(model)
#    feature_function = gridworld.etc.GWLocationFF(model)

    ## Define player
#    policy = mdp.agent.HumanAgent(model)
    opt_policy = mdp.solvers.ValueIteration(50).solve(model)
    
    j = 0
    for (s,a,r) in mdp.simulation.simulate(model,opt_policy, initial, 68):
        if (s.location[0] < observableStateLow):
            pass
        elif (s.location[0] > observableStateHigh):                
            pass
        else:
            perceptPub.publish(percept(mdpId=mdpId,state=stateToId(s),action=actionToId(a),time=j))
        j += 1
    
    
    centerObs = util.classes.NumMap()
    for s in model.S():
        centerObs[s] = 0
        if (s.location[0] == (observableStateLow + observableStateHigh) / 2):
            centerObs[s] = 1
    centerObs = centerObs.normalize() 
    s = mdpId
    calcPub.publish(String(s))
    
    
    raw_input("Percepts Sent, Press Enter to continue...")
    
    policyPxy = rospy.ServiceProxy('irlpolicy', policy)    
    est_p = policyPxy(policyRequest(mdpId))

    est_policy = util.classes.NumMap()    
    for (i, a) in enumerate(est_p.policy):
        est_policy[idToState(i)] = idToAction(a)
    
    
    mdp.etc.policy_report(opt_policy, est_policy, mdp.solvers.ExactPolicyEvaluator(), model, centerObs)
    
    for s in model.S():
        print 's = %s, pi*(s) = %s, pi_E(s) = %s' % ( s, opt_policy.actions(s), est_policy.actions(s) )
    print 'pi* and pi_E disagree on {} of {} states'.format( len([ s for s in model.S() if 
                                                            opt_policy.actions(s) != est_policy.actions(s) ]),
                                                            len(model.S()) )

    
    simulatePxy = rospy.ServiceProxy('irlsimulate', simulate)
    enc_policy = simulatePxy(simulateRequest(mdpId)).state_actions
    
    
    
if __name__ == "__main__":

    stupidPythonIdiots()
