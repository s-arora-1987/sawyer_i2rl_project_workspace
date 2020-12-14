#!/usr/bin/env python
# import roslib; roslib.load_manifest('navigation_irl')

import numpy as np
import rospy
from navigation_irl.srv import *
from navigation_irl.msg import *
import std_msgs.msg
import random
import mdp
import patrol
import util
import string
import mdp.agent
import mdp.solvers
import mdp.simulation
import os

home = os.environ['HOME']

configFile = home + "/catkin_ws/navigation_irl/mdpconfig"

config = None

mdps = {}
states = {}
actions = {}
percepts = {}
policies = {}
rewards = {}

# stolen from: http://stackoverflow.com/questions/452969/does-python-have-an-equivalent-to-java-class-forname
def get_class( kls ):
    parts = kls.split('.')
    module = ".".join(parts[:-1])
    m = __import__( module )
    for comp in parts[1:]:
        m = getattr(m, comp)            
    return m
    
    
def handle_init(req):
    global states
    global actions
    global mdps
    global percepts
    global policies
    global rewards
    
    mdpname = req.mdpType
    
    i = 1
    while(config.has_section("mdp"+str(i))):
        
        section = "mdp"+str(i)
        name = config.get(section, "name")
        if (name == mdpname):
        
            clazz = get_class(config.get(section, "model"))
            reward = get_class(config.get(section, "reward"))
            p_fail = config.getfloat(section,"p_fail")
            gamma = config.getfloat(section, "gamma")
            params = config.get(section, "params")        
            params = params.split(";")
            reward_params = config.get(section, "reward_params")
            reward_params = reward_params.split(";")
            
            p = [q for q in params]
            p.insert(0, p_fail)
            model = clazz(*p)
            
            p = [q for q in reward_params]
            reward_func = reward(*p)
            reward_weights = np.zeros( reward_func.dim )
            reward_func.params = reward_weights
            model.reward_function = reward_func
            model.gamma = gamma
            
            myid = ''.join(random.choice(string.ascii_uppercase + string.digits) for x in range(10))

            
            mdps[myid] = model
            states[myid] = model.S()
            actions[myid] = model.A()
            percepts[myid] = []
            rewards[myid] = []
            policies[myid] = mdp.agent.RandomAgent( model.A() )
            
            return initResponse(myid)
        
        i = i + 1
        
    return initResponse("")
        

def handle_reset(data):
    global percepts
    global policies
    global actions
    
    mdpId = data.data
    percepts[mdpId] = []
    policies[mdpId] = mdp.agent.RandomAgent( actions[mdpId] )

def handle_percept(data):
    global percepts
    global states
    global actions
    
    percepts[data.mdpId].append( ( states[data.mdpId][data.state], actions[data.mdpId][data.action], data.time ) )

def handle_calcpolicy(data):
    # create a (s,a) array for the given percepts, filling in missing percepts
    # give to the reward function
    # solve
    global states
    global actions
    global percepts
    global policies
    global mdps
    global rewards
    
    mdpId = data.data
    
    initial = util.classes.NumMap()
    for s in states[mdpId]:
        initial[s] = 1.0
    initial = initial.normalize()
    
    s_percepts = sorted(percepts[mdpId], key=lambda p: p[2])
    
    if len(s_percepts) == 0:
        policies[mdpId] = mdp.agent.RandomAgent( actions[mdpId] )
        return

    samples = []
    last_t = s_percepts[0][2]        
    for (s, a, t) in s_percepts:
        if t > last_t + 1:
            q = last_t
            last_percept = samples[len(samples) - 1]
            while t > last_t + 1:
                # create negative percepts
                last_t += 1
                samples.append(mdps[mdpId].NegState(last_percept, q, last_t))
        
        samples.append( (s,a) )
        last_t = t
    
    irl = mdp.solvers.IRLApprximateSolver(20,mdp.solvers.PolicyIteration(30, mdp.solvers.IteratingPolicyEvaluator(20)), 500)
    mdps[mdpId].reward_function.setSamples([samples,])
    
    (est_policy, w_est) = irl.solve( mdps[mdpId], initial, [samples,] )
    policies[mdpId] = est_policy
    rewards[mdpId] = w_est

def handle_simulate(req):
    global states
    global actions
    global models
    
    mdpId = req.mdpId
    count = req.timesteps
    initial_list = req.initial

    initial = util.classes.NumMap()
    for (t, s) in enumerate(states[mdpId]):
        initial[s] = initial_list[t]
    initial = initial.normalize()
    
    s_s = []
    a_s = []
    for (s,a,r) in mdp.simulation.simulate(models[mdpId], policies[mdpId], initial, count):
        s_s.append(states[mdpId].index(s))
        a_s.append(actions[mdpId].index(a))
        
    return simulateResponse([s_s, a_s])

def handle_reward(req):
    mdpId = req.mdpId
    
    global rewards
    
    return rewardResponse(rewards[mdpId])
    

def handle_policy(req):
    mdpId = req.mdpId
    
    global policies
    global states
    global actions
    
    p = policies[mdpId]    
    
    returnval = [0 for i in range(len(states[mdpId]))]
    for (t, s) in enumerate(states[mdpId]):
        returnval[t] = actions[mdpId].index(p.actions(s).keys()[0])
    
    return policyResponse(returnval)
    

def irl_server():
    rospy.init_node('navigation_irl')
    
    rospy.Subscriber("reset", std_msgs.msg.String, handle_reset)
    rospy.Subscriber("percept", percept , handle_percept)
    rospy.Subscriber("calcPolicy", std_msgs.msg.String, handle_calcpolicy)
    
    s = rospy.Service('irlinit', init, handle_init)
    t = rospy.Service('irlsimulate', simulate, handle_simulate)
    r = rospy.Service('irlreward', reward, handle_reward)
    p = rospy.Service('irlpolicy', policy, handle_policy)
    
    rospy.spin()
    
if __name__ == "__main__":
        
    import ConfigParser
    config = ConfigParser.ConfigParser()
    config.readfp(open(configFile))
    
    irl_server()
