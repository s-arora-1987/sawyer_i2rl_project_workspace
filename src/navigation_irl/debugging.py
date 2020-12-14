#!/usr/bin/env python
import patrol.model
from patrol.model import *
import mdp.agent
import sys
import Queue
import subprocess
import multiprocessing
import random
import cPickle as pickle
import os
import numpy
import operator
from patrol.time import *


home = os.environ['HOME']
def get_home():
    global home
    return home

startat = 0

def get_time():
    global startat
    return rospy.get_time() - startat


# ros publications
pub = None
goal = None
state = "w"

# mdp Settings
startState = None

# mdp States
lastPatroller0State = None
lastPatroller1State = None

# class for holding the MDP and utils
mdpWrapper = None

# actual Map positions
cur_waypoint = [0,0]
lastPositionBelief = None
lastActualPosition = None

lastPatroller0Position1 = None
lastPatroller0Position2 = None
lastPatroller1Position1 = None
lastPatroller1Position2 = None
global minGapBwDemos, patroller0LastSeenAt, patroller1LastSeenAt

maxsightdistance = 6

patroller0GroundTruth = None
patroller1GroundTruth = None

mapToUse = None
patroller = None

obstime = 900

interactionLength = 1

patrollersuccessrate = None
usesimplefeatures = False

use_em = 0

patrollerOGMap = None
attackerOGMap = None

equilibriumKnown = False

#I2RL
BatchIRLflag=False
sessionNumber=0
learned_mu_E=[[0.0]*6,[0.0]*6] # may not be needed
#learned_weights=[[0.0]*6,[0.0]*6] # needed for compute reldiff
num_Trajsofar=0
#wt_data=numpy.empty([1, 1, 2], dtype=float)
normedRelDiff=2.0 # start with a high value to call update()
stopI2RLThresh=0.5 #0.1 
curr_sessionStart=False
lineFoundWeights=""
lineFeatureExpec=""
lineLastBeta=""
lineLastZS=""
lineLastZcount=""
global wt_data
wt_data=numpy.empty([2, 6], dtype=float)

def parsePolicies(stdout, equilibrium):
    global lineFoundWeights, lineFeatureExpec, learned_weights, \
    num_Trajsofar, lineLastBeta, lineLastZS, lineLastZcount
    
    stateactions = ['[0, 1, 0] = MoveForwardAction', '[0, 1, 1] = TurnRightAction', '[0, 1, 2] = TurnLeftAction', '[0, 1, 3] = MoveForwardAction', '[0, 2, 0] = MoveForwardAction', '[0, 2, 1] = TurnRightAction', '[0, 2, 2] = MoveForwardAction', '[0, 2, 3] = TurnLeftAction', '[0, 3, 0] = MoveForwardAction', '[0, 3, 1] = TurnRightAction', '[0, 3, 2] = MoveForwardAction', '[0, 3, 3] = TurnLeftAction', '[0, 4, 0] = MoveForwardAction', '[0, 4, 1] = TurnRightAction', '[0, 4, 2] = TurnLeftAction', '[0, 4, 3] = TurnLeftAction', '[0, 5, 0] = TurnLeftAction', '[0, 5, 1] = TurnLeftAction', '[0, 5, 2] = TurnLeftAction', '[0, 5, 3] = TurnLeftAction', '[0, 6, 0] = TurnLeftAction', '[0, 6, 1] = TurnLeftAction', '[0, 6, 2] = MoveForwardAction', '[0, 6, 3] = TurnRightAction', '[0, 7, 0] = MoveForwardAction', '[0, 7, 1] = TurnLeftAction', '[0, 7, 2] = MoveForwardAction', '[0, 7, 3] = TurnRightAction', '[0, 8, 0] = TurnLeftAction', '[0, 8, 1] = TurnLeftAction', '[0, 8, 2] = MoveForwardAction', '[0, 8, 3] = TurnRightAction', '[1, 1, 0] = TurnRightAction', '[1, 1, 1] = MoveForwardAction', '[1, 1, 2] = TurnLeftAction', '[1, 1, 3] = MoveForwardAction', '[2, 1, 0] = TurnRightAction', '[2, 1, 1] = MoveForwardAction', '[2, 1, 2] = TurnLeftAction', '[2, 1, 3] = MoveForwardAction', '[3, 1, 0] = TurnRightAction', '[3, 1, 1] = MoveForwardAction', '[3, 1, 2] = TurnLeftAction', '[3, 1, 3] = MoveForwardAction', '[4, 1, 0] = TurnRightAction', '[4, 1, 1] = MoveForwardAction', '[4, 1, 2] = TurnLeftAction', '[4, 1, 3] = MoveForwardAction', '[5, 1, 0] = TurnRightAction', '[5, 1, 1] = MoveForwardAction', '[5, 1, 2] = TurnLeftAction', '[5, 1, 3] = MoveForwardAction', '[6, 1, 0] = TurnRightAction', '[6, 1, 1] = MoveForwardAction', '[6, 1, 2] = TurnLeftAction', '[6, 1, 3] = MoveForwardAction', '[7, 1, 0] = TurnRightAction', '[7, 1, 1] = MoveForwardAction', '[7, 1, 2] = TurnLeftAction', '[7, 1, 3] = MoveForwardAction', '[8, 1, 0] = TurnLeftAction', '[8, 1, 1] = MoveForwardAction', '[8, 1, 2] = TurnLeftAction', '[8, 1, 3] = MoveForwardAction', '[9, 1, 0] = TurnLeftAction', '[9, 1, 1] = MoveForwardAction', '[9, 1, 2] = TurnRightAction', '[9, 1, 3] = MoveForwardAction', '[10, 1, 0] = TurnLeftAction', '[10, 1, 1] = MoveForwardAction', '[10, 1, 2] = TurnRightAction', '[10, 1, 3] = MoveForwardAction', '[11, 1, 0] = TurnLeftAction', '[11, 1, 1] = MoveForwardAction', '[11, 1, 2] = TurnRightAction', '[11, 1, 3] = MoveForwardAction', '[12, 1, 0] = TurnLeftAction', '[12, 1, 1] = MoveForwardAction', '[12, 1, 2] = TurnRightAction', '[12, 1, 3] = MoveForwardAction', '[13, 1, 0] = TurnLeftAction', '[13, 1, 1] = MoveForwardAction', '[13, 1, 2] = TurnRightAction', '[13, 1, 3] = MoveForwardAction', '[14, 1, 0] = TurnLeftAction', '[14, 1, 1] = MoveForwardAction', '[14, 1, 2] = TurnRightAction', '[14, 1, 3] = MoveForwardAction', '[15, 1, 0] = TurnLeftAction', '[15, 1, 1] = MoveForwardAction', '[15, 1, 2] = TurnRightAction', '[15, 1, 3] = MoveForwardAction', '[16, 1, 0] = MoveForwardAction', '[16, 1, 1] = MoveForwardAction', '[16, 1, 2] = TurnRightAction', '[16, 1, 3] = TurnLeftAction', '[16, 2, 0] = MoveForwardAction', '[16, 2, 1] = TurnRightAction', '[16, 2, 2] = MoveForwardAction', '[16, 2, 3] = TurnLeftAction', '[16, 3, 0] = MoveForwardAction', '[16, 3, 1] = TurnRightAction', '[16, 3, 2] = MoveForwardAction', '[16, 3, 3] = TurnLeftAction', '[16, 4, 0] = MoveForwardAction', '[16, 4, 1] = TurnRightAction', '[16, 4, 2] = TurnLeftAction', '[16, 4, 3] = TurnLeftAction', '[16, 5, 0] = TurnLeftAction', '[16, 5, 1] = TurnLeftAction', '[16, 5, 2] = TurnLeftAction', '[16, 5, 3] = TurnLeftAction', '[16, 6, 0] = TurnLeftAction', '[16, 6, 1] = TurnLeftAction', '[16, 6, 2] = MoveForwardAction', '[16, 6, 3] = TurnRightAction', '[16, 7, 0] = MoveForwardAction', '[16, 7, 1] = TurnLeftAction', '[16, 7, 2] = MoveForwardAction', '[16, 7, 3] = TurnRightAction', '[16, 8, 0] = TurnLeftAction', '[16, 8, 1] = TurnLeftAction', '[16, 8, 2] = MoveForwardAction', '[16, 8, 3] = TurnRightAction', 'ENDPOLICY', '[0, 1, 0] = MoveForwardAction', '[0, 1, 1] = MoveForwardAction', '[0, 1, 2] = MoveForwardAction', '[0, 1, 3] = MoveForwardAction', '[0, 2, 0] = TurnLeftAction', '[0, 2, 1] = TurnLeftAction', '[0, 2, 2] = TurnLeftAction', '[0, 2, 3] = TurnLeftAction', '[0, 3, 0] = MoveForwardAction', '[0, 3, 1] = TurnLeftAction', '[0, 3, 2] = TurnLeftAction', '[0, 3, 3] = TurnLeftAction', '[0, 4, 0] = TurnLeftAction', '[0, 4, 1] = TurnLeftAction', '[0, 4, 2] = TurnLeftAction', '[0, 4, 3] = TurnLeftAction', '[0, 5, 0] = TurnLeftAction', '[0, 5, 1] = TurnLeftAction', '[0, 5, 2] = TurnLeftAction', '[0, 5, 3] = TurnLeftAction', '[0, 6, 0] = TurnLeftAction', '[0, 6, 1] = TurnLeftAction', '[0, 6, 2] = TurnLeftAction', '[0, 6, 3] = TurnLeftAction', '[0, 7, 0] = TurnLeftAction', '[0, 7, 1] = TurnLeftAction', '[0, 7, 2] = MoveForwardAction', '[0, 7, 3] = TurnLeftAction', '[0, 8, 0] = TurnLeftAction', '[0, 8, 1] = TurnLeftAction', '[0, 8, 2] = TurnLeftAction', '[0, 8, 3] = TurnLeftAction', '[1, 1, 0] = TurnLeftAction', '[1, 1, 1] = TurnLeftAction', '[1, 1, 2] = TurnLeftAction', '[1, 1, 3] = TurnLeftAction', '[2, 1, 0] = TurnLeftAction', '[2, 1, 1] = TurnLeftAction', '[2, 1, 2] = TurnLeftAction', '[2, 1, 3] = TurnLeftAction', '[3, 1, 0] = TurnLeftAction', '[3, 1, 1] = TurnLeftAction', '[3, 1, 2] = TurnLeftAction', '[3, 1, 3] = TurnLeftAction', '[4, 1, 0] = TurnLeftAction', '[4, 1, 1] = TurnLeftAction', '[4, 1, 2] = TurnLeftAction', '[4, 1, 3] = TurnLeftAction', '[5, 1, 0] = TurnLeftAction', '[5, 1, 1] = TurnLeftAction', '[5, 1, 2] = TurnLeftAction', '[5, 1, 3] = TurnLeftAction', '[6, 1, 0] = TurnLeftAction', '[6, 1, 1] = TurnLeftAction', '[6, 1, 2] = TurnLeftAction', '[6, 1, 3] = TurnLeftAction', '[7, 1, 0] = TurnLeftAction', '[7, 1, 1] = TurnLeftAction', '[7, 1, 2] = TurnLeftAction', '[7, 1, 3] = TurnLeftAction', '[8, 1, 0] = TurnLeftAction', '[8, 1, 1] = TurnLeftAction', '[8, 1, 2] = TurnLeftAction', '[8, 1, 3] = TurnLeftAction', '[9, 1, 0] = TurnLeftAction', '[9, 1, 1] = TurnLeftAction', '[9, 1, 2] = TurnLeftAction', '[9, 1, 3] = TurnLeftAction', '[10, 1, 0] = TurnLeftAction', '[10, 1, 1] = TurnLeftAction', '[10, 1, 2] = TurnLeftAction', '[10, 1, 3] = TurnLeftAction', '[11, 1, 0] = TurnLeftAction', '[11, 1, 1] = TurnLeftAction', '[11, 1, 2] = TurnLeftAction', '[11, 1, 3] = TurnLeftAction', '[12, 1, 0] = TurnLeftAction', '[12, 1, 1] = TurnLeftAction', '[12, 1, 2] = TurnLeftAction', '[12, 1, 3] = TurnLeftAction', '[13, 1, 0] = TurnLeftAction', '[13, 1, 1] = TurnLeftAction', '[13, 1, 2] = TurnLeftAction', '[13, 1, 3] = TurnLeftAction', '[14, 1, 0] = TurnLeftAction', '[14, 1, 1] = TurnLeftAction', '[14, 1, 2] = TurnLeftAction', '[14, 1, 3] = TurnLeftAction', '[15, 1, 0] = TurnLeftAction', '[15, 1, 1] = TurnLeftAction', '[15, 1, 2] = TurnLeftAction', '[15, 1, 3] = TurnLeftAction', '[16, 1, 0] = MoveForwardAction', '[16, 1, 1] = MoveForwardAction', '[16, 1, 2] = MoveForwardAction', '[16, 1, 3] = MoveForwardAction', '[16, 2, 0] = TurnLeftAction', '[16, 2, 1] = TurnLeftAction', '[16, 2, 2] = TurnLeftAction', '[16, 2, 3] = TurnLeftAction', '[16, 3, 0] = MoveForwardAction', '[16, 3, 1] = TurnLeftAction', '[16, 3, 2] = TurnLeftAction', '[16, 3, 3] = TurnLeftAction', '[16, 4, 0] = TurnLeftAction', '[16, 4, 1] = TurnLeftAction', '[16, 4, 2] = TurnLeftAction', '[16, 4, 3] = TurnLeftAction', '[16, 5, 0] = TurnLeftAction', '[16, 5, 1] = TurnLeftAction', '[16, 5, 2] = TurnLeftAction', '[16, 5, 3] = TurnLeftAction', '[16, 6, 0] = TurnLeftAction', '[16, 6, 1] = TurnLeftAction', '[16, 6, 2] = TurnLeftAction', '[16, 6, 3] = TurnLeftAction', '[16, 7, 0] = TurnLeftAction', '[16, 7, 1] = TurnLeftAction', '[16, 7, 2] = MoveForwardAction', '[16, 7, 3] = TurnLeftAction', '[16, 8, 0] = TurnLeftAction', '[16, 8, 1] = TurnLeftAction', '[16, 8, 2] = TurnLeftAction', '[16, 8, 3] = TurnLeftAction', 'ENDPOLICY', 'MoveForwardAction = 1', 'ENDE', 'StopAction = 1', 'ENDE', '[[15.0978, -13.0115, -13.4567, -14.9978, 15.3268, -14.6374], [0, 1.72723e-77, 0, 7.90505e-323, 6.91928e-310, 2.6267e-317]]', '[[0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]', '1', '[3.41525, 0.839565, 0.626002, -1.11151, 3.42275, -0.371946]', '[-295.074, -373.936, -522.361, -215.542, -609.893, -287.381]', '70000', '']
    counter = 0
    pmaps = []    
    p = {}
    for stateaction in stateactions:
        counter += 1        
        if stateaction == "ENDPOLICY":
            pmaps.append(p)
            p = {}
            if len(pmaps) == 2: # change this if we ever support more than two patrollers
                break        
        temp = stateaction.split(" = ")
        if len(temp) < 2: continue
        state = temp[0]
        action = temp[1]


        state = state[1 : len(state) - 1]
        pieces = state.split(",")    
        ps = patrol.model.PatrolState(np.array([int(pieces[0]), int(pieces[1]), int(pieces[2])]))
    
        if action == "MoveForwardAction":
            a = patrol.model.PatrolActionMoveForward()
        elif action == "TurnLeftAction":
            a = patrol.model.PatrolActionTurnLeft()
        elif action == "TurnRightAction":
            a = patrol.model.PatrolActionTurnRight()
        elif action == "TurnAroundAction":
            a = patrol.model.PatrolActionTurnAround()
        else:
            a = patrol.model.PatrolActionStop()
            
        p[ps] = a

    if (len(pmaps) < 2):        
        returnval = [mdp.agent.MapAgent(p), mdp.agent.MapAgent(p)]
    else:
        returnval = [mdp.agent.MapAgent(pmaps[0]), mdp.agent.MapAgent(pmaps[1])]
        
    pat = 0
    if (equilibrium is None):
        # now parse the equilibria
        p = {}
        for stateaction in stateactions[counter :]:
            counter += 1
            if stateaction == "ENDE":
                returnval.append(p)
                p = {}
                pat += 1
                if pat == 2: # change this if we ever support more than two patrollers
                    print("at 2nd ende")
                    print(stateactions[counter :])
                    break        
            temp = stateaction.split(" = ")
            if len(temp) < 2: continue
            action = temp[0]
            percent = temp[1]
    
    
            if action == "MoveForwardAction":
                a = patrol.model.PatrolActionMoveForward()
            elif action == "TurnLeftAction":
                a = patrol.model.PatrolActionTurnLeft()
            elif action == "TurnRightAction":
                a = patrol.model.PatrolActionTurnRight()
            elif action == "TurnAroundAction":
                a = patrol.model.PatrolActionTurnAround()
            else:
                a = patrol.model.PatrolActionStop()
    
            p[a] = float(percent)        
    
    else:
        global patroller
            
        if patroller == "ideal":
            p = {}
            p[patrol.model.PatrolActionMoveForward()] = 1.0
            returnval.append(p)
    
            p = {}
            p[patrol.model.PatrolActionMoveForward()] = 1.0
            returnval.append(p)
            
        else:
            p = {}
            if equilibrium[0] == "c":
                p[patrol.model.PatrolActionMoveForward()] = 1.0
            elif equilibrium[0] == "s":
                p[patrol.model.PatrolActionStop()] = 1.0
            else:
                p[patrol.model.PatrolActionTurnAround()] = 1.0
            returnval.append(p)
    
            p = {}
            if equilibrium[1] == "c":
                p[patrol.model.PatrolActionMoveForward()] = 1.0
            elif equilibrium[1] == "s":
                p[patrol.model.PatrolActionStop()] = 1.0
            else:
                p[patrol.model.PatrolActionTurnAround()] = 1.0
            returnval.append(p)

    print("reading params via stateactions. length array, counter:")
    print(len(stateactions[counter:]))
    print(counter)
    
    if len(stateactions[counter:])>0:
        print("len(stateactions[counter:])>counter")

rdiffSpikeTh=0.9

def computeRelDiff(weights2):
    
    global wt_data, normedRelDiff, num_Trajsofar
    
    wt_data=numpy.append(wt_data, [weights2], axis=0)
    # print(wt_data)
    
    for i in range(len(wt_data)-1):
        temp1=wt_data[i][0]/numpy.linalg.norm(wt_data[i][0],ord=2)
        temp2=wt_data[i+1][0]/numpy.linalg.norm(wt_data[i+1][0],ord=2)
        rel_diff1=numpy.linalg.norm(numpy.subtract(temp2,temp1),ord=2)/numpy.linalg.norm(temp1,ord=2)    
        temp3=wt_data[i][1]/numpy.linalg.norm(wt_data[i][1],ord=2)
        temp4=wt_data[i+1][1]/numpy.linalg.norm(wt_data[i+1][1],ord=2)
        rel_diff2=numpy.linalg.norm(numpy.subtract(temp4,temp3),ord=2)/numpy.linalg.norm(temp3,ord=2)
        
        if (rel_diff1 < 0.0001) and (rel_diff2 > 0.0001):
            # one of weights unchanged 
            rel_diff1 = rel_diff2 
        elif (rel_diff2 < 0.0001) and (rel_diff1 > 0.0001):
            rel_diff2 = rel_diff1
        elif (rel_diff1 < 0.0001) and (rel_diff2 < 0.0001):
            #print(str(rel_diff1)+" "+str(rel_diff2)+" "+str("continue"))
            continue
        
        if (rel_diff1 < rdiffSpikeTh) and (num_Trajsofar > 0):
            if (rel_diff2 < rdiffSpikeTh):
                normedRelDiff=min(rel_diff1,rel_diff2)
            else:
                normedRelDiff=rel_diff1
        elif (rel_diff2 < rdiffSpikeTh) and (num_Trajsofar > 1):
            normedRelDiff=rel_diff2
        elif (num_Trajsofar == 0):
            normedRelDiff=min(rel_diff1,rel_diff2)
        else:
            print("\n both differences for current session are spikes")
        
        print("\n rel differences for session, normedRelDiff :"+str(rel_diff1)+" "+str(rel_diff2)+" ")
        print(normedRelDiff)
    return


if __name__ == "__main__":
    
    #computeRelDiff(None,None)
    #parsePolicies(None,None)
    global learned_weights, wt_data
    learned_weights=[[0.0]*6,[0.0]*6] # needed for compute reldiff
    for i in range(2):
        for j in range(6):
            learned_weights[i][j]=random.uniform(.01,.1)
    
    wt_data=numpy.array([learned_weights])
    #wt_data=numpy.append(wt_data, [[[0.071, 0.071, 0.092, 0.084, 0.01, 0.088], [0.09, 0.05, 0.089, 0.09, 0.016, 0.04]]], axis=0)
    #print(wt_data)
    
    wt_data1= numpy.array([[[7.68774532e-002, 2.94449808e-002, 8.36294566e-002, 7.77130630e-002, \
                             4.99845368e-002, 4.55014437e-002], [7.81426536e-002, 4.62514610e-002, \
                                                                 1.51273691e-002, 5.06923683e-002, \
                                                                 2.22849103e-002, 8.12384267e-002]], \
                           [[3.38027, -3.74774, -2.44288, -3.56052, 3.92754, -3.81691], [0.0, 1.72723e-77, \
                                                                                         6.91084e-310, 1.2e-322, \
                                                                                         6.91084e-310, 0.0]], \
                           [[2.12647, 0.484203, -1.70786, -1.96425, 1.9943, -1.58764], [0.0, 1.72723e-77, 0.0, 5e-324, \
                                                                                      6.91386e-310, 0.0]], \
                           [[2.25002, -0.25734, -2.21497, -0.882719, 1.41191, -1.4469], [0.0, 1.72723e-77, 0.0, \
                                                                                       5e-324, 6.90919e-310, 0.0]], \
                           [[2.25002, -0.25734, -2.21497, -0.882719, 1.41191, -1.4469], [0.0, 1.72723e-77, 0.0, \
                                                                                      5e-324, 6.90919e-310, 0.0]]])
    
    #computeRelDiff([[0.071, 0.071, 0.092, 0.084, 0.01, 0.088], [0.09, 0.05, 0.089, 0.09, 0.016, 0.04]])
    computeRelDiff(wt_data1[1])
    computeRelDiff(wt_data1[2])
    computeRelDiff(wt_data1[3])
    computeRelDiff(wt_data1[4])
    computeRelDiff(wt_data1[3])

    