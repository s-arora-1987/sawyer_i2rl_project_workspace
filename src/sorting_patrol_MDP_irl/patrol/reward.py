# -*- coding: utf-8 -*-

from mdp.reward import LinearReward, Reward
import numpy
from model import PatrolState, AttackerState, PatrolActionTurnLeft, PatrolModel
import util.functions
import mdp.simulation
import math

#one last try, add time spent out of view (left or right) to the state and add features on it

class PatrolReward(LinearReward):
    '''
    Feature functions are whether the agent is in
    a box of a specific size
    '''
    
    def __init__(self, totalPatrolArea, farness, observableLow, observableHigh):
        super(PatrolReward,self).__init__()
        self._actions = list( PatrolModel(0.0).A() )
        self._totalPatrolArea = totalPatrolArea
        self._farness = farness
        self._observableLow = observableLow
        self._observableHigh = observableHigh
        
    @property
    def dim(self):
        return 3
    
    def features(self, state, action):
        
        location = state.location[0]
        result = numpy.zeros( self.dim )

        # the reward for moving forward
        result[0] = 1 if action.__class__.__name__ == "PatrolActionForward" and not (location == 0 and state.location[1] == 1 ) and not (location == self._totalPatrolArea -1 and state.location[1] == 0) else 0
#        result[0] = 1 if action.__class__.__name__ == "PatrolActionForward" else 0
        
        # the reward for the total size of patrolled area (greater at the extremes)
#        totalPatrolArea = self._totalPatrolArea + 1
#        halfPatrolArea = totalPatrolArea / 2.0

#        result[1] = 1 - ((location + 1) / (halfPatrolArea + 1)) if location <= halfPatrolArea else ((location - halfPatrolArea + 1) / (halfPatrolArea + 1))

# Ok so, just return the state's idleness number / the max idleness number
        # the reward for the time since the last time the location was visited        
#        result[2] = (state.time * 1.0 - state.lastvisits[state.location[0]]) / (state.time - state.lastvisits.max())
        
        #idea, replace 1 and 2 with the measure of a location's centrality, ie the number of states within a certain distance from them
        if location >= 0 and location < self._totalPatrolArea:
            result[1] = self._farness[location] / max(self._farness)
            result[2] = 1 - result[1]
        else:
            result[1] = 0
            result[2] = 0            

#        result[3] = 1 if location < self._observableLow and  action.__class__.__name__ == "PatrolActionForward" else 0 # in a state to the left of the observable
#        result[4] = 1 if location > self._observableHigh and  action.__class__.__name__ == "PatrolActionForward" else 0 # in a state to the right of the observable

        return result
    
    def __str__(self):
        return 'PatrolReward'
        
    def info(self, model = None):
        result = 'PatrolReward:\n'
        if model is not None:        
        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                    
#            for i in reversed(range(self._map_size[0])):
#                for j in range(self._map_size[1]):
#                    state = GWState( numpy.array( [i,j] ) )
#                    action = a
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
#                result += '\n'
                result += '\n\n'
        return result

class PatrolRewardSpecTurnAround(LinearReward):
    '''
    Feature functions are whether the agent is in
    a box of a specific size
    '''
    
    def __init__(self, turnAroundOne, turnAroundTwo):
        super(PatrolRewardSpecTurnAround,self).__init__()
        self._actions = list( PatrolModel(0.0).A() )
        self._turnAroundOne = turnAroundOne
        self._turnAroundTwo = turnAroundTwo
        
    @property
    def dim(self):
        return 2
    
    def features(self, state, action):
        
        location = state.location[0]
        direction = state.location[1]
        
        result = numpy.zeros( self.dim )

        # the reward for moving forward
        result[0] = 1 if action.__class__.__name__ == "PatrolActionForward" else 0
                
        if action.__class__.__name__ == "PatrolActionTurnAround":
            if location <= self._turnAroundOne and direction == 1:
                result[1] = 1
            elif location >= self._turnAroundTwo and direction == 0:
                result[1] = 1
            else:
                result[1] = -1

        else:
            result[1] = 0

        return result
    
    def __str__(self):
        return 'PatrolReward'
        
    def info(self, model = None):
        result = 'PatrolReward:\n'
        if model is not None:        
        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                    
#            for i in reversed(range(self._map_size[0])):
#                for j in range(self._map_size[1]):
#                    state = GWState( numpy.array( [i,j] ) )
#                    action = a
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
#                result += '\n'
                result += '\n\n'
        return result
                

class PatrolRewardIRLTurnAround(LinearReward):
    '''
    Feature functions are whether the agent is in
    a box of a specific size
    '''
    
    def __init__(self, patrolAreaSize):
        super(PatrolRewardIRLTurnAround,self).__init__()
        self._actions = list( PatrolModel(0.0).A() )
        self._patrolAreaSize = patrolAreaSize

    @property
    def dim(self):
        return self._patrolAreaSize + 1
    
    def features(self, state, action):
        
        location = state.location[0]
        direction = state.location[1]
        
        result = numpy.zeros( self.dim )

        # the reward for moving forward
        result[0] = 1 if action.__class__.__name__ == "PatrolActionForward" else 0
        
        
        if action.__class__.__name__ == "PatrolActionTurnAround":
            if (direction == 1):
                for i in range(self._patrolAreaSize / 2):
                    j = i + 1
                    if location == i and direction == 1:
                        result[j] = 1
                    else:
                        result[j] = -1

            if (direction == 0):
                for i in range(self._patrolAreaSize / 2):
                    j = i + 1 + self._patrolAreaSize / 2
                    if location == i + self._patrolAreaSize / 2 and direction == 0:
                        result[j] = 1
                    else:
                        result[j] = -1

        return result
    
    def __str__(self):
        return 'PatrolReward'
        
    def info(self, model = None):
        result = 'PatrolReward:\n'
        if model is not None:        
        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                    
#            for i in reversed(range(self._map_size[0])):
#                for j in range(self._map_size[1]):
#                    state = GWState( numpy.array( [i,j] ) )
#                    action = a
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
#                result += '\n'
                result += '\n\n'
        return result
        
class PatrolRewardIRL(LinearReward):
    '''
    Feature functions are whether the agent is in
    a box of a specific size
    '''
    
    def __init__(self, totalPatrolArea, farness, observableLow, observableHigh):
        super(PatrolRewardIRL,self).__init__()
        self._actions = list( PatrolModel(0.0).A() )
        self._totalPatrolArea = totalPatrolArea
        self._farness = farness
        self._observableLow = observableLow
        self._observableHigh = observableHigh
        
    @property
    def dim(self):
        return 5
    
    def features(self, state, action):
        
        location = state.location[0]
        result = numpy.zeros( self.dim )

        # the reward for moving forward
        result[0] = 1 if action.__class__.__name__ == "PatrolActionForward" and location > 0 and location < self._totalPatrolArea - 1 else 0
        
        # the reward for the total size of patrolled area (greater at the extremes)
#        totalPatrolArea = self._totalPatrolArea + 1
#        halfPatrolArea = totalPatrolArea / 2.0

#        result[1] = 1 - ((location + 1) / (halfPatrolArea + 1)) if location <= halfPatrolArea else ((location - halfPatrolArea + 1) / (halfPatrolArea + 1))

# Ok so, just return the state's idleness number / the max idleness number
        # the reward for the time since the last time the location was visited        
#        result[2] = (state.time * 1.0 - state.lastvisits[state.location[0]]) / (state.time - state.lastvisits.max())
        
        #idea, replace 1 and 2 with the measure of a location's centrality, ie the number of states within a certain distance from them
        result[1] = 0
        result[2] = 0
       
# lump all unobserveable states into a single feature, denoting how much time the robot spends not in an observable state (total time - observed time)

        result[3] = 1 if location < self._observableLow and  action.__class__.__name__ == "PatrolActionForward" else 0 # in a state to the left of the observable
        result[4] = 1 if location > self._observableHigh and  action.__class__.__name__ == "PatrolActionForward" else 0 # in a state to the right of the observable
       
       
        return result
    
    def __str__(self):
        return 'PatrolReward'
        
    def info(self, model = None):
        result = 'PatrolReward:\n'
        if model is not None:        
        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                    
#            for i in reversed(range(self._map_size[0])):
#                for j in range(self._map_size[1]):
#                    state = GWState( numpy.array( [i,j] ) )
#                    action = a
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
#                result += '\n'
                result += '\n\n'
        return result
        
class PatrolRewardIRL2(LinearReward):
    '''
    Feature functions are whether the agent is in
    a box of a specific size
    '''
    
    def __init__(self, totalPatrolArea, observableLow, observableHigh, expertTrajectory=None):
        super(PatrolRewardIRL2,self).__init__()
        self._actions = list( PatrolModel(0.0).A() )
        self._totalPatrolArea = int(totalPatrolArea)
        self._observableLow = int(observableLow)
        self._observableHigh = int(observableHigh)
        if (expertTrajectory != None):
            self.setSamples(expertTrajectory)
    
    def setSamples(self, t):
        self._trajectory = t
        self._calcDistances()
        
    @property
    def dim(self):
        return 5
    
    def features(self, state, action):
        
        location = state.location[0]
        result = numpy.zeros( self.dim )

        # the reward for moving forward
        if state.placeholder:
            result[0] = 1 if (location >= self._observableLow and location <= self._observableHigh and action.__class__.__name__ == "PatrolActionForward") or \
                             (location < self._observableLow and not state.negObsCount == round(self._leftDist)) or \
                             (location > self._observableHigh and not state.negObsCount == round(self._rightDist)) else 0
        else:
            result[0] = 1 if action.__class__.__name__ == "PatrolActionForward"  else 0
            if (location == 0 and state.location[1] == 1) or (location == self._totalPatrolArea - 1 and state.location[1] == 0):
                result[0] = 0

        # to solve this for the feature expectations of the expert, if the state's location < 0 then treat it as an index into the trajectory.  This allows us to calculate how far out
        # if the location >= 0 then this is the IRL expectations, just calculate for the given state how far it is away from an observable state        
        
        # reward for being x squares away from the last observable position and moving towards/away from the observer
        if state.placeholder:
            result[1] = 1 if (location < self._observableLow and state.negObsCount / 2 < self._leftDist) or \
                             (location > self._observableHigh and state.negObsCount / 2 < self._rightDist) else -.5
        else:
            result[1] = 1 if (location < self._observableLow and location > self._observableLow - self._leftDist and action.__class__.__name__ == "PatrolActionForward") or \
                             (location > self._observableHigh and location < self._observableHigh + self._rightDist and action.__class__.__name__ == "PatrolActionForward") else -.5

        # reward for being x squares away from the last observable position and turning around if moving away
        if state.placeholder:
            result[2] = 1 if (location < self._observableLow and state.negObsCount == round(self._leftDist)) or \
                             (location > self._observableHigh and state.negObsCount == round(self._rightDist)) else 0
        else:
            result[2] = 1 if (location < self._observableLow and state.location[1] == 1 and location == self._observableLow - int(self._leftDist) and action.__class__.__name__ == "PatrolActionTurnAround") or \
                             (location > self._observableHigh and state.location[1] == 0 and location == self._observableHigh + int(self._rightDist) and action.__class__.__name__ == "PatrolActionTurnAround") else 0

        # reward for being observable and moving forward
        result[3] = 1 if location <= self._observableHigh and  location >= self._observableLow and action.__class__.__name__ == "PatrolActionForward" else 0

        # reward for being observable and turning around
        result[4] = 1 if location <= self._observableHigh and  location >= self._observableLow and action.__class__.__name__ == "PatrolActionTurnAround" else 0
       
        return result
        
    def _calcDistances(self):
        leftSum = 0
        leftCount = 0
        rightSum = 0
        rightCount = 0

        for history in self._trajectory:
            highestLeft = 0
            highestRight = 0
            
            for sa in history:
                if sa[0].location[0] < self._observableLow:
                    if highestLeft < sa[0].negObsCount:
                        highestLeft = sa[0].negObsCount
                elif sa[0].location[0] > self._observableHigh:
                    if highestRight < sa[0].negObsCount:
                        highestRight = sa[0].negObsCount
                else:
                    if highestLeft > 0:
                        leftCount += 1
                        leftSum += highestLeft
                        highestLeft = 0
                    elif highestRight > 0:
                        rightCount += 1
                        rightSum += highestRight
                        highestRight = 0
        if (leftCount != 0):
            self._leftDist = (leftSum / 2.0) / leftCount
        else:
            self._leftDist = 0
        
        if (rightCount != 0):
            self._rightDist = (rightSum / 2.0) / rightCount
        else:
            self._rightDist = 0
        print(self._leftDist, self._rightDist)
        
    def __str__(self):
        return 'PatrolReward'
        
    def info(self, model = None):
        result = 'PatrolReward:\n'
        if model is not None:        
        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                    
#            for i in reversed(range(self._map_size[0])):
#                for j in range(self._map_size[1]):
#                    state = GWState( numpy.array( [i,j] ) )
#                    action = a
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
#                result += '\n'
                result += '\n\n'
        return result

class PatrolRewardIRL3(LinearReward):
    '''
    Feature functions are whether the agent is in
    a box of a specific size
    '''
    
    def __init__(self, observableStateActions):
        super(PatrolRewardIRL3,self).__init__()
        self._obsSA = observableStateActions
        self._dim = 3 + len(self._obsSA)
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):

        result = numpy.zeros( self.dim )
        
        isObservable = False
        for (t, sa) in enumerate(self._obsSA):
            if (sa[0] == state and sa[1] == action):
                result[t + 3] = 1
                isObservable = True
                break
            
        if not isObservable:
            result[0] = 1

        result[1] = 1 if state.location[1] == 0 else 0
        result[2] = 1 if state.location[1] == 1 else 0
            

        return result
    
    def __str__(self):
        return 'PatrolRewardIRL3'
        
    def info(self, model = None):
        result = 'PatrolRewardIRL3:\n'
        if model is not None:        
        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                    
#            for i in reversed(range(self._map_size[0])):
#                for j in range(self._map_size[1]):
#                    state = GWState( numpy.array( [i,j] ) )
#                    action = a
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
#                result += '\n'
                result += '\n\n'
        return result 

def convertPatrollerStateToAttacker(state):
	if state.location[0] < 3:
		x1 = state.location[0]
		y1 = 0
	elif state.location[0] >= 11:
		x1 = 2 - (state.location[0] - (11))
		y1 = 9
	else:
		x1 = 2
		y1 = state.location[0] - 2
	
	return AttackerState(numpy.array([x1, y1]), 0)
        
def convertPatrollerStateToAttackerBoydRight(state):
    return AttackerState(numpy.array([8 - state.location[1], state.location[0]]), 0)
    
class AttackerRewardPatrollerTraj(LinearReward):
    
    def __init__(self, destinationState, rewardAtDestination, penaltyForDiscovery, patrollerTrajectories, patrollerStartingStates, patrollerStartingPatrollerStates, patrollerStartingTimes, safeDistance):
        
        super(AttackerRewardPatrollerTraj, self).__init__()
        self._destinationState = destinationState
        self._rewardAtDestination = rewardAtDestination
        self._penaltyForDiscovery = penaltyForDiscovery
        self._patrollerTrajectories = patrollerTrajectories
        self._patrollerStartingPatrollerStates = patrollerStartingPatrollerStates
        self._patrollerStartingStates = patrollerStartingStates
        self._patrollerStartingTimes = patrollerStartingTimes
        self._safeDistance = safeDistance
        
        self.transferStateAction(patrollerTrajectories, patrollerStartingPatrollerStates, patrollerStartingStates, patrollerStartingTimes )
        
    @property
    def dim(self):
        return 1
    
    def features(self, state, action):
        
        result = numpy.zeros((1,))        
        
        if all(state.location == self._destinationState.location):
            result += self._rewardAtDestination
            
        # figure out the prob. that a patroller is able to see the attacker
        # this is when the patroller is one square away and facing the attacker

                        
        for patrollers in self.patrollerPositions:
            if (len(patrollers) > state.time):
                for states in patrollers[state.time]:
                    distance = state.distance(states[0])
                    distanceFactor = (distance / self._safeDistance)*(distance / self._safeDistance)
                    if (distanceFactor > 1.0):
                        distanceFactor = 1.0
                    result += (1.0 - distanceFactor) * states[1] * self._penaltyForDiscovery
        
        return result
    
    def __str__(self):
        return 'AttackerRewardPatrollerTraj'
        
    def info(self, model = None):
        return ""
        
    '''returns the provided trajectory of the patroller's model in the attacker's model'''
    def transferStateAction(self, traj, targetStartStates, attackerModelStartState, t):
        
        # for each patroller, find the target starting state
        # map this to the attacker's gridworld model, actions to the correct actions (will need to keep track of orientation)
        # save the resulting trajectory, stepped into the future t times steps

        returnval = []
        
        for (num, patroller) in enumerate(traj):
            # find the target state
            target = -1

            # need to change this so that it finds the Lastest point in the trajectory that we have seen both patrollers being in the desired states (example, a complete back and forth of an oscillating patroller)
            for timenum in range(len(patroller) - 1 - 30 - t[num] , -1, -1):
                timestep = patroller[timenum]
                for possibleState in timestep:
                    if possibleState[0] == targetStartStates[num]:
                        target = timenum
                        break

            if target >= 0:
                # found the target state, start translating after stepping forward to t
                target += t[num]
                patrollerTraj = []
                while target < len(patroller):
                    
                    curStates = []
                    for p in patroller[target]:
                        if p[0].__class__.__name__ == "PatrolState2":
                            curStates.append( (convertPatrollerStateToAttackerBoydRight(p[0]), p[2]))
                        else:    
                            curStates.append( (convertPatrollerStateToAttacker(p[0]), p[2]))
                        
                        
                    patrollerTraj.append(curStates)
                    target += 1
                    
                returnval.append(patrollerTraj)

        self.patrollerPositions = returnval
    

class FakeReward(Reward):

    def reward(self, state, action):
        return 1

def getDirectionFactor(patrollerState, patrollerOrientation, attackerState):

    if (patrollerState.location[1] == 0) and (patrollerState.location[0] < 2):
        if patrollerOrientation == 0:
            return attackerState.location[0] >= patrollerState.location[0] and attackerState.location[1] <= 1
        else:
            return attackerState.location[0] <= patrollerState.location[0] and attackerState.location[1] <= 1
            
    elif (patrollerState.location[1] == 9 and patrollerState.location[0] < 2):
        if patrollerOrientation == 1:
            return attackerState.location[0] >= patrollerState.location[0] and attackerState.location[1] >= 8
        else:
            return attackerState.location[0] <= patrollerState.location[0] and attackerState.location[1] >= 8
        
    if patrollerOrientation == 1:
        return attackerState.location[1] <= patrollerState.location[1] and attackerState.location[0] == 2
    else:
        return attackerState.location[1] >= patrollerState.location[1] and attackerState.location[0] == 2
        
class AttackerRewardPatrollerPolicy(LinearReward):
    
    def __init__(self, destinationState, rewardAtDestination, penaltyForDiscovery, patrollerModel, patrollerPolicies, patrollerStartingStates, patrollerStartingPatrollerStates, patrollerStartingTimes, safeDistance, maxTime, addDelay = False):
        
        super(AttackerRewardPatrollerPolicy, self).__init__()
        self._destinationState = destinationState
        self._rewardAtDestination = rewardAtDestination
        self._penaltyForDiscovery = penaltyForDiscovery
        self._patrollerModel = patrollerModel
        self._patrollerPolicies = patrollerPolicies
        self._patrollerStartingPatrollerStates = patrollerStartingPatrollerStates
        self._patrollerStartingStates = patrollerStartingStates
        self._patrollerStartingTimes = patrollerStartingTimes
        self._safeDistance = safeDistance
        self._maxTime = maxTime
        self._addDelay = addDelay
        
        self.createTrajectories()
        
    @property
    def dim(self):
        return 1
    
    def features(self, state, action):
        
        result = numpy.zeros((1,))        
        
        if all(state.location == self._destinationState.location):
            result += self._rewardAtDestination
            
        # figure out the prob. that a patroller is able to see the attacker
        # this is when the patroller is one square away and facing the attacker

                        
        for patrollers in self._patrollerPositions:
            if (len(patrollers) > state.time):
                for entry in patrollers[state.time]:
                   
    		        distance = state.distance(entry[0])
    		        distanceFactor = (distance / self._safeDistance)*(distance / self._safeDistance)
    		        if (distanceFactor > 1.0):
    		            distanceFactor = 1.0
#    		        print("State", state, "distaqnce", distanceFactor, "prob", entry[1]) 
    		        directionFactor = 1 if getDirectionFactor(entry[0], entry[2], state) else 0
    		        result += (1.0 - distanceFactor) * entry[1] * self._penaltyForDiscovery * directionFactor
        
        return result
        
    
    def __str__(self):
        return 'AttackerRewardPatrollerTraj'
        
    def info(self, model = None):
        return ""
  
        
    def createTrajectories(self):
        
        returnVal = []
        
        
        for (num, patrollerPolicy) in enumerate(self._patrollerPolicies):
            self._patrollerModel.reward_function = FakeReward()

            if num >= len(self._patrollerStartingStates):
                continue
            traj = []
            
            for i in range(1):
                initial = util.classes.NumMap()
                for s in self._patrollerModel.S():
                    initial[s] = 0
                initial[self._patrollerStartingPatrollerStates[num]] = 1
                initial = initial.normalize()
    
                lastS = self._patrollerStartingPatrollerStates[num]
                for (s,a,r) in mdp.simulation.simulate(self._patrollerModel, patrollerPolicy, initial, self._patrollerStartingTimes[num]):
                    lastS = s
    
                initial = util.classes.NumMap()
                for s in self._patrollerModel.S():
                    initial[s] = 0
                initial[lastS] = 1
                initial = initial.normalize()

                j = 0
                for (s,a,r) in mdp.simulation.simulate(self._patrollerModel, patrollerPolicy, initial, self._maxTime):
                    if s.__class__.__name__ == "PatrolState2":
                        patrollerLocation = convertPatrollerStateToAttackerBoydRight(s)
                    else:   
                        patrollerLocation = convertPatrollerStateToAttacker(s)
                    if (j >= len(traj)):
                        traj.append( [[patrollerLocation, 1.0, s.location[1]], ] )
                    else:
                        add = True
                        for x in traj[j]:
                            if patrollerLocation == x[0] and s.location[1] == x[2]:
                                x[1] += 1
                                add = False
                                break
                        if add:
			    if s.__class__.__name__ == "PatrolState2":
	                        traj[j].append([patrollerLocation, 1.0, s.location[2]])						
			    else:
	                        traj[j].append([patrollerLocation, 1.0, s.location[1]])
                    j += 1

            # normalize                
            for t in traj:
                sum = 0
                
                for entry in t:
                    sum += entry[1]
                
                for entry in t:
                    entry[1] = entry[1] / sum
            returnVal.append(traj)

        # if addDelay, scan through the simulated trajectories, if two patrollers are in the same space, delay.
        #   if two patrollers are headed the same direction, make sure they are at least two positions behind (delay one in back)
        
        if (len(returnVal) > 1) and self._addDelay:
            (hist1, hist2) = self.add_delay(returnVal[0], returnVal[1])
            returnVal = [hist1, hist2]
        self._patrollerPositions = returnVal


    def add_delay(self, hist1, hist2):
        newhist1 = []
        newhist2 = []
        for (i, sa) in enumerate(hist1):
            newhist1.append(sa)
            newhist2.append(hist2[i])
            if sa[0][0] == hist2[i][0][0] or (i > 0 and ( all(hist1[i-1][0][0].location == hist2[i][0][0].location) or all(sa[0][0].location == hist2[i-1][0][0].location) ) ):
                newhist1.append(sa)
                newhist2.append(hist2[i])
                newhist1.append(sa)
                newhist2.append(hist2[i])
                
            
        newhist1 = newhist1[0:len(hist1)]
        newhist2 = newhist2[0:len(hist2)]
        return (newhist1, newhist2)
        
class PatrolReward2(LinearReward):
    '''
    Feature functions are whether the agent is in
    a box of a specific size
    '''
    
    def __init__(self):
        super(PatrolReward2,self).__init__()
        
    @property
    def dim(self):
        return 6

    def setModel(self, model):
        self._model = model
        
    def features(self, state, action):
        
        
        newState = action.apply(state)
        
        if self._model.is_legal(newState):
            moved = True
        else:
            moved = False
            
            
        result = numpy.zeros( self.dim )


        # the reward for moving forward
        result[0] = 1 if action.__class__.__name__ == "PatrolActionMoveForward2" and moved else 0
        

        # the reward for not being in the hallway        
        result[1] = 1 if state.location[1] == 8 or (state.location[1] < 7 and state.location[0] > 0 and state.location[0] < 5) else 0
        
        # distance from top left
        result[2] = math.sqrt((state.location[1] - 0)*(state.location[1] - 0) + (state.location[0] - 0)*(state.location[0] - 0))
        # distance from top right
        result[3] =  math.sqrt((state.location[1] - 8)*(state.location[1] - 8) + (state.location[0] - 0)*(state.location[0] - 0))
        # distance from bottom left
        result[4] =  math.sqrt((state.location[1] - 0)*(state.location[1] - 0) + (state.location[0] - 5)*(state.location[0] - 5))
        # distance from bottom right
        result[5] =  math.sqrt((state.location[1] - 8)*(state.location[1] - 8) + (state.location[0] - 5)*(state.location[0] - 5))

        return result
    
    def __str__(self):
        return 'PatrolReward2'
        
    def info(self, model = None):
        result = 'PatrolReward2:\n'
        if model is not None:        
        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                    
#            for i in reversed(range(self._map_size[0])):
#                for j in range(self._map_size[1]):
#                    state = GWState( numpy.array( [i,j] ) )
#                    action = a
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
#                result += '\n'
                result += '\n\n'
        return result        
								
								

def getDirectionFactor2(patrollerState, patrollerOrientation, attackerState):

	if patrollerOrientation == 0:
		# facing right, row must be less than and column must be same
		return patrollerState.location[0] >= attackerState.location[0] and patrollerState.location[1] == attackerState.location[1]
	if patrollerOrientation == 1:
		# facing up, row must be same than and column must less than
		return patrollerState.location[0] == attackerState.location[0] and patrollerState.location[1] >= attackerState.location[1]
	if patrollerOrientation == 2:
		# facing left, row must be greater than and column must be same
		return patrollerState.location[0] <= attackerState.location[0] and patrollerState.location[1] == attackerState.location[1]
	if patrollerOrientation == 3:
		# facing down, row must be same than and column must greater than
		return patrollerState.location[0] == attackerState.location[0] and patrollerState.location[1] <= attackerState.location[1]
		
								
class AttackerRewardPatrollerPolicy2(LinearReward):
    
    def __init__(self, destinationState, rewardAtDestination, penaltyForDiscovery, patrollerModel, patrollerPolicies, patrollerStartingStates, patrollerStartingPatrollerStates, patrollerStartingTimes, safeDistance, maxTime, addDelay = False):
        
        super(AttackerRewardPatrollerPolicy2, self).__init__()
        self._destinationState = destinationState
        self._rewardAtDestination = rewardAtDestination
        self._penaltyForDiscovery = penaltyForDiscovery
        self._patrollerModel = patrollerModel
        self._patrollerPolicies = patrollerPolicies
        self._patrollerStartingPatrollerStates = patrollerStartingPatrollerStates
        self._patrollerStartingStates = patrollerStartingStates
        self._patrollerStartingTimes = patrollerStartingTimes
        self._safeDistance = safeDistance
        self._maxTime = maxTime
        self._addDelay = addDelay
        
        self.createTrajectories()
        
    @property
    def dim(self):
        return 1
    
    def features(self, state, action):
        
        result = numpy.zeros((1,))        
        
        if all(state.location == self._destinationState.location):
            result += self._rewardAtDestination
            
        # figure out the prob. that a patroller is able to see the attacker
        # this is when the patroller is one square away and facing the attacker

                        
        for patrollers in self._patrollerPositions:
            if (len(patrollers) > state.time):
                for entry in patrollers[state.time]:
                   
    		        distance = state.distance(entry[0])
    		        distanceFactor = (distance / self._safeDistance)*(distance / self._safeDistance)
    		        if (distanceFactor > 1.0):
    		            distanceFactor = 1.0
#    		        print("State", state, "distaqnce", distanceFactor, "prob", entry[1]) 
    		        directionFactor = 1 if getDirectionFactor2(entry[0], entry[2], state) else 0
    		        result += (1.0 - distanceFactor) * entry[1] * self._penaltyForDiscovery * directionFactor
        
        return result
        
    
    def __str__(self):
        return 'AttackerRewardPatrollerTraj'
        
    def info(self, model = None):
        return ""
  
        
    def createTrajectories(self):
        
        returnVal = []
        
        
        for (num, patrollerPolicy) in enumerate(self._patrollerPolicies):
            self._patrollerModel.reward_function = FakeReward()

            if num >= len(self._patrollerStartingStates):
                continue
            traj = []
            
            for i in range(1):
                initial = util.classes.NumMap()
                for s in self._patrollerModel.S():
                    initial[s] = 0
                initial[self._patrollerStartingPatrollerStates[num]] = 1
                initial = initial.normalize()
    
                lastS = self._patrollerStartingPatrollerStates[num]
                for (s,a,r) in mdp.simulation.simulate(self._patrollerModel, patrollerPolicy, initial, self._patrollerStartingTimes[num]):
                    lastS = s
    
                initial = util.classes.NumMap()
                for s in self._patrollerModel.S():
                    initial[s] = 0
                initial[lastS] = 1
                initial = initial.normalize()

                j = 0
                for (s,a,r) in mdp.simulation.simulate(self._patrollerModel, patrollerPolicy, initial, self._maxTime):
                    patrollerLocation = convertPatrollerStateToAttackerBoydRight(s)
                    if (j >= len(traj)):
                        traj.append( [[patrollerLocation, 1.0, s.location[2]], ] )
                    else:
                        add = True
                        for x in traj[j]:
                            if patrollerLocation == x[0] and s.location[2] == x[2]:
                                x[1] += 1
                                add = False
                                break
                        if add:
                            traj[j].append([patrollerLocation, 1.0, s.location[2]])						
                    j += 1

            # normalize                
            for t in traj:
                sum = 0
                
                for entry in t:
                    sum += entry[1]
                
                for entry in t:
                    entry[1] = entry[1] / sum
            returnVal.append(traj)

        # if addDelay, scan through the simulated trajectories, if two patrollers are in the same space, delay.
        #   if two patrollers are headed the same direction, make sure they are at least two positions behind (delay one in back)
        
        if (len(returnVal) > 1) and self._addDelay:
            (hist1, hist2) = self.add_delay(returnVal[0], returnVal[1])
            returnVal = [hist1, hist2]
        self._patrollerPositions = returnVal


    def add_delay(self, hist1, hist2):
        newhist1 = []
        newhist2 = []
        for (i, sa) in enumerate(hist1):
            newhist1.append(sa)
            newhist2.append(hist2[i])
            if sa[0][0] == hist2[i][0][0] or (i > 0 and ( all(hist1[i-1][0][0].location == hist2[i][0][0].location) or all(sa[0][0].location == hist2[i-1][0][0].location) ) ):
                newhist1.append(sa)
                newhist2.append(hist2[i])
#                newhist1.append(sa)
#                newhist2.append(hist2[i])
                
            
        newhist1 = newhist1[0:len(hist1)]
        newhist2 = newhist2[0:len(hist2)]
        return (newhist1, newhist2)