# -*- coding: utf-8 -*-
import numpy
import random
from mdp.reward import Reward

class BayesianPatrolReward(Reward):
    
    def __init__(self, numStates, gridSize, initVector=None):
        super(BayesianPatrolReward, self)
        self._numStates = numStates
        self._gridSize = gridSize
        if initVector == None:
            self._values = numpy.random.random((numStates))
        else:
            self._values = initVector
        
       
    @property
    def dim(self):
        return self._numStates
        
    def reward(self, state, action):
        #have to assume the state is a PatrolState here
        
        stateIdx =  state.location[0] + (state.location[0] if state.location[1] == 1 else 0 )
        
        return self._values[stateIdx]
        
    def randNeighbor(self):
        whichOne = random.randint(0, self._numStates - 1)
        newvector = numpy.array(self._values)
        if (random.randint(0,1) == 0):
            newvector[whichOne] += self._gridSize
        else:
            newvector[whichOne] -= self._gridSize            
            
        return BayesianPatrolReward(self._numStates, self._gridSize, newvector)

#        newvector = numpy.array(self._values)
#        for i in range(self._numStates):
#            if (random.randint(0,1) == 0):
#                newvector[i] += self._gridSize
#            else:
#                newvector[i] -= self._gridSize            
#            
#        return BayesianPatrolReward(self._numStates, self._gridSize, newvector)

    def __str__(self):
        return 'BayesianPatrolReward'
        
    def info(self, model = None):
        result = 'BayesianPatrolReward: '
        result += "["
        for i in self._values:
            result += str(i) + ", "
        result += "]\n"
        
        return result
        