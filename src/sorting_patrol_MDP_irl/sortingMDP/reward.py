# -*- coding: utf-8 -*-

from mdp.reward import LinearReward, Reward
import numpy
from sortingMDP.model import *
import util.functions
import mdp.simulation
import math

class sortingReward1(LinearReward):
    '''
    Feature functions:

    Good onion in conveyor
    Defective onion in conveyor
    Good onion in bin
    Defective onion in bin
    
    '''
    
    def __init__(self,dim):
        super(sortingReward1,self).__init__()
        self._dim = dim
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):        
        result = numpy.zeros( self._dim )
        next_state = action.apply(state)

        if next_state._prediction == 1 and next_state._onion_location == 4:
            result[0] = 1
        if next_state._prediction == 0 and next_state._onion_location == 4:
            result[1] = 1
        if next_state._prediction == 1 and next_state._onion_location == 2:
            result[2] = 1
        if next_state._prediction == 0 and next_state._onion_location == 2:
            result[3] = 1

        # keep inspecting 
        if state._prediction == 2 and next_state._prediction != 2:
            result[4] = 1

        # don't stay still, don't get trapped into cycles 
        if (state._onion_location == next_state._onion_location and\
        state._prediction == next_state._prediction and\
        state._EE_location == next_state._EE_location and\
        state._listIDs_status == next_state._listIDs_status): 
            result[5] = 1

        # keep claiming new onions 
        if not (state._prediction == next_state._prediction) and state._prediction == 2:
            result[6] = 1 

        # changing EE location
        if not (state._EE_location == next_state._EE_location): 
            result[7] = 1

        #empty the list
        if state._listIDs_status == 1 and next_state._listIDs_status == 0:
            result[8] = 1

        #create the list
        if state._listIDs_status == 2 and next_state._listIDs_status != 2:
            result[9] = 1 

        #make empty list unavailable
        if state._listIDs_status == 0 and next_state._listIDs_status == 2:
            result[10] = 1 

        # "keeping prediction from unknown in current state to unknown in next state" cycle
        if state._prediction == 2 and next_state._prediction == 2:
            result[11] = 1

        # inspecting an onion that is already inspected 
        if state._prediction != 2 and next_state._EE_location == 1: 
            result[12] = 1

        # pick a good (pick-place-pick cycles)
        if state._prediction == 1 and next_state._EE_location == 3: 
            result[13] = 1

        return result
    
    def __str__(self):
        return 'sortingReward'
        
    def info(self, model = None):
        result = 'sortingReward:\n'
        if model is not None:        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
                result += '\n\n'
        return result


class sortingReward2(LinearReward):
    '''
    Feature functions:

    Good onion in conveyor
    Defective onion in conveyor
    Good onion in bin
    Defective onion in bin
    
    '''
    
    def __init__(self,dim):
        super(sortingReward2,self).__init__()
        self._dim = dim
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):        
        result = numpy.zeros( self._dim )
        next_state = action.apply(state)

        if next_state._prediction == 1 and next_state._onion_location == 4:
            result[0] = 1
        if next_state._prediction == 0 and next_state._onion_location == 4:
            result[1] = 1
        if next_state._prediction == 1 and next_state._onion_location == 2:
            result[2] = 1
        if next_state._prediction == 0 and next_state._onion_location == 2:
            result[3] = 1

        # stay still
        if (state._onion_location == next_state._onion_location and\
        state._prediction == next_state._prediction and\
        state._EE_location == next_state._EE_location and\
        state._listIDs_status == next_state._listIDs_status): 
            result[4] = 1

        # inspect
        if not (state._prediction == next_state._prediction) and state._prediction == 2:
            result[5] = 1 

        # creating new list
        if state._listIDs_status == 2 and next_state._listIDs_status != 2:
            result[6] = 1 

        # picking already placed onion 
        if state._onion_location == 4 and next_state._EE_location == 3: 
            result[7] = 1

        return result
    
    def __str__(self):
        return 'sortingReward2'
        
    def info(self, model = None):
        result = 'sortingReward2:\n'
        if model is not None:        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
                result += '\n\n'
        return result

class sortingReward3(LinearReward):
    '''
    
    '''
    def __init__(self,dim):
        super(sortingReward3,self).__init__()
        self._dim = dim
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):        
        result = numpy.zeros( self._dim )
        next_state = action.apply(state)

        '''
        Feature functions:

        Good onion in conveyor
        Defective onion in conveyor
        Good onion in bin
        Defective onion in bin
        
        '''
        if next_state._prediction == 1 and next_state._onion_location == 4:
            result[0] = 1
        if next_state._prediction == 0 and next_state._onion_location == 4:
            result[1] = 1
        if next_state._prediction == 1 and next_state._onion_location == 2:
            result[2] = 1
        if next_state._prediction == 0 and next_state._onion_location == 2:
            result[3] = 1

        # stay still
        if (state._onion_location == next_state._onion_location and\
        state._prediction == next_state._prediction and\
        state._EE_location == next_state._EE_location and\
        state._listIDs_status == next_state._listIDs_status): 
            result[4] = 1

        # inspect after pick
        if not (state._prediction == next_state._prediction) and state._prediction == 2\
            and state._onion_location != 0:
            result[5] = 1 

        # creating new list
        if state._listIDs_status == 2 and next_state._listIDs_status != 2:
            result[6] = 1 

        # picking already placed onion 
        if state._onion_location == 4 and next_state._EE_location == 3: 
            result[7] = 1

        # inspect without pick
        if not (state._prediction == next_state._prediction) and state._prediction == 2\
            and state._onion_location == 0:
            result[8] = 1 

        # place an uninspected in bin
        if (state._prediction == 2 and next_state._EE_location == 2): 
            result[9] = 1

        return result
    
    def __str__(self):
        return 'sortingReward3'
        
    def info(self, model = None):
        result = 'sortingReward3:\n'
        if model is not None:        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
                result += '\n\n'
        return result


class sortingReward4(LinearReward):
    '''
    
    '''
    def __init__(self,dim):
        super(sortingReward4,self).__init__()
        self._dim = dim
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):        
        result = numpy.zeros( self._dim )
        next_state = action.apply(state)

        '''
        Feature functions:

        // good placed on belt
        // not placing bad on belt
        // not placing good in bin
        // bad placed in bin
        
        '''
        if next_state._prediction == 1 and next_state._onion_location == 4:
            result[0] = 1
        if next_state._prediction == 0 and next_state._onion_location != 4:
            result[1] = 1
        if next_state._prediction == 1 and next_state._onion_location != 2:
            result[2] = 1
        if next_state._prediction == 0 and next_state._onion_location == 2:
            result[3] = 1

        # not staying still
        if not (state._onion_location == next_state._onion_location and\
        state._prediction == next_state._prediction and\
        state._EE_location == next_state._EE_location and\
        state._listIDs_status == next_state._listIDs_status): 
            result[4] = 1

        # inspect after pick 
        if not (state._prediction == next_state._prediction) and state._prediction == 2\
            and state._onion_location != 0:
            result[5] = 1 

        # creating new list 
        if state._listIDs_status == 2 and next_state._listIDs_status != 2:
            result[6] = 1 

        # not picking already placed onion 
        if state._onion_location == 4 and next_state._EE_location != 3: 
            result[7] = 1

        # inspect without pick 
        if not (state._prediction == next_state._prediction) and state._prediction == 2\
            and state._onion_location == 0:
            result[8] = 1 

        # not placing uninspected in bin
        if (state._prediction == 2 and next_state._EE_location != 2): 
            result[9] = 1

        return result
    
    def __str__(self):
        return 'sortingReward4'
        
    def info(self, model = None):
        result = 'sortingReward4:\n'
        if model is not None:        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
                result += '\n\n'
        return result


class sortingReward5(LinearReward):
    '''
    
    '''
    def __init__(self,dim):
        super(sortingReward5,self).__init__()
        self._dim = dim
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):        
        result = numpy.zeros( self._dim )
        next_state = action.apply(state)

        '''
        Feature functions:

        // good placed on belt
        // not placing bad on belt
        // not placing good in bin
        // bad placed in bin
        
        '''
        if next_state._prediction == 1 and next_state._onion_location == 4:
            result[0] = 1
        if next_state._prediction == 0 and next_state._onion_location != 4:
            result[1] = 1
        if next_state._prediction == 1 and next_state._onion_location != 2:
            result[2] = 1
        if next_state._prediction == 0 and next_state._onion_location == 2:
            result[3] = 1

        # not staying still
        if not (state._onion_location == next_state._onion_location and\
        state._prediction == next_state._prediction and\
        state._EE_location == next_state._EE_location and\
        state._listIDs_status == next_state._listIDs_status): 
            result[4] = 1

        # claim new onion from belt 
        if state._prediction == 2 and \
        (state._onion_location == 2 or state._onion_location == 4)\
        and state._EE_location == 3: 
            result[5] = 1 

        # create list 
        if state._listIDs_status == 0 and next_state._listIDs_status == 1: 
            result[6] = 1 

        # picking an onion with unknown prediction 
        if state._onion_location == 0 and state._prediction == 2 \
        and (next_state._prediction == 2 and next_state._EE_location == 3): 
            result[7] = 1

        return result
    
    def __str__(self):
        return 'sortingReward5'
        
    def info(self, model = None):
        result = 'sortingReward5:\n'
        if model is not None:        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
                result += '\n\n'
        return result


class sortingReward6(LinearReward):
    '''
    
    '''
    def __init__(self,dim):
        super(sortingReward6,self).__init__()
        self._dim = dim
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):        
        result = numpy.zeros( self._dim )
        next_state = action.apply(state)

        '''
        Feature functions:

        // good placed on belt
        // not placing bad on belt
        // not placing good in bin
        // bad placed in bin
        
        '''
        if next_state._prediction == 1 and next_state._onion_location == 4:
            result[0] = 1
        if next_state._prediction == 0 and next_state._onion_location != 4:
            result[1] = 1
        if next_state._prediction == 1 and next_state._onion_location != 2:
            result[2] = 1
        if next_state._prediction == 0 and next_state._onion_location == 2:
            result[3] = 1

        # not staying still
        if not (state._onion_location == next_state._onion_location and\
        state._prediction == next_state._prediction and\
        state._EE_location == next_state._EE_location and\
        state._listIDs_status == next_state._listIDs_status): 
            result[4] = 1

        # claim new onion from belt 
        if state._prediction == 2 and \
        (state._onion_location == 2 or state._onion_location == 4)\
        and state._EE_location == 3: 
            result[5] = 1 

        # create list 
        if state._listIDs_status == 0 and next_state._listIDs_status == 1: 
            result[6] = 1 

        # picking an onion with unknown prediction 
        if state._onion_location == 0 and state._prediction == 2 \
        and (next_state._prediction == 2 and next_state._EE_location == 3): 
            result[7] = 1

        # picking an onion with known pred - blemished  
        if state._onion_location == 0 and state._prediction == 0 \
        and (next_state._prediction == 0 and next_state._EE_location == 3): 
            result[8] = 1

        # empty list 
        if state._listIDs_status == 1 and next_state._listIDs_status == 0: 
            result[9] = 1 

        # inspect picked onion 
        if state._onion_location == 3 and state._prediction == 2 and\
        next_state._prediction != 2: 
            result[10] = 1 

        return result
    
    def __str__(self):
        return 'sortingReward6'
        
    def info(self, model = None):
        result = 'sortingReward6:\n'
        if model is not None:        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
                result += '\n\n'
        return result

class sortingReward7WPlaced(LinearReward):
        
    def __init__(self,dim):
        super(sortingReward7WPlaced,self).__init__()
        self._dim = dim
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):        
        result = numpy.zeros( self._dim )
        next_state = action.apply(state)

        '''
        Feature functions:

        // good placed on belt
        // not placing bad on belt
        // not placing good in bin
        // bad placed in bin
        
        '''
        if state._prediction == 1 and next_state._onion_location == 4:
            result[0] = 1
        if state._prediction == 0 and next_state._onion_location != 4:
            result[1] = 1
        if state._prediction == 1 and next_state._onion_location != 2:
            result[2] = 1
        if state._prediction == 0 and next_state._onion_location == 2:
            result[3] = 1

        # not staying still
        if not (state._onion_location == next_state._onion_location and\
        state._prediction == next_state._prediction and\
        state._EE_location == next_state._EE_location and\
        state._listIDs_status == next_state._listIDs_status): 
            result[4] = 1

        # claim new onion from belt 
        if (next_state._prediction == 2 and \
        next_state._onion_location == 0): 
            result[5] = 1 

        # create list 
        if state._listIDs_status == 0 and next_state._listIDs_status == 1: 
            result[6] = 1 

        # picking an onion with unknown prediction 
        if state._onion_location == 0 and state._prediction == 2 \
        and (next_state._prediction == 2 and next_state._EE_location == 3): 
            result[7] = 1

        # picking an onion with known pred - blemished  
        if state._onion_location == 0 and state._prediction == 0 \
        and (next_state._prediction == 0 and next_state._EE_location == 3): 
            result[8] = 1

        # empty list 
        if state._listIDs_status == 1 and next_state._listIDs_status == 0: 
            result[9] = 1 

        # inspect picked onion 
        if state._onion_location == 3 and state._prediction == 2 and\
        next_state._prediction != 2: 
            result[10] = 1 

        return result
    
    def __str__(self):
        return 'sortingReward7'
        
    def info(self, model = None):
        result = 'sortingReward7:\n'
        if model is not None:        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
                result += '\n\n'
        return result

class sortingReward7(LinearReward):
    '''
    
    '''
    def __init__(self,dim):
        super(sortingReward7,self).__init__()
        self._dim = dim
        
    @property
    def dim(self):
        return self._dim
    
    def features(self, state, action):        
        result = numpy.zeros( self._dim )
        next_state = action.apply(state)

        '''
        Feature functions:

        // good placed on belt
        // not placing bad on belt
        // not placing good in bin
        // bad placed in bin
        
        '''
        if state._prediction == 1 and next_state._onion_location == 0:
            result[0] = 1
        if state._prediction == 0 and next_state._onion_location != 0:
            result[1] = 1
        if state._prediction == 1 and next_state._onion_location != 2:
            result[2] = 1
        if state._prediction == 0 and next_state._onion_location == 2:
            result[3] = 1

        # not staying still
        if not (state._onion_location == next_state._onion_location and\
        state._prediction == next_state._prediction and\
        state._EE_location == next_state._EE_location and\
        state._listIDs_status == next_state._listIDs_status): 
            result[4] = 1

        # claim new onion from belt 
        if (next_state._prediction == 2 and \
        next_state._onion_location == 0): 
            result[5] = 1 

        # create list 
        if state._listIDs_status == 0 and next_state._listIDs_status == 1: 
            result[6] = 1 

        # picking an onion with unknown prediction 
        if state._onion_location == 0 and state._prediction == 2 \
        and (next_state._prediction == 2 and next_state._EE_location == 3): 
            result[7] = 1

        # picking an onion with known pred - blemished  
        if state._onion_location == 0 and state._prediction == 0 \
        and (next_state._prediction == 0 and next_state._EE_location == 3): 
            result[8] = 1

        # empty list 
        if state._listIDs_status == 1 and next_state._listIDs_status == 0: 
            result[9] = 1 

        # inspect picked onion 
        if state._onion_location == 3 and state._prediction == 2 and\
        next_state._prediction != 2: 
            result[10] = 1 

        return result
    
    def __str__(self):
        return 'sortingReward7'
        
    def info(self, model = None):
        result = 'sortingReward7:\n'
        if model is not None:        
            for a in self._actions:
                result += str(a) + '\n'
                for s in model.S():
                     result += '|{: 4.4f}|'.format(self.reward(s, a))
                result += '\n\n'
        return result
