# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/saurabharora/catkin_ws/src/navigation_irl')

from mdp.model import *
from util.classes import NumMap
from numpy import array, all
import math
import numpy as np

num_objects = 8.0

class sortingState(State):
    def __init__(self, onion_location = 0, prediction = -1, EE_location = 1, listIDs_status = 1):
        '''
        onion_location: 
        on the conveyor, or 0
        in front of eye, or 1
        in bin or 2
        at home after begin picked or 3 (in superficial inspection, onion is picked and placed)
        placed_on_conveyor 4
        prediction:
        1 good
        0 bad
        2 unkonwn before inspection
        EE_location:
        conv 0
        inFront 1
        bin 2 
        at home 3
        listIDs_status: 
        0 empty
        1 not empty
        2 list not available (because rolling hasn't happened for current set of onions)
        ''' 
        
        self._onion_location = onion_location
        self._prediction = prediction
        self._EE_location = EE_location
        self._listIDs_status = listIDs_status
        self._hash_array = [5,3,4,3]

    @property
    def onion_location(self):
        return self._location_chosen_onion

    @onion_location.setter
    def onion_location(self, loc):
        self._location_chosen_onion = loc

    @property
    def prediction(self):
        return self._prediction

    @prediction.setter
    def onion_location(self, pred):
        self._prediction = pred

    @property
    def EE_location(self):
        return self._prediction

    @EE_location.setter
    def EE_location(self, ee_loc):
        self._EE_location = ee_loc

    @property
    def listIDs_status(self):
        return self._listIDs_status

    @listIDs_status.setter
    def listIDs_status(self, listIDs_status):
        self._listIDs_status = listIDs_status

    def __str__(self):
        return 'State: [{}, {}, {}, {}]'.format(self._onion_location, self._prediction,\
         self._EE_location, self._listIDs_status)
    
    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        try:
            return (self._onion_location == other._onion_location \
            and self._prediction == other._prediction\
            and other._EE_location == self._EE_location\
            and other._listIDs_status == self._listIDs_status) # epsilon error
        except Exception:
            print "Exception in __eq__(self, other)"
            return False
    
    def __hash__(self):
        row_major = self._onion_location+self._hash_array[0]*(self._prediction+self._hash_array[1]*(self._EE_location+self._hash_array[2]*self._listIDs_status))
        return (row_major).__hash__()
        # return row_major
        # row_major = self._onion_location+self._hash_array[0]*(self._prediction+self._hash_array[1]*self._EE_location)
        # return (row_major).__hash__()
        # return (self._onion_location,self._prediction,self._EE_location,self._listIDs_status).__hash__() 
        # return hash((self._onion_location,self._prediction,self._EE_location))

class sortingModel(Model):
    
    _actions = None
    _p_fail = None
    _terminal = None

    def __init__(self, p_fail=0.05, terminal=sortingState(-1,-1,-1) ):
        super(sortingModel,self)
                
        self._actions = [InspectAfterPicking(),InspectWithoutPicking(),\
        Pick(),PlaceOnConveyor(),PlaceInBin(),GoHome(),ClaimNewOnion(),\
        ClaimNextInList()] 
        self._p_fail = float(p_fail) 
        self._terminal = terminal

        result = []
        onion_locations = [0,1,2,3,4]
        predictions = [0,1,2] 
        EE_locations = [0,1,2,3]
        listIDs_status_opts = [0,1,2]

        for ol in onion_locations:
            for pr in predictions:
                for el in EE_locations:
                    for le in listIDs_status_opts:
                        # invalid if onion infront/ athome and EE loc not same
                        if (ol == 1 and el != 1) or \
                        (ol == 3 and el != 3): # (ol == 2 and el != 2) (ol == 4 and el != 0)
                            pass
                        else:
                            result.append( sortingState(ol,pr,el,le) )
        
        self._states = result
        
    def S(self):
        """All states in the MDP"""
        return self._states
        
    def A(self,state=None):
        # Pick, InspectWithoutPicking can't be done if onion is already picked (home-pose 3 or infront 1), 
        # PlaceOnConveyor,PlaceInBin, InspectAfterPicking can't be done if onion is not picked yet ()
        # onion prediction must always be unknown before inspection and known after inspection
        #
        
        res = []
        if state._onion_location == 1 or state._onion_location == 3: 
        # home or front, onion is picked
            # res = [InspectAfterPicking(),PlaceOnConveyor(),PlaceInBin()] 
            if (state._listIDs_status == 2) :
                if (state._onion_location == 1): 
                    # no inspect after inspecting
                    res = [PlaceOnConveyor(), PlaceInBin()]
                else: 
                    res = [InspectAfterPicking(), PlaceInBin()]
                
            else: 
                # do not re-inspect if list exists
                res = [ PlaceInBin()]

        if state._onion_location == 0 or state._onion_location == 4: 
        # on conveyor (not picked yet or already placed) 
            if state._listIDs_status == 2: # can not claim from list if list not available 
                res = [Pick(),ClaimNewOnion(),InspectWithoutPicking()] 
            else: # can not create list again if a list is already available 
                res = [Pick(),ClaimNextInList()]# if we allow ClaimNewOnion with a list available
                # then it will do *,0,2,1 ClaimNewOnion 0,2,2,1 ClaimNextInList 0,0,2,1
                # and will assume onion is bad without inspection
        if state._onion_location == 2: # in bin, can't pick from bin because not reachable 
            if state._listIDs_status == 2:# sorter can claim new onion only when a list of predictions has not been pending 
                res = [ClaimNewOnion(),InspectWithoutPicking()] 
            else:
                res = [ClaimNextInList()]# if we allow ClaimNewOnion with a list available
                # then it will do *,0,2,1 ClaimNewOnion 0,2,2,1 ClaimNextInList 0,0,2,1
                # and will assume onion is bad without inspection
        return res
   
    def is_terminal(self, state):
        return False
        '''returns whether or not good onion is on conv (0) and bad onion is in bin (2)'''
        # test = (state._prediction == 1 and state._onion_location == 0) or \
        #        (state._prediction == 0 and state._onion_location == 2)
        # return test

    def T(self,state,action):
        """Returns a function state -> [0,1] for probability of next state
        given currently in state performing action"""
        result = NumMap()
        s_p = action.apply(state)
        if not self.is_legal(s_p) or s_p.__eq__(state):
            result[state] = 1
        else:  
            result[s_p] = 1 - self._p_fail
            result[state] = self._p_fail

        return result 
                
    def is_legal(self,state):
        # if onion is unknown, then it should not be in bin 
        # return not (state._prediction == 2 and state._onion_location == 2)
        return True
    
    def generate_matrix(self,dict_stateEnum,dict_actEnum):

        acts = [InspectAfterPicking(),PlaceOnConveyor(),PlaceInBin(),\
        Pick(),ClaimNewOnion(),InspectWithoutPicking(),ClaimNextInList()] 

        for s in self._states:
            for a in acts:
                sum_t = 0
                for (ns,p) in self.T(s,a).items():
                    # print sum_t 
                    sum_t = sum_t + p
                if not (sum_t == 1):                    
                    print "row doesn't sum to 1 "

        # exit(0)

        # dummy_states = self._states
        # dummy_states.append(sortingState(-1,-1,-1,-1))

        # acts = [InspectAfterPicking(),PlaceOnConveyor(),PlaceInBin(),\
        # Pick(),ClaimNewOnion(),InspectWithoutPicking(),ClaimNextInList()] 
        # dict_stateEnum = {}
        # ind = 0
        # for s in dummy_states:
        #     ind = ind +1
        #     dict_stateEnum[ind] = s

        # dict_actEnum = {}
        # ind = 0
        # for a in acts:
        #     ind = ind +1
        #     dict_actEnum[ind] = a

        # exit(0)

        T = {}
        num_elements = 0
        for ind1 in range(1,len(dict_actEnum)+1):
            a = dict_actEnum[ind1]
            T[ind1] = {}
            for ind2 in range(1,len(dict_stateEnum)+1):
                s = dict_stateEnum[ind2]
                T[ind1][ind2] = {}
                sum_t = 0
                trans_dict = self.T(s,a)
                # print "\n\nstarted here"
                # print "s:"+str(s)+",a:"+str(a)+", T(a,s):"+str(trans_dict)\
                # +", self.T(s,a).keys() "+str(trans_dict.keys())
                # print " (a in self.A(s))?"+str((a in self.A(s)))+" (a in self.A(s))?"+str((a in self.A(s)))
                for ind3 in range(1,len(dict_stateEnum)+1):
                    ns = dict_stateEnum[ind3]
                    # if ns._onion_location == -1:
                    #     print "ns:"+str(ns)
                    # T[impossible][a][impossible] = 1, T[impossible][a][ns] = 0 for any ns and a
                    # T[s][a(not allowed)][impossible] = 1, T[s][a(not allowed)][ns] = 0 for any s and a and ns
                    # T[s][a(allowed)][impossible] = 0
                    num_elements = num_elements+1
                    if s._onion_location == -1:
                        # if current state is impossible
                        # for every action, impossible state to impossible state
                        if ns._onion_location == -1: 
                            # print "current and next state is impossible. s,a:"+str((s,a))
                            T[ind1][ind2][ind3] = 1
                            sum_t = sum_t + T[ind1][ind2][ind3]
                            break
                        else:
                            T[ind1][ind2][ind3] = 0
                    else:
                        if a in self.A(s):
                            # action allowed
                            if ns in trans_dict.keys():
                                # print "ns in self.T(s,a).keys(), " +str((ns in trans_dict.keys()))
                                T[ind1][ind2][ind3] = trans_dict[ns]
                            else:
                                T[ind1][ind2][ind3] = 0
                        else:
                            # action not allowed
                            if ns._onion_location == -1: 
                                # if action not allowed and next state is impossible
                                # print "if action not allowed and next state is impossible. s,a:"+str((s,a))
                                T[ind1][ind2][ind3] = 1
                                sum_t = sum_t + T[ind1][ind2][ind3]
                                break
                            else:
                                T[ind1][ind2][ind3] = 0

                    sum_t = sum_t + T[ind1][ind2][ind3]
                    # if ns._onion_location == -1:
                        # print "sum_t:"+str(sum_t)

                if not (sum_t == 1):
                    print "row doesn't sum to 1 for a,s:"

                    ol = None
                    pr = None
                    el = None
                    ls = None

                    if s._onion_location == 0:
                        ol = 'Onconveyor'
                    elif s._onion_location == 1:
                        ol = 'Infront'
                    elif s._onion_location == 2:
                        ol = 'Inbin'
                    elif s._onion_location == 4:
                        ol = 'Placed'
                    else:
                        ol = 'Picked/AtHomePose'

                    if s._prediction == 0:
                        pr = 'bad'
                    elif s._prediction == 1:
                        pr = 'good'
                    else:
                        pr = 'unknown'

                    if s._EE_location == 0:
                        el = 'Onconveyor'
                    elif s._EE_location == 1:
                        el = 'Infront'
                    elif s._EE_location == 2:
                        el = 'Inbin'
                    else:
                        el = 'Picked/AtHomePose'

                    if s._listIDs_status == 0:
                        ls = 'Empty'
                    elif s._listIDs_status == 1:
                        ls = 'NotEmpty' 
                    else:
                        ls = 'Unavailable' 

                    print str((s,a,(ol,pr,el,ls)))
                    print T[ind1][ind2]
                    exit(0)
                    # print str((ol,pr,el,ls)

        return (T,num_elements,dict_actEnum,dict_stateEnum) #,order_s,order_a,order_ns)

    def __str__(self):
        format = 'PatrolModel [p_fail={},terminal={}]'
        return format.format(self._p_fail, self._terminal)
    
    def info(self):
        result = [str(self) + '\n']
        return ''.join(result)

class sortingModelbyPSuresh(sortingModel):
    
    def __init__(self, p_fail=0.05, terminal=sortingState(-1,-1,-1) ):
        super(sortingModelbyPSuresh,self).__init__(p_fail, terminal)

        # print("__init__(self ", self._p_fail)
        self._p_fail = self._p_fail

        result2 = []
        onion_locations = [0,1,2,3,4]
        predictions = [0,1,2] 
        EE_locations = [0,1,2,3]
        listIDs_status_opts = [0,1,2]

        for ol in onion_locations:
            for pr in predictions:
                for el in EE_locations:
                    for le in listIDs_status_opts:
                        # invalid if onion infront/ athome and EE loc not same
                        if (ol == 1 and el != 1) or \
                        (ol == 3 and el != 3) or\
                        (ol == 2 and el != 2): # (ol == 4 and el != 0)
                            pass
                        else:
                            result2.append( sortingState(ol,pr,el,le) )
        
        self._states = result2
        print("number of states in mdp ",len(self._states))

    def A(self,state=None):
        ''' {0: 'InspectAfterPicking', 1: 'PlaceOnConveyor', 
        2: 'PlaceInBin', 3: 'Pick', 
        4: 'ClaimNewOnion', 5: 'InspectWithoutPicking', 
        6: 'ClaimNextInList'}'''

        res = []
        if state._onion_location == 0: 
            # on conveyor (not picked yet) 
            if state._listIDs_status == 2: 
                res = [Pick()] 
            elif state._listIDs_status == 0:  
                # res = [InspectWithoutPicking()]
                res = [InspectWithoutPicking(),Pick()]
            else:
                # list not empty
                if state._prediction == 2:
                    res = [ClaimNextInList()]
                else:
                    res = [Pick()]

        elif state._onion_location == 1:
            ##
            # if result of inspection is good, then placeinbin is not allowed 
            # if result of inspection is bad, then placeonconveyor is not allowed 
            ##
            if state._prediction == 0:
                res = [PlaceInBin()]
            elif state._prediction == 1:
                res = [PlaceOnConveyor()]
            else: 
                res = [InspectAfterPicking()]

        elif state._onion_location == 2:
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                res = [InspectWithoutPicking()]
            else:
                res = [ClaimNextInList()]

        elif state._onion_location == 3:
            if state._prediction == 2: 
                res = [InspectAfterPicking()] 
            elif state._prediction == 0: 
                res = [PlaceInBin()] 
            else: 
                res = [PlaceOnConveyor()] 

        else: # onion placed
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0: 
                res = [InspectWithoutPicking()] 
            else: 
                res = [ClaimNextInList()] 

        return res
   
class sortingModelbyPSuresh2(sortingModel):
    
    def __init__(self, p_fail=0.05, terminal=sortingState(-1,-1,-1) ):
        super(sortingModelbyPSuresh2,self).__init__(p_fail, terminal)

        # print("__init__(self ", self._p_fail)
        self._p_fail = self._p_fail

        statesList = [[ 0, 2, 0, 0],\
        [ 3, 2, 3, 0],\
        [ 1, 0, 1, 2],\
        [ 2, 2, 2, 2],\
        [ 0, 2, 2, 2],\
        [ 3, 2, 3, 2],\
        [ 1, 1, 1, 2],\
        [ 4, 2, 0, 2],\
        [ 0, 0, 0, 1],\
        [ 3, 0, 3, 1],\
        [ 2, 2, 2, 1],\
        [ 0, 0, 2, 1],\
        [ 2, 2, 2, 0],\
        [0, 2, 0, 2],\
        [0, 2, 2, 0],\
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],\
        [3,1,3,0],[0,0,1,1],[0,0,3,1]]

        # rolling states start from [ 0, 0, 0, 1],\ and last two are needed for completing value iteration

        result2 = []
        for ls in statesList:
            ol,pr,el,le = ls[0], ls[1], ls[2], ls[3]
            result2.append( sortingState(ol,pr,el,le) )

        # onion_locations = [0,1,2,3,4]
        # predictions = [0,1,2] 
        # EE_locations = [0,1,2,3]
        # listIDs_status_opts = [0,1,2]

        # for ol in onion_locations:
        #     for pr in predictions:
        #         for el in EE_locations:
        #             for le in listIDs_status_opts:
        #                 # invalid if onion infront/ athome and EE loc not same
        #                 if (ol == 1 and el != 1) or \
        #                 (ol == 3 and el != 3) or\
        #                 (ol == 2 and el != 2): # (ol == 4 and el != 0)
        #                     pass
        #                 else:
        #                     result2.append( sortingState(ol,pr,el,le) )
        
        self._states = result2
        print("number of states in mdp ",len(self._states))


    def A(self,state=None):
        ''' {0: 'InspectAfterPicking', 1: 'PlaceOnConveyor', 
        2: 'PlaceInBin', 3: 'Pick', 
        4: 'ClaimNewOnion', 5: 'InspectWithoutPicking', 
        6: 'ClaimNextInList'}'''

        res = []
        if state._onion_location == 0: 
            # on conveyor (not picked yet) 
            if state._listIDs_status == 2: 
                res = [Pick()] 
            elif state._listIDs_status == 0:  
                # res = [InspectWithoutPicking()]
                res = [InspectWithoutPicking(),Pick()]
            else:
                # list not empty
                if state._prediction == 2:
                    res = [ClaimNextInList()]
                else:
                    res = [Pick()]

        elif state._onion_location == 1:
            ##
            # if result of inspection is good, then placeinbin is not allowed 
            # if result of inspection is bad, then placeonconveyor is not allowed 
            ##
            if state._prediction == 0:
                res = [PlaceInBin()]
            elif state._prediction == 1:
                res = [PlaceOnConveyor()]
            else: 
                res = [InspectAfterPicking()]

        elif state._onion_location == 2:
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                res = [InspectWithoutPicking()]
            else:
                res = [ClaimNextInList()]

        elif state._onion_location == 3:
            if state._prediction == 2: 
                res = [InspectAfterPicking()] 
            elif state._prediction == 0: 
                res = [PlaceInBin()] 
            else: 
                res = [PlaceOnConveyor()] 

        else: # onion placed
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0: 
                res = [InspectWithoutPicking()] 
            else: 
                res = [ClaimNextInList()] 

        return res

class sortingModelbyPSuresh2WOPlaced(sortingModel):
    
    def __init__(self, p_fail=0.05, terminal=sortingState(-1,-1,-1) ):
        super(sortingModelbyPSuresh2WOPlaced,self).__init__(p_fail, terminal)

        # print("__init__(self ", self._p_fail)
        self._p_fail = self._p_fail

        statesList = [[ 0, 2, 0, 0],\
        [ 3, 2, 3, 0],\
        [ 1, 0, 1, 2],\
        [ 2, 2, 2, 2],\
        [ 0, 2, 2, 2],\
        [ 3, 2, 3, 2],\
        [ 1, 1, 1, 2],\
        [ 0, 0, 0, 1],\
        [ 3, 0, 3, 1],\
        [ 2, 2, 2, 1],\
        [ 0, 0, 2, 1],\
        [ 2, 2, 2, 0],\
        [0, 2, 0, 2],\
        [0, 2, 2, 0],\
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],\
        [3,1,3,0],[0,0,1,1],[0,0,3,1] #,\
        # [0,0,0,0],[0,0,1,0],[0,0,2,0],[0,0,3,0],\
        # [3,0,3,0]
        ]

        # rolling states start from [ 0, 0, 0, 1],\ and last two are needed for completing value iteration

        result2 = []
        for ls in statesList:
            ol,pr,el,le = ls[0], ls[1], ls[2], ls[3]
            result2.append( sortingState(ol,pr,el,le) )
        
        self._states = result2
        print("number of states in mdp ",len(self._states))


    def A(self,state=None):
        ''' {0: 'InspectAfterPicking', 1: 'PlaceOnConveyor', 
        2: 'PlaceInBin', 3: 'Pick', 
        4: 'ClaimNewOnion', 5: 'InspectWithoutPicking', 
        6: 'ClaimNextInList'}'''

        res = []
        if state._onion_location == 0: 
            # on conveyor (not picked yet) 
            if state._listIDs_status == 2: 
                res = [Pick()] 
            elif state._listIDs_status == 0:  
                # res = [InspectWithoutPicking()]
                res = [InspectWithoutPicking(),Pick()]
            else:
                # list not empty
                if state._prediction == 2:
                    res = [ClaimNextInList()]
                else:
                    res = [Pick()]

        elif state._onion_location == 1:
            ##
            # if result of inspection is good, then placeinbin is not allowed 
            # if result of inspection is bad, then placeonconveyor is not allowed 
            ##
            if state._prediction == 0:
                res = [PlaceInBin()]
            elif state._prediction == 1:
                res = [PlaceOnConveyor()]
            else: 
                res = [InspectAfterPicking()]

        elif state._onion_location == 2:
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                res = [InspectWithoutPicking()]
            else:
                res = [ClaimNextInList()]

        elif state._onion_location == 3:
            if state._prediction == 2: 
                res = [InspectAfterPicking()] 
            elif state._prediction == 0: 
                res = [PlaceInBin()] 
            else: 
                res = [PlaceOnConveyor()] 

        else: # onion placed
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0: 
                res = [InspectWithoutPicking()] 
            else: 
                res = [ClaimNextInList()] 

        return res


class sortingModelbyPSuresh3(sortingModel): # alllowed claim new onion in 0202
    
    def __init__(self, p_fail=0.05, terminal=sortingState(-1,-1,-1,-1) ):
        super(sortingModelbyPSuresh3,self).__init__(p_fail, terminal)

        # print("__init__(self ", self._p_fail)
        self._p_fail = self._p_fail

        statesList = [[ 0, 2, 0, 0],\
        [ 3, 2, 3, 0],\
        [ 1, 0, 1, 2],\
        [ 2, 2, 2, 2],\
        [ 0, 2, 2, 2],\
        [ 3, 2, 3, 2],\
        [ 1, 1, 1, 2],\
        [ 4, 2, 0, 2],\
        [ 0, 0, 0, 1],\
        [ 3, 0, 3, 1],\
        [ 2, 2, 2, 1],\
        [ 0, 0, 2, 1],\
        [ 2, 2, 2, 0],\
        [0, 2, 0, 2],\
        [0, 2, 2, 0],\
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],\
        [3,1,3,0],[0,0,1,1],[0,0,3,1],\
        [0,2,1,2],[0,2,3,2]]

        # rolling states start from [ 0, 0, 0, 1],\ and last two are needed 
        # for completing value iteration

        result2 = []
        for ls in statesList:
            ol,pr,el,le = ls[0], ls[1], ls[2], ls[3]
            result2.append( sortingState(ol,pr,el,le) )

        # onion_locations = [0,1,2,3,4]
        # predictions = [0,1,2] 
        # EE_locations = [0,1,2,3]
        # listIDs_status_opts = [0,1,2]

        # for ol in onion_locations:
        #     for pr in predictions:
        #         for el in EE_locations:
        #             for le in listIDs_status_opts:
        #                 # invalid if onion infront/ athome and EE loc not same
        #                 if (ol == 1 and el != 1) or \
        #                 (ol == 3 and el != 3) or\
        #                 (ol == 2 and el != 2): # (ol == 4 and el != 0)
        #                     pass
        #                 else:
        #                     result2.append( sortingState(ol,pr,el,le) )
        
        self._states = result2
        print("number of states in mdp ",len(self._states))


    def A(self,state=None):
        ''' {0: 'InspectAfterPicking', 1: 'PlaceOnConveyor', 
        2: 'PlaceInBin', 3: 'Pick', 
        4: 'ClaimNewOnion', 5: 'InspectWithoutPicking', 
        6: 'ClaimNextInList'}'''

        res = []
        if state._onion_location == 0: 
            # on conveyor (not picked yet) 
            if state._listIDs_status == 2: 
                res = [Pick(),ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                # res = [InspectWithoutPicking()]
                res = [InspectWithoutPicking(), Pick(), ClaimNewOnion()]
            else:
                # list not empty
                if state._prediction == 2:
                    res = [ClaimNextInList()]
                else:
                    res = [Pick()]

        elif state._onion_location == 1:
            ##
            # if result of inspection is good, then placeinbin is not allowed 
            # if result of inspection is bad, then placeonconveyor is not allowed 
            ##
            if state._prediction == 0:
                res = [PlaceInBin()]
            elif state._prediction == 1:
                res = [PlaceOnConveyor()]
            else: 
                res = [InspectAfterPicking()]

        elif state._onion_location == 2:
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                res = [InspectWithoutPicking()]
            else:
                res = [ClaimNextInList()]

        elif state._onion_location == 3:
            if state._prediction == 2: 
                res = [InspectAfterPicking()] 
            elif state._prediction == 0: 
                res = [PlaceInBin()] 
            else: 
                res = [PlaceOnConveyor()] 

        else: # onion placed
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0: 
                res = [InspectWithoutPicking()] 
            else: 
                res = [ClaimNextInList()] 

        return res

class sortingModelbyPSuresh3multipleInit(sortingModel): # alllowed claim new onion in 0202
    
    def __init__(self, p_fail=0.05, terminal=sortingState(-1,-1,-1,-1) ):
        super(sortingModelbyPSuresh3multipleInit,self).__init__(p_fail, terminal)

        # print("__init__(self ", self._p_fail)
        self._p_fail = self._p_fail

        statesList = [[ 0, 2, 0, 0],\
        [ 3, 2, 3, 0],\
        [ 1, 0, 1, 2],\
        [ 2, 2, 2, 2],\
        [ 0, 2, 2, 2],\
        [ 3, 2, 3, 2],\
        [ 1, 1, 1, 2],\
        [ 4, 2, 0, 2],\
        [ 0, 0, 0, 1],\
        [ 3, 0, 3, 1],\
        [ 2, 2, 2, 1],\
        [ 0, 0, 2, 1],\
        [ 2, 2, 2, 0],\
        [0, 2, 0, 2],\
        [0, 2, 2, 0],\
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],\
        [3,1,3,0],[0,0,1,1],[0,0,3,1],\
        [0,2,1,2],[0,2,3,2]]

        # rolling states start from [ 0, 0, 0, 1],\ and last two are needed 
        # for completing value iteration

        result2 = []
        for ls in statesList:
            ol,pr,el,le = ls[0], ls[1], ls[2], ls[3]
            result2.append( sortingState(ol,pr,el,le) )

        # onion_locations = [0,1,2,3,4]
        # predictions = [0,1,2] 
        # EE_locations = [0,1,2,3]
        # listIDs_status_opts = [0,1,2]

        # for ol in onion_locations:
        #     for pr in predictions:
        #         for el in EE_locations:
        #             for le in listIDs_status_opts:
        #                 # invalid if onion infront/ athome and EE loc not same
        #                 if (ol == 1 and el != 1) or \
        #                 (ol == 3 and el != 3) or\
        #                 (ol == 2 and el != 2): # (ol == 4 and el != 0)
        #                     pass
        #                 else:
        #                     result2.append( sortingState(ol,pr,el,le) )
        
        self._states = result2
        print("number of states in mdp ",len(self._states))


    def A(self,state=None):
        ''' {0: 'InspectAfterPicking', 1: 'PlaceOnConveyor', 
        2: 'PlaceInBin', 3: 'Pick', 
        4: 'ClaimNewOnion', 5: 'InspectWithoutPicking', 
        6: 'ClaimNextInList'}'''

        res = []
        if state._onion_location == 0: 
            # on conveyor (not picked yet) 
            if state._listIDs_status == 2: 
                res = [Pick(),ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                # res = [InspectWithoutPicking()]
                res = [InspectWithoutPicking(), Pick(), ClaimNewOnion()]
            else:
                # list not empty
                if state._prediction == 2:
                    res = [ClaimNextInList()]
                else:
                    res = [Pick()]

        elif state._onion_location == 1:
            ##
            # if result of inspection is good, then placeinbin is not allowed 
            # if result of inspection is bad, then placeonconveyor is not allowed 
            ##
            if state._prediction == 0:
                res = [PlaceInBin()]
            elif state._prediction == 1:
                res = [PlaceOnConveyor()]
            else: 
                res = [InspectAfterPicking()]

        elif state._onion_location == 2:
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                res = [InspectWithoutPicking()]
            else:
                res = [ClaimNextInList()]

        elif state._onion_location == 3:
            if state._prediction == 2: 
                res = [InspectAfterPicking()] 
            elif state._prediction == 0: 
                res = [PlaceInBin()] 
            else: 
                res = [PlaceOnConveyor()] 

        else: # onion placed
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0: 
                res = [InspectWithoutPicking()] 
            else: 
                res = [ClaimNextInList()] 

        return res

class sortingModelbyPSuresh4(sortingModel): # alllowed claim new onion in 0202, placeinbin for good onion
    
    def __init__(self, p_fail=0.05, terminal=sortingState(-1,-1,-1,-1) ):
        super(sortingModelbyPSuresh4,self).__init__(p_fail, terminal)

        # print("__init__(self ", self._p_fail)
        self._p_fail = self._p_fail

        statesList = [[ 0, 2, 0, 0],\
        [ 3, 2, 3, 0],\
        [ 1, 0, 1, 2],\
        [ 2, 2, 2, 2],\
        [ 0, 2, 2, 2],\
        [ 3, 2, 3, 2],\
        [ 1, 1, 1, 2],\
        [ 4, 2, 0, 2],\
        [ 0, 0, 0, 1],\
        [ 3, 0, 3, 1],\
        [ 2, 2, 2, 1],\
        [ 0, 0, 2, 1],\
        [ 2, 2, 2, 0],\
        [0, 2, 0, 2],\
        [0, 2, 2, 0],\
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],\
        [3,1,3,0],[0,0,1,1],[0,0,3,1],\
        [0,2,1,2],[0,2,3,2]]

        # rolling states start from [ 0, 0, 0, 1],\ and last two are needed 
        # for completing value iteration

        result2 = []
        for ls in statesList:
            ol,pr,el,le = ls[0], ls[1], ls[2], ls[3]
            result2.append( sortingState(ol,pr,el,le) )

        # onion_locations = [0,1,2,3,4]
        # predictions = [0,1,2] 
        # EE_locations = [0,1,2,3]
        # listIDs_status_opts = [0,1,2]

        # for ol in onion_locations:
        #     for pr in predictions:
        #         for el in EE_locations:
        #             for le in listIDs_status_opts:
        #                 # invalid if onion infront/ athome and EE loc not same
        #                 if (ol == 1 and el != 1) or \
        #                 (ol == 3 and el != 3) or\
        #                 (ol == 2 and el != 2): # (ol == 4 and el != 0)
        #                     pass
        #                 else:
        #                     result2.append( sortingState(ol,pr,el,le) )
        
        self._states = result2
        print("number of states in mdp ",len(self._states))


    def A(self,state=None):
        ''' {0: 'InspectAfterPicking', 1: 'PlaceOnConveyor', 
        2: 'PlaceInBin', 3: 'Pick', 
        4: 'ClaimNewOnion', 5: 'InspectWithoutPicking', 
        6: 'ClaimNextInList'}'''

        res = []
        if state._onion_location == 0: 
            # on conveyor (not picked yet) 
            if state._listIDs_status == 2: 
                res = [Pick(),ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                # res = [InspectWithoutPicking()]
                res = [InspectWithoutPicking(), Pick(), ClaimNewOnion()]
            else:
                # list not empty
                if state._prediction == 2:
                    res = [ClaimNextInList()]
                else:
                    res = [Pick()]

        elif state._onion_location == 1:
            ##
            # if result of inspection is good, then placeinbin is not allowed 
            # if result of inspection is bad, then placeonconveyor is not allowed 
            ##
            if (state._listIDs_status == 2) :
                if state._prediction == 0:
                    res = [PlaceInBin()] #,PlaceOnConveyor()]
                elif state._prediction == 1:
                    res = [PlaceOnConveyor(),PlaceInBin()]
                else: 
                    res = [InspectAfterPicking()]
            else :
                if state._prediction == 0:
                    res = [PlaceInBin()]
                elif state._prediction == 1:
                    res = [PlaceOnConveyor()]
                else: 
                    res = [InspectAfterPicking()]

        elif state._onion_location == 2:
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0:  
                res = [InspectWithoutPicking()]
            else:
                res = [ClaimNextInList()]

        elif state._onion_location == 3:
            if state._prediction == 2: 
                res = [InspectAfterPicking()] 
            elif state._prediction == 0: 
                res = [PlaceInBin()] 
            else: 
                res = [PlaceOnConveyor()] 

        else: # onion placed
            if state._listIDs_status == 2: 
                res = [ClaimNewOnion()] 
            elif state._listIDs_status == 0: 
                res = [InspectWithoutPicking()] 
            else: 
                res = [ClaimNextInList()] 

        return res

class sortingModel2(sortingModel):
    
    def __init__(self, p_fail=0.05, terminal=sortingState(-1,-1,-1) ):
        super(sortingModel2, self).__init__(p_fail, terminal)

        result2 = []
        onion_locations = [0,1,2,3,4]
        predictions = [0,1,2] 
        EE_locations = [0,1,2,3]
        listIDs_status_opts = [0,1,2]

        for ol in onion_locations:
            for pr in predictions:
                for el in EE_locations:
                    for le in listIDs_status_opts:
                        # invalid if onion infront/ athome and EE loc not same
                        if (ol == 1 and el != 1) or \
                        (ol == 3 and el != 3) or\
                        (ol == 2 and (le == 0 or le == 1) ): # (ol == 2 and el != 2) (ol == 4 and el != 0)
                            pass
                        else:
                            result2.append( sortingState(ol,pr,el,le) )
        
        self._states = result2

    def A(self,state=None):
        # Pick, InspectWithoutPicking can't be done if onion is already picked (home-pose 3 or infront 1), 
        # PlaceOnConveyor,PlaceInBin, InspectAfterPicking can't be done if onion is not picked yet ()
        # onion prediction must always be unknown before inspection and known after inspection
        #
        
        res = []
        if state._onion_location == 1 or state._onion_location == 3: 
        # home or front, onion is picked
            # res = [InspectAfterPicking(),PlaceOnConveyor(),PlaceInBin()] 
            if (state._listIDs_status == 2) :
                if (state._onion_location == 1): 
                    # no inspect after inspecting
                    res = [PlaceOnConveyor(), PlaceInBin()]
                else: 
                    res = [InspectAfterPicking(), PlaceInBin()]
                
            else: 
                # new action specific to rolling
                res = [PlaceInBinClaimNextInList()]

        if state._onion_location == 0 or state._onion_location == 4: 
        # on conveyor (not picked yet or already placed) 
            if state._listIDs_status == 2: # can not claim from list if list not available 
                res = [Pick(),ClaimNewOnion(),InspectWithoutPicking()] 
            else: # can not create list again if a list is already available 
                res = [Pick()]# if we allow ClaimNewOnion with a list available
                # then it will do *,0,2,1 ClaimNewOnion 0,2,2,1 ClaimNextInList 0,0,2,1
                # and will assume onion is bad without inspection
        if state._onion_location == 2: # in bin, can't pick from bin because not reachable 
            if state._listIDs_status == 2:# sorter can claim new onion only when a list of predictions has not been pending 
                res = [ClaimNewOnion(),InspectWithoutPicking()] 
            else:
                # with new
                res = []# if we allow ClaimNewOnion with a list available
                # then it will do *,0,2,1 ClaimNewOnion 0,2,2,1 ClaimNextInList 0,0,2,1
                # and will assume onion is bad without inspection
        return res


class InspectAfterPickingold(Action):
    
    def apply(self,state):
        # prediction that onion is bad. 95% accuracy of detection
        # 50% of claimable onions on conveyor are bad
        if state._prediction == 2:# it can predict claimed-gripped onion only if prediction is unknown
            pp = 0.5*0.95
            pred = np.random.choice([1,0],1,p=[1-pp,pp])[0]
            return sortingState( 1, pred, 1, state._listIDs_status )
        else:
            return sortingState( 1, state._prediction, 1, state._listIDs_status )

    def __str__(self):
        return "InspectAfterPicking"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "InspectAfterPicking"
        except Exception:
            return False
    
    def __hash__(self):
        return (0).__hash__() 

class InspectAfterPicking(Action):
    
    def apply(self,state):
        if state._prediction == 2:# it can predict claimed-gripped onion only if prediction is unknown
            pp = 0.5
            pred = np.random.choice([1,0],1,p=[1-pp,pp])[0]
            return sortingState( 1, pred, 1, 2 )
        else:
            # return sortingState( state._onion_location, state._prediction, 1, state._listIDs_status )
            return sortingState( 1, state._prediction, 1, state._listIDs_status )

    def __str__(self):
        return "InspectAfterPicking"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "InspectAfterPicking"
        except Exception:
            return False
    
    def __hash__(self):
        return (8).__hash__() 
    
class InspectWithoutPickingold(Action):  
    
    def apply(self,state): 
        #  can not apply this action if a list is already available
        # It is detecting many onions simultaneously. assuming half are bad with 95% probability,
        # it should be derived from
        # chance of not detecting any of bad onions = 0.3^(num/objects/2) . 
        # Then prob is 0.95*(1-0.3^(num/objects/2)) ~ 1        
        # 
        # if state._prediction == 2:
        global num_objects
        pp = 0.95*(1 - pow((1-0.7),(num_objects/2)) )
        ls = np.random.choice([1,0],1,p=[pp,1-pp])[0] 
        if (ls == 0):
            pred = 2
        else:
            pred = 0
        return sortingState( 0, pred, state._EE_location, ls )
        # else:
        #     return sortingState( state._onion_location, state._prediction, state._EE_location, state._listIDs_status )
   
    def __str__(self):
        return "InspectWithoutPicking"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "InspectWithoutPicking"
        except Exception:
            return False
    
    def __hash__(self):
        return (1).__hash__()

class InspectWithoutPicking(Action):  
    
    def apply(self,state): 
        #  can not apply this action if a list is already available
        global num_objects
        pp = 0.5
        pp = 1*0.95
        ls = np.random.choice([1,0],1,p=[pp,1-pp])[0] 
        if (ls == 0):
            pred = 2
        else:
            pred = 0
        return sortingState( 0, pred, state._EE_location, ls )
        # else:
        #     return sortingState( state._onion_location, state._prediction, state._EE_location, state._listIDs_status )
   
    def __str__(self):
        return "InspectWithoutPicking"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "InspectWithoutPicking"
        except Exception:
            return False
    
    def __hash__(self):
        return (13).__hash__()

class Pick(Action):
    
    def apply(self,state): 
        # onion picked and is at home-pose of sawyer
        return sortingState( 3, state._prediction, 3, state._listIDs_status )
    
    def __str__(self):
        return "Pick"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "Pick"
        except Exception:
            return False
    
    def __hash__(self):
        return (2).__hash__()


class Pickpip(Action):
    
    def apply(self,state): 
        # onion picked and is at home-pose of sawyer
        return sortingState( 3, state._prediction, 3, state._listIDs_status )
    
    def __str__(self):
        return "Pickpip"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "Pickpip"
        except Exception:
            return False
    
    def __hash__(self):
        return (20).__hash__()

class PlaceOnConveyorold(Action):
    
    def apply(self,state): 
        return sortingState( 4, state._prediction, 0, state._listIDs_status )
    
    def __str__(self):
        return "PlaceOnConveyor"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PlaceOnConveyor"
        except Exception:
            return False
    
    def __hash__(self):
        return (3).__hash__()

class PlaceOnConveyorWPlaced(Action):
    
    def apply(self,state): 
        return sortingState( 4, 2, 0, 2 )
    
    def __str__(self):
        return "PlaceOnConveyor"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PlaceOnConveyor"
        except Exception:
            return False
    
    def __hash__(self):
        return (9).__hash__()

class PlaceOnConveyor(Action):
    
    def apply(self,state): 
        return sortingState( 0, 2, 0, 2 )
    
    def __str__(self):
        return "PlaceOnConveyor"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PlaceOnConveyor"
        except Exception:
            return False
    
    def __hash__(self):
        return (24).__hash__()

class PlaceInBinold(Action):
    
    def apply(self,state): 
        # most of attempts won't make list empty if it is not already empty or unavailable
        # if list is available and 50% of objects are bad, then 1 out of 6 attempts make 
        # list empty
        global num_objects
        if state._listIDs_status == 1:
            pp = 1-(2/num_objects)
            ls = np.random.choice([1,0],1,p=[pp,1-pp])[0] 
            return sortingState( 2, state._prediction, 2, ls )
        else:
            return sortingState( 2, state._prediction, 2, state._listIDs_status )

    def __str__(self):
        return "PlaceInBin"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PlaceInBin"
        except Exception:
            return False
    
    def __hash__(self):
        return (4).__hash__()

class PlaceInBin(Action):
    
    def apply(self,state): 
        # most of attempts won't make list empty if it is not already empty or unavailable
        # if list is available and 50% of objects are bad, then 1 out of 6 attempts make 
        # list empty
        global num_objects
        if state._listIDs_status == 1:
            pp = 0.5
            ls = np.random.choice([1,0],1,p=[pp,1-pp])[0] 
            return sortingState( 2, 2, 2, ls )
        else:
            return sortingState( 2, 2, 2, state._listIDs_status )

    def __str__(self):
        return "PlaceInBin"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PlaceInBin"
        except Exception:
            return False
    
    def __hash__(self):
        return (10).__hash__()

class PlaceInBinpip(Action):
    
    def apply(self,state): 
        # most of attempts won't make list empty if it is not already empty or unavailable
        # if list is available and 50% of objects are bad, then 1 out of 6 attempts make 
        # list empty
        global num_objects
        if state._listIDs_status == 1:
            pp = 0.5
            ls = np.random.choice([1,0],1,p=[pp,1-pp])[0] 
            return sortingState( 2, 2, 2, ls )
        else:
            return sortingState( 2, 2, 2, state._listIDs_status )

    def __str__(self):
        return "PlaceInBinpip"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PlaceInBinpip"
        except Exception:
            return False
    
    def __hash__(self):
        return (21).__hash__()

class ClaimNewOnionold(Action):

    def apply(self,state):
        # on conv, unknown, 
        return sortingState( 0, 2, state._EE_location, state._listIDs_status )
    
    def __str__(self):
        return "ClaimNewOnion"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "ClaimNewOnion"
        except Exception:
            return False
    
    def __hash__(self):
        return (5).__hash__()

class ClaimNewOnion(Action):

    def apply(self,state):
        # on conv, unknown, 
        return sortingState( 0, 2, state._EE_location, 2 )
    
    def __str__(self):
        return "ClaimNewOnion"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "ClaimNewOnion"
        except Exception:
            return False
    
    def __hash__(self):
        return (12).__hash__()

class ClaimNextInListold(Action):

    def apply(self,state):
        
        if state._listIDs_status == 1:
            # if list not empty, then 
            return sortingState( 0, 0, state._EE_location, 1 )
        else:
            # else make onion unknown and list not available
            return sortingState( 0, 2, state._EE_location, 2 )
    
    def __str__(self):
        return "ClaimNextInList"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "ClaimNextInList"
        except Exception:
            return False
    
    def __hash__(self):
        return (6).__hash__()


class ClaimNextInList(Action):

    def apply(self,state):
        
        if state._listIDs_status == 1:
            # if list not empty, then 
            return sortingState( 0, 0, state._EE_location, 1 )
        else:
            # else make onion unknown and list not available
            return sortingState( 0, 2, state._EE_location, state._listIDs_status )
    
    def __str__(self):
        return "ClaimNextInList"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "ClaimNextInList"
        except Exception:
            return False
    
    def __hash__(self):
        return (14).__hash__()

class GoHome(Action):
    
    def apply(self,state): 
        # 50-50 chance of bringing onion home gripped in gripper?
        return sortingState( state._onion_location, state._prediction, 3, state._listIDs_status )
    
    def __str__(self):
        return "GoHome"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "GoHome"
        except Exception:
            return False
    
    def __hash__(self):
        return (7).__hash__()


class PlaceInBinClaimNextInList(Action):

    def apply(self,state):
        
        global num_objects
        if state._listIDs_status == 1:
            # paceinbin part
            pp = 1-(2/num_objects)
            ls = np.random.choice([1,0],1,p=[pp,1-pp])[0] 

            # claimnext inlist part 
            return sortingState( 0, 0, state._EE_location, 1 )
        else:
            # claimnext inlist part 
            return sortingState( 0, 2, state._EE_location, 2 )

    def __str__(self):
        return "PlaceInBinClaimNextInList"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PlaceInBinClaimNextInList"
        except Exception:
            return False
    
    def __hash__(self):
        return (7).__hash__()
