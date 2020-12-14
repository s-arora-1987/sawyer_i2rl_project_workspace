# -*- coding: utf-8 -*-
from mdp.model import *
from util.classes import NumMap
from numpy import array, all
import math
import numpy as np
import sys



class AttackerState(State):
    def __init__(self, location = array( [0, 0] ), orientation = 0, time = 0):
        self._location = location
        self._isPlaceholder = False
        self._orientation = orientation
        self._time = time
                                                                
        

    @property
    def location(self):
        return self._location
    
    @location.setter
    def location(self, loc):
        self._location = loc

    @property
    def time(self):
        return self._time
    
    @time.setter
    def time(self, loc):
        self._time = loc

    @property
    def orientation(self):
        return self._orientation
    
    @orientation.setter
    def orientation(self, loc):
        self._orientation = loc

    def __str__(self):
        return 'AttackerState: [{}, {}] @ {}'.format(self.location, self.orientation, self.time)
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        try:
#            return all( self.location == other.location) and self.time == other.time and all(self.lastvisits == other.lastvisits) # epsilon error
            return all( self.location == other.location) and other.orientation == self.orientation and other.time == self.time # epsilon error
        except Exception:
            return False
    
    def __hash__(self):
        loc = self.location # hash codes for numpy.array not consistent?

#        return (loc[0], loc[1], self.time, self.lastvisits.sum()).__hash__()
        return (loc[0], loc[1], self.orientation, self.time).__hash__()
        
    def distance(self, otherState):
        return math.sqrt((self.location[0] - otherState.location[0])*(self.location[0] - otherState.location[0]) + (self.location[1] - otherState.location[1])*(self.location[1] - otherState.location[1]))
        
class AttackerMoveForward(Action):
    
    
    def apply(self,gwstate):
        orientation = gwstate.orientation
        if (orientation == 0):
            dir = array([0, 1])
        if (orientation == 1):
            dir = array([-1, 0])
        if (orientation == 2):
            dir = array([0, -1])
        if (orientation == 3):
            dir = array([1, 0])
        
        return AttackerState( gwstate.location + dir, orientation, gwstate.time + 1)
    
    def __str__(self):
        return "AttackerMoveForward"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "AttackerMoveForward"
        except Exception:
            return False
    
    def __hash__(self):
        return 0
        
class AttackerTurnLeft(Action):
    
    def apply(self,gwstate):
        orientation = gwstate.orientation
        
        orientation += 1
        if (orientation > 3):
            orientation = 0
        
        return AttackerState( gwstate.location, orientation, gwstate.time + 1)
    
    def __str__(self):
        return "AttackerTurnLeft"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "AttackerTurnLeft"
        except Exception:
            return False
    
    def __hash__(self):
        return 1
    
    
class AttackerTurnRight(Action):
    
    def apply(self,gwstate):
        orientation = gwstate.orientation
        
        orientation -= 1
        if (orientation < 0):
            orientation = 3
        
        return AttackerState( gwstate.location, orientation, gwstate.time + 1)
    
    def __str__(self):
        return "AttackerTurnRight"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "AttackerTurnRight"
        except Exception:
            return False
    
    def __hash__(self):
        return 2
        
        
class AttackerAction(Action):
    
    def __init__(self, direction):
        self._direction = direction
     
    @property   
    def direction(self):
        return self._direction
    
    @direction.setter
    def direction(self, dir):
        self._direction = dir
        
    def apply(self,gwstate):
        return AttackerState( gwstate.location + self.direction, gwstate.time + 1)
    
    def __str__(self):
        return "AttackerAction: [direction={}]".format(self.direction)
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return all(self.direction == other.direction)
        except Exception:
            return False
    
    def __hash__(self):
        dir = self.direction    # hash codes for numpy.array not consistent?
        return (dir[0], dir[1]).__hash__()            


class AttackerModel(Model):
    
    def __init__(self, p_fail=0.05, map=None, maxTime = 0,
                 terminal=AttackerState(array( [-1,-1] ), 0) ):
        super(AttackerModel,self)
                
        self._actions = [AttackerMoveForward(), AttackerTurnLeft(), AttackerTurnRight()]
        self._p_fail = float(p_fail)
        self._map = map
        self._terminal = terminal
        self._maxTime = maxTime
        
    def T(self,state,action):
        """Returns a function state -> [0,1] for probability of next state
        given currently in state performing action"""
        result = NumMap()
        actions = self.A(state)
        # unfortunately have to assume we're using attackerStates here
        newState = AttackerState(state.location, state.time + 1)
        for a in actions:
            p = 0
            if a == action:
                p = 1 - self._p_fail
            else:
                p = self._p_fail / ( len(actions)-1 )
            s_p = a.apply(state)
            if not self.is_legal(s_p):
                result[newState] += p
            else:
                result[s_p] += p  
        return result 
        
    def S(self):
        """All states in the MDP"""
        result = []
        (r,c) = self._map.shape
        for x in range(r):
            for y in range(c):
                if (self._map[x,y] == 1):
                    for i in range(4):
                        for t in range(self._maxTime):
                            result.append( AttackerState( array([x,y]) , i, t) )
        return result
        
    def A(self,state=None):
        """All actions in the MDP is state=None, otherwise actions available
        from state"""
        return self._actions
    
    def is_terminal(self, state):
        '''returns whether or not a state is terminal'''
        return all(state.location == self._terminal.location)
    
    
    def is_legal(self,state):
        loc = state.location
        (r,c) = self._map.shape
        
        return loc[0] >= 0 and loc[0] < r and \
            loc[1] >= 0 and loc[1] < c and \
            self._map[ loc[0],loc[1] ] == 1
    
    def __str__(self):
        format = 'PatrolModel [p_fail={},terminal={}]'
        return format.format(self._p_fail, self._terminal)
    
    def info(self):
        result = [str(self) + '\n']
        return ''.join(result)


class PatrolState(State):
    
    def __init__(self, location=array( [0,0,0] )):
        self._extension = 0
        self._location = location
    
    @property
    def location(self):
        return self._location
    
    @location.setter
    def location(self, loc):
        self._location = loc
        
    def __str__(self):
        return 'PatrolState: [location={}]'.format(self.location)
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        try:
            return all( self.location == other.location) # epsilon error
        except Exception:
            return False
    
    def __hash__(self):
        loc = self.location # hash codes for numpy.array not consistent?
        return (loc[0], loc[1], loc[2]).__hash__()
    
    def conflicts(self, otherstate):
        return self.location[0] == otherstate.location[0] and self.location[1] == otherstate.location[1]


class PatrolExtendedState(State):
    
    def __init__(self, location=array( [0,0,0] ), current_goal=0):
        self._extension = 1
        self._location = location
        self._current_goal = current_goal
    
    @property
    def location(self):
        return self._location

    @property
    def current_goal(self):
        return self._current_goal
    
    @location.setter
    def location(self, loc):
        self._location = loc

    @current_goal.setter
    def current_goal(self, cg):
        self._current_goal = cg
        
    def __str__(self):
        return 'PatrolState: [location='+str(self._location)+']'+'-'+str(self._current_goal)
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self, other):
        try:
            return (all( self.location == other.location)\
                    and self.current_goal == other.current_goal) # epsilon error
        except Exception:
            return False
    
    def __hash__(self):
        
        loc = self._location # hash codes for numpy.array not consistent?
        current_goal = self._current_goal
        code = ((loc[0], loc[1], loc[2], current_goal)).__hash__()
#         2*(4*(9*location[0] + location[1]) + location[2]) \
#                         + current_goal
        return code
    
    def conflicts(self, otherstate):
        return self.location[0] == otherstate.location[0] \
            and self.location[1] == otherstate.location[1] \
            and self.current_goal == otherstate._current_goal 

def check_goalprogress_and_increment(next_loc, prev_goal):

    if (all(next_loc == array([6,0,2])) and (prev_goal == 0) or\
         (all(next_loc == array([6,0,2])) and (prev_goal == 4))): 
        return 1
    if (all(next_loc == array([0,8,0])) and (prev_goal == 1)):
        return 2
    if (all(next_loc == array([3,8,0])) and (prev_goal == 2)):
        return 3
    if (all(next_loc == array([9,0,2])) and (prev_goal == 3)): 
        return 4
    return prev_goal

class PatrolActionMoveForward(Action):
    
    def apply(self,gwstate):
        
        if gwstate._extension == 0:
            if gwstate.location[2] == 0:
                return PatrolState( gwstate.location + array( [0,1,0] ) )
            if gwstate.location[2] == 1:
                return PatrolState( gwstate.location + array( [-1,0,0] ) )
            if gwstate.location[2] == 2:
                return PatrolState( gwstate.location + array( [0,-1,0] ) )
            if gwstate.location[2] == 3:
                return PatrolState( gwstate.location + array( [1,0,0] ) )
        else:
            if gwstate.location[2] == 0:
                next_loc = gwstate.location + array( [0,1,0] )
            if gwstate.location[2] == 1:
                next_loc = gwstate.location + array( [-1,0,0] )
            if gwstate.location[2] == 2:
                next_loc = gwstate.location + array( [0,-1,0] )
            if gwstate.location[2] == 3:
                next_loc = gwstate.location + array( [1,0,0] )
                
            next_goal = check_goalprogress_and_increment(next_loc, gwstate._current_goal)
            return PatrolExtendedState( next_loc, next_goal )
            
    def __str__(self):
        return "PatrolActionMoveForward"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PatrolActionMoveForward"
        except Exception:
            return False

    def __hash__(self):
        return 0
           

def check_goalprogress_and_incrementR(next_loc, prev_goal): 

    if (all(next_loc == array([0,8,0])) and (prev_goal == 4)):
        return 2
    if (all(next_loc == array([9,0,2])) and (prev_goal == 2)): 
        return 4
    return prev_goal

class PatrolActionMoveForwardR(Action):
    
    def apply(self,gwstate):
        
        if gwstate._extension == 0:
            if gwstate.location[2] == 0:
                return PatrolState( gwstate.location + array( [0,1,0] ) )
            if gwstate.location[2] == 1:
                return PatrolState( gwstate.location + array( [-1,0,0] ) )
            if gwstate.location[2] == 2:
                return PatrolState( gwstate.location + array( [0,-1,0] ) )
            if gwstate.location[2] == 3:
                return PatrolState( gwstate.location + array( [1,0,0] ) )
        else:
            if gwstate.location[2] == 0:
                next_loc = gwstate.location + array( [0,1,0] )
            if gwstate.location[2] == 1:
                next_loc = gwstate.location + array( [-1,0,0] )
            if gwstate.location[2] == 2:
                next_loc = gwstate.location + array( [0,-1,0] )
            if gwstate.location[2] == 3:
                next_loc = gwstate.location + array( [1,0,0] )
                
            next_goal = check_goalprogress_and_incrementR( next_loc, gwstate._current_goal ) 
            return PatrolExtendedState( next_loc, next_goal )
            
    def __str__(self):
        return "PatrolActionMoveForward"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PatrolActionMoveForward"
        except Exception:
            return False

    def __hash__(self):
        return 0
           

class PatrolActionStop(Action):
    
    def apply(self,gwstate):
        if gwstate._extension == 0:
            return PatrolState( gwstate.location )
        else:
            return PatrolExtendedState( gwstate.location, gwstate.current_goal )
    
    def __str__(self):
        return "PatrolActionStop"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PatrolActionStop"
        except Exception:
            return False
           
    def __hash__(self):
        return 1
           
class PatrolActionTurnLeft(Action):
    
    def apply(self,gwstate):
        
        next_loc = gwstate.location + array( [0,0,1] )
        if next_loc[2] > 3:
            next_loc[2] = 0
        if gwstate._extension == 0:
            returnval = PatrolState( next_loc )
        else:
            returnval = PatrolExtendedState( next_loc, gwstate.current_goal )
            
        return returnval
    
    def __str__(self):
        return "PatrolActionTurnLeft"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PatrolActionTurnLeft"
        except Exception:
            return False

    def __hash__(self):
        return 2
           
           
class PatrolActionTurnAround(Action):
    
    def apply(self,gwstate):

        next_loc = gwstate.location + array( [0,0,2] )
        if next_loc[2] > 3:
            next_loc[2] = next_loc[2] - 4
        if gwstate._extension == 0:
            returnval = PatrolState( next_loc )
        else:
            returnval = PatrolExtendedState( next_loc, gwstate.current_goal)

        return returnval
    
    def __str__(self):
        return "PatrolActionTurnAround"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PatrolActionTurnAround"
        except Exception:
            return False
    def __hash__(self):
        return 3
        

class PatrolActionTurnRight(Action):
    
    def apply(self,gwstate):

        next_loc = gwstate.location + array( [0,0,-1] )
        if next_loc[2] < 0:
            next_loc[2] = 3
            
        if gwstate._extension == 0:
            returnval = PatrolState( next_loc )
        else:
            returnval = PatrolExtendedState( next_loc, gwstate.current_goal )
            
        return returnval
    
    def __str__(self):
        return "PatrolActionTurnRight"
    
    def __repr__(self):
        return self.__str__()
    
    def __eq__(self,other):
        try:
            return other.__class__.__name__ == "PatrolActionTurnRight"
        except Exception:
            return False
    
    def __hash__(self):
        return 4

        
class PatrolModel(Model):
    
    def __init__(self, p_fail=0.2, 
                 terminal=PatrolState(np.array( [-1,-1, -1] )), mapToUse=None ):
        super(PatrolModel,self)
        
        self._actions = [PatrolActionMoveForward(), PatrolActionTurnLeft(), PatrolActionTurnRight(), PatrolActionStop()]
        self._p_fail = float(p_fail)
        self._map = mapToUse
        self._terminal = terminal
        
        """All states in the MDP"""
        result = []
        nz = np.nonzero(self._map == 1)
        for o in range(4):
            for c, ind in enumerate(nz[0]):
                s = array( [nz[0][c], nz[1][c], o] )
                result.append( PatrolState( s ) )
        self._S = result
        
        print "\n total number of states: "+str(len(result))+"\n"

    def T(self,state,action):
        """Returns a function state -> [0,1] for probability of next state
        given currently in state performing action"""
        result = NumMap()
        valid_actions = self.A(state)
        s_p = action.apply(state)
        if action not in valid_actions or not self.is_legal(s_p) or s_p.__eq__(state):
            result[state] = 1
        else:
            result[state] = self._p_fail
            result[s_p] = 1-self._p_fail

        # totalP = 0
        # for a in actions:
        #     p = 0
        #     if a == action:
        #         p = 1 - self._p_fail
        #     else:
        #         p = self._p_fail / ( len(actions)-1 )
        #     s_p = a.apply(state)
        #     if not self.is_legal(s_p):
        #         result[state] += p
        #     else:
        #         result[s_p] += p  
        #     totalP += p
        # result[state] += 1.0 - totalP
        return result 
        
    def S(self):
        return self._S
        
    def A(self,state=None):
        """All actions in the MDP is state=None, otherwise actions available
        from state"""
#        if state is None:
 #           return self._actions
#        if state.location[1] == 3:
 #           if state.location[0] == 1 and state.location[2] == 1:
  #              return [PatrolActionTurnLeft(), PatrolActionTurnRight()]
   #         if state.location[0] == 0 and state.location[2] == 3:
    #            return [PatrolActionTurnLeft(), PatrolActionTurnRight()]
                
                
        return self._actions
    
    def is_terminal(self, state):
        '''returns whether or not a state is terminal'''
        return all(state == self._terminal)
    
    def is_legal(self,state):
        loc = state.location
        (r,c) = self._map.shape
        
        return loc[0] >= 0 and loc[0] < r and \
            loc[1] >= 0 and loc[1] < c and \
            loc[2] >= 0 and loc[2] < 4 and \
            self._map[ loc[0],loc[1] ] == 1
    
    def __str__(self):
        format = 'GWModel [p_fail={},terminal={}]'
        return format.format(self._p_fail, self._terminal)
    
    def info(self):
        result = [str(self) + '\n']
        map_size = self._map.shape
        for i in reversed(range(map_size[0])):
            for j in range(map_size[1]):
                if self._map[i,j] == 1:
                    result.append( '[O]' )
                else:
                    result.append( '[X]')
            result.append( '\n' )
        return ''.join(result)
       
class PatrolExtendedModel(Model):
    
    def __init__(self, p_fail=0.01, 
                 terminal=PatrolExtendedState(np.array( [-1,-1, -1] ),0), mapToUse=None ):
        super(PatrolExtendedModel,self)
        
        self._actions = [PatrolActionMoveForward(), PatrolActionTurnLeft(), PatrolActionTurnRight(), PatrolActionStop()]
        self._p_fail = float(p_fail)
        self._map = mapToUse
        self._terminal = terminal
        
        """All states in the MDP"""
        result2 = []
        nz = np.nonzero(self._map == 1)
        for o in range(4):
            for c, ind in enumerate(nz[0]):
                s = array( [nz[0][c], nz[1][c], o] )
                result2.append( PatrolState( s ) )
        
        result = []
        goals = [0,1,2,3,4]
        pmodel= PatrolModel(self._p_fail,PatrolState(np.array( [-1,-1,-1] )),self._map)
        
        for g in goals:
            for st in result2:
                l = st.location
                nextstatelegal = pmodel.is_legal(st)
                
#                 validgoal0 = g==0 and l[0]>=6 and not all(l[0:2] == array([6,0]))
                validgoal4 = g==4 and l[0]>=6 and not all(l[0:2] == array([6,0]))
                validgoal1 = g==1 and l[0]<=6 and (not (l[0] == 3 and l[1] > 4)) and not all(l[0:2] == array([0,8]))
                validgoal2 = g==2 and l[0]<=3 and not all(l[0:2] == array([3,8]))
                validgoal3 = g==3 and l[0]>=3 and (not (l[0] == 6 and l[1] < 4)) and not all(l[0:2] == array([9,0]))
                
                validgoal = validgoal4 or validgoal1 or validgoal2 or validgoal3
                
                if nextstatelegal and validgoal:
                    result.append(PatrolExtendedState(l, g))

        print "total number of extended states-"+str(len(result))
        visible = 0
        for pst in result:
            if pst.location[1] == 4 and pst.location[0] >= 3:
                visible += 1
        print "total states in largest hallway 4th row onwards:"+str(visible)
        
        self._S = result
        
    def T(self,state,action):
        """Returns a function state -> [0,1] for probability of next state
        given currently in state performing action"""
        result = NumMap()
        s_p = action.apply(state)
        if not self.is_legal(s_p):
            result[state] = 1
        else:
            result[state] = self._p_fail
            result[s_p] = 1 - self._p_fail  
        
#         actions = self.A(state)
#         totalP = 0
#         for a in actions:
#             p = 0
#             if a == action:
#                 p = 1 - self._p_fail
#             else:
#                 p = self._p_fail / ( len(actions)-1 )
#             s_p = a.apply(state)
#             
#             if not self.is_legal(s_p):
#                 result[state] += p
#             else:
#                 result[s_p] += p  
#             totalP += p
#         result[state] += 1.0 - totalP
        return result 
        
    def S(self):
        return self._S
        
    def A(self,state=None):
        """All actions in the MDP is state=None, otherwise actions available
        from state"""
#        if state is None:
 #           return self._actions
#        if state.location[1] == 3:
 #           if state.location[0] == 1 and state.location[2] == 1:
  #              return [PatrolActionTurnLeft(), PatrolActionTurnRight()]
   #         if state.location[0] == 0 and state.location[2] == 3:
    #            return [PatrolActionTurnLeft(), PatrolActionTurnRight()]
        
        return self._actions
    
    def is_terminal(self, state):
        '''returns whether or not a state is terminal'''
        
#        state = PatrolExtendedState(state.location,None,None)
        return all(state.location == self._terminal.location)
    
    def is_legal(self,state):
        
        l = state.location
        (r,c) = self._map.shape
        g = state.current_goal
        
        legal_loc = l[0] >= 0 and l[0] < r and \
            l[1] >= 0 and l[1] < c and \
            l[2] >= 0 and l[2] < 4 and \
            self._map[ l[0],l[1] ] == 1
        legal_cg = g >= 0 and g <= 4
        
#         validgoal0 = g==0 and l[0]>=6 and not all(l[0:2] == array([6,0]))
        validgoal4 = g==4 and l[0]>=6 and not all(l[0:2] == array([6,0]))
        validgoal1 = g==1 and l[0]<=6 and (not (l[0] == 3 and l[1] > 4)) and not all(l[0:2] == array([0,8]))
        validgoal2 = g==2 and l[0]<=3 and not all(l[0:2] == array([3,8]))
        validgoal3 = g==3 and l[0]>=3 and (not (l[0] == 6 and l[1] < 4)) and not all(l[0:2] == array([9,0]))
        validgoal = validgoal4 or validgoal1 or validgoal2 or validgoal3
                
        return (legal_loc and legal_cg and validgoal)
    
    def __str__(self):
        format = 'GWModel [p_fail={},terminal={}]'
        return format.format(self._p_fail, self._terminal)
    
    def info(self):
        result = [str(self) + '\n']
        map_size = self._map.shape
        for i in reversed(range(map_size[0])):
            for j in range(map_size[1]):
                if self._map[i,j] == 1:
                    result.append( '[O]' )
                else:
                    result.append( '[X]')
            result.append( '\n' )
        return ''.join(result)

class PatrolExtendedModelR(Model):
    
    def __init__(self, p_fail=0.01, 
                 terminal=PatrolExtendedState(np.array( [-1,-1, -1] ),0), mapToUse=None ):
        super(PatrolExtendedModelR,self) 
        
        self._actions = [PatrolActionMoveForwardR(), PatrolActionTurnLeft(), 
                         PatrolActionTurnRight(), PatrolActionStop()] 
        self._p_fail = float(p_fail)
        self._map = mapToUse
        self._terminal = terminal
        
        """All states in the MDP"""
        result2 = []
        nz = np.nonzero(self._map == 1)
        for o in range(4):
            for c, ind in enumerate(nz[0]):
                s = array( [nz[0][c], nz[1][c], o] )
                result2.append( PatrolState( s ) )
        
        result = []
        goals = [2,4] 
        pmodel= PatrolModel(self._p_fail,PatrolState(np.array( [-1,-1,-1] )),self._map) 
        
        for g in goals:
            for st in result2:
                
                l = st.location
                nextstatelegal = pmodel.is_legal(st)
                
                validgoal2 = g==2 and not all(l[0:2] == array([9,0]))
                validgoal4 = g==4 and not all(l[0:2] == array([0,8]))
                
                validgoal = validgoal4 or validgoal2
                
                if nextstatelegal and validgoal:
                    result.append(PatrolExtendedState(l, g))

        print "total number of extended states-"+str(len(result))
        
        self._S = result
        
    def T(self,state,action):
        """Returns a function state -> [0,1] for probability of next state
        given currently in state performing action"""
        result = NumMap()
        s_p = action.apply(state)
        if not self.is_legal(s_p):
            result[state] = 1
        else:
            result[state] = self._p_fail
            result[s_p] = 1 - self._p_fail  

        return result 
        
    def S(self):
        return self._S
        
    def A(self,state=None):
        """All actions in the MDP is state=None, otherwise actions available
        from state"""
        
        return self._actions
    
    def is_terminal(self, state):
        '''returns whether or not a state is terminal'''
        
#        state = PatrolExtendedState(state.location,None,None)
        return all(state.location == self._terminal.location)
    
    def is_legal(self,state):
        
        l = state.location
        (r,c) = self._map.shape
        g = state.current_goal
        
        legal_loc = l[0] >= 0 and l[0] < r and \
            l[1] >= 0 and l[1] < c and \
            l[2] >= 0 and l[2] < 4 and \
            self._map[ l[0],l[1] ] == 1
        legal_cg = g >= 0 and g <= 4
        
        validgoal2 = g==2 and not all(l[0:2] == array([9,0]))
        validgoal4 = g==4 and not all(l[0:2] == array([0,8]))
        
        validgoal = validgoal4 or validgoal2 
                
        return (legal_loc and legal_cg and validgoal)
    
    def __str__(self):
        format = 'GWModel [p_fail={},terminal={}]'
        return format.format(self._p_fail, self._terminal)
    
    def info(self):
        result = [str(self) + '\n']
        map_size = self._map.shape
        for i in reversed(range(map_size[0])):
            for j in range(map_size[1]):
                if self._map[i,j] == 1:
                    result.append( '[O]' )
                else:
                    result.append( '[X]')
            result.append( '\n' )
        return ''.join(result)

class OGMap:
    
    def __init__(self, themap, topRow, bottomRow, leftCol, rightCol):
        self.themap = themap
        self.topRow = topRow
        self.bottomRow = bottomRow
        self.leftCol = leftCol
        self.rightCol = rightCol
        
        (self.rowSquares, self.colSquares) = themap.shape
      
    def toState(self, pos, useAttackerState = True):
        
        # first do a straightforward conversion:
        row = int( (self.rowSquares -1) - round ( (pos[1] - self.bottomRow) / ((self.topRow - self.bottomRow) / (self.rowSquares - 1)) ) )
        col = int( round ( (pos[0] - self.leftCol) / ((self.rightCol - self.leftCol) / (self.colSquares - 1)) ) )
    
        if pos[2] < math.pi / 4 or pos[2] > 7 * math.pi / 4:
            direction = 0
        elif pos[2] > 3 * math.pi / 4 and pos[2] < 5 * math.pi / 4:
            direction = 2
        elif  pos[2] <= 3 * math.pi / 4 and pos[2] >=  math.pi / 4:
            direction = 1
        else:
            direction = 3
        
        if self.is_valid((row, col)):
            return self.buildState((row, col, direction), useAttackerState)
        
        # that state was invalid, are we outside of the map boundaries?
        row = max(0, min(self.rowSquares - 1, row))
        col = max(0, min(self.colSquares - 1, col))
        
        if self.is_valid((row, col)):
            return self.buildState((row, col, direction), useAttackerState)
        
        # Do a search for the closest valid state
        
        minValid = None
        minValidDistance = sys.maxint
        x = 2
        while minValid is None:
            for i in range(row - x, row + x + 1):
                for j in range(col - x, col + x + 1):
                    if self.is_valid((i, j)):
                        state = self.buildState((i, j, direction), useAttackerState)
                        s = self.toPos(state)
                        dist = (pos[0] - s[0])*(pos[0] - s[0]) + (pos[1] - s[1])*(pos[1] - s[1]) 
                        
                        if (dist < minValidDistance):
                            minValidDistance = dist
                            minValid = state
            x += 1                
        
        return minValid            
    
    def buildState(self, coords, useAttackerState):
        if (useAttackerState):
            return AttackerState(np.array([coords[0], coords[1]]), coords[2], 0)
        else:
            return PatrolState(np.array([coords[0], coords[1], coords[2]]))
        
    def toPos(self, state):    

        row = (- state.location[0] + (self.rowSquares - 1)) * ((self.topRow - self.bottomRow) / (self.rowSquares - 1)) + self.bottomRow
        col = (state.location[1]) * ((self.rightCol - self.leftCol) / (self.colSquares - 1) ) + self.leftCol
        if (state.__class__.__name__ == "AttackerState"):
            a = math.pi * state.orientation / 2.0
        else:
            a = math.pi * state.location[2] / 2.0
    
        return (col,row, a)
    
    def is_valid(self, coords):
        return coords[0] >= 0 and coords[1] >= 0 and coords[0] < self.rowSquares and coords[1] < self.colSquares and self.themap[coords[0], coords[1]] == 1
    
    def theMap(self):
        return self.themap

def boyd2MapParams(attacker):
    if attacker:
        themap = np.array( [[1, 1, 1, 1, 1, 1, 1, 1, 1], 
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 1, 1, 1, 1, 1, 1, 1]])
    else:
        themap = np.array( [[0, 1, 1, 1, 1, 1, 1, 1, 1], 
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 1, 1, 1, 1, 1, 1, 1]])
        
    return ( themap, 25.55, 8.65, 7.25, 16.52 )

def boydrightMapParams(attacker):
    
    # if (attacker):
    #     themap =      np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    #                             [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0]])
    # else:
    #     themap =      np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
    #                             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0]])
        
    if (attacker):
        themap =      np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                                [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0]])
    else:
        themap =      np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0]])

    return ( themap, 25.91, 13.19, 35, 52)


def boydright2MapParams(attacker):
    if (attacker):
        themap = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0]])
    else:
        themap = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0]])

    return (themap, 26.81, 13.19, 35, 52)


def largeGridMapParams(attacker):
#     if attacker:
#         themap = np.array( [[1, 1, 1, 0, 0, 1, 1, 1, 1],
#                  [1, 1, 1, 0, 0, 1, 1, 1, 1],
#                  [1, 1, 1, 1, 1, 1, 1, 1, 1],
#                  [0, 1, 0, 0, 0, 0, 0, 1, 0],
#                  [1, 1, 1, 0, 0, 1, 1, 1, 1],
#                  [1, 1, 1, 0, 0, 1, 1, 1, 1],
#                  [1, 1, 1, 1, 1, 1, 1, 1, 1],
#                  [1, 1, 1, 0, 0, 1, 1, 1, 1]])
#     else:
#         themap = np.array( [[1, 1, 1, 0, 0, 1, 1, 1, 1],
#                  [1, 1, 1, 0, 0, 1, 1, 1, 1],
#                  [1, 1, 1, 1, 1, 1, 1, 1, 1],
#                  [0, 1, 0, 0, 0, 0, 0, 1, 0],
#                  [1, 1, 1, 0, 0, 1, 1, 1, 1],
#                  [1, 1, 1, 0, 0, 1, 1, 1, 1],
#                  [1, 1, 1, 1, 1, 1, 1, 1, 1],
#                  [1, 1, 1, 0, 0, 1, 1, 1, 1]])
#         
#     return ( themap, 25.55, 8.65, 7.25, 16.52 )
    if attacker:
        themap = np.array( [[0, 0, 0, 0, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0]])
    else:
        themap = np.array( [[0, 0, 0, 0, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0]] )
        
    return ( themap, 25.55, 8.35, 7.65, 19.15 )

def reducedGridMapParams(attacker):
#     if attacker:
#         themap = np.array( [[0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [1, 1, 1, 1, 1, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0],
#             [1, 1, 1, 1, 1, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0]])
#     else:
#         themap = np.array( [[0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [1, 1, 1, 1, 1, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0],
#             [1, 1, 1, 1, 1, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0]] )
    if attacker:
        themap = np.array( [[0, 0, 0, 0, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0]])
    else:
        themap = np.array( [[0, 0, 0, 0, 1, 1, 1, 1, 1],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0]] )
                
    return ( themap, 25.55, 8.35, 7.65, 19.15 )

def convertPatrollerStateToAttacker(state):
    return AttackerState(np.array([state.location[0], state.location[1]]), state.location[2], 0)
