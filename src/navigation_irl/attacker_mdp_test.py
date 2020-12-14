import sys
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

def convertPatrollerStateToAttacker(state):
	if state.location[0] <= 2:
		x1 = state.location[0]
		y1 = 0
	elif state.location[0] >= 11:
		x1 = 2 - (state.location[0] - (11))
		y1 = 9
	else:
		x1 = 2
		y1 = state.location[0] - 2
	
	return AttackerState(np.array([x1, y1]), 0)

from pandac.PandaModules import NodePath 
from pandac.PandaModules import Point3 
from pandac.PandaModules import Vec4 
from pandac.PandaModules import LineSegs   

from panda3d.core import Vec3, Quat, WindowProperties 

squareSize = .5
distBetweenLevels = 2

def drawmap(model, basePos, num):
    global squareSize
    global distBetweenLevels
    
    from panda3d.core import TextNode
    text = TextNode("levellabel" + str(num))
    text.setText(str(num))
    t = render.attachNewNode(text)
    t.setPos(-1, 0, basePos * distBetweenLevels)
    
    np = NodePath( 'modelpen' + str(num) ) 
    squares = [ ] 
    
    for x in range(model._map.shape[0]):
        for y in range(model._map.shape[1]):
            if (model._map[x,y] == 1):
                points = []
                np.setPos( 1 + (y * squareSize), 1 + (-x * squareSize), basePos * distBetweenLevels ) 
                np.setH( 0) 
                points.append( np.getPos( ) ) 
                np.setPos( np, Point3( 0, squareSize, 0) ) 
                points.append( np.getPos( ) ) 
                np.setH( np, 90) 
                np.setPos( np, Point3( 0, squareSize, 0) ) 
                points.append( np.getPos( ) ) 
                np.setH( np, 90) 
                np.setPos( np, Point3( 0, squareSize, 0) ) 
                points.append( np.getPos( ) ) 
                np.setH( np, 90) 
                np.setPos( np, Point3( 0, squareSize, 0) ) 
                points.append( np.getPos( ) ) 
                np.setH( np, 90) 
            
                squares.append(points)
            
    segs = LineSegs( ) 
    segs.setThickness( 2.0 ) 
    segs.setColor( Vec4(1,1,0,1) )
    for points in squares:
        segs.moveTo( points[0] ) 
        for p in points[1:]: segs.drawTo( p ) 
    render.attachNewNode(segs.create( ))

def drawpolicy(model, policy, basePos, num):
    global squareSize
    global distBetweenLevels
    
    np = NodePath( 'modelarrows' + str(num) ) 
    arrows = [ ] 
    
    for s in model.S():
        if model.is_legal(s) and not model.is_terminal(s) and s._time == num - 1:
            for (a, p) in policy.actions(s).iteritems():

                points = []
                np.setPos( 1 + (s._location[1] * squareSize) - .5*squareSize, 1 + (-s._location[0] * squareSize) + .5*squareSize, basePos * distBetweenLevels ) 
                if a._direction[0] == -1:
                    # x up
                    np.setH( 0) 
                elif  a._direction[0] == 1:
                    # x down
                    np.setH( 180) 
                elif  a._direction[1] == -1:
                    # y left
                    np.setH( 90) 
                else:
                    # y right
                    np.setH( 270) 
                    
                points.append( np.getPos( ) ) 
                np.setPos( np, Point3( 0, - (.75*squareSize) / 2, 0) ) 
                points.append( np.getPos( ) ) 
                np.setPos( np, Point3( 0, .75 *squareSize, 0) ) 
                points.append( np.getPos( ) ) 
                np.setH( np, 135) 
                np.setPos( np, Point3( 0, .1*squareSize, 0) ) 
                points.append( np.getPos( ) ) 
                np.setPos( np, Point3( 0, -.1*squareSize, 0) ) 
                points.append( np.getPos( ) ) 
                np.setH( np, 90) 
                np.setPos( np, Point3( 0, .1*squareSize, 0) ) 
                points.append( np.getPos( ) ) 
            
                arrows.append(points)
            
    segs = LineSegs( ) 
    segs.setThickness( 2.0 ) 
    segs.setColor( Vec4(1,1,1,1) )
    for points in arrows:
        segs.moveTo( points[0] ) 
        for p in points[1:]: segs.drawTo( p ) 
    render.attachNewNode(segs.create( ))

def drawPlan(model, policy, startState):
    global squareSize
    global distBetweenLevels
    
    i = startState._time

    np = NodePath( 'optPlan')     
    points = [ ] 

    np.setPos( 1 + (startState._location[1] * squareSize) - .5*squareSize, 1 + (-startState._location[0] * squareSize) + .5*squareSize, startState._time * distBetweenLevels ) 
    points.append( np.getPos( ) )
    while not model.is_terminal(startState) and i < model._maxTime:
        action = policy.actions(startState).keys()[0]
        newMdpState = action.apply(startState)
        if not model.is_legal(newMdpState):
            newMdpState = startState
            newMdpState._time += 1
            
        np.setPos( np, Point3( squareSize * (newMdpState._location[1] - startState._location[1]), -squareSize * (newMdpState._location[0] - startState._location[0]), distBetweenLevels) )     
        points.append( np.getPos( ) )
        startState = newMdpState
        
    segs = LineSegs( ) 
    segs.setThickness( 4.0 ) 
    segs.setColor( Vec4(0,1,0,1) )
    segs.moveTo( points[0] ) 
    for p in points[1:]: segs.drawTo( p ) 
    render.attachNewNode(segs.create( ))    

def drawPatrollers(patrollers_traj):
    global squareSize
    global distBetweenLevels
    
    counter = 0
    for patroller in patrollers_traj:
        t = 0
        for timestep in patroller:
            for (state, prob, orientation) in timestep:
                np = NodePath( 'patrollerarrows' + str(counter) )
                counter += 1
                points = [ ] 
                
                if orientation == 1:
                    offset = .1        
                else:
                    offset = -.1
                    
                if orientation == 1 and state._location[0] < 2:
                    if state._location[1] == 0:
                        np.setH( 180 )
                    else:
                        np.setH( 0 )                 
                elif orientation == 0 and state._location[0] < 2:
                    if state._location[1] == 0:
                        np.setH( 0 )                 
                    else:
                        np.setH( 180 )                 
                else:
                    if orientation == 1:
                        np.setH( 270 )                 
                    else:
                        np.setH( 90 )                    
                np.setPos( 1 + (state._location[1] * squareSize) - .5*squareSize, offset + 1 + (-state._location[0] * squareSize) + .5*squareSize, t * distBetweenLevels + .1) 
                points.append(np.getPos())
                np.setPos(np, Point3( -squareSize * .1, squareSize * .33, 0 ))
                points.append(np.getPos())
                np.setPos(np, Point3( squareSize * .2, 0, 0 ))
                points.append(np.getPos())
                np.setPos(np, Point3( -squareSize * .1, -squareSize * .33, 0 ))
                points.append(np.getPos())

                segs = LineSegs( ) 
                segs.setThickness( 6.0 * prob ) 
                segs.setColor( Vec4(1,0,0,1) )
                segs.moveTo( points[0] ) 
                for p in points[1:]: segs.drawTo( p ) 
                render.attachNewNode(segs.create( ))
            t += 1

class MouseLook(object): 
    
   MLMFPPNoCenter = 0 
   MLMOrbit = 1 
   MLMPan = 2 
   MLMFPPCenter = 3 
    
   def __init__(self, targetCam = None, targetWin = None): 
      self.setTargetCamera(targetCam) 
      self.setTargetWin(targetWin) 
       
      self.orbitCenter = Point3(0, 0, 0) 
       
      self.mouseLookMode = self.MLMFPPCenter 
      self.zoomOrtho = False 
       
      self.limitH = { 
         "left": None, 
         "right": None, 
         "relative": render, 
      } 
       
      self.movementSpeed = 6.80 
      self.wheelSpeed = 8.0 
      self.mouseLookSpeed = [0.1, 0.1] 
       
      self.camMove = dict( 
         forward = False, 
         backward = False, 
         strafeLeft = False, 
         strafeRight = False, 
         down = False, 
         up = False, 
      ) 
    
   def zoom(self, direction): 
      step = 64 
      top = 32768 
      bottom = 64 
       
      fsH, size = self.lens.getFilmSize() 
      size -= direction * step 
       
      if size < bottom: 
         size = bottom 
       
      for vp in self.editorBase.viewports[1:4]: 
         vp.zoomLocal(size) 
    
   def zoomLocal(self, size): 
      fsH = size * self.aspectRatio 
      fsV = size 
      self.lens.setFilmSize(fsH, fsV) 
    
    
    
   def enable(self, ownsTask = True): 
      self.prevX = self.targetWin.getPointer(0).getX() 
      self.prevY = self.targetWin.getPointer(0).getY() 
      if ownsTask: 
         taskMgr.add(self.update, "UpdateMouseLook") 
    
   def disable(self): 
      taskMgr.remove("UpdateMouseLook") 
    
   def clearMovement(self): 
      self.camMove = dict( 
         forward = False, 
         backward = False, 
         strafeLeft = False, 
         strafeRight = False, 
         down = False, 
         up = False, 
         ) 
    
   def setLimitH(self, left, right, relative = None): 
      if relative is None: 
         relative = render 
      self.limitH = { 
         "left": left, 
         "right": right, 
         "relative": relative 
      } 
    
   def enableMovement(self): 
      base.accept("w", self.setCamMove, extraArgs = ["forward", True]) 
      base.accept("w-up", self.setCamMove, extraArgs = ["forward", False]) 
      base.accept("s", self.setCamMove, extraArgs = ["backward", True]) 
      base.accept("s-up", self.setCamMove, extraArgs = ["backward", False]) 
      base.accept("a", self.setCamMove, extraArgs = ["strafeLeft", True]) 
      base.accept("a-up", self.setCamMove, extraArgs = ["strafeLeft", False]) 
      base.accept("d", self.setCamMove, extraArgs = ["strafeRight", True]) 
      base.accept("d-up", self.setCamMove, extraArgs = ["strafeRight", False]) 
       
      base.accept("control", self.setCamMove, extraArgs = ["down", True]) 
      base.accept("control-up", self.setCamMove, extraArgs = ["down", False]) 
      base.accept("space", self.setCamMove, extraArgs = ["up", True]) 
      base.accept("space-up", self.setCamMove, extraArgs = ["up", False]) 
       
      base.accept("wheel_up", self.moveCamera, extraArgs = [Vec3(0, 1, 0)]) 
      base.accept("wheel_down", self.moveCamera, extraArgs = [Vec3(0, -1, 0)]) 
    
   def setCamMove(self, key, val): 
      self.camMove[key] = val 
    
   def update(self, task = None): 
      mouse = self.targetWin.getPointer(0) 
      x = mouse.getX() 
      y = mouse.getY() 
       
      deltaX = (x - self.prevX) * self.mouseLookSpeed[0] 
      deltaY = (y - self.prevY) * self.mouseLookSpeed[1] 
       
      self.prevX = x 
      self.prevY = y 
       
      if self.mouseLookMode == self.MLMFPPNoCenter: 
#         print "update fpp no center" 
         self.updateFPPNoCenter(deltaX, deltaY) 
      elif self.mouseLookMode == self.MLMOrbit: 
#         print "update orbit" 
         self.updateOrbit(deltaX, deltaY) 
      elif self.mouseLookMode == self.MLMPan: 
 #        print "update pan" 
         self.updatePan(deltaX, deltaY) 
      if self.mouseLookMode == self.MLMFPPCenter: 
  #       print "update fpp center" 
         self.updateFPPCenter() 
       
      if self.limitH["left"] is not None: 
         rel = self.limitH["relative"] 
         h = self.targetCamera.getH(rel) 
          
         if h < self.limitH["left"]: 
            h = self.limitH["left"] 
         elif h > self.limitH["right"]: 
            h = self.limitH["right"] 
          
         self.targetCamera.setH(rel, h) 
       
      linVel = Vec3(0,0,0) 
      if self.camMove["forward"]: linVel[1] = self.movementSpeed 
      if self.camMove["backward"]: linVel[1] = -self.movementSpeed 
      if self.camMove["strafeLeft"]: linVel[0] = -self.movementSpeed 
      if self.camMove["strafeRight"]: linVel[0] = self.movementSpeed 
      if self.camMove["up"]: linVel[2] = self.movementSpeed 
      if self.camMove["down"]: linVel[2] = -self.movementSpeed 
       
      linVel *= globalClock.getDt() 
      self.moveCamera(linVel) 
       
      if task is not None: 
         return task.cont 
    
   def moveCamera(self, vector): 
      self.targetCamera.setPos(self.targetCamera, vector * self.wheelSpeed) 
    
   def rotateAround(self, node, point, axis, angle, relative): 
      quat = Quat() 
      quat.setFromAxisAngle(angle, render.getRelativeVector(relative, axis)) 
       
      relativePos = node.getPos(render) - point 
      relativePosRotated = quat.xform(relativePos) 
      absolutePosRotated = relativePosRotated + point 
       
      node.setPos(render, absolutePosRotated) 
      node.setQuat(render, node.getQuat(render) * quat) 
    
   def setTargetCamera(self, cam): 
      if cam is None: 
         self.targetCamera = base.camera 
      else: 
         self.targetCamera = cam 
    
   def setTargetWin(self, win): 
      if win is None: 
         self.targetWin = base.win 
      else: 
         self.targetWin = win 
    
   def setMouseModeRelative(self, state): 
      props = WindowProperties() 
      if not state: 
         props.setMouseMode(WindowProperties.MAbsolute) 
      else: 
         props.setMouseMode(WindowProperties.MRelative) 
      self.targetWin.requestProperties(props) 
    
   def setCursorHidden(self, state): 
      props = WindowProperties() 
      props.setCursorHidden(state) 
      self.targetWin.requestProperties(props) 
    
   def updateFPPNoCenter(self, deltaX, deltaY): 
      p = self.targetCamera.getP() - deltaY 
      h = self.targetCamera.getH() - deltaX 
      if abs(p) > 90: 
         p = 90 * cmp(p, 0) 
      self.targetCamera.setP(p) 
      self.targetCamera.setH(h) 
    
   def updateFPPCenter(self): 
      winSizeX = self.targetWin.getXSize()/2 
      winSizeY = self.targetWin.getYSize()/2 
       
      mouse = self.targetWin.getPointer(0) 
       
      x = mouse.getX() - winSizeX 
      y = mouse.getY() - winSizeY 
       
      h = x * (self.mouseLookSpeed[0]) 
      h = self.targetCamera.getH() - h 
      self.targetCamera.setH(h) 
       
      p = y * (self.mouseLookSpeed[1]) 
      p = self.targetCamera.getP() - p 
      if p < 90.0 and p > -90.0: 
         self.targetCamera.setP(p) 
       
      self.targetWin.movePointer(0, winSizeX, winSizeY) 
    
   def updateOrbit(self, deltaX, deltaY): 
      self.rotateAround(self.targetCamera, self.orbitCenter, Vec3(0, 0, 1), -deltaX, render) 
      self.rotateAround(self.targetCamera, self.orbitCenter, Vec3(1, 0, 0), -deltaY, self.targetCamera) 
    
   def updatePan(self, deltaX, deltaY): 
      vector = Vec3(-deltaX, 0, deltaY) * 1/globalClock.getDt() * 0.01 
      self.moveCamera(vector) 
      
mode = None
      
def visualize(model, policy, values, patroller_positions, startState):
    
    # init panda3d
    # draw base base MDP
    # draw policy arrows
    # find optimum go time
    # draw plan connection lines
    # draw patroller positions, use the alpha color value to display the prob of a position
    global mode

    import direct.directbase.DirectStart     

    
    for t in range(model._maxTime):
        drawmap(model, t, t + 1)
    
        drawpolicy(model, policy, t, t + 1)
        
    
    maxval = -sys.maxint - 1
    goTime = -1
    for t in range(model._maxTime):
        startState._time = t
        if values[startState] > maxval:
            maxval = values[startState]
            goTime = t
            
    print ("Maximum Value", maxval, "@", goTime + 1)
    startState._time = goTime
    drawPlan(model, policy, startState)
    
    drawPatrollers(patroller_positions)
    
        
    
    
    base.accept("escape", sys.exit) 
    base.disableMouse() 
    
    m = MouseLook() 
    m.enable() 
    m.enableMovement() 
    m.setMouseModeRelative(True) 
    m.setCursorHidden(True) 
    m.mouseLookMode = 3
    
    mode = m.mouseLookMode 
    def cycle(): 
      global mode
      mode += 1 
      if mode > 3: 
         mode = 0 
      print mode 
      m.mouseLookMode = mode 
    
    base.accept("f1", cycle) 
   
    base.camera.setPos( 0, 0, 100 ) 
    base.camera.lookAt( 0, 0, 0 ) 
    run ()


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
#    feature_function = mdp.etc.StateFeatureFunction(model)
#    feature_function = gridworld.etc.GWLocationFF(model)

    ## Define player
#    policy = mdp.agent.HumanAgent(model)
    opt_policy = mdp.solvers.ValueIteration(.1).solve(model)

    pmodel = model

    pmodel._p_fail = 0

    map = np.array( [[1, 0, 0, 0, 0, 0, 0, 0, 0, 1], 
			     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
			     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]])

    predictTime = 30
    reward = 10
    penalty = -10
    policies = [opt_policy, opt_policy]
    detectDistance = 3
    

    ## Create Model
    goalState = patrol.model.AttackerState(np.array([2,0]), 0)
    model = patrol.model.AttackerModel(p_fail, map, predictTime, goalState)
    model.gamma = 0.99

    patrollerStartStates = [patrol.model.PatrolState(np.array([8,0])), patrol.model.PatrolState(np.array([6,1]))]
    patrollerStartStatesAttackerEquiv = [convertPatrollerStateToAttacker(patrollerStartStates[0]), convertPatrollerStateToAttacker(patrollerStartStates[1])]
    patrollerTimes = [0,0]

#    pmodel._p_fail = 0
 
    reward = patrol.reward.AttackerRewardPatrollerPolicy(goalState, reward, penalty, pmodel, policies, patrollerStartStatesAttackerEquiv, patrollerStartStates , patrollerTimes, detectDistance, predictTime)
    reward_weights = np.zeros( reward.dim )
    reward_weights[0] = 1


    reward.params = reward_weights
    model.reward_function = reward

    (attacker_policy, values) = mdp.solvers.ValueIteration2(1).solve(model)
    
#    policy = mdp.solvers.QValueIterator(100).solve(model)
#    policy = mdp.solvers.LSPI(40,100, feature_f=feature_function).solve(model)
#    policy = mdp.solvers.LSTD(50, 100, feature_f=feature_function).solve(model)
#    policy = mdp.solvers.PolicyIterator(20, mdp.solvers.IteratingPolicyEvaluator(100)).solve(model)
#    policy = mdp.solvers.PolicyIterator(20, mdp.solvers.SamplingPolicyEvaluator(100,50)).solve(model)
#    policy = mdp.solvers.PolicyIterator(20, mdp.solvers.ExactPolicyEvaluator()).solve(model)
    
    ## Print out world information
    print model.info()
    print reward.info()
    print 'States: ' + str( [str(state) for state in model.S()] )
    print 'Action: ' + str( [str(action) for action in model.A()] )
    print 'Policy: '
    for s in model.S():
        if not model.is_terminal(s):
            print '\tpi({}) = {}'.format(s, attacker_policy.actions(s))
    
    
    # read in the bag files, and using the time offset and timescale parameters generate a list of actual patroller and attacker positions at each mdp timestep
    
    visualize(model, attacker_policy, values, reward._patrollerPositions, patrol.model.AttackerState(np.array([2,4]), 0))
    
    exit()    
    
    ## Estimate policy quality
    print 'Sample run:'
    for (s,a,r) in mdp.simulation.simulate(model, policy, initial, 20):
        print '%s, %s, %f' % (s,a,r)
#    print 'Average Score: %f' % (evaluate_policy(model, initial, policy, t_max),)
    mdp.etc.policy_report(opt_policy, policy, mdp.solvers.ExactPolicyEvaluator(), model, initial)
    
    ## Do IRL
#    irl = mdp.solvers.IRLExactSolver(20, mdp.solvers.ValueIteration(40))
#    (est_policy, w_est) = irl.solve(model, initial, opt_policy) 

#    irl = mdp.solvers.IRLApprximateSolver(50, mdp.solvers.ValueIteration(100), 1)
#    irl = mdp.solvers.IRLApprximateSolver(20, mdp.solvers.ValueIteration(50), 500)
    irl = patrol.solvers.IRLApprximateSolver(30,mdp.solvers.PolicyIteration(50, mdp.solvers.IteratingPolicyEvaluator2(.1)), 200)
        
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