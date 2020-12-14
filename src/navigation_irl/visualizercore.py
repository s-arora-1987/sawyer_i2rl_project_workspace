# import roslib; roslib.load_manifest('navigation_irl')

import sys
import patrol.model
import numpy as np
import random
import mdp.simulation
import mdp.solvers
#import mdp.agent
import util.classes
import patrol.solvers
import math
from patrol.model import *
from mdp.simulation import simulate
import tf


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
        if model.is_legal(s) and not model.is_terminal(s) and s._time == num:
            for (a, p) in policy.actions(s).iteritems():
                continue
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
        if (action is None):
        	break
        newMdpState = action.apply(startState)
        if not model.is_legal(newMdpState):
            newMdpState = startState
            newMdpState._time += 1
            
        np.setPos( np, Point3( squareSize * (newMdpState._location[1] - startState._location[1]), -squareSize * (newMdpState._location[0] - startState._location[0]), distBetweenLevels) )     
        points.append( np.getPos( ) )
        startState = newMdpState
        i += 1
        
    segs = LineSegs( ) 
    segs.setThickness( 4.0 ) 
    segs.setColor( Vec4(0,1,0,1) )
    segs.moveTo( points[0] ) 
    for p in points[1:]: segs.drawTo( p ) 
    render.attachNewNode(segs.create( ))    


def drawRealAttacker(states):
    global squareSize
    global distBetweenLevels

    np = NodePath( 'optPlan')     
    points = [ ] 

    np.setPos( 1 + (states[0]._location[1] * squareSize) - .5*squareSize, 1 + (-states[0]._location[0] * squareSize) + .5*squareSize, 0) 
    points.append( np.getPos( ) )
    prevState = states[0]
    for (time, state) in enumerate(states):
        if time > 0:    
            np.setPos( np, Point3( squareSize * (state._location[1] - prevState._location[1]), -squareSize * (state._location[0] - prevState._location[0]), distBetweenLevels) )     
            points.append( np.getPos( ) )
        prevState = state
        
    segs = LineSegs( ) 
    segs.setThickness( 4.0 ) 
    segs.setColor( Vec4(0,1,1,1) )
    segs.moveTo( points[0] ) 
    for p in points[1:]: segs.drawTo( p ) 
    render.attachNewNode(segs.create( ))    
    
    
def drawPatrollers(patrollers_traj):
    global squareSize
    global distBetweenLevels
    
    counter = 0
    for patroller in patrollers_traj:
        t = 0
        for (state, prob) in patroller:
            np = NodePath( 'patrollerarrows' + str(counter) )
            counter += 1
            points = [ ] 
            orientation = state.location[2]
                            
            if orientation == 1:
                offset = .1        
            else:
                offset = -.1
                    
#                if orientation == 1 and state._location[0] < 2:
#                    if state._location[1] == 0:
#                        np.setH( 180 )
#                    else:
#                        np.setH( 0 )                 
#                elif orientation == 0 and state._location[0] < 2:
#                    if state._location[1] == 0:
#                        np.setH( 0 )                 
#                    else:
#                        np.setH( 180 )                 
#                else:
#                    if orientation == 1:
#                        np.setH( 270 )                 
#                    else:
 #                       np.setH( 90 )
            if orientation == 0:
                np.setH(90)
            if orientation == 1:
                np.setH(180)
            if orientation == 2:
                np.setH(270)
            if orientation == 3:
                np.setH(0)
																			
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

def drawRealPatrollers(patrollerStates):
    global squareSize
    global distBetweenLevels
    
    counter = 0
    for patroller in patrollerStates:
        t = -10
        for (state, orientation) in patroller:
            np = NodePath( 'patrollerarrows' + str(counter) )
            counter += 1
            points = [ ] 
            
            np.setH( orientation + 90)
            np.setPos( 1 + (state._location[1] * squareSize) - .5*squareSize,  1 + (-state._location[0] * squareSize) + .25*squareSize, t * distBetweenLevels + .5) 
            points.append(np.getPos())
            np.setPos(np, Point3( -squareSize * .1, squareSize * .33, 0 ))
            points.append(np.getPos())
            np.setPos(np, Point3( squareSize * .2, 0, 0 ))
            points.append(np.getPos())
            np.setPos(np, Point3( -squareSize * .1, -squareSize * .33, 0 ))
            points.append(np.getPos())

            segs = LineSegs( ) 
            segs.setThickness( 10.0 ) 
            segs.setColor( Vec4(1,1,0,1) )
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
      
def visualize(timeLeftAt, model, policy, values, patroller_positions, attackerStates, patrollerStates):
    
    # init panda3d
    # draw base base MDP
    # draw policy arrows
    # find optimum go time
    # draw plan connection lines
    # draw patroller positions, use the alpha color value to display the prob of a position
    global mode

    import direct.directbase.DirectStart     

    
    for t in range(-10, model._maxTime):
        drawmap(model, t, t)
    
        drawpolicy(model, policy, t, t)
        
    
    maxval = -sys.maxint - 1
    goTime = -1
    for t in range(model._maxTime):
        attackerStates[0]._time = t
        if values[attackerStates[0]] > maxval:
            maxval = values[attackerStates[0]]
            goTime = t
            
    print ("Maximum Value", maxval, "@", goTime )
    print ("Attacker Left At", timeLeftAt)
    attackerStates[0]._time = timeLeftAt
    goTime = timeLeftAt
    drawPlan(model, policy, attackerStates[0])
    
    drawPatrollers(patroller_positions)
    
    drawRealAttacker(attackerStates)
    drawRealPatrollers(patrollerStates)
    
    
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
   
    base.camera.setPos( 0, 50, 0 ) 
    base.camera.lookAt( 0, 0, 0 ) 
    run ()


def grid_main(bestTime, timeScale, patrollerModels, patrollerStartStates, patrollerTimes, attackerPositions, patrollerPositions, attacker_policy, values, mapToUse, predicttime, interactionlength):
    random.seed()
    np.random.seed()
    
    ## Initialize constants

    p_fail = 0.05
    longHallway = 10
    shortSides = 2
    patrolAreaSize = longHallway + shortSides + shortSides
    observableStateLow = 6
    observableStateHigh = 7
    
    
    # calculate farness for each node in the patrolled area
    farness = np.zeros(patrolAreaSize)
    for i in range(patrolAreaSize):
        sum = 0
        for j in range(patrolAreaSize):
            sum += abs(i - j)
    
        farness[i] = sum
        

    if (mapToUse == "boydright"):
    	themap = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 0], 
				     [1, 0, 1, 1, 1, 1, 0, 1, 0],
				     [1, 0, 0, 0, 1, 1, 1, 1, 1],
				     [1, 0, 0, 0, 0, 0, 1, 1, 1],
				     [1, 1, 1, 1, 1, 0, 0, 1, 0],
				     [0, 0, 0, 0, 1, 1, 1, 1, 0]])    
    else:

		themap = np.array(  [[1, 1, 1, 1, 1], 
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],
				     [1, 0, 0, 0, 0],				     
				     [1, 0, 0, 0, 0],				     
				     [1, 1, 1, 1, 1]])	
    	
    	
    ## Create Model
    model = patrol.model.PatrolModel(p_fail, None, themap)
    model.gamma = 0.99

    t_max = math.ceil( math.log(0.01)/math.log(model.gamma) )   
    print(t_max)
    ## Create initial distribution
    initial = util.classes.NumMap()
    for s in model.S():
        initial[s] = 1.0
    initial = initial.normalize()
    
    
    policies = patrollerModels

#    print 'Policy: '
#    for s in model.S():
#        if not model.is_terminal(s):
#            print '\tpi({}) = {}'.format(s, policies[0].actions(s))

#    print()
#    for s in model.S():
#        if not model.is_terminal(s):
#            print '\tpi({}) = {}'.format(s, policies[1].actions(s))

    pmodel = model

    pmodel._p_fail = 0

    reward = 10
    penalty = -10
    detectDistance = 3
    

    ## Create Model
    p_fail = .20
    goalState = patrol.model.AttackerState(np.array([4,2]), 1)
    model = patrol.model.AttackerModel(p_fail, themap, predicttime, goalState)
    model.gamma = 0.95
	
    for s in model.S():
		print(s, values[s])

    # read in the bag files, and using the time offset and timescale parameters generate a list of actual patroller and attacker positions at each mdp timestep
    trajectory = mdp.simulation.create_trajectories(pmodel, policies, patrollerStartStates, patrollerTimes, predicttime, interactionlength)

    visualize(bestTime, model, attacker_policy, values, trajectory, attackerPositions, patrollerPositions)
    
    exit()    
    
    

def runVis(gotimesLog, attackerpolicyLog, attackerBag, patroller1Bag, patroller2Bag, ptime, delaylength, mapToUse, policyLog, timeScale):   
    # read in and process bag and log files
    
    import pickle
    
    # need to know when the attacker left at, the patroller start states and start times
    
    # load the  patroller model if exists
    
    # look through each of the bags to generate a list of actual positions for each robot at each timestep of the mdp
    f = open(gotimesLog, "r")
    decisions = pickle.load(f)
    f.close()
        
    lastOne = decisions[len(decisions) - 1]
    print(lastOne)
    goTime = lastOne[0]
    patrollerStates = lastOne[3]
    patrollerTimes = lastOne[4]
    bestTime = lastOne[5]
    
    predicttime = int(ptime)	
    interactionlength = int(delaylength)

    
    baseTime = goTime - (bestTime * timeScale)
    patrollerModels = None
    if not policyLog is None:
        f = open(policyLog, "r")
        patrollerModels = pickle.load(f)
        f.close()

    attackerPolicy = None
    values = None
    f = open(attackerpolicyLog, "r")
    l = pickle.load(f)
    attackerPolicy = l[0][lastOne[6]]
    values = l[1][lastOne[6]]
    f.close()


    import roslib; roslib.load_manifest('navigation_irl')

    import rospy
    import rosbag
    
    attackerPositions = []
    curState = 0
    curGoalTime = baseTime
    bag = rosbag.Bag(attackerBag)
    for topic, msg, t in bag.read_messages(topics=['/robot_2/base_pose_ground_truth']):
        if t.to_sec() >= curGoalTime:
            q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))

            x, y, angle = tf.transformations.euler_from_quaternion(q)
            if (angle < 0):
                angle = 2 * math.pi + angle
            attackerPositions.append(convertPositionToState( (msg.pose.pose.position.x, msg.pose.pose.position.y, angle), mapToUse))
            
            curState += 1
            curGoalTime += timeScale
    bag.close()

   
    patrollerPositions = [[],[]]
    i = 0
    curState = 0
    curGoalTime = baseTime - (10 * timeScale)
    bag = rosbag.Bag(patroller1Bag)

    for topic, msg, t in bag.read_messages(topics=['/robot_0/base_pose_ground_truth']):
        if t.to_sec() >= curGoalTime:
            q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))

            x, y, angle = tf.transformations.euler_from_quaternion(q)
            if (angle < 0):
                angle = 2 * math.pi + angle

            patrollerPositions[i].append((convertPositionToState( (msg.pose.pose.position.x, msg.pose.pose.position.y, angle), mapToUse), angle * 57.2957795))
            
            curState += 1
            curGoalTime += timeScale
    bag.close()
    i = 1
    curState = 0
    curGoalTime = baseTime - (10 * timeScale)
    bag = rosbag.Bag(patroller2Bag)
    for topic, msg, t in bag.read_messages(topics=['/robot_1/base_pose_ground_truth']):
        if t.to_sec() >= curGoalTime:
            q = np.array((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w ))

            x, y, angle = tf.transformations.euler_from_quaternion(q)
            if (angle < 0):
                angle = 2 * math.pi + angle
            
            patrollerPositions[i].append((convertPositionToState( (msg.pose.pose.position.x, msg.pose.pose.position.y, angle), mapToUse), angle * 57.2957795))
            
            curState += 1
            curGoalTime += timeScale
    bag.close()
    
    grid_main(bestTime, timeScale, patrollerModels, patrollerStates, patrollerTimes, attackerPositions, patrollerPositions, attackerPolicy, values, mapToUse, predicttime, interactionlength)
