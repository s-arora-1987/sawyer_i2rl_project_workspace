#!/usr/bin/env python
# import roslib; roslib.load_manifest('navigation_irl')

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
import tf
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
learned_weights=[[0.0]*6,[0.0]*6] # needed for compute reldiff
num_Trajsofar=0
normedRelDiff=2.0 # start with a high value to call update()
stopI2RLThresh=0.5 #0.1 
sessionStart=False
sessionFinish=False
lineFoundWeights=""
lineFeatureExpec=""
lineLastBeta=""
lineLastZS=""
lineLastZcount=""

print_once=0

class FakeModel():

	def T(self, state, action):

		return {0 : 1}

	def A(self):
		return [0, ]




# a strategy that waits a random amount of time and then goes, ignores all percepts
class WrapperRandom():

	def __init__(self, mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance):

		global obstime
		self.goTime = random.randint(obstime, obstime+600)
		self.policy = None
		print("The Go time is at: " + str(self.goTime))
		self.predictTime = 30

		self.map = mapToUse	
		self.observableStateLow = 0
		self.observableStateHigh = 0

		global attackerOGMap
		
		p_fail = 0.05
		
		## Create Model
		self.goalStates = [self.getStateFor(goalPos), ]
		self.goalState = self.goalStates[0]

		model = patrol.model.AttackerModel(p_fail, attackerOGMap.theMap(), self.predictTime, self.goalStates[0])
		model.gamma = 0.99

		self.model = model
		self.startState = self.getStateFor(startPos)
					
		import threading

		t = threading.Thread(target=self.solve)
		self.thread = t
		self.thread.start()

	def solve(self):
		import time
		global patroller0GroundTruth
		global patroller1GroundTruth
		
		while(patroller0GroundTruth is None or patroller1GroundTruth is None):
			time.sleep(1)
		
		
		print("Starting MDP Solver")
		
		lastPatroller0State = getGroundTruthStateFor(0)
		lastPatroller1State = getGroundTruthStateFor(1)
	
		patrollerStartStates = [lastPatroller0State, lastPatroller1State]
		patrollerTimes = [0,0]
	
	
		print("PatrollerStartStates", patrollerStartStates)
		print("PatrollerTimes", patrollerTimes, "curRospy", get_time())
	
		args = ["boydattacker", ]
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
		
		# build trajectories
		outtraj = self.map + "\n"
		outtraj += "false\n"
		
		for stateS in patrollerStartStates:
			outtraj += "["
			outtraj += str(int(stateS.location[0]))
			outtraj += ", "
			outtraj += str(int(stateS.location[1]))
			outtraj += ", "
			outtraj += str(int(stateS.location[1]))
			outtraj += "];"		
		
		outtraj += "\n"
		
		for startTime in patrollerTimes:
			outtraj += str(startTime)
			outtraj += ";"
		outtraj += "\n"
	
		outtraj += str(1)
		outtraj += "\n"
		outtraj += str(30)
		outtraj += "\n"
	
		outtraj += str(0)	
		outtraj += "\n"
		outtraj += str(10)	
		outtraj += "\n"
		outtraj += str(0)	
		outtraj += "\n"
		outtraj += str(1)	
		outtraj += "\n"
				

		f = open(get_home() + "/patrolstudy/toupload/patpolicy.log", "w")
		f.write(outtraj)
		f.close()		
		
		(stdout, stderr) = p.communicate(outtraj)

		# stdout now needs to be parsed into a hash of state => action, which is then sent to mapagent
		pols = []
		vs = []
		p = {}
		v = {}
		stateactions = stdout.split("\n")
		for stateaction in stateactions:
	
			if (stateaction == "ENDPROBS"):
				continue
			
			if len(stateaction) < 2:
				# we've found where the marker between two policies
				pols.append(p)
				vs.append(v)
				p = {}
				v = {}
				continue
			temp = stateaction.split(" = ")
			if len(temp) < 2: continue
			state = temp[0]
			value = temp[1]
			action = temp[2]
	
			
			state = state[2 : len(state) - 1]
			pieces = state.split(",")	
			
			ps = patrol.model.AttackerState(np.array([int(pieces[0]), int(pieces[1][0 : len(pieces[1]) - 1])]) , int(pieces[2]), int(pieces[3]))
		
			if action.strip() == "null":
				p[ps] = None
			else:	
				if action.strip() == "AttackerMoveForward":
					a = patrol.model.AttackerMoveForward()
				elif action.strip() == "AttackerTurnLeft":
					a = patrol.model.AttackerTurnLeft()
				else:
					a = patrol.model.AttackerTurnRight()
	
				p[ps] = a
			v[ps] = float(value)
	
	
		policies = []
		for q in pols:
			policies.append(mdp.agent.MapAgent(q))			
		
		# choose randomly between available goals
		# print(policies)
		whichone = random.randint(0,len(policies) - 1)
		self.policy = policies[whichone]
		self.goalState = self.goalStates[whichone]
				
		print("Finished MDP Solving")
		

	def getStateFor(self, position):
		global attackerOGMap
		
		return attackerOGMap.toState(position, True)

	def getPositionFor(self, mdpState):
		global attackerOGMap
		
		return attackerOGMap.toPos(mdpState)


	def latestPolicy(self):
		return self.policy

	def goNow(self):
		return fromRospyTime(get_time()) >= fromRospyTime(self.goTime)

	def getPatrollerStateActionFor(self, curPos, lastPos, lastPos2, lastTime):
		return (0, 1)

	def patrollerModel(self):
		return FakeModel()

	def addPercept(self, patroller, state, time):
		pass

	def getStartState(self):
		return self.startState

	def getGoalState(self):
		return self.goalState

	def update(self):
		pass

	def getModel(self):
		return self.model


def classifyAction(outputVec):
	global mapToUse
	
	vecs = []
	if (mapToUse == "boyd2"):
		vecs.append(numpy.array([0.10471529247895256,	0.27178226770618075,	0.33862569390802433,	0.375])) # moveforward
		vecs.append(numpy.array([0.5,	0.5,	0.9841229182759271,	0.375])) # turnleft
		vecs.append(numpy.array([0.5,	0.5,	0.5,	1.0])) # turnright
		vecs.append(numpy.array([0.5,	0.9564354645876385,	0.33862569390802433,	0.375])) # turnaround
		vecs.append(numpy.array([0.8952847075210475,	0.27178226770618075,	0.33862569390802433,	0.375])) # stop
	else:
		# these need to be redone when a NN for boydright is built
		vecs.append(numpy.array([0.10471529247895256,	0.27178226770618075,	0.33862569390802433,	0.375])) # moveforward
		vecs.append(numpy.array([0.5,	0.5,	0.9841229182759271,	0.375])) # turnleft
		vecs.append(numpy.array([0.5,	0.5,	0.5,	1.0])) # turnright
		vecs.append(numpy.array([0.5,	0.9564354645876385,	0.33862569390802433,	0.375])) # turnaround
		vecs.append(numpy.array([0.8952847075210475,	0.27178226770618075,	0.33862569390802433,	0.375])) # stop
		
	closest = -1
	closestVal = sys.float_info.max
	
	for i, v in enumerate(vecs):
		dist = numpy.linalg.norm(outputVec - v)
		if (dist < closestVal):
			closest = i
			closestVal = dist
	
	if (closest == 0):
		return PatrolActionMoveForward()
	if (closest == 1):
		return PatrolActionTurnLeft()
	if (closest == 2):
		return PatrolActionTurnRight()
	if (closest == 3):
		return PatrolActionTurnAround()
	if (closest == 4):
		return PatrolActionStop()
	
	return None


def mean_and_variance(data):
	n	= 0
	sum1 = 0
	sum2 = 0

	for x in data:
		n	= n + 1
		sum1 = sum1 + x

	mean = sum1/n

	for x in data:
		sum2 = sum2 + (x - mean)*(x - mean)

	variance = sum2/(n - 1)
	return (mean, variance)

actionNN = None

def getPatrollerActionNN(readings):
	from pybrain.tools.shortcuts import buildNetwork
	from pybrain.structure import TanhLayer

	if (len(readings) <= 2):
		return None
	
	global actionNN
	if (actionNN is None):
	
		global mapTouse
		
		if (mapToUse == "boyd2"):
			nnWeights = numpy.array([-0.3304985698,-2.0350233909,1.4252249243,0.2963310102,4.9266064369,-0.1472800993,0.061455457,0.223676191,0.0025171819,-0.517450847,2.543845079,0.2942835103,-0.0823201426,0.3745702746,-0.0281529442,0.2206690533,-0.0985171095,0.0054585868,-36.1536614237,0.0980005071,-10.0402714801,4107.922303848,1.1756230206,-11.173458288,-78.4043090397,1.4706464857,545.5495422035,83336.9157690647,222.1574064718,5.3629004192,-11354.3282550292,482.0961046906,3.627796326,18485.1522666649,3.8994555386,-2.0162859479,-711.4848036675,-0.095789549,-5.1913028932,12165.1412947653,0.9587726743,0.4053123806,20.7827205806,0.3963309953,953.7460206174,41254.089509375,13.8877636884,-55.7693899285,107854.5579788469,-115.024748763,-161.8372335656,-229.7124016302,5.2355727181,-21.1921433795,6814.8934279982,11.8144628413,-1516.9409075926,-178436.1615841909,45.4057518591,-1116.6717947839,-1231.3555799918,26.5367868278,0.2946896214,1107.3431771647,0.0970529269,15.9504145508,1435.8551242887,-0.4690070429,-0.0752631963,-0.7997826301,0.3840072903,0.6206706334,1621.9304807605,-0.1605863922,20.9367198862,5351.8031245516,-0.9362696644,3.1561242116,195.9966687042,-0.1201570823,-4.9454580981,184.8361757175,0.160018912,-33.0211667279,-18176.7060075939,0.871384414,3.5221200995,240.7632822965,-0.2080971545,-1.0405001885,-856.4937261407,0.1298636107,0.5177587901,3069.1714221005,-0.3756751408,-3.9467678449,-33.7275333761,0.040441488,0.132976886,724.3911891873,0.218647283,24.8460698888,2181.8699154502,-0.9915000002,-3.3831296597,10.2921621295,0.7589195007,572.6591220021,32336.6540817552,11.444372836,165.7540263761,-41587.6179129253,-7.3935572673,21.5767608761,-4238.8100528394,36.9535371488,-0.1774941572,1115.1030922934,-0.0162510915,-17.6029184258,2815.2262040795,0.6461238176,1.6241572646,21.2920354298,0.0356162823,0.197993219,-1393.7550083931,0.188329235,10.5948853241,-3788.7678184607,0.2442199395,5.9897149839,-154.4688647851,0.3610844208,-0.0618358173,-0.7957428078,0.3269138991,0.3342662757,-0.2142045179,-0.7582433437,-0.2780321189,0.1074139583,0.0390365181,0.37232585,-0.1611990394,1.1381611431,-0.0017546681,-0.9970171851,-0.0387787452,0.1102100351,0.1756448881,-0.1405135597,-3.407333392,0.2331252469,-0.3326675975,-1.5063271236,2.1723786962,-0.031489985,2.3384290129,1.1644199035,1.0954118938,-0.1759323485,0.2932166683,0.4798292085,-0.4851544844,1.4122326915,-1.2010908245,0.5232572567,1.4147323864,-1.1501400115,-0.1435034775,0.0709863455,-1.6319298998,-0.0440954912,-0.0462593535,0.9498094825,0.0793733352,-0.0069429358,1.6773864397,0.1632380311,0.2338671661,0.6668889845,-0.8915792212,-0.480782618,-0.6649133478,0.0568622766])
		else:
			# this is totally wrong, but the NN is worse than the manual method so whatever for now 
			nnWeights = numpy.array([-0.3304985698,-2.0350233909,1.4252249243,0.2963310102,4.9266064369,-0.1472800993,0.061455457,0.223676191,0.0025171819,-0.517450847,2.543845079,0.2942835103,-0.0823201426,0.3745702746,-0.0281529442,0.2206690533,-0.0985171095,0.0054585868,-36.1536614237,0.0980005071,-10.0402714801,4107.922303848,1.1756230206,-11.173458288,-78.4043090397,1.4706464857,545.5495422035,83336.9157690647,222.1574064718,5.3629004192,-11354.3282550292,482.0961046906,3.627796326,18485.1522666649,3.8994555386,-2.0162859479,-711.4848036675,-0.095789549,-5.1913028932,12165.1412947653,0.9587726743,0.4053123806,20.7827205806,0.3963309953,953.7460206174,41254.089509375,13.8877636884,-55.7693899285,107854.5579788469,-115.024748763,-161.8372335656,-229.7124016302,5.2355727181,-21.1921433795,6814.8934279982,11.8144628413,-1516.9409075926,-178436.1615841909,45.4057518591,-1116.6717947839,-1231.3555799918,26.5367868278,0.2946896214,1107.3431771647,0.0970529269,15.9504145508,1435.8551242887,-0.4690070429,-0.0752631963,-0.7997826301,0.3840072903,0.6206706334,1621.9304807605,-0.1605863922,20.9367198862,5351.8031245516,-0.9362696644,3.1561242116,195.9966687042,-0.1201570823,-4.9454580981,184.8361757175,0.160018912,-33.0211667279,-18176.7060075939,0.871384414,3.5221200995,240.7632822965,-0.2080971545,-1.0405001885,-856.4937261407,0.1298636107,0.5177587901,3069.1714221005,-0.3756751408,-3.9467678449,-33.7275333761,0.040441488,0.132976886,724.3911891873,0.218647283,24.8460698888,2181.8699154502,-0.9915000002,-3.3831296597,10.2921621295,0.7589195007,572.6591220021,32336.6540817552,11.444372836,165.7540263761,-41587.6179129253,-7.3935572673,21.5767608761,-4238.8100528394,36.9535371488,-0.1774941572,1115.1030922934,-0.0162510915,-17.6029184258,2815.2262040795,0.6461238176,1.6241572646,21.2920354298,0.0356162823,0.197993219,-1393.7550083931,0.188329235,10.5948853241,-3788.7678184607,0.2442199395,5.9897149839,-154.4688647851,0.3610844208,-0.0618358173,-0.7957428078,0.3269138991,0.3342662757,-0.2142045179,-0.7582433437,-0.2780321189,0.1074139583,0.0390365181,0.37232585,-0.1611990394,1.1381611431,-0.0017546681,-0.9970171851,-0.0387787452,0.1102100351,0.1756448881,-0.1405135597,-3.407333392,0.2331252469,-0.3326675975,-1.5063271236,2.1723786962,-0.031489985,2.3384290129,1.1644199035,1.0954118938,-0.1759323485,0.2932166683,0.4798292085,-0.4851544844,1.4122326915,-1.2010908245,0.5232572567,1.4147323864,-1.1501400115,-0.1435034775,0.0709863455,-1.6319298998,-0.0440954912,-0.0462593535,0.9498094825,0.0793733352,-0.0069429358,1.6773864397,0.1632380311,0.2338671661,0.6668889845,-0.8915792212,-0.480782618,-0.6649133478,0.0568622766])

		#build neural net
		
		actionNN = buildNetwork(9, 13, 4, hiddenclass = TanhLayer, outclass = TanhLayer, bias = True)
		actionNN._setParameters(nnWeights)

	# calculate inputs
	
	deltaxs = []
	lastx = None
	deltays = []
	lasty = None
	deltathetas = []
	lasttheta = None
	
	for r in readings:
				
		if (lasttheta is not None):
			test = r[2] - lasttheta
			while (test < -math.pi): test += 2*math.pi;
			while (test > math.pi): test -= 2*math.pi;
			
			deltathetas.append(test)
			
		lasttheta = r[2]
					
		if (lastx is not None):
			x = math.cos(lasttheta) * (r[0] - lastx) + math.sin(lasttheta)* (r[1] - lasty) 
#				print("x", msg.pose.pose.position.x, lastx, x, firsttheta)
			deltaxs.append(x)
		
		lastx = r[0]

		if (lasty is not None):
			y = math.sin(lasttheta) * (r[0] - lastx) + math.cos(lasttheta) * (r[1] - lasty) 
#				print("y", msg.pose.pose.position.y, lasty, y, firsttheta)				
			deltays.append(y)
		
		lasty = r[1]
			
			
	sumx = reduce(operator.add, deltaxs, 0)
	(meanx, variancex) = mean_and_variance(deltaxs)
	sumy = reduce(operator.add, deltays, 0)
	(meany, variancey) = mean_and_variance(deltays)
	sumtheta = reduce(operator.add, deltathetas, 0)
	(meantheta, variancetheta) = mean_and_variance(deltathetas)
	
	
	return classifyAction(actionNN.activate((meanx, variancex, sumx, meany, variancey, sumy, meantheta, variancetheta, sumtheta)))
	

def getPatrollerActionForStates(states):
	# returns the action that was performed at states[0] to get to states[1] and states[2 - 3] (if given)
		
	if (len(states) <= 1 or states[1] is None):

		return PatrolActionMoveForward()


	if states[0].location[0] == states[1].location[0] and states[0].location[1] == states[1].location[1]:
		# position is the same, have we turned or just stopped?       

		# check if we've turned
		if not (states[0].location[2] == states[1].location[2]):
#			if (abs(states[0].location[2] - states[1].location[2]) == 2):
#				return PatrolActionTurnAround()
			
			if (states[0].location[2] == 0) and (states[1].location[2] == 3):
				return PatrolActionTurnRight()
					
			if (states[0].location[2] == 3) and (states[1].location[2] == 0):
				return PatrolActionTurnLeft()
				
			if states[0].location[2] > states[1].location[2]:
				return PatrolActionTurnRight()
			
			return PatrolActionTurnLeft()
		
		
		# nope we're the same, we must've stopped
		return PatrolActionStop()
		
		
	else:
		# position is different, check if we moved forward by looking at the orientation
		
		if states[0].location[2] == 0 and (states[0].location[0] == states[1].location[0]) and (states[0].location[1] == states[1].location[1] - 1):
			return PatrolActionMoveForward()

		if states[0].location[2] == 2 and (states[0].location[0] == states[1].location[0]) and (states[0].location[1] - 1 == states[1].location[1]):
			return PatrolActionMoveForward()
		
		if states[0].location[2] == 1 and (states[0].location[0] -1 == states[1].location[0]) and (states[0].location[1] == states[1].location[1]):
			return PatrolActionMoveForward()

		if states[0].location[2] == 3 and (states[0].location[0] == states[1].location[0] - 1) and (states[0].location[1] == states[1].location[1]):
			return PatrolActionMoveForward()
		
		# donno ?
		return None			
	
	
def getPatrollerSubActions(readingsWithinTimestep, state1, state2 ):

	actionCount = [0, 0, 0, 0]
	
	for i in range(len(readingsWithinTimestep) - 1):
	
		# calculate angle to turn for point 1 to face point 2
		# calculate distance travelled
		# calculate angle to turn to final orientation

		# calculate the angle between the points
		# calculate the amount turned between the current orientation and the intermediate angle
		# calculate the amount turned from the intermediate angle to the final one
		
		distance = math.sqrt((readingsWithinTimestep[i][0]-readingsWithinTimestep[i + 1][0])*(readingsWithinTimestep[i][0]-readingsWithinTimestep[i + 1][0]) + (readingsWithinTimestep[i][1]-readingsWithinTimestep[i + 1][1])*(readingsWithinTimestep[i][1]-readingsWithinTimestep[i + 1][1]))		
		
		if (distance > 0.0):
			diff = math.atan2(readingsWithinTimestep[i + 1][1] - readingsWithinTimestep[i][1], readingsWithinTimestep[i + 1][0] - readingsWithinTimestep[i][0])
		else:
			diff = 0
		if (diff < 0):
			diff += 2 * math.pi

		angle1 = diff - readingsWithinTimestep[i][2] 
		while (angle1 < -math.pi/2): angle1 += math.pi;
		while (angle1 > math.pi/2): angle1 -= math.pi;
		
		angle2 = readingsWithinTimestep[i + 1][2] - diff
		while (angle2 < -math.pi/2): angle2 += math.pi;
		while (angle2 > math.pi/2): angle2 -= math.pi;
		
		angle = angle1 + angle2
		
#		angle = readingsWithinTimestep[i + 1][2] - readingsWithinTimestep[i][2] 
#		while (angle < -3.1415/2): angle += 3.1415;
#		while (angle > 3.1415/2): angle -= 3.1415;

#		print(str(readingsWithinTimestep[i]) + ", " + str(readingsWithinTimestep[i+1]) + " => " + str(angle) + " " + str(distance))
		# classify this action:
		# a forward will have good forward movement (duh)
		# A stop will have very little forward or angular movement
		# A turn will have very little forward movement, but good angular
		
		if (distance < .005) and abs(angle) < 0.03:
			actionCount[1] += 1 # stop
		elif abs(angle) >= 0.02:  
			if angle > 0:
				actionCount[2] += 1 # leftTurn
			else:
				actionCount[3] += 1 # rightTurn
		else: # .15 meters a second for a tenth of a second
			actionCount[0] += 1 # moveforward
	
	return actionCount


def getPatrollerAction(readingsWithinTimestep, state1, state2 ):

	
	states = []
	states.append(state1)
	states.append(state2)	
	discreteAction = getPatrollerActionForStates(states)

	actionCount = getPatrollerSubActions(readingsWithinTimestep, state1, state2)	
	
	# now decide whether to use the discrete-states-calulated action or the substates one		
	# need to decide if the substates dictate a strong action or not, this really should be done probabilistically, or with a NN, but
	# conference paper limits being what they are, we're going to go with something quickly explainable. YAY SCIENCE!
	
	highest = 0
	for i in range(len(actionCount)):
			
		if (len(readingsWithinTimestep) > 0):
			actionCount[i] /= float(len(readingsWithinTimestep))

		if actionCount[i] > actionCount[highest]:
			highest = i
			
	
	
#	if (len(readingsWithinTimestep) >= (getTimeConv() / 2 / 0.1)) and ((actionCount[2] > 0.7 * len(readingsWithinTimestep) - 1) or (actionCount[3] > 0.7 * len(readingsWithinTimestep) - 1)):
#		return PatrolActionTurnAround()
	
	# detect a quick turn left or right by comparing the start and end orientations
	# if change >= 90 degrees, it's definitely a turn
	
	if len(readingsWithinTimestep) > 0 and not PatrolActionTurnAround().__eq__(discreteAction):
		orientationNet = readingsWithinTimestep[len(readingsWithinTimestep) - 1][2] - readingsWithinTimestep[0][2]
		while (orientationNet < -math.pi): orientationNet += math.pi * 2;
		while (orientationNet > math.pi): orientationNet -= math.pi * 2;
		
#		if abs(orientationNet) > math.pi / 1.2:
#			return PatrolActionTurnAround()
		
		if abs(orientationNet) > math.pi/2.2:
			# force a left or right turn
						
			if (orientationNet > 0):
				return PatrolActionTurnLeft()
			else:
				return PatrolActionTurnRight()
	
	
#	if (actionCount[1]) > 0.5 * len(readingsWithinTimestep) - 1:
#		return PatrolActionStop()
	
#	if (actionCount[2] >= 0.25 * len(readingsWithinTimestep) - 1) or (actionCount[3] > 0.25 * len(readingsWithinTimestep) - 1):
		# we have a turn
		
#		if actionCount[2] > actionCount[3]:
#			return PatrolActionTurnLeft()
#		return PatrolActionTurnRight()

	if (highest == 0 and (discreteAction is None or (actionCount[highest] > 0.5 and len(readingsWithinTimestep) > 4))):	
		return PatrolActionMoveForward()
	
	if (highest == 1 and (discreteAction is None or (actionCount[highest] > 0.75 and len(readingsWithinTimestep) > 4))):	
		return PatrolActionStop()

	if (highest == 2 and (discreteAction is None or (actionCount[highest] > 0.75 and len(readingsWithinTimestep) > 4))):	
		return PatrolActionTurnLeft()

	if (highest == 3 and (discreteAction is None or (actionCount[highest] > 0.75 and len(readingsWithinTimestep) > 4))):
		return PatrolActionTurnRight()
	
	return discreteAction
	
def convertObsToDiscreteStates(obsin):
	# convert raw position observations to states
	obs = obsin[:]
	global patrollerOGMap
	
		
	discreteStates = []
	
	for i in range(fromRospyTime(get_time())):
		timeLowerRange = i * getTimeConv()
		timeUpperRange = (i + 1) * getTimeConv()
		
		if (len(obs) > 0):
			sumObsInTimeStep = [0, 0, 0, 0]

		countObsInTimeStep = 0
	
		while (len(obs) > 0 and obs[0][1] >= timeLowerRange and obs[0][1] < timeUpperRange):
#			for j in range(len(obsInTimestep) - 1, max(len(obsInTimestep) - numOfReadings - 1, -1), -1):
#			for j in range(0, min(len(obsInTimestep), numOfReadings)):
#			for j in range(0, len(obsInTimestep)):
			curObs =  obs.pop(0)[0]
			
			if countObsInTimeStep < 3:
		
				sumObsInTimeStep[0] += curObs[0]
				sumObsInTimeStep[1] += curObs[1]
				sumObsInTimeStep[2] += math.cos(curObs[2])
				sumObsInTimeStep[3] += math.sin(curObs[2])
				
			
				countObsInTimeStep += 1

		if countObsInTimeStep > 0:
			avg = [sumObsInTimeStep[0] / countObsInTimeStep, sumObsInTimeStep[1] / countObsInTimeStep, math.atan2(sumObsInTimeStep[3], sumObsInTimeStep[2]) ]
			
			if avg[2] < 0:
				avg[2] += 2 * math.pi
			s = patrollerOGMap.toState(avg, False)
			
			discreteStates.append(s)
		else:
			discreteStates.append(None)
		
	
	discreteStates.append(None)
	
	return discreteStates
		
		
def convertObsToTrajectoryWPolicy(obsin, policy):
	discreteStates = convertObsToDiscreteStates(obsin)
		
	traj = []
	for ds in discreteStates:
		if (ds is None):
			traj.append(None)
		else:
			a = policy.actions(ds).keys()[0]
			traj.append((ds, a))
			
	return traj	
		
		
def convertObsToTrajectory(obsin):
	
	discreteStates = convertObsToDiscreteStates(obsin)
	
	obs = obsin[:]
	
	traj = []
	
	
	for i in range(fromRospyTime(get_time())):
		timeLowerRange = i * getTimeConv()
		timeUpperRange = (i + 1) * getTimeConv()
		
		obsInTimestep = []
	
		while (len(obs) > 0 and obs[0][1] >= timeLowerRange and obs[0][1] < timeUpperRange):
			obsInTimestep.append(obs.pop(0)[0])


		if len(obsInTimestep) > 0:
			
			a = getPatrollerAction(obsInTimestep, discreteStates[len(traj)], discreteStates[len(traj) + 1])
			
			traj.append((discreteStates[len(traj)], a))
		else:
			traj.append(None)
		
	
	return traj	
		
def convertObsToTrajectoryUncertainActions(obsin):
	
	discreteStates = convertObsToDiscreteStates(obsin)
	
	obs = obsin[:]
	
	traj = []
	
	
	for i in range(fromRospyTime(get_time())):
		timeLowerRange = i * getTimeConv()
		timeUpperRange = (i + 1) * getTimeConv()
		
		obsInTimestep = []
	
		while (len(obs) > 0 and obs[0][1] >= timeLowerRange and obs[0][1] < timeUpperRange):
			obsInTimestep.append(obs.pop(0)[0])


		if len(obsInTimestep) > 0:
			
			a = getPatrollerSubActions(obsInTimestep, discreteStates[len(traj)], discreteStates[len(traj) + 1])
			
			traj.append((discreteStates[len(traj)], a))
		else:
			traj.append(None)
		
	
	return traj	



def filter_traj_for_t_solver(traj1, traj2):
	
	# remove entries that correspond to interaction states
	
	# remove unmodellable entries by inserting a blank space between them
	a = PatrolActionMoveForward()
	returnval1 = []
	returnval2 = []
	
	for i in range(len(traj1)):
		if (i <= 1):
			returnval1.append(None)
			returnval2.append(None)
			continue
			
		
		if (traj1[i] is not None and traj2[i] is not None):
			t1 = a.apply(traj1[i][0])
			t2 = a.apply(traj2[i][0])
			
			if (traj1[i][0].conflicts(traj2[i][0])) or (traj1[i][0].conflicts(t2)) or (traj2[i][0].conflicts(t1)):
				returnval1.append(None)
				returnval2.append(None)
			else:
				returnval1.append(traj1[i])
				returnval2.append(traj2[i])
		else:
			returnval1.append(traj1[i])
			returnval2.append(traj2[i])				
	
	return (returnval1, returnval2)
	

def printTrajectories(trajs):
	outtraj = ""
	for patroller in trajs:
		for sap in patroller:
			if (sap is not None):
				outtraj += "["
				outtraj += str(int(sap[0].location[0]))
				outtraj += ", "
				outtraj += str(int(sap[0].location[1]))
				outtraj += ", "
				outtraj += str(int(sap[0].location[2]))
				outtraj += "]:"
				if sap[1].__class__.__name__ == "PatrolActionMoveForward":
					outtraj += "MoveForwardAction"
				elif sap[1].__class__.__name__ == "PatrolActionTurnLeft":
					outtraj += "TurnLeftAction"
				elif sap[1].__class__.__name__ == "PatrolActionTurnRight":
					outtraj += "TurnRightAction"
				elif sap[1].__class__.__name__ == "PatrolActionTurnAround":
					outtraj += "TurnAroundAction"
				else:
					outtraj += "StopAction"
					
				outtraj += ":"
				outtraj += "1"
				outtraj += ";"
			
			outtraj += "\n"
		outtraj += "ENDTRAJ\n"
	return outtraj
	
def printTrajectoriesStatesOnly(trajs):
	outtraj = ""
	for patroller in trajs:
		for sap in patroller:
			if (sap is not None):
				outtraj += "["
				outtraj += str(int(sap[0].location[0]))
				outtraj += ", "
				outtraj += str(int(sap[0].location[1]))
				outtraj += ", "
				outtraj += str(int(sap[0].location[2]))
				outtraj += "]"			
			outtraj += ";"
		outtraj += "\nENDTRAJ\n"
	return outtraj	

def parseTs(stdout):
	T = []
	t = []
	weights = []
	transitions = stdout.split("\n")
	counter = 0
	for transition in transitions:
		counter += 1		
		if transition == "ENDT":
			T.append(t)
			t = []
			continue
		temp = transition.split(":")
		if temp[0] == "WEIGHTS":
			weights.append(temp[1])
			continue
		if len(temp) < 4: continue
		state = temp[0]
		action = temp[1]
		state_prime = temp[2]
		prob = float(temp[3])


		state = state[1 : len(state) - 1]
		state_prime = state_prime[1 : len(state_prime) - 1]
		pieces = state.split(",")	

		ps = [int(pieces[0]), int(pieces[1]), int(pieces[2])]

		pieces = state_prime.split(",")	
		ps_prime = [int(pieces[0]), int(pieces[1]), int(pieces[2])]

		if action == "MoveForwardAction":
			a = 0
		elif action == "TurnLeftAction":
			a = 1
		elif action == "TurnRightAction":
			a = 2
		elif action == "TurnAroundAction":
			a = 3
		else:
			a = 4
			
		t.append( (ps, a, ps_prime, prob))


	if (len(t)) > 0:
		T.append(t)
		
	while (len(T) < 2):
		T.append(T[0])
	
	return (T, weights)
	
def printTs(T):
	outtraj = ""
	for t1 in T:
		for t in t1:
			outtraj += "["
			outtraj += str(t[0][0])
			outtraj += ", "
			outtraj += str(t[0][1])
			outtraj += ", "
			outtraj += str(t[0][2])
			outtraj += "]:"
			if t[1] == 0:
				outtraj += "MoveForwardAction"
			elif t[1] == 1:
				outtraj += "TurnLeftAction"
			elif t[1] == 2:
				outtraj += "TurnRightAction"
			elif t[1] == 3:
				outtraj += "TurnAroundAction"
			else:
				outtraj += "StopAction"
			outtraj += ":["
			outtraj += str(t[2][0])
			outtraj += ", "
			outtraj += str(t[2][1])
			outtraj += ", "
			outtraj += str(t[2][2])
			outtraj += "]:"
			outtraj += str(t[3])
			outtraj += "\n"
			
		outtraj += "ENDT\n"	
	return outtraj


def printStochasticPolicies(pmodel, obs):
	
	# need to convert the observed subactions into distributions over actions for each state, with unseen states being uniform
	outtraj = ""
	
	for o in obs:
		traj = convertObsToTrajectoryUncertainActions(o)
		
		states = pmodel.S()
		
		action_counts = {}
		for s in states:
			action_counts[s] = [0, 0, 0, 0]
		
		for entry in traj:
			if entry is not None:
				for (i, count) in enumerate(entry[1]):
					action_counts[entry[0]][i] += count
					
		
		for (state, counts) in action_counts.iteritems():
			total = sum(counts)
			
			if total == 0:
				counts = [1 for i in range(len(counts))]
				total = len(counts)
				
			for (i, count) in enumerate(counts):
				if count > 0:
					outtraj += "["
					outtraj += str(int(state.location[0]))
					outtraj += ", "
					outtraj += str(int(state.location[1]))
					outtraj += ", "
					outtraj += str(int(state.location[2]))
					outtraj += "]:"
	
					if i == 0:
						outtraj += "MoveForwardAction"
					elif i == 1:
						outtraj += "StopAction"
					elif i == 2:
						outtraj += "TurnLeftAction"
					else:
						outtraj += "TurnRightAction"
				
					outtraj += ":"
					
					outtraj += str(float(count) / float(total))
					outtraj += "\n"
					
		
		outtraj += "ENDPOLICY\n"					

	return outtraj

def printUniformNEPriors():
	
	numNEs = 5
		
	outtraj = ""
	
	outtraj += "MoveForwardAction:MoveForwardAction:"
	outtraj += str(1.0 / numNEs)
	outtraj += "\n"
	
	outtraj += "StopAction:MoveForwardAction:"
	outtraj += str(1.0 / numNEs)
	outtraj += "\n"
	
	outtraj += "MoveForwardAction:StopAction:"
	outtraj += str(1.0 / numNEs)
	outtraj += "\n"

	outtraj += "TurnLeftAction:MoveForwardAction:"
	outtraj += str(1.0 / numNEs)
	outtraj += "\n"

	outtraj += "MoveForwardAction:TurnLeftAction:"
	outtraj += str(1.0 / numNEs)
	outtraj += "\n"
	
	return outtraj	
	

def parsePolicies(stdout, equilibrium):
	
	global lineFoundWeights, lineFeatureExpec, learned_weights, \
	num_Trajsofar, lineLastBeta, lineLastZS, lineLastZcount, \
	sessionFinish, BatchIRLflag

	if stdout is None:
		print("no stdout in parse policies")
	
	stateactions = stdout.split("\n")
	print("\n parse Policies from contents:")
	#print(stateactions)
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
					#print("at 2nd ende")
					#print(stateactions[counter :])
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
	
	#print("policies parsed")
	#print(returnval)
	
	#check if contents of session variables exist in results
	#if cond'n and sessionFinish are specific to i2rl 
	
	if len(stateactions[counter:])>0 and BatchIRLflag==False:
		sessionFinish = True
		print("\n sessionFinish = True, results after i2rl session:")
		#file = open("/home/saurabh/patrolstudy/i2rl_troubleshooting/I2RLOPread_rosctrl.txt","r")
		lineFoundWeights = stateactions[counter]
		counter += 1
		
		found_weights = [[0.0]*6,[0.0]*6]
		
		found_weights = [[float(x) for x in \
		lineFoundWeights.split("\n")[0]. \
		strip("[]").split("], [")[0].split(", ")], \
		[float(x) for x in \
		lineFoundWeights.split("\n")[0]. \
		strip("[]").split("], [")[1].split(", ")]]
		
		computeRelDiff(found_weights)
		
		learned_weights = found_weights
		
		print("lineFoundWeights:"+lineFoundWeights)
		
		lineFeatureExpec = stateactions[counter]
		counter += 1
		print("lineFeatureExpec:"+lineFeatureExpec)
		
		num_Trajsofar = int(stateactions[counter].split("\n")[0])
		counter += 1
		print("num_Trajsofar:"+str(num_Trajsofar))
		
		lineLastBeta = stateactions[counter]
		counter += 1
		print("lineLastBeta:"+lineLastBeta)
		
		lineLastZS = stateactions[counter]
		counter += 1
		print("lineLastZS:"+lineLastZS)
		
		lineLastZcount = stateactions[counter]
		print("lineLastZcount:"+lineLastZcount)
		
		'''
		LastBeta = [float(x) for x in \
		lineLastBeta.split("\n")[0]. \
		strip("[]").split(", ")]
		
		LastZS = [float(x) for x in \
		lineLastZS.split("\n")[0]. \
		strip("[]").split(", ")]
		
		LastZcount = int(lineLastZcount.split("\n")[0])
	
		print("info extracted. signalling abortrun in parsepolicies() method")
		abortRun()
		'''
	else:
		print("\n no results after i2rl")
				
	return returnval

rdiffSpikeTh=0.9

def computeRelDiff(weights2):
	
	global wt_data, normedRelDiff, num_Trajsofar
	
	print("wt_data")
	wt_data=numpy.append(wt_data, [weights2], axis=0)
	print(wt_data)
	
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
			#both unchanged
			#print(str(rel_diff1)+" "+str(rel_diff2)+" "+str("continue"))
			continue
		
		if (rel_diff1 < rdiffSpikeTh) and (num_Trajsofar > 0):
			# no spike comparison needed for first session
			if (rel_diff2 < rdiffSpikeTh):
				normedRelDiff=min(rel_diff1,rel_diff2)
			else:
				normedRelDiff=rel_diff1
		elif (rel_diff2 < rdiffSpikeTh) and (num_Trajsofar > 0):
			normedRelDiff=rel_diff2
		elif (num_Trajsofar == 0):
			normedRelDiff=min(rel_diff1,rel_diff2)
		else:
			print("\n both differences for current session are spikes")
		
		print("\n rel differences for session "+str(i+1)+", normedRelDiff :")
		print(str(rel_diff1)+" "+str(rel_diff2)+" "+str(normedRelDiff))

	return

def compareTrajectories(lastTrajectory, newTrajectory, prevcorrectentries = 0):
	cutoffpercent = 0.95
	cutoffchangeamount = 0.02
	correctentries = 0
	totalentries = 0
	
	for i, t in enumerate(lastTrajectory):
		for k, entry in enumerate(t):
			
			if entry is None:
				pass
			else:
				totalentries += 1
				if entry[1] == newTrajectory[i][k][1]:
					correctentries += 1
			
	print("Traj compare: " + str(correctentries) + " : " + str(totalentries))
	return ( ((float(correctentries) / totalentries ) > cutoffpercent or (abs(correctentries - prevcorrectentries)) < cutoffchangeamount * totalentries) , correctentries)

def idealirlsolve(q, add_delay, mapToUse, obs, pmodel, visibleStatesNum, equilibrium = "cs"):

	global patrollersuccessrate
	global usesimplefeatures

	traj = []
	traj.append(convertObsToTrajectory(obs[0]))
	traj.append(convertObsToTrajectory(obs[1]))

	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "w")
	f.write("")
	f.close()

	lasttrajchangeamount = 0
	returnval = None

	if patrollersuccessrate >= 0:

	
		args = [ 'boydsimple_t', ]
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		outtraj = ""
		
		outtraj += mapToUse + "\n"
		outtraj += str(patrollersuccessrate)
		
	else:

		t2 = traj[:]
		outtraj = ""
	
		if patrollersuccessrate == -2:
			args = [ 'boyd_em_t', ]	
			outtraj += str(visibleStatesNum / 14.0)	
			outtraj += "\n"
		else:
			args = [ 'boyd_t', ]
			t2 = filter_traj_for_t_solver(t2[0], t2[1])
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
		
		outtraj += mapToUse + "\n"
		outtraj += str(usesimplefeatures) + "\n"
		
		outtraj += printTrajectories(t2)
	
		f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "a")
		f.write(outtraj)
		f.write("ITERATE\n")
		f.close()			
	(stdout, stderr) = p.communicate(outtraj)


	(T, weights) = parseTs(stdout)
	
	stdout = None
	stderr = None
	outtraj = None
	
	f = open(get_home() + "/patrolstudy/toupload/t_weights.log", "w")
	for weight in weights:
		f.write(weight)
		f.write("\n")
	f.close()
	
	
	args = ["boydpatroller", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
	outtraj = mapToUse + "\n"
	
	outtraj += printTs(T)
	
	T = None
	(stdout, stderr) = p.communicate(outtraj)
	
	returnval = parsePolicies(stdout, equilibrium)
	
	stdout = None
	stderr = None
	outtraj = None
	

	if patrollersuccessrate >= 0:

	
		args = [ 'boydsimple_t', ]
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		outtraj = ""
		
		outtraj += mapToUse + "\n"
		outtraj += str(patrollersuccessrate)
		
	else:

		t2 = traj[:]
		outtraj = ""
	
		if patrollersuccessrate == -2:
			args = [ 'boyd_em_t', ]	
			outtraj += str(visibleStatesNum / 14.0)	
			outtraj += "\n"
		else:
			args = [ 'boyd_t', ]
			t2 = filter_traj_for_t_solver(t2[0], t2[1])
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
		
		outtraj += mapToUse + "\n"
		outtraj += str(usesimplefeatures) + "\n"
		
		outtraj += printTrajectories(t2)
	
		f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "a")
		f.write(outtraj)
		f.write("ITERATE\n")
		f.close()			
	(stdout, stderr) = p.communicate(outtraj)


	(T, weights) = parseTs(stdout)
	
	stdout = None
	stderr = None
	outtraj = None
	
	f = open(get_home() + "/patrolstudy/toupload/t_weights.log", "w")
	for weight in weights:
		f.write(weight)
		f.write("\n")
	f.close()
	
	returnval.append(T)
	q.put(returnval)
	
	q.close()
	q.join_thread()
	
	return

def irlsolve(q, obs, add_delay, algorithm, pmodel, mapToUse, NE, visibleStatesNum):

	#  create the interpolated trajectory, then give this to the attacker reward and solve,
	# use the last time we saw both attackers as the starting states
	#  now search the policy for the best values at the embarking point in time, 
	# set the go time to go at this time + the last time we saw both attackers
	# print("Starting MDP Solver")

	# augment the trajectory here
	## Create transfer reward function
	# find the patroller starting positions and times, use the latst seen time
	# execute the external solver
	
	#print("converting obs to trajectories")
	
	traj = []
	traj.append(convertObsToTrajectory(obs[0]))
	traj.append(convertObsToTrajectory(obs[1]))


	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "w")
	f.write("")
	f.close()

	f = open(get_home() + "/patrolstudy/toupload/traj_irl.log", "w")
	f.write("")
	f.close()
	
	global patrollersuccessrate
	global usesimplefeatures
	lasttrajchangeamount = 0
	
	policies = None
	
	#print("computing transitions for traj_irl.log")
		
	if patrollersuccessrate >= 0:

	
		args = [ 'boydsimple_t', ]
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		outtraj = ""
		
		outtraj += mapToUse + "\n"
		outtraj += str(patrollersuccessrate)
		

	else:


		t2 = traj[:]

		outtraj = ""
	
		if patrollersuccessrate == -2:
			args = [ 'boyd_em_t', ]	
			outtraj += str(visibleStatesNum / 14.0)	
			outtraj += "\n"
		else:
			args = [ 'boyd_t', ]
			t2 = filter_traj_for_t_solver(t2[0], t2[1])

	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		
		outtraj += mapToUse + "\n"
		outtraj += str(usesimplefeatures) + "\n"
	
		outtraj += printTrajectories(t2)

	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "a")
	f.write(outtraj)
	f.write("ITERATE\n")
	f.close()



	(stdout, stderr) = p.communicate(outtraj)


	(T, weights) = parseTs(stdout)
	
	stdout = None
	stderr = None
	outtraj = None

	f = open(get_home() + "/patrolstudy/toupload/t_weights.log", "w")
	for weight in weights:
		f.write(weight)
		f.write("\n")
	f.close()
		
		
	args = ["boydirl", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	# print("boydirl subprocess.Popen")
	
	# build trajectories
	outtraj = ""
	
	outtraj += mapToUse + "\n"
	
	if add_delay:
		outtraj += "true\n"
	else:
		outtraj += "false\n"


	if NE == "cc":
		equilibriumCode = 0
	elif NE == "sc":
		equilibriumCode = 1
	elif NE == "cs":
		equilibriumCode = 2
	elif NE == "tc":
		equilibriumCode = 3
	elif NE == "ct":
		equilibriumCode = 4
		
	outtraj += algorithm
	outtraj += "\n"
	outtraj += str(equilibriumCode)	
	outtraj += "\n"
	global equilibriumKnown
	if equilibriumKnown:
		outtraj += "true\n"
	else:
		outtraj += "false\n"
	global interactionLength
	outtraj += str(interactionLength)	
	outtraj += "\n"
	outtraj += str(visibleStatesNum / 14.0)	
	outtraj += "\n"
	#print("visibleStatesNum in traj_irl.log: ",str(visibleStatesNum / 14.0))
	
	outtraj += printTs(T)

	T = None
			
	outtraj += printTrajectories(traj)

	global learned_weights, learned_mu_E, lineFoundWeights, \
	num_Trajsofar, BatchIRLflag, lineLastBeta, \
	lineLastZS, lineLastZcount, sessionStart, wt_data
	
	#Even if i2rl params returned by boydirl aren't used for 
	#batch irl, they still must be used in both forms of irl
	#for calling boydirl
	if num_Trajsofar == 0:
		
		#create initial weights
		print("creating initial weights for num_Trajsofar == 0")
		for i in range(2):
			for j in range(6):
				learned_weights[i][j]=random.uniform(.01,.1)
		
		wt_data=numpy.array([learned_weights])
		
		lineFoundWeights = str(learned_weights)+"\n"
		#create initial feature expectations
		for i in range(2):
			for j in range(6):
				learned_mu_E[i][j]=0.0
		
		lineFeatureExpec = str(learned_mu_E)+"\n"
		#create initial LastBeta
		temp=[0.0]*6
		import math
		
		for j in range(6):
			temp[j]=-math.log(6)
		lineLastBeta = str(temp)+"\n"
		
		#create initial LastZS
		temp=[0.0]*6
		lineLastZS = str(temp)+"\n"
		
		#create initial zcount
		lineLastZcount = str(0)+"\n"
	
	print("params sent to boydirl")
	print("lineFoundWeights:"+lineFoundWeights+"\n lineFeatureExpec"+\
		lineFeatureExpec+"\n num_Trajsofar:"+str(num_Trajsofar)+"\n"\
		+"\n lineLastBeta: "+ lineLastBeta + "\n lineLastZS"+\
		lineLastZS + "\n lineLastZcount"+lineLastZcount)
	#pass previous weights +\n 
	#pass previous feature expectations
	#pass number of trajectories already used for training
	outtraj += lineFoundWeights+lineFeatureExpec+ str(num_Trajsofar)+"\n"
	
	#pass previous beta +\n , zs +\n , zcount +\n 
	outtraj += lineLastBeta + lineLastZS + lineLastZcount
	
	f = open(get_home() + "/patrolstudy/toupload/traj_irl.log", "a")
	f.write(outtraj)
	f.close()
	
	# abortRun()
	#print("calling boydirl at")
	#print(rospy.Time.now().to_sec())
				
	(stdout, stderr) = p.communicate(outtraj)
	print("boydirl (stdout, stderr) = p.communicate(outtraj)")
	
	policies = parsePolicies(stdout, None)
	
	stdout = None
	stderr = None
	outtraj = None
	

	if patrollersuccessrate >= 0:

	
		args = [ 'boydsimple_t', ]
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		outtraj = ""
		
		outtraj += mapToUse + "\n"
		outtraj += str(patrollersuccessrate)
		

	else:


		t2 = traj[:]

		outtraj = ""
	
		if patrollersuccessrate == -2:
			args = [ 'boyd_em_t', ]	
			outtraj += str(visibleStatesNum / 14.0)	
			outtraj += "\n"
		else:
			args = [ 'boyd_t', ]
			t2 = filter_traj_for_t_solver(t2[0], t2[1])

	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		
		outtraj += mapToUse + "\n"
		outtraj += str(usesimplefeatures) + "\n"
	
		outtraj += printTrajectories(t2)

	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "a")
	f.write(outtraj)
	f.write("ITERATE\n")
	f.close()

	(stdout, stderr) = p.communicate(outtraj)
	print("boydsimple_t (stdout, stderr) = p.communicate(outtraj)")

	(T, weights) = parseTs(stdout)
	
	stdout = None
	stderr = None
	outtraj = None

	f = open(get_home() + "/patrolstudy/toupload/t_weights.log", "w")
	for weight in weights:
		f.write(weight)
		f.write("\n")
	f.close()

	policies.append(T)			
	q.put(policies)	
	print("q.put(policies)")
	
	return
		
		
def emirlsolve(q, obs, add_delay, algorithm, pmodel, mapToUse, NE, visibleStatesNum):

	#  create the interpolated trajectory, then give this to the attacker reward and solve, use the last time we saw both attackers as the starting states
	#  now search the policy for the best values at the embarking point in time, set the go time to go at this time + the last time we saw both attackers
	print("Starting EMIRL Solver")

	# execute the external solver

	# convert the raw observatiosn to a trajectory
	traj = []
	traj.append(convertObsToTrajectory(obs[0]))
	traj.append(convertObsToTrajectory(obs[1]))


	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "w")
	f.write("")
	f.close()

	f = open(get_home() + "/patrolstudy/toupload/traj_irl.log", "w")
	f.write("")
	f.close()
	
	global patrollersuccessrate
	global usesimplefeatures
	lasttrajchangeamount = 0
	
	policies = None
	
	if patrollersuccessrate >= 0:

	
		args = [ 'boydsimple_t', ]
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		outtraj = ""
		
		outtraj += mapToUse + "\n"
		outtraj += str(patrollersuccessrate)
		

	else:


		t2 = traj[:]

		outtraj = ""
	
		if patrollersuccessrate == -2:
			args = [ 'boyd_em_t', ]	
			outtraj += str(visibleStatesNum / 14.0)	
			outtraj += "\n"
		else:
			args = [ 'boyd_t', ]
			t2 = filter_traj_for_t_solver(t2[0], t2[1])

	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		
		outtraj += mapToUse + "\n"
		outtraj += str(usesimplefeatures) + "\n"
	
		outtraj += printTrajectories(t2)

	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "a")
	f.write(outtraj)
	f.write("ITERATE\n")
	f.close()



	(stdout, stderr) = p.communicate(outtraj)


	(T, weights) = parseTs(stdout)
	
	stdout = None
	stderr = None
	outtraj = None

	f = open(get_home() + "/patrolstudy/toupload/t_weights.log", "w")
	for weight in weights:
		f.write(weight)
		f.write("\n")
	f.close()
		
	args = ["boydemirl", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
	# build trajectories
	outtraj = ""
	
	outtraj += mapToUse + "\n"
	
	if add_delay:
		outtraj += "true\n"
	else:
		outtraj += "false\n"


	if NE == "cc":
		equilibriumCode = 0
	elif NE == "sc":
		equilibriumCode = 1
	elif NE == "cs":
		equilibriumCode = 2
	elif NE == "tc":
		equilibriumCode = 3
	elif NE == "ct":
		equilibriumCode = 4
		
	outtraj += algorithm
	outtraj += "\n"
	outtraj += str(equilibriumCode)	
	outtraj += "\n"
	global interactionLength
	outtraj += str(interactionLength)	
	outtraj += "\n"
	outtraj += str(visibleStatesNum / 14.0)	
	outtraj += "\n"
	
	outtraj += printTs(T)

	T = None
			
	outtraj += printTrajectoriesStatesOnly(traj)

	outtraj += printStochasticPolicies(pmodel, obs)
	
	outtraj += printUniformNEPriors()
	
	
	f = open(get_home() + "/patrolstudy/toupload/traj_irl.log", "a")
	f.write(outtraj)
	f.close()
	
	(stdout, stderr) = p.communicate(outtraj)

	policies = parsePolicies(stdout, None)
	
	
	stdout = None
	stderr = None
	outtraj = None
	

	if patrollersuccessrate >= 0:

	
		args = [ 'boydsimple_t', ]
	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		outtraj = ""
		
		outtraj += mapToUse + "\n"
		outtraj += str(patrollersuccessrate)
		

	else:


		t2 = traj[:]

		outtraj = ""
	
		if patrollersuccessrate == -2:
			args = [ 'boyd_em_t', ]	
			outtraj += str(visibleStatesNum / 14.0)	
			outtraj += "\n"
		else:
			args = [ 'boyd_t', ]
			t2 = filter_traj_for_t_solver(t2[0], t2[1])

	
		p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
		
		outtraj += mapToUse + "\n"
		outtraj += str(usesimplefeatures) + "\n"
	
		outtraj += printTrajectories(t2)

	f = open(get_home() + "/patrolstudy/toupload/t_traj_irl.log", "a")
	f.write(outtraj)
	f.write("ITERATE\n")
	f.close()



	(stdout, stderr) = p.communicate(outtraj)


	(T, weights) = parseTs(stdout)
	
	stdout = None
	stderr = None
	outtraj = None

	f = open(get_home() + "/patrolstudy/toupload/t_weights.log", "w")
	for weight in weights:
		f.write(weight)
		f.write("\n")
	f.close()

	policies.append(T)			
	q.put(policies)	
	return		


def getattackerpolicy(q, policies, obs, pmodel, goalState, reward, penalty, detectDistance, predictTime, add_delay, pfail, mapToUse, T):
	
	calcStart = get_time()

	# align calcStart on a discrete time point	
	calcStart = (getTimeConv() - (calcStart % getTimeConv())) + calcStart

	#  create the interpolated trajectory, then give this to the attacker reward and solve, use the last time we saw both attackers as the starting states
	#  now search the policy for the best values at the embarking point in time, set the go time to go at this time + the last time we saw both attackers
	print("Starting Policy Solver")

	## Create transfer reward function
	global patrollersuccessrate

	traj = []
	traj.append(convertObsToTrajectoryWPolicy(obs[0], policies[0]))
	traj.append(convertObsToTrajectoryWPolicy(obs[1], policies[1]))

	patrollerStartStates = []
	patrollerTimes = []

	for patroller in traj:
		for n in range( len(patroller) - 1, -1, -1):
			timestep = patroller[n]
			if timestep is not None:
				patrollerStartStates.append(timestep[0])
				patrollerTimes.append(fromRospyTime(calcStart) - n)
				break

	while (len(patrollerStartStates) < len(traj)):
		# looks like we haven't observed a patroller or two, add some random stuff ...

		patrollerStartStates.append(PatrolState())
		patrollerTimes.append(0)

	print("PatrollerStartStates", patrollerStartStates)
	print("PatrollerTimes", patrollerTimes, "curRospy", get_time())



	args = ["boydattacker", ]			
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
	# build trajectories
	outtraj = mapToUse + "\n"
	if add_delay:
		outtraj += "true\n"
	else:
		outtraj += "false\n"
	
	for stateS in patrollerStartStates:
		outtraj += "["
		outtraj += str(int(stateS.location[0]))
		outtraj += ", "
		outtraj += str(int(stateS.location[1]))
		outtraj += ", "
		outtraj += str(int(stateS.location[2]))
		outtraj += "];"		
	
	outtraj += "\n"
	
	for startTime in patrollerTimes:
		outtraj += str(startTime)
		outtraj += ";"
	outtraj += "\n"

	outtraj += str(detectDistance)
	outtraj += "\n"
	outtraj += str(predictTime)
	outtraj += "\n"
	outtraj += str(pfail)	
	outtraj += "\n"
	outtraj += str(reward)	
	outtraj += "\n"
	outtraj += str(penalty)	
	outtraj += "\n"
	global interactionLength
	outtraj += str(interactionLength)	
	outtraj += "\n"
	
	outtraj += printTs(T)
		
	for patroller in policies:
		if (patroller.__class__.__name__ == "MapAgent"):
			for state in pmodel.S():
				action = patroller.actions(state).keys()[0]
				outtraj += "["
				outtraj += str(int(state.location[0]))
				outtraj += ", "
				outtraj += str(int(state.location[1]))
				outtraj += ", "
				outtraj += str(int(state.location[2]))
				outtraj += "] = "
				if action.__class__.__name__ == "PatrolActionMoveForward":
					outtraj += "MoveForwardAction"
				elif action.__class__.__name__ == "PatrolActionTurnLeft":
					outtraj += "TurnLeftAction"
				elif action.__class__.__name__ == "PatrolActionTurnRight":
					outtraj += "TurnRightAction"
				elif action.__class__.__name__ == "PatrolActionTurnAround":
					outtraj += "TurnAroundAction"
				else:
					outtraj += "StopAction"
					
				outtraj += "\n"
			outtraj += "ENDPOLICY\n"
		else:
			for (action, value) in patroller.iteritems():
				
				if action.__class__.__name__ == "PatrolActionMoveForward":
					outtraj += "MoveForwardAction"
				elif action.__class__.__name__ == "PatrolActionTurnLeft":
					outtraj += "TurnLeftAction"
				elif action.__class__.__name__ == "PatrolActionTurnRight":
					outtraj += "TurnRightAction"
				elif action.__class__.__name__ == "PatrolActionTurnAround":
					outtraj += "TurnAroundAction"
				else:
					outtraj += "StopAction"
					
				outtraj += " = "
				outtraj += str(value)
				outtraj += "\n"

			outtraj += "ENDE\n"
				
	
	f = open(get_home() + "/patrolstudy/toupload/patpolicy.log", "w")
	f.write(outtraj)
	f.close()
	
	(stdout, stderr) = p.communicate(outtraj)
	
	splitit = stdout.split("ENDPROBS")
	
	
	stdout = None
	stderr = None
	outtraj = None
	
	towrite = splitit[0] 
	if (len(towrite) == 0):
		exit()
	
	print("Writing out predictions")
	f = open(get_home() + "/patrolstudy/toupload/prediction.log", "w")
	f.write(towrite)
	f.close()
		
	
	# stdout now needs to be parsed into a hash of state => action, which is then sent to mapagent
	pols = []
	vs = []
	p = {}
	v = {}
	stateactions = splitit[1][1 : len(splitit[1])].split("\n")
	for stateaction in stateactions:

		if len(stateaction) < 2:
			# we've found where the marker between two policies
			pols.append(p)
			vs.append(v)
			p = {}
			v = {}
			continue
		temp = stateaction.split(" = ")
		if len(temp) < 2: continue
		
		state = temp[0]
		value = temp[1]
		action = temp[2]

		
		state = state[2 : len(state) - 1]
		pieces = state.split(",")	
		
		ps = patrol.model.AttackerState(np.array([int(pieces[0]), int(pieces[1][0 : len(pieces[1]) - 1])]) , int(pieces[2]), int(pieces[3]))
	
		if action.strip() == "null":
			p[ps] = None
		else:	
			if action.strip() == "AttackerMoveForward":
				a = patrol.model.AttackerMoveForward()
			elif action.strip() == "AttackerTurnLeft":
				a = patrol.model.AttackerTurnLeft()
			else:
				a = patrol.model.AttackerTurnRight()

			p[ps] = a
		v[ps] = float(value)

	
	policiesarr = []
	for temp2 in pols:
		policiesarr.append(mdp.agent.MapAgent(temp2))

	print("Finished MDP Solving")
		
	q.put([policiesarr, vs, predictTime, calcStart, patrollerStartStates, patrollerTimes])		

	

def perfectgetattackerpolicy(q, policies, obs, pmodel, goalState, reward, penalty, detectDistance, predictTime, add_delay, pfail, mapToUse):

	calcStart = get_time()

	# align calcStart on a discrete time point	
	calcStart = (getTimeConv() - (calcStart % getTimeConv())) + calcStart

	#  create the interpolated trajectory, then give this to the attacker reward and solve, use the last time we saw both attackers as the starting states
	#  now search the policy for the best values at the embarking point in time, set the go time to go at this time + the last time we saw both attackers
	print("Starting Policy Solver")

	## Create transfer reward function

	# find the patroller starting positions and times, use the latst seen time

	lastPatroller0State = getGroundTruthStateFor(0)
	lastPatroller1State = getGroundTruthStateFor(1)

	patrollerStartStates = [lastPatroller0State, lastPatroller1State]
	patrollerTimes = [0,0]


	print("PatrollerStartStates", patrollerStartStates)
	print("PatrollerTimes", patrollerTimes, "curRospy", get_time())



	args = ["boydattacker", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)				
	
	# build trajectories
	outtraj = mapToUse + "\n"
	if add_delay:
		outtraj += "true\n"
	else:
		outtraj += "false\n"
	
	for stateS in patrollerStartStates:
		outtraj += "["
		outtraj += str(int(stateS.location[0]))
		outtraj += ", "
		outtraj += str(int(stateS.location[1]))
		outtraj += ", "
		outtraj += str(int(stateS.location[2]))
		outtraj += "];"		
	
	outtraj += "\n"
	
	for startTime in patrollerTimes:
		outtraj += str(startTime)
		outtraj += ";"
	outtraj += "\n"

	outtraj += str(detectDistance)
	outtraj += "\n"
	outtraj += str(predictTime)
	outtraj += "\n"

	outtraj += str(pfail)	
	outtraj += "\n"
	outtraj += str(reward)	
	outtraj += "\n"
	outtraj += str(penalty)	
	outtraj += "\n"
	global interactionLength
	outtraj += str(interactionLength)	
	outtraj += "\n"
	
	
	for patroller in policies:
		if (patroller.__class__.__name__ == "MapAgent"):
			for state in pmodel.S():
				action = patroller.actions(state).keys()[0]
				outtraj += "["
				outtraj += str(int(state.location[0]))
				outtraj += ", "
				outtraj += str(int(state.location[1]))
				outtraj += ", "
				outtraj += str(int(state.location[2]))
				outtraj += "] = "
				if action.__class__.__name__ == "PatrolActionMoveForward":
					outtraj += "MoveForwardAction"
				elif action.__class__.__name__ == "PatrolActionTurnLeft":
					outtraj += "TurnLeftAction"
				elif action.__class__.__name__ == "PatrolActionTurnRight":
					outtraj += "TurnRightAction"
				elif action.__class__.__name__ == "PatrolActionTurnAround":
					outtraj += "TurnAroundAction"
				else:
					outtraj += "StopAction"
					
				outtraj += "\n"
			outtraj += "ENDPOLICY\n"
		else:
			for (action, value) in patroller.iteritems():
				
				if action.__class__.__name__ == "PatrolActionMoveForward":
					outtraj += "MoveForwardAction"
				elif action.__class__.__name__ == "PatrolActionTurnLeft":
					outtraj += "TurnLeftAction"					
				elif action.__class__.__name__ == "PatrolActionTurnRight":
					outtraj += "TurnRightAction"					
				elif action.__class__.__name__ == "PatrolActionTurnAround":
					outtraj += "TurnAroundAction"
				else:
					outtraj += "StopAction"
					
				outtraj += " = "
				outtraj += str(value)
				outtraj += "\n"

			outtraj += "ENDE\n"
	
	
	f = open(get_home() + "/patrolstudy/toupload/patpolicy.log", "w")
	f.write(outtraj)
	f.close()
	
	(stdout, stderr) = p.communicate(outtraj)

	# stdout now needs to be parsed into a hash of state => action, which is then sent to mapagent
	pols = []
	vs = []
	p = {}
	v = {}
	stateactions = stdout.split("\n")
	for stateaction in stateactions:

		if len(stateaction) < 2:
			# we've found where the marker between two policies
			pols.append(p)
			vs.append(v)
			p = {}
			v = {}
			continue
		temp = stateaction.split(" = ")
		if len(temp) < 2: continue
		state = temp[0]
		value = temp[1]
		action = temp[2]

		
		state = state[2 : len(state) - 1]
		pieces = state.split(",")	
		
		ps = patrol.model.AttackerState(np.array([int(pieces[0]), int(pieces[1][0 : len(pieces[1]) - 1])]) , int(pieces[2]), int(pieces[3]))
	
		if action.strip() == "null":
			p[ps] = None
		else:	
			if action.strip() == "AttackerMoveForward":
				a = patrol.model.AttackerMoveForward()
			elif action.strip() == "AttackerTurnLeft":
				a = patrol.model.AttackerTurnLeft()
			else:
				a = patrol.model.AttackerTurnRight()

			p[ps] = a
		v[ps] = float(value)


	policiesarr = []
	for temp2 in pols:
		policiesarr.append(mdp.agent.MapAgent(temp2))

	print("Finished MDP Solving")
		
	q.put([policiesarr, vs, predictTime, calcStart, patrollerStartStates, patrollerTimes])		
	
	


class WrapperIRL():

	def __init__(self, mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, detectableStatesNum, p_fail, add_delay, eq):
		self.goTime = sys.maxint
		self.policy = None
		self.patPolicies = None
		self.map = mapToUse
		self.isSolving = False
		self.predictTime = predictTime
		self.eq = eq

		self.reward = reward
		self.penalty = penalty
		self.detectDistance = detectDistance
		self.traj = [[], []]
		self.trajOffsets = [[] , [] ]
		self.finalQ = None
		self.maxTime = 0
		self.gotimes = []
		self.p_fail = p_fail
		self.add_delay = add_delay
		self.searchCount = 0
		self.lastSolveAttempt = -1

		global patrollerOGMap
		global attackerOGMap
		

		p_fail = self.p_fail

		self.observableStateLow = detectableStatesNum
		self.observableStateHigh = detectableStatesNum

		self.pmodel = patrol.model.PatrolModel(p_fail, None, patrollerOGMap.theMap())
		self.pmodel.gamma = 0.99

		## Create Model
		self.goalStates = [self.getStateFor(goalPos), ]
		self.goalState = self.goalStates[0]

		model = patrol.model.AttackerModel(p_fail, attackerOGMap.theMap(), self.predictTime, self.goalStates[0])
		model.gamma = 0.99

		self.model = model
		self.startState = self.getStateFor(startPos)


	def getStateFor(self, position):
		# will have to set the time component of the state correctly based on the current rospy time relative to when the attacker mdp's policy starts
		global attackerOGMap
		
		s = attackerOGMap.toState(position, True)

		if get_time() >= self.goTime:
			s.time = fromRospyTime(get_time() - self.goTime) + self.mdpTimeOffset
			s.time = min(s.time, self.maxTime-1)
			return s
		return s

	def getPositionFor(self, mdpState):
		global attackerOGMap

		return attackerOGMap.toPos(mdpState)


	def latestPolicy(self):
		return self.policy

	def goNow(self):

		if self.finalQ is not None and not self.goTime < sys.maxint:
			# check if there's something in the queue, if so we've got a new goTime
			try:
				newStuff = self.finalQ.get(False)
				self.p.join()
				print("\n Got the queue items at: " + str(get_time()))
				
				policies = newStuff[0]
				valuearr = newStuff[1]

				print("Pickling policy at: " + str(get_time()))
				
				f = open(get_home() + "/patrolstudy/toupload/attackerpolicy.log", "w")
				pickle.dump((policies, valuearr), f)
				f.close()

				self.maxTime = newStuff[2]
				calcStart = newStuff[3]
				patrollerStartStates = newStuff[4]
				patrollerTimes = newStuff[5]

				print("Finding Value Per timestep: " + str(get_time()))

				totalMaxValue = -sys.maxint - 1
				totalBestTime = -1
				totalBestPolicy = 0
				for (idx, values) in enumerate(valuearr):
					maxValue = -sys.maxint - 1
					bestTime = -1
	
					attackerStartState = self.startState
					for i in range(fromRospyTime(get_time() - calcStart), self.predictTime):
						attackerStartState.time = i
						print(i, values[attackerStartState])
						if values[attackerStartState] > maxValue:
							maxValue = values[attackerStartState]
							bestTime = i
					
					if maxValue > totalMaxValue:
						totalMaxValue = maxValue
						totalBestTime = bestTime
						totalBestPolicy = idx
	
				print("\n Start Search at",fromRospyTime(get_time() - calcStart), "BestTime", totalBestTime, "MaxValue", totalMaxValue)

				global mapToUse
				
				if totalBestTime >= 0 and totalMaxValue > 0:
					goTime = toRospyTime(totalBestTime) + calcStart
					self.policy = policies[totalBestPolicy]
				
					self.goalState = self.goalStates[totalBestPolicy] 
				else:
					goTime = sys.maxint
					self.finalQ = None  # didn't get a go time, retry


				self.gotimes.append( (goTime, get_time(), totalMaxValue, patrollerStartStates, patrollerTimes, totalBestTime, totalBestPolicy, mapToUse, getTimeConv() ) )
				f = open(get_home() + "/patrolstudy/toupload/gotimes.log", "w")
				pickle.dump(self.gotimes, f)
				f.close()

				self.mdpTimeOffset = totalBestTime
				self.goTime = goTime
				
				print("\n Got GoTime of "  + str(self.goTime))
				
#				self.searchCount += 1
#				if (self.searchCount > 300):
#					abortRun();
			except Queue.Empty:
				pass

		return get_time() >= self.goTime


			
	def patrollerModel(self):
		return self.pmodel
	

	def addPercept(self, patroller, state, time):
		
		self.traj[patroller].append((state, time))



	def getStartState(self):
		return self.startState

	def getGoalState(self):
		return self.goalState

	def update(self):
		self.updateWithAlg("NG", False)
	
	def updateWithAlg(self, alg, patrollerdelay):

		global obstime, sessionStart, sessionNumber, normedRelDiff, \
		stopI2RLThresh, sessionFinish, num_Trajsofar, print_once
		
		if not self.isSolving and self.policy is None:
			# print(count, get_time(), self.policy, max(len(a) for a in self.traj), self.predictTime)
			
			'''
			print("NEXT if conditions in update:")
			print(((get_time() > obstime and self.policy is None) and \
			(max(len(a) for a in self.traj) > self.predictTime))) 

			print(((current_time - patroller0LastSeenAt > minGapBwDemos) and \
			(current_time - patroller1LastSeenAt > minGapBwDemos)))
			'''
			
			current_time=rospy.Time.now().to_sec()
			
			if (((get_time() > obstime and self.policy is None) and \
			(max(len(a) for a in self.traj) > self.predictTime) and \
			BatchIRLflag == True) or \
			((current_time - patroller0LastSeenAt > minGapBwDemos) and \
			(current_time - patroller1LastSeenAt > minGapBwDemos) and \
			(sessionStart == False) and (self.patPolicies is None) and\
			BatchIRLflag == False)):
			# found enough states for both patrollers or 
			# current session starts because current demonstration is over
				
				self.isSolving = True
				
				self.q = multiprocessing.Queue()
				global vis
				print("self.q irlsolve initialized")
				#print("calling irlsolve")
				
				global use_em
				
				# current session started
				sessionStart = True
				# i2rl specific flag
				sessionFinish = False
				sessionNumber += 1

				print("\n sessionStart = True at, new q for sessionNumber")
				print(rospy.Time.now().to_sec())
				print(sessionNumber)
				
				if use_em:
					self.p = multiprocessing.Process(target=emirlsolve, args=(self.q, self.traj, patrollerdelay, alg, self.pmodel, self.map, self.eq, self.observableStateLow) )
				else:
					self.p = multiprocessing.Process(target=irlsolve, args=(self.q, self.traj, patrollerdelay, alg, self.pmodel, self.map, self.eq, self.observableStateLow) )
				
				print("multiprocessing.Process before self.p.start() ")
				self.p.start()
				print("multiprocessing.Process after self.p.start() ")

				#num_Trajsofar=num_Trajsofar+1 increment happened in boydirl

		elif (self.isSolving and (sessionStart == True and \
			 self.patPolicies is None)):
			# check if irl solver computed any patroller policies 
			try:
				# pulling results from queue, it may not have optimization params
				# false in arg, return an item if 
				# one is immediately available, else raise the Empty exception
				newStuff = self.q.get(False)

				# Block the calling thread until
				# the process whose join() method is called terminates
				print("\n multiprocessing.Process before self.p.join() ")
				self.p.join() 
				print("\n multiprocessing.Process after self.p.join() ")
				
				print("Is queue empty? "+str(self.q.empty()))
				
				# sessionFinish == True only if boydirl output optimization 
				# params with policies, not needed for batch version
				# Then, for i2rl, check if current session is finished
				# batchirl will automatically do only one session because 
				# the batchirlflag need to be False for next session to happen
				if (BatchIRLflag == False and sessionFinish == True): 
					self.traj = [[], []] # keep recording for next session
					# ready for next session
					sessionStart = False
					self.isSolving = False
					num_Trajsofar += 1

					print("\n desired params computed by i2rl at,")
					print(" for sessionNumber, num_Trajsofar:")
					
					print(rospy.Time.now().to_sec())
					print(sessionNumber)
					print(num_Trajsofar)
					
					print_once = 0
					
					if (sessionNumber == 4):
						print("abort run for session 4 uncommented ")
						abortRun()

				# if i2rl converged or irl is batch type, then save the policies
				# else continue for next session
				if (normedRelDiff < stopI2RLThresh and sessionFinish == True) or (BatchIRLflag == True):
					print("\n batchirl converged or normedRelDiff < stopI2RLThresh. writing patPolicies. ")
					print("abort run commented ")
					# abortRun()
					
					self.T = newStuff.pop()
					self.patPolicies = newStuff
					
					# save the generated policy to the logs
					f = open(get_home() + "/patrolstudy/toupload/policy.log", "w")
					pickle.dump(self.patPolicies,f)
					f.close()
					
# 					self.q = multiprocessing.Queue()
# 					print("self.q initialized again")
					
			except Queue.Empty: # no item in queue boydirl is not done
				if print_once == 0:
					print("print_once queue self.q empty")
				print_once = 1
				pass		

		if self.finalQ is None and self.patPolicies is not None and rospy.get_time() - self.lastSolveAttempt > getTimeConv():
			# compute attacker policy in new process and share results in new queue
			print("\n compute attacker policy because self.patPolicies is not None")
			self.finalQ = multiprocessing.Queue()
			self.p = multiprocessing.Process(target=getattackerpolicy, args=(self.finalQ, self.patPolicies, self.traj, self.pmodel, self.goalState, self.reward, self.penalty, self.detectDistance, self.predictTime, self.add_delay, self.p_fail, self.map, self.T) )
				
			self.p.start()
			self.lastSolveAttempt = rospy.get_time()
	

	def getModel(self):
		return self.model

class WrapperIRLDelay(WrapperIRL):


	def update(self):
		self.updateWithAlg("NG", True)


class WrapperMaxEntIRL(WrapperIRL):


	def update(self):
		self.updateWithAlg("MAXENT", False)

class WrapperMaxEntIRLZExact(WrapperIRL):


	def update(self):
		self.updateWithAlg("MAXENTZEXACT", False)

class WrapperMaxEntIRLDelay(WrapperIRL):


	def update(self):
		self.updateWithAlg("MAXENT", True)


class WrapperLMEIRL(WrapperIRL):


	def update(self):
		self.updateWithAlg("LME", True)

class WrapperLMEI2RL(WrapperIRL):


	def update(self):
		self.updateWithAlg("LME2", True)

class WrapperLMEIRLBLOCKEDGIBBS(WrapperIRL):

	def update(self):
		self.updateWithAlg("LMEBLOCKEDGIBBS", True)

class WrapperLMEIRLBLOCKEDGIBBSTIMESTEP(WrapperIRL):

	def update(self):
		self.updateWithAlg("LMEBLOCKEDGIBBSTIMESTEP", True)

class WrapperLMEIRLBLOCKEDGIBBSTIMESTEPSA(WrapperIRL):

	def update(self):
		self.updateWithAlg("LMEBLOCKEDGIBBSSATIMESTEP", True)

class WrapperIdealIRL(WrapperIRL):

	def update(self):
		global obstime
	
		if not self.isSolving and self.policy is None:
					
#			print(count, get_time(), self.policy, max(len(a) for a in self.traj))
			if ((get_time() > obstime and self.policy is None)) and max(len(a) for a in self.traj) > self.predictTime:  # found enough states for both patrollers
				self.isSolving = True

				self.q = multiprocessing.Queue()

				self.p = multiprocessing.Process(target=idealirlsolve, args=(self.q,self.add_delay, self.map, self.traj, self.pmodel, self.observableStateLow, self.eq) )

				self.p.start()

		elif self.isSolving:
			try:
				newStuff = self.q.get(False)
				
				self.p.join()
				# got a new set of patroller policies, kickoff the final step
				self.T = newStuff.pop()
				self.patPolicies = newStuff
				
				# save the generated policy to the logs
				f = open(get_home() + "/patrolstudy/toupload/policy.log", "w")
				pickle.dump(self.patPolicies,f)
				f.close()
				
				self.q = multiprocessing.Queue()
				
			except Queue.Empty:
				pass		

		if self.finalQ is None and self.patPolicies is not None and rospy.get_time() - self.lastSolveAttempt > getTimeConv():
			self.finalQ = multiprocessing.Queue()

			self.p = multiprocessing.Process(target=getattackerpolicy, args=(self.finalQ, self.patPolicies, self.traj, self.pmodel, self.goalState, self.reward, self.penalty, self.detectDistance, self.predictTime, self.add_delay, self.p_fail, self.map, self.T) )
			self.p.start()
			self.lastSolveAttempt = rospy.get_time()


# Note, will need to change this for each map


def getGroundTruthStateFor(pat):
	
	global patroller0GroundTruth
	global patroller1GroundTruth
	
	if (pat == 0):
		gt = patroller0GroundTruth
	else:
		gt = patroller1GroundTruth
	
	
	a = gt.pose.pose.orientation
	
	angles = tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])

	if (angles[2] < 0):
		angles = (0,0, 2 * math.pi + angles[2])
	
	pos = (gt.pose.pose.position.x, gt.pose.pose.position.y, angles[2])	
	
	global patrollerOGMap
	return patrollerOGMap.toState(pos, False)
		

class WrapperPerfect(WrapperIRL):


	def update(self):

		global obstime
		
		if not self.isSolving and self.policy is None:

#			print(count, get_time(), self.policy, max(len(a) for a in self.traj))
			if ((get_time() > obstime and self.policy is None)) and max(len(a) for a in self.traj) > self.predictTime:  # found enough states for both patrollers
				self.isSolving = True

				self.q = multiprocessing.Queue()

				p = multiprocessing.Process(target=idealirlsolve, args=(self.q,self.add_delay, self.map, self.traj, self.pmodel, self.observableStateLow, self.eq) )
				p.start()

		elif self.isSolving:
			try:
				newStuff = self.q.get(False)
				# got a new set of patroller policies, kickoff the final step
				self.patPolicies = newStuff

				# save the generated policy to the logs
				f = open(get_home() + "/patrolstudy/toupload/policy.log", "w")
				pickle.dump(self.patPolicies,f)
				f.close()
				
			except Queue.Empty:
				pass		

		if self.finalQ is None and self.patPolicies is not None and rospy.get_time() - self.lastSolveAttempt > getTimeConv():
			self.finalQ = multiprocessing.Queue()

			p = multiprocessing.Process(target=perfectgetattackerpolicy, args=(self.finalQ, self.patPolicies, self.traj, self.pmodel, self.goalState, self.reward, self.penalty, self.detectDistance, self.predictTime, self.add_delay, self.p_fail, self.map) )

			p.start()
			self.lastSolveAttempt = rospy.get_time()

def Attack():
	global pub
	
	pub.publish(String("attackerleft"))
	global state
	state = "a"


def Gooooooal():
	global pub
	pub.publish(String("reachgoal"))

def abortRun():
	global pub
	pub.publish(String("abortrun"))

def handle_pose(req):
	global state
	global goal
	global lastPositionBelief
	global mdpWrapper
	global cur_waypoint
	global percepts
	global perceptsUpdated
	global amclmessageReceived


	x = req.pose.pose.position.x
	y = req.pose.pose.position.y
	angle = req.pose.pose.orientation

	a = tf.transformations.euler_from_quaternion([angle.x, angle.y, angle.z, angle.w])

	if (a[2] < 0):
		a = (0,0, 2 * math.pi + a[2])
	
	lastPositionBelief = (x, y, a[2])
	amclmessageReceived = get_time();

currentAttackerState = None
amclmessageReceived = 0 
lastpublish = 0

waitUntilGoalState = False


def step():
	
	global state
	global goal
	global lastPositionBelief
	global mdpWrapper
	global cur_waypoint
	global percepts
	global perceptsUpdated
	global currentAttackerState     
	global amclmessageReceived
	global lastpublish
	global mapToUse
	
	if mdpWrapper is None:
		return

	if not state == "w":
		if mdpWrapper.latestPolicy() is None:
			return
		# we're attacking

#		if math.sqrt( (cur_waypoint[0] - lastPositionBelief[0])*(cur_waypoint[0] - lastPositionBelief[0]) + (cur_waypoint[1] - lastPositionBelief[1])*(cur_waypoint[1] - lastPositionBelief[1])) < .1:
#			lastPositionBelief = cur_waypoint
		# what mdp state are we closest to? 
		mdpState = mdpWrapper.getStateFor(lastPositionBelief)
#		print("CurState: ", mdpState)
		# are we at the goal?
		goalState = mdpWrapper.getGoalState()
		if np.all( mdpState.location == goalState.location):
			newwaypoint = mdpWrapper.getPositionFor(mdpState)
			if newwaypoint[1] != cur_waypoint[1] or newwaypoint[0] != cur_waypoint[0]:
				returnval = PoseStamped()
				returnval.pose.position = Point(cur_waypoint[0], cur_waypoint[1], 0)
				returnval.pose.orientation = Quaternion()
				returnval.pose.orientation.w = 1

				returnval.header.frame_id = "/map"
				goal.publish(returnval)
				cur_waypoint = newwaypoint        

#			print("belief", lastPositionBelief, "goal", cur_waypoint, math.sqrt( (cur_waypoint[0] - lastPositionBelief[0])*(cur_waypoint[0] - lastPositionBelief[0]) + (cur_waypoint[1] - lastPositionBelief[1])*(cur_waypoint[1] - lastPositionBelief[1])))

			if math.sqrt( (cur_waypoint[0] - lastPositionBelief[0])*(cur_waypoint[0] - lastPositionBelief[0]) + (cur_waypoint[1] - lastPositionBelief[1])*(cur_waypoint[1] - lastPositionBelief[1])) < .4:
				print("GOOOOOOOOOOOOAL")
				# we've Won!
				Gooooooal()
			return

		global waitUntilGoalState
		if waitUntilGoalState:
			return

		# special case for boydright, if we are currently in a state with no actions, give it the goal state and set a variable
		if mapToUse == "boydright":
			if mdpWrapper.latestPolicy().actions(mdpState).keys()[0] is None:
				goalwaypoint = mdpWrapper.getPositionFor(goalState)
				returnval = PoseStamped()
				returnval.pose.position = Point(goalwaypoint[0], goalwaypoint[1], 0)
				returnval.pose.orientation = Quaternion()
				returnval.pose.orientation.w = 1

				returnval.header.frame_id = "/map"
				goal.publish(returnval)
				cur_waypoint = goalwaypoint
				
				waitUntilGoalState = True
				return
		
		
		

		# ask that state for the policy, run policy to get next state
		#	Look at what position that state corresponds to, if it is different than cur_waypoint,
		#	update cur_waypoint and send out a new goal


#		action = mdpWrapper.latestPolicy().actions(mdpState).keys()[0]
#		newMdpState = action.apply(mdpState)
#		if not mdpWrapper.getModel().is_legal(newMdpState):
#			newMdpState = mdpState

#		print("newMDPState", newMdpState, "action", action)
#		newWayPoint = mdpWrapper.getPositionFor(newMdpState)
#		if not newWayPoint[0] == cur_waypoint[0] or not newWayPoint[1] == cur_waypoint[1]:
		if np.all(mdpState.location == currentAttackerState.location):
#		if math.sqrt( (cur_waypoint[0] - lastPositionBelief[0])*(cur_waypoint[0] - lastPositionBelief[0]) + (cur_waypoint[1] - lastPositionBelief[1])*(cur_waypoint[1] - lastPositionBelief[1])) < .75:
			test = mdpWrapper.latestPolicy().actions(mdpState).keys()[0]
			testState = test.apply(mdpState)
			if testState == currentAttackerState:
				return


			action = mdpWrapper.latestPolicy().actions(currentAttackerState).keys()[0]
			newMdpState = action.apply(currentAttackerState)
			if not mdpWrapper.model.is_legal(newMdpState):
				print("INVALID", currentAttackerState, "action", action, "newstate", newMdpState)
				newMdpState =  mdpWrapper.getStateFor(lastPositionBelief)

			newWayPoint = mdpWrapper.getPositionFor(newMdpState)
			currentAttackerState = newMdpState
			
			# skip ahead one state to prevent slowdowns from occuring
			if action.__class__.__name__ == "AttackerMoveForward":
				action = mdpWrapper.latestPolicy().actions(currentAttackerState).keys()[0]
				if not action is None:
					newMdpState = action.apply(currentAttackerState)
					if mdpWrapper.model.is_legal(newMdpState):
						newWayPoint = mdpWrapper.getPositionFor(newMdpState)
			
			
			cur_waypoint = newWayPoint

			returnval = PoseStamped()
			returnval.pose.position = Point(cur_waypoint[0], cur_waypoint[1], 0)
			q = tf.transformations.quaternion_from_euler(0,0,cur_waypoint[2])   
			returnval.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

			returnval.header.frame_id = "/map"
			goal.publish(returnval)
			print("cur_waypoint", cur_waypoint)
			lastpublish = get_time()
			
		elif get_time() - lastpublish > 2.5 and get_time() - amclmessageReceived > 1.5:
			# attacker stopped, send a future goal
			
			if np.all( currentAttackerState.location == goalState.location):

				newWayPoint = mdpWrapper.getPositionFor(currentAttackerState)
				cur_waypoint = newWayPoint
	
				returnval = PoseStamped()
				returnval.pose.position = Point(random.gauss(cur_waypoint[0], .1), random.gauss(cur_waypoint[1], .1), 0)
				q = tf.transformations.quaternion_from_euler(0,0,cur_waypoint[2])   
				returnval.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
	
				returnval.header.frame_id = "/map"
				goal.publish(returnval)
				print("Skipping Ahead cur_waypoint", cur_waypoint)
				lastpublish = get_time()				
				
				return
			
			action = mdpWrapper.latestPolicy().actions(currentAttackerState).keys()[0]
			
			newMdpState = action.apply(currentAttackerState)

			if not mdpWrapper.model.is_legal(newMdpState):
				print("INVALID", currentAttackerState, "action", action, "newstate", newMdpState)
				lastpublish = get_time()
				return
			
			newWayPoint = mdpWrapper.getPositionFor(newMdpState)
			currentAttackerState = newMdpState
			cur_waypoint = newWayPoint

			returnval = PoseStamped()
			returnval.pose.position = Point(cur_waypoint[0], cur_waypoint[1], 0)
			q = tf.transformations.quaternion_from_euler(0,0,cur_waypoint[2])   
			returnval.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

			returnval.header.frame_id = "/map"
			goal.publish(returnval)
			print("Skipping Ahead cur_waypoint", cur_waypoint)
			lastpublish = get_time()
											
			
	else:
		global normedRelDiff, stopI2RLThresh, BatchIRLflag
		
		# same call for irl and computing attackerpolicy 
		# while (normedRelDiff > stopI2RLThresh):
			 
		#	mdpWrapper.update()
		#	if BatchIRLflag == True:
		#		break

		mdpWrapper.update()
		# should we go now?
		# check if we got a valid go time
		if mdpWrapper.goNow():
			Attack()
#			import std_srvs.srv
#			try:
#				resetservice = rospy.ServiceProxy('move_base_node/clear_costmaps', std_srvs.srv.Empty)
#
#				resetservice()
#			except:
#				# ros can't be counted on for anything, at least we tried.
#				pass

			
			# get the current state (to base the plan on), then give the first goal message
			currentAttackerState = mdpWrapper.getStateFor(lastPositionBelief)
			action = mdpWrapper.latestPolicy().actions(currentAttackerState).keys()[0]   
			newMdpState = action.apply(currentAttackerState)
			if not mdpWrapper.model.is_legal(newMdpState):
				newMdpState.location = currentAttackerState.location
				newMdpState.orientation = currentAttackerState.orientation
				if not mdpWrapper.model.is_legal(newMdpState):
					newMdpState.time = currentAttackerState.time

			newWayPoint = mdpWrapper.getPositionFor(newMdpState)
			print("cur_waypoint", newWayPoint, " curstate:", currentAttackerState, "newstate", newMdpState)
			currentAttackerState = newMdpState
			cur_waypoint = newWayPoint

			returnval = PoseStamped()
			returnval.pose.position = Point(cur_waypoint[0], cur_waypoint[1], 0)

			q = tf.transformations.quaternion_from_euler(0,0,cur_waypoint[2])   
			returnval.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

			returnval.header.frame_id = "/map"
			goal.publish(returnval)
			lastpublish = get_time()			   




def boydrightisvisible(state, visiblenum):
	
	if visiblenum == 14:
		return True


	if visiblenum == 1:
		# 4 states visible
		if state.location[0] == 0 and state.location[1] >= 14:
			return True
		if state.location[0] == 1 and state.location[1] == 16:
			return True
		return False
		
	if visiblenum == 2:
		# 7 states visible
		if state.location[0] == 0 and state.location[1] >= 13:
			return True
		if state.location[0] <= 3 and state.location[1] == 16:
			return True
		return False
		
	if visiblenum == 4:
		# 15 states visible
		
		if state.location[0] == 0 and state.location[1] >= 10:
			return True
		if state.location[0] == 1 and state.location[1] >= 10:
			return True
		if state.location[0] <= 6 and ( state.location[1] == 15 or state.location[1] == 16) :
			return True
		return False	
		
	if visiblenum == 6:
		# 22 states visible
		
		if state.location[0] == 0 and state.location[1] >= 8:
			return True
		if state.location[0] == 1 and state.location[1] >= 10:
			return True
		if state.location[0] == 2 and state.location[1] >= 9:
			return True
		if state.location[0] == 5 and state.location[1] >= 14:
			return True
		if state.location[0] <= 7 and state.location[1] == 16:
			return True
		
		return False
		
	if visiblenum == 10:
		# 37 states visible
		
		if state.location[0] == 0 and state.location[1] >= 6:
			return True
		if state.location[0] <= 4 and state.location[1] >= 10:
			return True
		if state.location[0] == 5 and state.location[1] >= 12:
			return True
		if state.location[0] <= 7 and state.location[1] >= 14:
			return True
		if state.location[0] <= 11 and state.location[1] >= 15:
			return True
		
		return False
		
	return False
		
		
def boydisvisible(state, visiblenum):
	
	if visiblenum == 14:
		return True
	
	
	if visiblenum == 1:
		if (state.location[0] == 0 and state.location[1] == 1):
			return True
		if (state.location[0] == 0 and state.location[1] == 2):
			return True
		
		
	if visiblenum == 2:
		if (state.location[0] == 0 and state.location[1] <= 4):
			return True

		
	if visiblenum == 4:
		if (state.location[0] <= 2 and state.location[1] <= 6):
			return True
	
	if visiblenum == 6:
		if (state.location[0] <= 5):
			return True

	
	if visiblenum == 10:
		if (state.location[0] <= 14):
			return True

	return False
	

def isvisible(state, visiblenum):
	global mapToUse
	
	if (mapToUse == "boyd2"):
		return boydisvisible(state, visiblenum)
	
	return boydrightisvisible(state, visiblenum)

	
	
def handle_patroller0(req):

	global mdpWrapper
	global patroller0GroundTruth
	patroller0GroundTruth = req
	global patrollerOGMap, patroller0LastSeenAt, patroller1LastSeenAt
	
	if (mdpWrapper is None):
		return
	
	

	pos = req.pose.pose.position
	a = req.pose.pose.orientation

	angles = tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])

	# that function returns negative angles :facepalm: convert this to positive rotations

	if (angles[2] < 0):
		angles = (0,0, 2 * math.pi + angles[2])

	pState = patrollerOGMap.toState((pos.x, pos.y, angles[2]), False)
		
	observable = isvisible(pState, mdpWrapper.observableStateLow)
	
	if observable:
		mdpWrapper.addPercept(0, (pos.x, pos.y, angles[2]), get_time())
		patroller0LastSeenAt = rospy.Time.now().to_sec()
		''' print(last_observedtime) 
		if (rospy.get_time()-last_observedtime > 2.0):
			last_observedtime=rospy.get_time()
			print("p0 observable at")
			print(last_observedtime)'''
		

def handle_patroller1(req):

	global mdpWrapper
	global patroller1GroundTruth
	global patroller0GroundTruth
	global patrollerOGMap, patroller0LastSeenAt, patroller1LastSeenAt
	
	patroller1GroundTruth = req
	
	if (mdpWrapper is None):
		return

	pos = req.pose.pose.position
	a = req.pose.pose.orientation

	angles = tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])

	# that function returns negative angles :facepalm: convert this to positive rotations

	if (angles[2] < 0):
		angles = (0,0, 2 * math.pi + angles[2])

	pState = patrollerOGMap.toState((pos.x, pos.y, angles[2]), False)
	observable = isvisible(pState, mdpWrapper.observableStateLow)
	
	if observable:
		mdpWrapper.addPercept(1, (pos.x, pos.y, angles[2]), get_time())
		patroller1LastSeenAt = rospy.Time.now().to_sec()



def handle_attacker(req):
	global lastActualPosition

	lastActualPosition = req.pose.pose.position
#	tmp = lastActualPosition.x
#	lastActualPosition.x = - lastActualPosition.y
#	lastActualPosition.y = tmp

if __name__ == "__main__":
	# Need to know:
	#  Which policy to use
	#  Which Map to use
	#  Starting position on the MDP
	#  Goal position on the MDP
	#  Reward for making it to the goal
	#  Penalty for getting caught
	#  Assumed detection distance from each attacker (in mdp states)
	#  
	rospy.init_node('attacker_controller')
	
	global minGapBwDemos, patroller0LastSeenAt,\
	patroller1LastSeenAt, BatchIRLflag
	patroller0LastSeenAt=100.0
	patroller1LastSeenAt=100.0
	minGapBwDemos = 1.0
	
	startat = rospy.get_time()

	pub = rospy.Publisher("/study_attacker_status", String, latch=True)

	goal = rospy.Publisher("move_base_simple/goal", PoseStamped)


	policy = rospy.get_param("~policy")
	mapToUse = rospy.get_param("~map")
	
	if mapToUse == "boyd2":
		mapparams = boyd2MapParams(False)
		patrollerOGMap = OGMap(*mapparams)
		
		mapparams = boyd2MapParams(True)
		attackerOGMap = OGMap(*mapparams)
		
	else:
		mapparams = boydrightMapParams(False)
		patrollerOGMap = OGMap(*mapparams)
		
		mapparams = boydrightMapParams(True)
		attackerOGMap = OGMap(*mapparams)
	

	reward = float(rospy.get_param("~goalReward"))
	penalty = float(rospy.get_param("~detectionPenalty"))
	detectDistance = int(rospy.get_param("~detectDistance")) 
	predictTime = int(rospy.get_param("~predictionTime"))
	observableStates = int(rospy.get_param("~observableStates"))
	pfail = float(rospy.get_param("~pfail"))
	patroller = rospy.get_param("~patroller")
	patrollersuccessrate = float(rospy.get_param("~patrollersuccessrate"))
	usesimplefeatures = rospy.get_param("~usesimplefeatures") == "true"
	add_delay = int(rospy.get_param("~add_delay"))
	add_delay = (add_delay == 1)
	interactionLength = int(rospy.get_param("~interactionLength"))
	eq = rospy.get_param("~equilibrium")
	use_em = int(rospy.get_param("~use_em"))
	use_em = (use_em == 1)
	BatchIRLflag = (int(rospy.get_param("~BatchIRLflag")))==1
	print("\n BatchIRLflag:"+str(BatchIRLflag)+"\n")
	
#	equilibriumKnown = int(rospy.get_param("~equilibriumKnown"))	
#	equilibriumKnown = (equilibriumKnown == 1)
	equilibriumKnown = True # Fix for the current experiment
	
	obstime = int(rospy.get_param("~obstime"))

	f = open(rospy.get_param("~configFile"))

	startPos = f.readline().strip()
	goalPos = f.readline().strip()

	startPos = startPos.split(" ")
	startPos = (float(startPos[0]), float(startPos[1]), float(startPos[3]) * 0.0174532925)

	goalPos = goalPos.split(" ")
	goalPos = (float(goalPos[0]), float(goalPos[1]), 90 * 0.0174532925)
	
	goalPos2 = None
#	if mapToUse == "boyd2":
#		goalPos2 = f.readline().strip()
#		goalPos2 = goalPos2.split(" ")
#		goalPos2 = (float(goalPos2[0]), float(goalPos2[1]), 90 * 0.0174532925)
	f.close()
	
	# non-modified irl assumes no interaction
	if policy == "irl" or policy == "maxentirl":
		interactionLength = 0

	if policy == "random":
		mdpWrapper = WrapperRandom(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance)
	elif policy == "irl":
		mdpWrapper = WrapperIRL(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)
	elif policy == "irldelay":
		mdpWrapper = WrapperIRLDelay(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)
	elif policy == "maxentirl":
		mdpWrapper = WrapperMaxEntIRL(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)
	elif policy == "maxentirlzexact":
		mdpWrapper = WrapperMaxEntIRLZExact(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)
	elif policy == "maxentirldelay":
		mdpWrapper = WrapperMaxEntIRLDelay(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)
	elif policy == "lmeirl":
		mdpWrapper = WrapperLMEIRL(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)		
	elif policy == "lmei2rl":
		mdpWrapper = WrapperLMEI2RL(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)		
	elif policy == "lmeirlblockedgibbs":
		mdpWrapper = WrapperLMEIRLBLOCKEDGIBBS(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)		
	elif policy == "lmeirlblockedgibbstimestep":
		mdpWrapper = WrapperLMEIRLBLOCKEDGIBBSTIMESTEP(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)		
	elif policy == "lmeirlblockedgibbstimestepsa":
		mdpWrapper = WrapperLMEIRLBLOCKEDGIBBSTIMESTEPSA(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)				
	elif policy == "knownpolicy":
		mdpWrapper = WrapperIdealIRL(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)
	else:
		mdpWrapper = WrapperPerfect(mapToUse, startPos, goalPos, goalPos2, reward, penalty, detectDistance, predictTime, observableStates, pfail, add_delay, eq)

	startState = mdpWrapper.getStartState()
	print(startPos, " Start is", startState)

	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, handle_pose)
	rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, handle_patroller0)
	rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, handle_patroller1)
	rospy.Subscriber("base_pose_ground_truth", Odometry, handle_attacker)


	r = rospy.Rate(30) # 10hz
	count = 0
	while not rospy.is_shutdown():
		step()
		if state == "w" and count == 2000:
			# initialize 
			cur_waypoint = mdpWrapper.getPositionFor(startState)
			returnval = PoseStamped()
			returnval.pose.position = Point(cur_waypoint[0], cur_waypoint[1], 0)

			q = tf.transformations.quaternion_from_euler(0,0,cur_waypoint[2])   
			returnval.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

			returnval.header.frame_id = "/map"
			goal.publish(returnval)

		count += 1

		r.sleep()

