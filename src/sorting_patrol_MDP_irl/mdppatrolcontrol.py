#!/usr/bin/env python
import roslib; roslib.load_manifest('navigation_irl')
import time
import rospy
import std_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
from nav_msgs.msg import Odometry
import random
import math

import patrol.model
from patrol.model import *
import patrol.reward
import numpy as np
import mdp.simulation
import mdp.solvers
import tf.transformations
import std_srvs.srv
import dynamic_reconfigure.client
from patrol.time import *

import os
from posix import abort

home = os.environ['HOME']

tolerance = .1
pub = None
all_stop_pub = None
sd = .1
model = None
policy = None
savedPose = None
goalPos = None
lastGoal = None
m = None
add_delay = False
skip_interaction = False
resetservice = None
current_cg = 0

lastpublish = -100
amclmessageReceived = -100
dr_client = None

# states 0 & 1 = top hallway, in
# 2 - 11 = main hall
# 12 & 13 = bottom hallway, in


go = True
maxsightdistance = 2


ogmap = None


def sendGoal(goal):
	global sd
	global lastpublish
	global lastGoal
	global pub
	global goalPos
	global ogmap
	
	goalpos = ogmap.toPos(goal)
	goalPos = goalpos
	goalPos = (random.gauss(goalPos[0], sd), random.gauss(goalPos[1], sd), random.gauss(goalPos[2], sd))
#	print ("New Goal: " + str(goalPos) + " " + str(goal) )
	
	returnval = PoseStamped()
	returnval.pose.position = Point(goalpos[0], goalpos[1], 0)
	
	returnval.pose.orientation = Quaternion()
	q = tf.transformations.quaternion_from_euler(0,0,goalpos[2])

	returnval.pose.orientation.x = q[0]
	returnval.pose.orientation.y = q[1]
	returnval.pose.orientation.z = q[2]
	returnval.pose.orientation.w = q[3]
	
	returnval.header.frame_id = "/map"
	lastpublish = rospy.get_time()
	pub.publish(returnval)
	
	lastGoal = goal


otherPose = None

def canSee(mePose, themPose):
	global maxsightdistance
	distance = math.sqrt( (mePose.position.x - themPose.position.x)*(mePose.position.x - themPose.position.x) + (mePose.position.y - themPose.position.y)*(mePose.position.y - themPose.position.y) )
	
	if distance <= maxsightdistance:
		# now check if the angle is right, the attacker must be within the patroller's field of view
		angle = math.atan2(themPose.position.y - mePose.position.y, themPose.position.x - mePose.position.x)
		newangle = tf.transformations.quaternion_from_euler(0,0,angle)

		q1 = np.array((mePose.orientation.x, mePose.orientation.y, mePose.orientation.z, mePose.orientation.w ))
		q = tf.transformations.quaternion_multiply(tf.transformations.quaternion_inverse(q1), newangle)
		
		x, y, z = tf.transformations.euler_from_quaternion(q)
		if abs(z) <= (57.0 / 2 * .017453):

			# difference in the two orientations must be > 90 degrees
			a = mePose.orientation
			meAngle = tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])[2] 

			a = themPose.orientation
			themAngle = tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])[2] 
			
			diff = math.atan2(math.sin(meAngle-themAngle), math.cos(meAngle-themAngle))
			
			if abs(diff) > math.pi / 2:
#				print("Saw Other: " + str(distance))
				
				return True
	return False

def handle_other(req):

	global otherPose
		
	otherPose = req.pose.pose


savedState = None

	
def handle_pose(req):
	global savedPose
	global amclmessageReceived

	changed = False
		
	if savedPose is None:
		changed = True
	else:
		if not savedPose.position.x == req.pose.pose.position.x or not savedPose.position.y == req.pose.pose.position.y or not req.pose.pos.orientation.z == savedPose.orientation.z or not req.pose.pos.orientation.w == savedPose.orientation.w:
			changed = True
	
	savedPose = req.pose.pose

	if changed:
		amclmessageReceived = rospy.get_time()

		
		
ground_truth = None

def save_ground_truth(msg):
	global ground_truth
	ground_truth = msg

sentReset = 0

def clear_costmaps():
	global resetservice
	try:
		if resetservice is None:
			resetservice = rospy.ServiceProxy('move_base_node/clear_costmaps', std_srvs.srv.Empty)

		resetservice()
	except:
		# for fucks sake guys.
		pass	

lasttimestep = -1

def nextGoal():
	
	global add_delay
	global lastGoal
	global go
	global savedPose
	global otherPose
	global goalPos
	global savedState
	global amclmessageReceived
	global sentReset
	global m
	global all_stop_pub
	global lastpublish
	global lasttimestep
	global ogmap
	global current_cg, curr_st, next_st
	
	if add_delay and go == False:
		go = True
		topub = std_msgs.msg.Bool()
		topub.data = False

		all_stop_pub.publish(topub)
		
		clear_costmaps()
				
		if (lastGoal is not None):
			sendGoal(lastGoal)
		
	# can't see the other guy, continue on as usual

	x = savedPose.position.x
	y = savedPose.position.y
	a = savedPose.orientation
	
	angles = tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])
	
	# that function returns negative angles :facepalm: convert this to positive rotations
	
	if (angles[2] < 0):
		angles = (0,0, 2 * math.pi + angles[2])
	
	
	# what i want to to throttle the goal messages in the case of duplicates, but not others

	state = ogmap.toState((x,y, angles[2]), False)

	if state == lastGoal:
	
#	if goalPos is None or (goalPos is not None and math.sqrt( (goalPos[0] - x)*(goalPos[0] - x) + (goalPos[1] - y)*(goalPos[1] - y) ) < tolerance):
#	if state == lastGoal:
		#	if rospy.get_time() - lastpublish > 10 or (goalPos is not None and math.sqrt( (goalPos[0] - x)*(goalPos[0] - x) + (goalPos[1] - y)*(goalPos[1] - y)) < tolerance):

	#			print(str(state) + " - Sent: " + str(policy.actions(state).keys()[0]))

		if m=="largeGridPatrol":
			curr_st = PatrolExtendedState(state.location,current_cg)
# 			print "nextgoal() state == lastGoal next_st = policy.actions(curr_st).keys()[0].apply(curr_st)"
			next_st = policy.actions(curr_st).keys()[0].apply(curr_st)
			goal = PatrolState(next_st.location)
		else:
			goal = policy.actions(state).keys()[0].apply(state)
			
	#		while state.conflicts(goal):
	#			goal = policy.actions(goal).keys()[0].apply(goal)
		
		newgoal = goal
		# skip ahead one state to prevent slowdowns from occuring
# 		if policy.actions(state).keys()[0].__class__.__name__ == "PatrolActionMoveForward":
# 			newgoal = policy.actions(goal).keys()[0].apply(goal)
		if m=="largeGridPatrol":
			curr_st = PatrolExtendedState(state.location,current_cg)
# 			print "nextgoal() state == lastGoal if policy.actions(curr_st).keys()[0].__class__.__name__ == PatrolActionMoveForward"
			if policy.actions(curr_st).keys()[0].__class__.__name__ == "PatrolActionMoveForward":
# 				print " state == lastGoal __name__ == PatrolActionMoveForward next_st = policy.actions(curr_st).keys()[0].apply(curr_st) "
				next_st = policy.actions(curr_st).keys()[0].apply(curr_st)
				newgoal = PatrolState(next_st.location)
		else:
			if policy.actions(state).keys()[0].__class__.__name__ == "PatrolActionMoveForward":
				newgoal = policy.actions(state).keys()[0].apply(state)
# 			newgoal = policy.actions(goal).keys()[0].apply(goal)
			
		sentReset = 0				
# 		print "newgoal == lastGoal"
		
		if lastGoal is None or not newgoal == lastGoal:
# 			print "not newgoal == lastGoal"
			if m=="largeGridPatrol":
				# print "current_cg = next_st.current_goal"
				global current_cg
				current_cg = next_st.current_goal
				
			sendGoal(newgoal)
			savedState = state
			lastGoal = goal

	if rospy.get_time() - lastpublish > 2.5 and rospy.get_time() - amclmessageReceived > 1.5:
		#print("Resent Goal for " + str(add_delay))
		if lastGoal is None:
			print "if lastGoal is None"
# 			goal = policy.actions(state).keys()[0].apply(state)
			if m=="largeGridPatrol":
				curr_st = PatrolExtendedState(state.location,current_cg)
				# print "nextgoal() lastGoal is None next_st = policy.actions(curr_st).keys()[0].apply(curr_st) "
				next_st = policy.actions(curr_st).keys()[0].apply(curr_st)
				goal = PatrolState(next_st.location)
			else:
				goal = policy.actions(state).keys()[0].apply(state)

			sendGoal(goal)
		else:
# 			print "if lastGoal is not None, compute goal for next state"
# 			sendGoal(policy.actions(lastGoal).keys()[0].apply(lastGoal))
			if m=="largeGridPatrol":
				curr_st = PatrolExtendedState(lastGoal.location,current_cg)
				# print "nextgoal() lastGoal is not None next_st = policy.actions(curr_st).keys()[0].apply(curr_st) "
				next_st = policy.actions(curr_st).keys()[0].apply(curr_st)
				goal = PatrolState(next_st.location)
			else:
				goal = policy.actions(lastGoal).keys()[0].apply(lastGoal)
			
			if rospy.get_time() - amclmessageReceived >= 3 and sentReset < 2:
				sentReset = sentReset + 1
				clear_costmaps()
			
		savedState = state
	
def largeGridisvisible(state, visiblenum):
	
# 	print("state.location[1]: ",state.location[1]," visiblenum :",visiblenum)
	if visiblenum == 14:
		return True
	
	
	if visiblenum == 1:
		if (state.location[0] == 0 and state.location[1] == 1):
			return True
		if (state.location[0] == 0 and state.location[1] == 2):
			return True
		
		
	if visiblenum == 2:
		if (state.location[1] <= 1 or state.location[1] >= 8):
			return True

		
	if visiblenum == 4:
		if (state.location[1] <= 2 or state.location[1] >= 8):
			return True
	
	if visiblenum == 6:
		if (state.location[1] <= 3 or state.location[1] >= 8):
			return True

	
	if visiblenum == 10:
		if (state.location[1] <= 5 or state.location[1] >= 8):
			return True
	
	return False

def init():
	
	global pub
	global all_stop_pub
	global pub_status
	global model
	global policy
	global lastpublish
	global savedPose
	global amclmessageReceived
	global ground_truth
	global add_delay
	global otherPose
	global lastGoal
	global go
	global maxsightdistance
	global dr_client
	global home
	global timestep
	global current_cg, curr_st, next_st 
	
	rospy.init_node('patrolcontrol')

	global recordConvTraj, useRegions
	recordConvTraj = 0
	useRegions=0
	recordConvTraj = int(rospy.get_param("~recordConvTraj"))
	observableStates = int(rospy.get_param("~observableStates"))
	useRegions = int(rospy.get_param("~useRegions"))
	data_log = str(rospy.get_param("~dataLog"))
	
	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, handle_pose)
	pub = rospy.Publisher("move_base_simple/goal", PoseStamped)

	all_stop_pub = rospy.Publisher("all_stop_enabled", std_msgs.msg.Bool)

	initialpose = rospy.Publisher("initialpose", PoseWithCovarianceStamped)
	rospy.Subscriber("base_pose_ground_truth", Odometry, save_ground_truth)
	
	pub_status = rospy.Publisher("/study_attacker_status", String, latch=True)

	global skip_interaction
	if not skip_interaction:				
		if add_delay:
			rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, handle_other)
		else:
			rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, handle_other)
					
	random.seed()
	np.random.seed()
	
	## Initialize constants

	p_fail = 0.05
	longHallway = 10
	shortSides = 2
	patrolAreaSize = longHallway + shortSides + shortSides
	observableStateLow = patrolAreaSize / 2 - 1
	observableStateHigh = patrolAreaSize / 2
	observableStateLow = 0
	observableStateHigh = patrolAreaSize
	
	
	# calculate farness for each node in the patrolled area
	farness = np.zeros(patrolAreaSize)
	for i in range(patrolAreaSize):
		sum = 0
		for j in range(patrolAreaSize):
			sum += abs(i - j)
	
		farness[i] = sum
	
	
	global m
	global ogmap
	current_cg = None	
	## Create reward function

	if (m == "boydright"):
		mapparams = boydrightMapParams(False)
		ogmap = OGMap(*mapparams)
		## Create Model
		model = patrol.model.PatrolModel(p_fail, None, ogmap.theMap()) 
		model.gamma = 0.9
		
	elif m == "largeGridPatrol":
		current_cg = 4
		mapparams = largeGridMapParams(False)
		ogmap = OGMap(*mapparams)
		## Create Model
# 		model = patrol.model.PatrolModel(p_fail, PatrolState(np.array( [6, 8, 0] )), ogmap.theMap())
		model = patrol.model.PatrolExtendedModel(0.01, PatrolExtendedState\
												(np.array( [-1,-1, -1] ),0), \
												ogmap.theMap())
		model.gamma = 0.9


	elif (m == "boydright2"):
		mapparams = boydright2MapParams(False)

		ogmap = OGMap(*mapparams)

		## Create Model

		model = patrol.model.PatrolModel(p_fail, None, ogmap.theMap())

		model.gamma = 0.9

	else:
		mapparams = boyd2MapParams(False)
		ogmap = OGMap(*mapparams)
		## Create Model
		model = patrol.model.PatrolModel(p_fail, None, ogmap.theMap())
		model.gamma = 0.9
	
	# Call external solver here instead of Python based
	args = ["boydsimple_t", ]
	import subprocess			

	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

	stdin = m + "\n"
	stdin += "1.0\n"
	
	(transitionfunc, stderr) = p.communicate(stdin)
	
	
	
	args = ["boydpatroller", ]
	p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	
	stdin = m + "\n"
	stdin += str(useRegions)+"\n"

	stdin += transitionfunc
					
					
	f = open(home + "/patrolstudy/toupload/patrollerT.log", "w")
	f.write(stdin)
	f.close()
	
	(stdout, stderr) = p.communicate(stdin)
	print("\n computed patroller's policy")

	if m == "largeGridPatrol":
		correctpolicy = home + "/patrolstudy/largeGridpolicy_mdppatrol"
	if m == "boydright":
		correctpolicy = home + "/patrolstudy/boydright_policy"
	if m == "boydright2":
		correctpolicy = home + "/patrolstudy/boydright2_policy"
	if m == "boyd2":
		correctpolicy = home + "/patrolstudy/boydpolicy_mdppatrolcontrol"
	
	# stdout now needs to be parsed into a hash of state => action, which is then sent to map agent
# 	correctpolicy = "/home/saurabh/patrolstudy/largeGridpolicy_mdppatrol"
 	f = open(correctpolicy,"w")
  	f.write("") 
 	f.close() 
 	
 	f = open(correctpolicy,"w")
	outtraj = ""
	p = {}
	stateactions = stdout.split("\n")
	for stateaction in stateactions:
		temp = stateaction.split(" = ")
		if len(temp) < 2: continue
		state = temp[0]
		action = temp[1]
 						
		outtraj += state+" = "+action+"\n"
 		
# 		ps = patrol.model.PatrolState(np.array([int(pieces[0]), int(pieces[1]), int(pieces[2])]))
		if m == "largeGridPatrol":
			state = state.split(";")
			loc = state[0]
			current_goal = int(state[1])
			loc = loc[1 : len(loc) - 1]
			pieces = loc.split(",")
			location = np.array([int(pieces[0]), int(pieces[1]), int(pieces[2])])
			ps = patrol.model.PatrolExtendedState(location,current_goal)
			
		else:
			state = state[1 : len(state) - 1]
			pieces = state.split(",")	
			ps = patrol.model.PatrolState(np.array([int(pieces[0]), int(pieces[1]), int(pieces[2])]))
 
		if action == "MoveForwardAction":
			a = patrol.model.PatrolActionMoveForward()
		elif action == "TurnLeftAction":
			a = patrol.model.PatrolActionTurnLeft()
		elif action == "TurnAroundAction":
			a = patrol.model.PatrolActionTurnAround()
		elif action == "TurnRightAction":
			a = patrol.model.PatrolActionTurnRight()
		else:
			a = patrol.model.PatrolActionStop()
 
		p[ps] = a
 		
  	f.write(outtraj)
 	f.close()

#  	pub_status.publish(String("abortrun"))
	
	from mdp.agent import MapAgent
	policy = MapAgent(p)

# 	sample trajectories for m == "largeGrid":
	global recordConvTraj
	if m == "largeGridPatrol" and recordConvTraj == 1: 
		from util.classes import NumMap
		state_dict = {}
		for s in model.S():
			if s.location[0]<=7 and s.location[1]<=1:
				state_dict[s] = 1.0
# 		state_dict = { PatrolState([0,0,0]):1.0 }
		initial = NumMap( state_dict ).normalize()
# 		data_log = "/home/saurabh/patrolstudy/toupload/recd_convertedstates_14_14_200states_LGR.log"
		f = open(data_log,"w")
		from mdp.simulation import sample_traj
		n_samples = 45
		t_max = 50
		outtraj = ""
# 		outtraj2 = "observableStates: " 
# 		outtraj2 += str(observableStates)
# 		s = ogmap.toState((6, 0, 0), False)
# 		s = PatrolState(np.array( [6, 0, 0] ))
# 		outtraj2 += str(largeGridisvisible(s, observableStates))
# 		pub_status.publish(String("datacollected"))
		
		pair_ct = 0 
		for i in range(n_samples): 
			traj_list = sample_traj(model, t_max, initial, policy) 
			for (s,a,s_p) in traj_list: 
# 				outtraj2 += str(s.location[0:3])
# 				outtraj2 += str(largeGridisvisible(s, observableStates))+"\n"    
				if largeGridisvisible(s, observableStates):
					s = s.location[0:3]
					# print(s)
					# print(sap[1].__class__.__name__)
					outtraj += str(s[0])+","+str(s[1])+","+str(s[2])+","
					if a.__class__.__name__ == "PatrolActionMoveForward":
						outtraj += "PatrolActionMoveForward"
					elif a.__class__.__name__ == "PatrolActionTurnLeft":
						outtraj += "PatrolActionTurnLeft"
					elif a.__class__.__name__ == "PatrolActionTurnRight":
						outtraj += "PatrolActionTurnRight"
					elif a.__class__.__name__ == "PatrolActionTurnAround":
						outtraj += "PatrolActionTurnAround"
					else:
						outtraj += "PatrolActionStop"
				else:
					outtraj += "None"

				outtraj += "\n"
				pair_ct += 1
# 			outtraj2 += "\n"
			if  pair_ct >= 200:
				outtraj += "ENDREC\n" 
				pair_ct = 0
				
		f.write(outtraj)
# 		f.write(outtraj2)
		f.close()
	
#	policy = mdp.solvers.PolicyIteration(30, mdp.solvers.IteratingPolicyEvaluator2(.5)).solve(model)
#	policy = mdp.solvers.PolicyIteration(30, mdp.solvers.IteratingPolicyEvaluator(20)).solve(model)
#	policy = mdp.solvers.ValueIteration(.5).solve(model)
	print("Finished solving mdp")
				
	
	has_slowed_down = False
	dr_client = dynamic_reconfigure.client.Client("move_base_node/TrajectoryPlannerROS")
	print("dr_client initialized")
	
	r = rospy.Rate(8)
	while not rospy.is_shutdown():
		
# 		print "savedPose is not None and otherPose is not None and canSee(savedPose, otherPose)"
# 		print (savedPose is not None and otherPose is not None and canSee(savedPose, otherPose))
		if m=="largeGridPatrol":
			print current_cg
		
		if savedPose is not None and otherPose is not None and canSee(savedPose, otherPose):
			
			if not add_delay:
				params = { 'max_vel_x' : 0.2 }
				config = dr_client.update_configuration(params)
				has_slowed_down = True
			
			if not add_delay and lastGoal is not None:			

				a = otherPose.orientation
				
				angles2 = tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])
	
				if (angles2[2] < 0):
					angles2 = (0,0, 2 * math.pi + angles2[2])
				otherState = ogmap.toState((otherPose.position.x,otherPose.position.y, angles2[2]), False)
				
				if lastGoal.conflicts(otherState):
#					if rospy.get_time() - lastpublish >= timestep:	
					lg = lastGoal
					while (lg.conflicts(otherState)):
						if m=="largeGridPatrol":
							curr_st = PatrolExtendedState(lg.location,current_cg)
							# print "(lg.conflicts(otherState)) next_st = policy.actions(curr_st).keys()[0].apply(curr_st)"
							next_st = policy.actions(curr_st).keys()[0].apply(curr_st)
							lg = PatrolState(next_st.location)
						else:
							lg = policy.actions(lg).keys()[0].apply(lg)
					
					sendGoal(lg)
				else:
					nextGoal()
					
			if add_delay and go:

				go = False
				
				topub = std_msgs.msg.Bool()
				topub.data = True

				all_stop_pub.publish(topub)
					

		elif savedPose is not None:
			
			if otherPose is not None and not canSee(savedPose, otherPose) and has_slowed_down:
				params = { 'max_vel_x' : 0.5 }
				config = dr_client.update_configuration(params)
				has_slowed_down = False
				
# 			print "nextGoal() in elif savedPose is not None"
			nextGoal()
			
		time.sleep(0.25)


if __name__ == "__main__":

	import argparse
	
	parser = argparse.ArgumentParser(description="Patroller Controller")
	parser.add_argument("tolerance")
	parser.add_argument("sd")
	parser.add_argument("map")				
	parser.add_argument("num")
	parser.add_argument("name")
	parser.add_argument("log")

	args = parser.parse_args()

	tolerance = float(args.tolerance)
	sd = float(args.sd)
	m = args.map
	if int(args.num) == 1:
		add_delay = True
								
	if int(args.num) == 2:
		add_delay = False
		skip_interaction = True
							
#	resetservice = rospy.ServiceProxy('move_base_node/clear_costmaps', std_srvs.srv.Empty)

	init()
