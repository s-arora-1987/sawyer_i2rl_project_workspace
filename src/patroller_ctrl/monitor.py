#!/usr/bin/python
# import roslib; roslib.load_manifest('patroller_ctrl')

import rospy
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math

# from navigation_irl.ros_ctrl import statesfromstarttogoal
# from navigation_irl.patrol.time import getTimeConv

# subscribe to the overall run status node

# record when the attacker leaves

# record if the attacker is detected or if it gets to its destination

# write out when the attacker left to /tmp/studyresults

# if there's an error somewhere, or we run out of time, don't write out the file, just exit

# messages

attackerLeftAt = -1

goalX = 0
goalY = 0

# print ("Waiting time before shutdown after reachgoal of spotattacker"\
#        +str((statesfromstarttogoal*getTimeConv())))

def getTimeConv():
	return 2

def handle_attacker_status(req):
    global attackerLeftAt
    global goalX
    global goalY
    global lastPos2
    
    data = req.data

    receivedAt = rospy.get_time()

    if data == "attackerleft":
        if attackerLeftAt > 0:
            return
        attackerLeftAt = receivedAt
        print("Attacker left at: " + str(rospy.get_time()));
    elif data == "datacollected":
        rospy.signal_shutdown('')
    elif data == "emptyinput":
        f = open("/tmp/studyresults","w")
        f.write("e -1 -1")
        f.close()
        print("\n\n Shutting down due to empty input data \n ")
        rospy.signal_shutdown('')
    elif data == "reachgoal":
        dist = math.sqrt((goalX - lastPos2.x)*(goalX - lastPos2.x) + (goalY - lastPos2.y)*(goalY - lastPos2.y))
        f = open("/tmp/studyresults","w")
        f.write("y -1 -1")#+ str(attackerLeftAt) + " " + str(dist))
        f.close()
        print("\n\n Shutting down due to reaching the goal at \n "+str(receivedAt))
        rospy.signal_shutdown('')
    elif data == "abortrun2":
        dist = math.sqrt((goalX - lastPos2.x) * (goalX - lastPos2.x) + (goalY - lastPos2.y) * (goalY - lastPos2.y))
        f = open("/tmp/studyresults", "w")
        f.write("t -1 -1")
        f.close()
        print("\n\n Shutting down due to unfinished learning within time threshold \n " + str(receivedAt))
        rospy.signal_shutdown('')
    elif data == "abortrun":
        global statesfromstarttogoal
        # time.sleep(5*statesfromstarttogoal*getTimeConv())
        dist = math.sqrt((goalX - lastPos2.x)*(goalX - lastPos2.x) + (goalY - lastPos2.y)*(goalY - lastPos2.y))
        f = open("/tmp/studyresults","w")
        f.write("o -1 -1")
        f.close()
        print("\n\n Shutting down due to maximum searches reached at \n "+str(receivedAt))
        rospy.signal_shutdown('')


def handle_status(req):
    global attackerLeftAt
    global goalX
    global goalY
    global lastPos2
    
    data = req.data

    receivedAt = rospy.get_time()

    if data == "spotattacker":
        #print("Attacker Spotted! at: " + str(rospy.get_time()));
        # time.sleep(statesfromstarttogoal*getTimeConv())

        if attackerLeftAt > 0:
            # attacker spotted, it has failed
             
            dist = math.sqrt((goalX - lastPos2.x)*(goalX - lastPos2.x) + (goalY - lastPos2.y)*(goalY - lastPos2.y))
            f = open("/tmp/studyresults","w")
            f.write("n -1 -1")# + str(attackerLeftAt) + " " + str(dist))
            f.close()
            print("\n\n Shutting down due to attacker spotted at \n "+str(receivedAt))
            rospy.signal_shutdown('')

lastPos0 = None
lastPos0Time = -1

def handle_patroller0(req):
	global lastPos0
	global lastPos0Time

	if lastPos0 is None:
		lastPos0 = req.pose.pose.position
		lastPos0Time = rospy.get_time()
	else:
		if not lastPos0.x == req.pose.pose.position.x or not lastPos0.y == req.pose.pose.position.y:
			lastPos0 = req.pose.pose.position
			lastPos0Time = rospy.get_time()
		elif rospy.get_time() - lastPos0Time > 60 * 2:
			f = open("/tmp/studyresults","w")
			f.write("d -1 -3")
			f.close()
			print("\n\n Shutting down at "+str(rospy.get_time())+" due to patroller 0 death\n ")
			rospy.signal_shutdown("")


lastPos1 = None
lastPos1Time = -1

def handle_patroller1(req):
	global lastPos1
	global lastPos1Time

	if lastPos1 is None:
		lastPos1 = req.pose.pose.position
		lastPos1Time = rospy.get_time()
	else:
		if not lastPos1.x == req.pose.pose.position.x or not lastPos1.y == req.pose.pose.position.y:
			lastPos1 = req.pose.pose.position
			lastPos1Time = rospy.get_time()
		elif rospy.get_time() - lastPos1Time > 60 * 2:
			f = open("/tmp/studyresults","w")
			f.write("d -1 -3")
			f.close()
			print("\n\n Shutting down at "+str(rospy.get_time())+" due to patroller 1 death\n ")
			rospy.signal_shutdown("")


lastPos2 = None
def handle_attacker(req):
	global lastPos2
	lastPos2 = req.pose.pose.position	

def init():

    global statesfromstarttogoal
    # forward + 1 turn + 7 forwards
    statesfromstarttogoal = 1 + 1 + 7

    rospy.init_node('study_monitor')
    
    rospy.Subscriber("/study_status", String, handle_status)
    
    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, handle_patroller0)
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, handle_patroller1)
    rospy.Subscriber("/study_attacker_status", String, handle_attacker_status)
    rospy.Subscriber("/robot_2/base_pose_ground_truth", Odometry, handle_attacker)

    import argparse
    parser = argparse.ArgumentParser(description="Patroller Controller")
    parser.add_argument("configfile")
    
    args = parser.parse_args()

    f = open(args.configfile)

    startPos = f.readline().strip()
    goalPos = f.readline().strip()

    f.close()

    goalPos = goalPos.split(" ")
    global goalX
    global goalY
    goalX = float(goalPos[0])
    goalY = float(goalPos[1])
 
    import time
    end = time.time() + (100 * 60) #(25 * 60)
    while not rospy.is_shutdown() and time.time() < end:
        time.sleep(1)
 
    if time.time() >= end:
        print("\n \n Run ran out of time at "+str(rospy.get_time())+" \n ")
        import os.path
        if not os.path.isfile("/tmp/studyresults"):
            f = open("/tmp/studyresults","w")
            f.write("ro -1 -1")
            f.close()

init() 
