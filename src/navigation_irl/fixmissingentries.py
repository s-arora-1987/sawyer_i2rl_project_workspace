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
import roslib; roslib.load_manifest('navigation_irl')

import rospy
import rosbag

import psycopg2
import os
from subprocess import call


if __name__ == '__main__':
	
	
	host = "localhost"
	sshhost = "kbogert.homelinux.net"
	
	conn = psycopg2.connect("host="+host+" dbname=uai2014 user=patrol password=patrolrunner")
	cur = conn.cursor()

	cur.execute("select id from run6 where successful is null and id > 2611  order by id");
	rows = cur.fetchall()


	downloadDir = "/tmp"
	
	
	if not os.path.exists(downloadDir):
		os.makedirs(downloadDir)

	for row in rows:
		print(row[0])
		call(["scp", sshhost + ":/home/patrol/bags/" + str(row[0]) + "-*", downloadDir + "/"])
		
		# uncompress bag files
		call(["tar", "-xzf", downloadDir + "/" + str(row[0]) + "-bags.tar.gz", "-C", downloadDir, "--strip-components", "4"])
	
		attackerBag = downloadDir + "/attacker.bag"
		
		try:
			bag = rosbag.Bag(attackerBag)
		except:
			continue
		
		
		lastPos = None
		for topic, msg, t in bag.read_messages(topics=['/robot_2/base_pose_ground_truth']):
			lastPos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
		bag.close()
		
		print(lastPos)
		
		
		distance = math.sqrt((lastPos[0] - 16.52)*(lastPos[0] - 16.52) + (lastPos[1] - 25.55)*(lastPos[1] - 25.55))
		
		cur.execute("update run6 set successful = %s, distancetogoal = %s where id = %s", ((distance <= 0.6), distance, row[0]))
		
		
		conn.commit()
		
		os.unlink(attackerBag)
		os.unlink(downloadDir + "/" + str(row[0]) + "-bags.tar.gz")