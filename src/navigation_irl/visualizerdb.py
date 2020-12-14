import visualizercore
import argparse
import psycopg2
import os
from subprocess import call


if __name__ == '__main__':
	
	# read in and process bag and log files
	parser = argparse.ArgumentParser(description="Visualizer")
	parser.add_argument("db")
	parser.add_argument("id")	
	
	args = parser.parse_args()
	
	host = "shutter"
	sshhost = "shutter"
	
	conn = psycopg2.connect("host="+host+" dbname=patrol user=patrol password=patrolrunner")
	cur = conn.cursor()

	cur.execute("select id, attacker, penalty, detect, predict, obsstates, attackerdelay, pfail, interactionlength, map from " + args.db + " where id = " + str(args.id));
	rows = cur.fetchall()


	downloadDir = "/tmp"
	
	
	if not os.path.exists(downloadDir):
		os.makedirs(downloadDir)

	row = rows[0]	
	call(["scp", sshhost + ":/home/patrol/bags/" + str(row[0]) + "-*", downloadDir + "/"])
	call(["scp", sshhost + ":/home/patrol/logupload/" + str(row[0]) + "-*", downloadDir + "/"])
	
	# uncompress bag files
	call(["tar", "-xzf", downloadDir + "/" + str(row[0]) + "-bags.tar.gz", "-C", downloadDir, "--strip-components", "4"])
	
	gotimesLog = downloadDir + "/" + str(row[0]) + "-gotimes.log"
	predicttime = int(row[4])
	interactionlength = row[8]
	attackerpolicyLog = downloadDir + "/" + str(row[0]) + "-attackerpolicy.log"
	policyLog = downloadDir + "/" + str(row[0]) + "-policy.log"
	attackerBag = downloadDir + "/attacker.bag"
	patroller1Bag = downloadDir + "/robot0.bag"
	patroller2Bag = downloadDir + "/robot1.bag"
			

	if (row[9] == "boyd2"):
		timeScale = 3.5
	else:
		timeScale = 4.8
	
	if row[1] == "irl" or row[1] == "maxentirl":
		interactionlength = 0
	
	visualizercore.runVis(gotimesLog, attackerpolicyLog, attackerBag, patroller1Bag, patroller2Bag, predicttime, interactionlength, row[9], policyLog, timeScale)
