#import psycopg2
import os
#from subprocess import call

home = os.environ['HOME']


def run(patpolicylog):
	
	try:
		f = open(patpolicylog)
	except:
		return


	fout = open("/tmp/studyresults","a")
	
	
	for line in f:
		if (len(line) > 1):
			temp = line[line.find("[") + 1 : line.find("]")]
			
			components = temp.split(", ")
	
			for component in components:
				fout.write(" " + str(component))
			
			
	fout.close()
		

if __name__ == '__main__':
	
# 	host = "shutter"
# #	sshhost = "kdbappt.dyn.sevenbowlabs.com"
# 	sshhost = "shutter"
# 	
# 	conn = psycopg2.connect("host="+host+" dbname=current user=patrol password=patrolrunner")
# 	cur = conn.cursor()
# 
# 	cur.execute("select id, map from run3 where successful is not null and ppsucces = -1 order by id");
# 	rows = cur.fetchall()
# 
# 
# 	downloadDir = "/tmp"
# 	
# 	
# 	if not os.path.exists(downloadDir):
# 		os.makedirs(downloadDir)
# 
# 	for row in rows:
# 		print(row[0])
# 		themap = row[1]
# 		call(["scp", sshhost + ":/home/patrol/logupload/" + str(row[0]) + "-*", downloadDir + "/"])
# 		
# 		# uncompress bag files
# 		call(["tar", "-xzf", downloadDir + "/" + str(row[0]) + "-logs.tar.gz", "-C", downloadDir, "--strip-components", "4"])
	
	run(home + "/patrolstudy/toupload/t_weights.log")
	
			