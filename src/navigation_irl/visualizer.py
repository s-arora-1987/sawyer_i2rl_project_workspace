import visualizercore2
	
if __name__ == '__main__':
	
	# read in and process bag and log files
	import argparse
	parser = argparse.ArgumentParser(description="Visualizer")
	parser.add_argument("gotimesLog")
	parser.add_argument("attackerpolicyLog")
	parser.add_argument("attackerBag")
	parser.add_argument("patroller1Bag")
	parser.add_argument("patroller2Bag")
	parser.add_argument("predictionLog")
	
	args = parser.parse_args()

	visualizercore2.runVis(args.gotimesLog, args.attackerpolicyLog, args.attackerBag, args.patroller1Bag, args.patroller2Bag, args.predictionLog)
