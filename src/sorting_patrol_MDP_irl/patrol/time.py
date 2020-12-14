
def getTimeConv():
	return 2

def fromRospyTime(t):
	return int(t / getTimeConv())

def toRospyTime(t):
	return t * getTimeConv()