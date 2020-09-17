import mdp;
import boydmdp;
import jointboydmdp;
import irl;
import std.stdio;
import std.format;
import std.string;
import std.math;
import std.random;


int main() {
	
	// Read in stdin to get settings and trajectory
	
	bool addDelay = false;
	sar [][] SAR;
	int interactionLength;

	string mapToUse;
	string buf;
	buf = readln();
	
	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	buf = readln();	
	formattedRead(buf, "%s", &addDelay);
	buf = readln();
	string algorithm;
	
	formattedRead(buf, "%s", &algorithm);
	algorithm = strip(algorithm);

	buf = readln();
	formattedRead(buf, "%s", &interactionLength);
	
	int curPatroller = 0;
	
	string[][] percepts;

	int tempCounter= 0;
    while ((buf = readln()) != null) {
    	buf = strip(buf);
    
    	if (buf == "ENDTRAJ") {
    		tempCounter = 0;
    	} else {
    		if (percepts.length <= tempCounter) {
    			percepts.length = tempCounter + 1;
    		}	
    		percepts[tempCounter] ~= buf;
    		
    		tempCounter ++;
    	}
    	
    }

	foreach (string [] p; percepts) {
		sar [] newtraj;

		string percept = p[0][0..p[0].indexOf(";")];
		
		string state;
		string action;
		double prob;
		
		formattedRead(percept, "%s:%s:%s", &state, &action, &prob);
		
		int x;
		int y;
		int z;
		
		formattedRead(state[1..state.length], "%s, %s, %s]", &x, &y, &z);


		int action1;

		if (action == "MoveForwardAction") {
			action1 = 0;
		} else if (action == "StopAction") {
			action1 = 1;
		} else if (action == "TurnLeftAction") {
			action1 = 2;
		} else if (action == "TurnAroundAction") {
			action1 = 4;
		} else {
			action1 = 3;
		}


		percept = p[1][0..p[1].indexOf(";")];
			
		formattedRead(percept, "%s:%s:%s", &state, &action, &prob);
		
		int i;
		int j;
		int k;
		
		formattedRead(state[1..state.length], "%s, %s, %s]", &i, &j, &k);

		int action2;

		if (action == "MoveForwardAction") {
			action2 = 0;
		} else if (action == "StopAction") {
			action2 = 1;
		} else if (action == "TurnLeftAction") {
			action2 = 2;
		} else if (action == "TurnAroundAction") {
			action2 = 4;
		} else {
			action2 = 3;
		}

		int interaction = 0;
		if (JointBoydState.isConflict([x, y, z],[x, y, z],[i,j,k],[i,j,k])) {
			interaction = interactionLength;
		}
		
		Action a = new JointBoydAction(action1, action2, interactionLength);		
		
		newtraj ~= sar(new JointBoydState([x, y, z],[i,j,k], interaction), a, prob);
		
		SAR ~= newtraj;
	
	}

	double p_fail = 0.1;
//	double p_fail = 0.05;


	byte[][] map;
	LinearReward reward;
	 
	map  = 		[[1, 1, 1, 1, 1], 
			     [1, 0, 0, 0, 0],
			     [1, 0, 0, 0, 0],
			     [1, 0, 0, 0, 0],
			     [1, 0, 0, 0, 0],
			     [1, 0, 0, 0, 0],
			     [1, 0, 0, 0, 0],
			     [1, 0, 0, 0, 0],
			     [1, 0, 0, 0, 0],
			     [1, 0, 0, 0, 0],
			     [1, 1, 1, 1, 1]];
		
		
	JointBoydAction NE = new JointBoydAction(0, 1, interactionLength);
	JointBoydModel model = new JointBoydModel(p_fail, null, map, NE, interactionLength);

	int[State] distance;
		
	BoydModel smallmodel = new BoydModel(p_fail, null, map);
	assignDistance(smallmodel, new BoydState([5,0,0]), distance);

	
	int [][][] distances;
	
	distances.length = map.length;
	foreach (ref x; distances)
		x.length = map[0].length;
		
	foreach (x; distances)
		foreach (ref y; x)
			y.length = 4;

	
	foreach(i; 0..distances.length) {
		foreach(j; 0..distances[i].length) {
			foreach (k; 0..distances[i][j].length) {
				if (map[i][j] == 1)
					distances[i][j][k] = distance[new BoydState([i,j,k])];
			}
		}
	}
	
	
	reward = new JointBoyd2Reward(model, distances);
	double [] reward_weights;
	reward_weights.length = reward.dim();
	reward_weights[] = 0;

	reward.setParams(reward_weights);
	
	model.setReward(reward);
	
	model.setGamma(0.9);


	double[State] initials;
	
/*	foreach (int num, sar [] temp; SAR) {
		foreach(sar SAR2; temp) {
			initials[num][SAR2.s] = 1.0;
		}
		Distr!State.normalize(initials[num]);
		
	} */
	
	
	foreach (s; model.S()) {
		initials[s] = 0;
	}
	
	initials[SAR[0][0].s] = 1.0;
	Distr!State.normalize(initials);
	
	
	Agent policy = new RandomAgent(model.A(null));
	
	int counter = 0;
	
	double [] featureExpectations;
	featureExpectations.length =  reward.dim();
	featureExpectations[] = 0;
	
	double [] lastWeights = new double[featureExpectations.length];
	for (int i = 0; i < lastWeights.length; i ++)
		lastWeights[i] = uniform(-.1, .1);
	
	
	int chosenEquilbrium = 0;
	

	double [] foundWeights;
	
	double val;
	
	State [] observableStates;
	
	foreach (s; model.S())
		observableStates ~= s;
	
	
	MaxEntIrlPartialVisibility irl = new MaxEntIrlPartialVisibility(100,new ValueIteration(), 1000, .1, .01, .009, observableStates);
	
	policy = irl.solve(model, initials, SAR, lastWeights, val, foundWeights);
	
	
/*	ValueIteration vi = new ValueIteration();
	
	reward_weights = [1, -1, -1, -1, -1, -1, -1, -1, -1, -1, .5, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, .5, -1, -1, -1];
					 
	reward.setParams(reward_weights);
	double[State] V = vi.solve(model, .1);
	
	policy = vi.createPolicy(model, V); */	
	
	
	foreach(i, row; map) {
		foreach(j, col; row) {
			if (col == 1) {
				foreach (k; 0..4) {
					int [] otherOne = [0,0,0];
					if (i <= 2)
						otherOne = [10,0,0];
					JointBoydState ps = new JointBoydState([i, j, k], otherOne, 0);
					JointBoydAction act = cast(JointBoydAction)policy.actions(ps).keys()[0];
					writeln( ps.getLocation1(), " = ", JointBoydAction.printAction(act.getAction1()));
				}
			
			}
		}
	}
	
	writeln("ENDPOLICY");

	foreach(i, row; map) {
		foreach(j, col; row) {
			if (col == 1) {
				foreach (k; 0..4) {
					int [] otherOne = [0,0,0];
					if (i <= 2)
						otherOne = [10,0,0];
					
					JointBoydState ps = new JointBoydState(otherOne, [i, j, k], 0);
					JointBoydAction act = cast(JointBoydAction)policy.actions(ps).keys()[0];
					writeln( ps.getLocation2(), " = ", JointBoydAction.printAction(act.getAction2()));
				}
			}
		}
	}
	writeln("ENDPOLICY");
	
	
	writeln(JointBoydAction.printAction(NE.getAction1()), " = 1");
	
	writeln("ENDE");
	
	writeln(JointBoydAction.printAction(NE.getAction2()), " = 1");
	
	writeln("ENDE");
	
	
	return 0;
}