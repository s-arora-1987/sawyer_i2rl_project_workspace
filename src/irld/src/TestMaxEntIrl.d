import mdp;
import boydmdp;
import std.stdio;
import std.math;
import std.string;
import std.format;
import std.random;
import irl;


    double [] feature_expectations(Model model, double[StateAction] D) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	// modified to only include observed states
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
    	foreach(sa, v; D) {
    		double [] f = ff.features(sa.s, sa.a);
    		f[] *= v;
    		
    		returnval[] += f[];
    	
    	}

        return returnval;
	}	
	
int main() {
	
	string mapToUse;
	
	string buf;
//	buf = readln();
//	formattedRead(buf, "%s", &mapToUse);
	
	mapToUse = "boyd2";
		
//	double p_fail = 0.3;
	double p_fail = 0.1;

	byte[][] map;
	LinearReward reward;
	 
	if (mapToUse == "boyd2") {
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
		
	} else {
		map = [[1, 1, 1, 1, 1, 1, 1, 1, 0], 
				     [0, 0, 1, 1, 1, 1, 0, 1, 0],
				     [0, 0, 0, 0, 1, 1, 1, 1, 1],
				     [0, 0, 0, 0, 0, 0, 1, 1, 1],
				     [0, 0, 0, 0, 0, 0, 0, 1, 0],
				     [0, 0, 0, 0, 0, 0, 1, 1, 0]];
		
	}
	
	BoydModel model = new BoydModel(p_fail, null, map);

	if (mapToUse == "boyd2") {
		
		int[State] distances;
		
		assignDistance(model, new BoydState([5,0,0]), distances);
				
/*		reward = new Boyd2Reward(model, distances);
		double [14] reward_weights = [1, -1, -1, -1, -1, -1, -1, -1, -1, -1, .5, -1, -1, -1];*/
		
		reward = new Boyd2RewardReducedFeatures(model, distances);
		double [2] reward_weights = [1, .91];
	
		reward.setParams(reward_weights);
	} else {				     
		reward = new BoydRightReward(model);		     
		double [4] reward_weights = [1, -.1, 0, 0];
	
		reward.setParams(reward_weights);
		
	}
	
	
	
	model.setReward(reward);
	
	model.setGamma(0.99);
	
	ValueIteration vi = new ValueIteration();
	
	double[State] V = vi.solve(model, .1);
	
	Agent a = vi.createPolicy(model, V);
	
	
	int length = 100;
	
	
	
		
	double[State] initials;
	foreach (s; model.S()) {
		initials[s] = 0;
	}
/*	Distr!State.normalize(initials);*/
	
	initials[new BoydState([5,0,0])] = 1.0;
	
	
	double[StateAction] freq = calcStateActionFreq(a, initials, model, length);
	
	double [] exp_f = feature_expectations(model, freq);
	
	writeln("True Expected Features: ", exp_f);
	
	
	sar[][] traj;
	
	// Run Simulate to get a trajectory
	foreach( sim; simulate(model, a, initials, length)) {
		sar [] temp;
		sim.p = 1.0;
		temp ~= sim;
		
		traj ~= temp;
	}
	
	// remove all states that aren't visible
	
	foreach (ref sararr; traj) {
		if ((cast(BoydState)(sararr[0].s)).getLocation()[0] < 4 || (cast(BoydState)(sararr[0].s)).getLocation()[0] > 5) {
			sararr = sararr[0..$-1];
		} 
	}
	
	writeln(traj);
	
	// setup a maxent solver
	
	State [] observedStatesList;
	
	foreach (s; model.S()) {
		bool add = false;
		outerLoop: foreach (int timestep, sar [] SAR2; traj) {
		
			foreach (int item, sar SAR3; SAR2) {
			
				if (s.samePlaceAs(SAR3.s)) {
					add = true;
					break outerLoop;
				}
			}
							
		}
		if (add) {
			observedStatesList ~= s;
		}
	}
	
	
//	double [] lastWeights = new double[reward.dim()];
	double [] lastWeights = new double[2];

	for (int i = 0; i < lastWeights.length; i ++)
		lastWeights[i] = uniform(-.1, .1);
	
//	lastWeights = [1, -1, -1, -1, -1, -1, -1, -1, -1, -1, .5, -1, -1, -1];
	double [] foundWeights;
	
	double val;
	
	MaxEntIrlPartialVisibility irl = new MaxEntIrlPartialVisibility(100,new ValueIteration(), 0, .1, .01, .009, observedStatesList);
	
	// Run, see if we get the same weights

	foreach (s; model.S()) {
		initials[s] = 1;
	}
	Distr!State.normalize(initials);
	
	Agent policy = irl.solve(model, initials, traj, lastWeights, val, foundWeights);
	
	
	int sum = 0;
	int count = 0;
	
	foreach (s; model.S()) {
		double[Action] testPolicy = policy.actions(s);
		double[Action] truePolicy = a.actions(s);
		
		foreach(act, v; testPolicy) {
			if (act in truePolicy)
				sum ++;
		}
		
		
		count ++;
	}
	writeln();
	writeln(val, " ", foundWeights);
	writeln();
	writeln("Percent Correct: ", (sum * 1.0) / (count * 1.0));
	
	foreach (s; model.S()) {
		initials[s] = 0;
	}	
	
	initials[new BoydState([5,0,0])] = 1.0;
	
	model.p_fail = 0;
	
	
	traj.length = 0;
	
	foreach( sim; simulate(model, a, initials, length)) {
		sar [] temp;
		sim.p = 1.0;
		temp ~= sim;
		
		traj ~= temp;
	}
	
	
	sar[][] traj_test;
	
	// Run Simulate to get a trajectory
	foreach( sim; simulate(model, policy, initials, length)) {
		sar [] temp;
		sim.p = 1.0;
		temp ~= sim;
		
		traj_test ~= temp;
	}	

	foreach (n; 0..traj_test.length) {
		writeln(traj[n][0].s, " -> ", traj[n][0].a, "  VS  ", traj_test[n][0].s, " -> ", traj_test[n][0].a); 
	}
	 
	return 0;	
	
}