import boydmdp;
import mdp;
import std.stdio;
import irl;
import std.random;
import std.math;
import std.range;
import std.traits;
import std.numeric;
import std.format;
import std.algorithm;
import std.string;
import core.stdc.stdlib : exit;
import std.file;
import std.conv;
import solverApproximatingObsModel;

byte [][] patrolToyMap() {
	//return [[1,1]]; 
	return [[1,1],
			[0,1]]; 
}


class keepTurning : LinearReward {
	
	Model model;
	
	public this (Model model) {
		
		this.model = model;
	}
	
	public override int dim() {
		return 2;
	}
			
	public override double [] features(State state, Action action) {

		BoydState currState = cast(BoydState)state;
		BoydState newState = cast(BoydState)action.apply(state);

		double [] result;
		result.length = dim();
		result[] = 0;

		// encourage turning left
		bool turned = false; 
		int oldOrient = currState.getLocation()[2];
		int newOrient = newState.getLocation()[2];
		if ((newOrient == 0 && oldOrient == 3) ||
			(newOrient == oldOrient+1) )
			turned = true;

		if (! model.is_legal(newState) )
			turned = false;

		if (turned)
			result[0] = 1;

		// encourage moving
		bool moved = true;		
		if (! model.is_legal(newState) || newState.samePlaceAs(state)) 
			moved = false;
		if (moved) 
			result[1] = 1;

		// encourage stopping
		//bool stopped = false; 
		//if ((cast(StopAction)action) && newState.samePlaceAs(state)) {
		//	//writeln("action ",action);
		//	stopped = true;
		//}
		//if (! model.is_legal(newState))
		//	stopped = false;
		//if (stopped) 
		//	result[1] = 1;
		
		return result;
	}

}

int main() {
	
	sac [][] SAC;
    sac [][] SACfull;
	string mapToUse;

	byte[][] map;

	map = patrolToyMap();
    BoydModelWdObsFeatures model = new BoydModelWdObsFeatures(null, map, null, 0);

    State ts = model.S()[0];
    auto features = model.obsFeatures(ts,model.A(ts)[0],ts,model.A(ts)[0]);

    int numObFeatures = cast(int)(features.length);
    model.setNumObFeatures(numObFeatures);

	debug {
		writeln("number of states ",(model.S()).length);
		writeln("number of obs features ",(model.getNumObFeatures()));
	}

	double[State][Action][State] T;
	double p_fail = 0.05;
	
	foreach(s;model.S()) {
		foreach(a;model.A(s)){
			State ins = a.apply(s);
			foreach (ns;model.S()) {
				if (cast(StopAction)a) {
					if (ns==s) {
						T[s][a][ns] = 1.0;
					} else T[s][a][ns] = 0.0;

				} else  {
					if (ns==s) {
						T[s][a][ns] = p_fail;
					} else {
						if (ns == ins) T[s][a][ns] = 1-p_fail;
						else T[s][a][ns] = 0.0;
					}
				}
			}
		}
	}
	model.setT(T);

	// for comparing complete obs model 
	StateAction [] all_sa_pairs;
	foreach(s;model.S()) {
		foreach(a;model.A(s)){
			all_sa_pairs ~= new StateAction(s,a);
		}
	}

	
	debug {
		int totalSA_pairs = 0;
		writeln("number of states ",(model.S()).length);
		writeln("number of actions ",(model.A()).length);
		//writeln("created trans function");
		//foreach(s;model.S()) {
		//	foreach(a;model.A(s)){
		//		writeln(new StateAction(s,a));
		//		writeln(model.T(s,a));
		//		writeln("\n");
		//		totalSA_pairs +=1;
		//	}
		//}
		writeln("# valid sa pairs ",totalSA_pairs);
	}

	// Generate samples with random prediction scores
	LinearReward reward;
	double [] reward_weights = [0.4,0.6]; 
    reward = new keepTurning(model);
	reward.setParams(reward_weights);
    model.setReward(reward);

	debug {
		writeln("created reward features ");
		foreach(s;model.S()) {
			foreach(a;model.A(s)){
				double [] fv = reward.features(s,a);
				double r = model.R(s, a);
			}
		}
		writeln("started policy computation ");
	}
    model.setGamma(0.99);

	ValueIteration vi = new ValueIteration();
    double[State] V = vi.solve(model, .5);
	debug {
		writeln("computed V",V);
	}

	Agent policy = vi.createPolicy(model, V);;
	double[State] initial;
	foreach (s; model.S()) {
		initial[s] = 1.0;
	}
	Distr!State.normalize(initial); 
	
	debug {
		writeln("created policy");
	}
	int num_trajs = 1;
	int size_traj = 3;
	bool useHierDistr = 0;	
	// decide what values of features will help lbfgs dsitinguish events
	bool lbfgs_use_ones = 1;
	int num_sessions = 10;

	int num_trials_perSession = 1;
	bool allGTdata = 0;
	double[][] arr_metric3data;
	
	auto learnedDistrFeat = simulateNoisyDemo_Incremental_ObsModLearning(model, all_sa_pairs, policy, initial, num_trajs, 
	size_traj, useHierDistr, lbfgs_use_ones, num_sessions, num_trials_perSession, allGTdata, arr_metric3data);	

	//writeln("BEGPARSING");

	//writeln("ENDPARSING");
	
	return 0;
}
