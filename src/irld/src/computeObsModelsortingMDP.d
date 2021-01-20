import mdp;
import std.stdio;
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
import sortingMDP;

int main() {
	
	sac [][] SAC;
    sac [][] SACfull;
	LinearReward reward;
	Model model;
	double chanceNoise=0.90;

    model = new sortingMDPWdObsFeatures(0.05,null, 0, chanceNoise);
    State ts = model.S()[0];
    auto features = model.obsFeatures(ts,model.A(ts)[0],ts,model.A(ts)[0]);
    int numObFeatures = cast(int)(features.length);
    model.setNumObFeatures(numObFeatures);

	// for comparing complete obs model 
	StateAction [] all_sa_pairs;
	foreach(s;model.S()) {
		foreach(a;model.A(s)){
			all_sa_pairs ~= new StateAction(s,a);
		}
	}

	int dim = 11;
	reward = new sortingReward7(model,dim); 
	
    double [] reward_weights = new double[dim];
	reward_weights[] = 0;

	// The new sorting MDP class is specific to pip behavior and therefore 
	// have more relaxed transition dyanamics. That introduces more 
	// options in state and action combinations for introducing noise.
	double [] params_pip_sortingModelbyPSuresh4multipleInit_onlyPIP = [0.13986013986013984, 
	0.13986013986013984, 0.13986013986013984, 0.13986013986013984, 0.013986013986013986, 
	0.006993006993006993, 0.0, 0.2797202797202797, 0.0, 0.0, 0.2797202797202797];
	reward_weights[] = params_pip_sortingModelbyPSuresh4multipleInit_onlyPIP[]; 

    model.setReward(reward);
	reward.setParams(reward_weights);
    model.setGamma(0.99);

	//ValueIteration vi = new ValueIteration();
	int vi_duration_thresh_secs = 30;
	TimedValueIteration vi = new TimedValueIteration(int.max,false,vi_duration_thresh_secs); 
	Agent policy; 
    double vi_threshold;
    vi_threshold = 0.25; 

    double[State] V; 
    V = vi.solve(model, vi_threshold); 
    policy = vi.createPolicy(model, V); 

	double[State] initial;
	foreach (s; model.S()) {
		initial[s] = 1.0;
	}
	Distr!State.normalize(initial); 
	
	debug {
		writeln("created policy");
		//foreach(s;model.S()) {
			//writeln("s:",s,",act:",policy.sample(s));
			//writeln("\n");
		//}
	}
	debug {
		foreach(i;0..3){
			//writeln(simulate(model, policy, initial, 15));
			;
		}
		
		//exit(0);
	}

	int num_trajs = 1;
	int size_traj = 1;
	bool useHierDistr = 1;	
	// decide what values of features will help lbfgs dsitinguish events
	bool lbfgs_use_ones = 0;
	int num_sessions = 10;
	int num_trials_perSession = 1;
	bool allGTdata = 0;
	int num_runs = 100;
	double[][] arr_metric3data;

	foreach(i;0..num_runs) {
		double [] learnedDistrFeat = simulateNoisyDemo_Incremental_ObsModLearning(model, all_sa_pairs, policy, initial, num_trajs, 
		size_traj, useHierDistr, lbfgs_use_ones, num_sessions, num_trials_perSession, allGTdata, arr_metric3data); 
		//double[StateAction][StateAction] obsModel = createObsModel(model, learnedDistrFeat, lbfgs_use_ones, useHierDistr); 
		//model.setObsMod(obsModel);
		writeln("\n\n\n a run finished \n\n\n");
	}

	double[] avg_metric3_per_session;
	avg_metric3_per_session.length = num_sessions;
	foreach(i; 0 .. num_sessions ) {
		avg_metric3_per_session[i] = sum(arr_metric3data.transversal(i))/cast(double)(arr_metric3data.length);
	}

	double[] stdev_metric3_per_session;
	stdev_metric3_per_session.length = num_sessions;
	auto n = cast(double)(arr_metric3data.length);
	foreach(i; 0 .. num_sessions ) {
		auto a = arr_metric3data.transversal(i);
		double avg = avg_metric3_per_session[i];
		//double var = 0.0;
		//foreach (arr;arr_metric3data) var += pow(arr[i] - avg, 2) / n;
		auto var = reduce!((a, b) => a + pow(b - avg, 2) / n)(0.0f, a);
	    stdev_metric3_per_session[i] = cast(double)sqrt(var);
	}

	File file3 = File("/home/saurabharora/Downloads/resultsApproxObsModelMetric3.csv", "a"); 
	string str_avg_metric3_per_session = to!string(avg_metric3_per_session);
	str_avg_metric3_per_session = str_avg_metric3_per_session[1 .. (str_avg_metric3_per_session.length-1)];
	file3.writeln("\n");
	file3.writeln(str_avg_metric3_per_session);
	string str_stdev_metric3_per_session = to!string(stdev_metric3_per_session);
	str_stdev_metric3_per_session = str_stdev_metric3_per_session[1 .. (str_stdev_metric3_per_session.length-1)];
	file3.writeln("\n");
	file3.writeln(str_stdev_metric3_per_session);
	file3.close(); 

	writeln("all runs finished and results are in file resultsApproxObsModelMetric3.csv");

	return 0;

}

