import sortingMDP;
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

int main() {
	
	sac [][] SAC;
    sac [][] SACfull;
	LinearReward reward;
	Model model;

    model = new sortingMDPWdObsFeatures(0.05,null, 0);
    State ts = model.S()[0];
    int numObFeatures = cast(int)(model.obsFeatures(ts,model.A(ts)[0]).length);
    model.setNumObFeatures(numObFeatures);

	int dim = 11;
	reward = new sortingReward7(model,dim); 
	
    double [] reward_weights = new double[dim];
	reward_weights[] = 0;
	double [] params_pickinspectplace_reward7woplacedmixedinit = [0.13986013986013984, 
	0.06993006993006992, 0.13986013986013984, 0.06993006993006992, 0.013986013986013986, 
	0.006993006993006993, 0.0, 0.2797202797202797, 0.0, 0.0, 0.2797202797202797];
	reward_weights[] = params_pickinspectplace_reward7woplacedmixedinit[]; 

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
	
	sar [][] GT_trajs;
	sac [][] obs_trajs;
	int num_trajs = 1;
	int size_traj = 3;
	sar [] temp_traj;
	sac [] obs_traj;
	
	// collect all possible values of arrays output of features function
	// Option 1 for testing efficacy of approximation: Create random distr over features. assume multiplicative model for computing 
	// the cumulative probs for all possible combinations of feature values
	double [] trueDistr_obsfeatures = new double[model.getNumObFeatures()];
	double totalmass_untilNw = 0.0;
	double currp;
	foreach (i; 0 .. model.getNumObFeatures()-1) {
		currp = uniform(0.0,1-totalmass_untilNw);
		trueDistr_obsfeatures[i] = currp;
		totalmass_untilNw += currp;
		
	} 
	trueDistr_obsfeatures[model.getNumObFeatures()-1] = 1-totalmass_untilNw;
	debug { 
		writeln("true distr obsfeatures ",trueDistr_obsfeatures); 
		//exit(0);
	}

	// Option 2: HARDCODE the cumulative probs for all possible combinations of feature values
	//cum_prob_fvs = [0.1,0.3,0.6];


	// Option 2: HARDCODE the cumulative probs for all possible combinations of feature values
	//cum_prob_fvs = [0.1,0.3,0.6];

	double [] cum_prob_fvs;
	int [][] possible_obs_fvs;
	int [] obs_fv;
	double prod;
	//int idx_fv = 0;
	foreach(s;model.S()) {
		foreach(a;model.A(s)){
			obs_fv = model.obsFeatures(s, a);
			if (! possible_obs_fvs.canFind(obs_fv)) {
				possible_obs_fvs ~= obs_fv;
				prod = 1.0;
				foreach(i,f;obs_fv) {
					if (f==1) prod *= trueDistr_obsfeatures[i];
				}
				cum_prob_fvs ~= prod;
				//idx_fv += 1;
			}
		}
	} 

	double[StateAction][StateAction] trueObsMod;
	trueObsMod = createObsModel(model, trueDistr_obsfeatures); 

	double [] learnedDistr_obsfeatures = new double[model.getNumObFeatures()]; 
	// learned distribution incrementally averaged over sessions
	double [] runAvg_learnedDistr_obsfeatures = new double[model.getNumObFeatures()]; 
	runAvg_learnedDistr_obsfeatures[] = 0.0;
	double numSessionsSoFar = 0.0;
	// arrays of error values for both metrics 
	double [] arr_avg_cum_diff1, arr_avg_cum_diff2;
	int num_sessions = 10;
	int num_trials_perSession = 1;
	debug {
		double [][] arr_arr_cum_diff1;
	}

	// For metric 1 to accommodate the new s-a pairs with every next session, the noise corrupted feature 
	// arrays has to collected incrementally. That also avoids the chances of divide by 0 in computation of
	// metric 1 below 
	int [][] noiseCorrupted_obs_fvs;
	noiseCorrupted_obs_fvs.length = 0;
	// unlike toy mdp, we can not afford noise always occuring for specific s-a pairs
	// so, noise corruption happens a pre-defined percentage of time, for specific s-a pairs
	double chanceNoise = 0.2; 

	foreach(idxs; 0..num_sessions) {
		////////////////////////////////// Demonstration  /////////////////////////////


		for(int i = 0; i < num_trajs; i++) {

			temp_traj = simulate(model, policy, initial, size_traj);
			GT_trajs ~= temp_traj;
			obs_traj.length = 0; 

			foreach(e_sar;temp_traj) { 
				// give specific prob value to each obs feature 
				obs_fv = model.obsFeatures(e_sar.s,e_sar.a);

				// multiple noise corrupted sa pairs with multiple shared features
				auto insertNoise = dice(chanceNoise, 1-chanceNoise);
				if (insertNoise) {
					sortingState ss = cast(sortingState)e_sar.s;
					// Blemished onion has been picked for placing in bin. 
					// It was perceived as unblemished. 
					if ((ss._prediction == 0) && cast(PlaceInBin)e_sar.a) {
						// save to list of unseen combinations of tau values
						if (! noiseCorrupted_obs_fvs.canFind(obs_fv)) noiseCorrupted_obs_fvs ~= obs_fv;

						// introduce faulty input
						ss._prediction = 1;
						obs_fv = model.obsFeatures(ss,e_sar.a); 
					} 
				} 

				obs_traj ~= sac(e_sar.s,e_sar.a,cum_prob_fvs[countUntil(possible_obs_fvs,obs_fv)]);
			}

			obs_trajs ~= obs_traj;
		}

		debug{
			writeln("observed trajectories ");
			int actv;
			foreach(obstraj;obs_trajs) {
				writeln("\n",obstraj);
				foreach(e_sac;obs_traj) {
					actv = 0; 
					foreach (f; model.obsFeatures(e_sac.s,e_sac.a)) {
						if (f==1) actv += 1;
					}
					
				}
			}
			
		}

		////////////////////////////////// Session Starts /////////////////////////////
		double avg_cum_diff1, avg_cum_diff2;

		learnedDistr_obsfeatures = 
		runAvgSessionLearnObsModel(num_trials_perSession, model,
		trueDistr_obsfeatures, noiseCorrupted_obs_fvs, GT_trajs,
		obs_trajs, trueObsMod, numSessionsSoFar,  runAvg_learnedDistr_obsfeatures,
		avg_cum_diff1, avg_cum_diff2, arr_arr_cum_diff1);


		arr_avg_cum_diff1 ~= avg_cum_diff1; 
		arr_avg_cum_diff2 ~= avg_cum_diff2; 
		writeln("arr_avg_cum_diff1 ",arr_avg_cum_diff1);

		///////////////////////////////////////// Session Finished /////////////////////////////////////////////

		// updating global variable for runing average
		runAvg_learnedDistr_obsfeatures[] = (runAvg_learnedDistr_obsfeatures[]*(numSessionsSoFar-1) + learnedDistr_obsfeatures[]);
		runAvg_learnedDistr_obsfeatures[] /= numSessionsSoFar; 

	}

	File file1 = File("/home/saurabharora/Downloads/resultsApproxObsModelMetric1.csv", "a"); 
	string str_arr_avg_cum_diff1 = to!string(arr_avg_cum_diff1);
	str_arr_avg_cum_diff1 = str_arr_avg_cum_diff1[1 .. (str_arr_avg_cum_diff1.length-1)];
	file1.writeln(str_arr_avg_cum_diff1);
	file1.close(); 

	File file2 = File("/home/saurabharora/Downloads/resultsApproxObsModelMetric2.csv", "a"); 
	string str_arr_avg_cum_diff2 = to!string(arr_avg_cum_diff2);
	str_arr_avg_cum_diff2 = str_arr_avg_cum_diff2[1 .. (str_arr_avg_cum_diff2.length-1)];
	file2.writeln(str_arr_avg_cum_diff2);
	file2.close(); 

	writeln(arr_avg_cum_diff1,arr_avg_cum_diff2);
	writeln(runAvg_learnedDistr_obsfeatures);
	
	return 0;
}




class sortingMDPWdObsFeatures : sortingModelbyPSuresh3multipleInit {
	// This class takes has as its member an observation feature function 
	// for estimating observation model

	int numObFeatures;
	// observation model to be estimated 
	double [StateAction][StateAction] obsMod;

	public this(double p_fail, State terminal, int inpNumObFeatures) {
		super(p_fail, terminal);
		this.numObFeatures = inpNumObFeatures;
			
	}

	public override int [] obsFeatures(State state, Action action) {
		/*

		blemish is present on onion 
		onion moves with hand (onion rolling on conveyor doesn't count as moving) 
		onion rotates (either rotating on table for rolling or rotating in hand for inspection)
		onion moves to conveyor 
		orientation of all onions changed 
		onion was on conveyor before action 
		onion moves to bin  
		hand leaves atEye region in 2D image 
		hand moves to bin 
		hand moves to conveyor 
		hand moves over all onions - 0/1

		*/

		int [] result;

		// This is where number of features is decided 
		result.length = 11;
		result[] = 0;

		sortingState ss = cast(sortingState)state;

		if (ss._prediction == 0) result[0] = 1;

		if ((cast(PlaceOnConveyor)action) || (cast(PlaceInBin)action) 
			|| (cast(Pick)action)) result[1] = 1;

		if ((cast(InspectWithoutPicking)action) || (cast(InspectAfterPicking)action))
			result[2] = 1;

		if (cast(PlaceOnConveyor)action) result[3] = 1;

		if ((cast(InspectWithoutPicking)action)) 
			result[4] = 1;

		if (ss._onion_location == 0) 
			result[5] = 1;

		if (cast(PlaceInBin)action) result[6] = 1;

		if ( (ss._EE_location == 1) && 
			( (cast(PlaceOnConveyor)action) || (cast(PlaceInBin)action) 
			|| (cast(Pick)action) ) ) result[7] = 1;

		if (cast(PlaceInBin)action) result[8] = 1;

		if (cast(PlaceOnConveyor)action) result[9] = 1;

		if ((cast(InspectWithoutPicking)action))
			result[10] = 1;

		return result;

	}

	override public void setNumObFeatures(int inpNumObFeatures) {
		this.numObFeatures = inpNumObFeatures;
	}

	public override int getNumObFeatures() {
		return numObFeatures;
	}
		
	public override void setObsMod(double [StateAction][StateAction] newObsMod) {
		obsMod = newObsMod;
	}

}
