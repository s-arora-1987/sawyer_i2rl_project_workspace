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
    writeln("here ts ",ts," ",model.A(ts)[0]);
    auto features = model.obsFeatures(ts,model.A(ts)[0],ts,model.A(ts)[0]);
    writeln("here ");
    //exit(0);

    int numObFeatures = cast(int)(features.length);
    writeln("here ");
    model.setNumObFeatures(numObFeatures);
    writeln("here ");

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
	int totalSA_pairs = 0;
	debug {
		writeln("number of states ",(model.S()).length);
		writeln("number of actions ",(model.A()).length);
		//writeln("created trans function");
		foreach(s;model.S()) {
			foreach(a;model.A(s)){
				//writeln(model.T(s,a));
				//writeln("\n");
				totalSA_pairs +=1;
			}
		}
		writeln("# valid sa pairs ",totalSA_pairs);
	}

	// Generate samples with random prediction scores
	LinearReward reward;
	double [] reward_weights = [0.4,0.6]; 
    reward = new keepTurning(model);
	reward.setParams(reward_weights);

	debug {
		writeln("created reward features ");
		foreach(s;model.S()) {
			foreach(a;model.A(s)){
				double [] fv = reward.features(s,a);
				if (fv[0] == 1 ) {
					//writeln(s,a);
					//writeln("\n");
				}
			}
		}
		//writeln("started policy computation ");
	}
    model.setReward(reward);
    model.setGamma(0.99);

	ValueIteration vi = new ValueIteration();
    double[State] V = vi.solve(model, .5);
	debug {
		//writeln("computed V",V);
	}

	Agent policy = vi.createPolicy(model, V);;
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

	sar [][] GT_trajs;
	sac [][] obs_trajs;
	int num_trajs = 1;
	int size_traj = 3;
	double [] noise_range = [0.8,0.99];
	sar [] temp_traj;
	sac [] obs_traj;
	
	// collect all possible values of arrays output of features function

	// Option 1 for testing efficacy of approximation: Create random distr over features. assume multiplicative model for computing 
	// the cumulative probs for all possible combinations of feature values

	double [] trueDistr_obsfeatures = new double[2*model.getNumObFeatures()];
	double totalmass_untilNw = 0.0;
	double currp;
	foreach (i; 0 .. model.getNumObFeatures()) {
		currp = uniform(0.0,1);
		trueDistr_obsfeatures[i] = currp;
		trueDistr_obsfeatures[i+model.getNumObFeatures()] = 1-currp;
	} 
	//trueDistr_obsfeatures[model.getNumObFeatures()-1] = 1-totalmass_untilNw;
	debug { 
		writeln("true distr obsfeatures ",trueDistr_obsfeatures); 
		//exit(0);
	}

	// Option 2: HARDCODE the cumulative probs for all possible combinations of feature values
	//cum_prob_fvs = [0.1,0.3,0.6];

	double [] cum_prob_fvs;
	int [][] possible_obs_fvs;
	int [] obs_fv;
	double prod;
	//int idx_fv = 0;
	foreach(s;model.S()) {
		foreach(a;model.A(s)){
			foreach(os;model.S()) {
				foreach(oa;model.A(os)){

					obs_fv = model.obsFeatures(s, a, os, oa);
					if (! possible_obs_fvs.canFind(obs_fv)) {
						possible_obs_fvs ~= obs_fv;
						prod = 1.0;
						foreach(i,f;obs_fv) {
							//if (f==1) prod *= trueDistr_obsfeatures[i];
							//else prod *= trueDistr_obsfeatures[i+model.getNumObFeatures()];
							if (f==0) prod *= trueDistr_obsfeatures[i+model.getNumObFeatures()];
						}
						cum_prob_fvs ~= prod;
					}

				}
			} 
		}
	}

	double[StateAction][StateAction] trueObsMod;
	trueObsMod = createObsModel(model, trueDistr_obsfeatures); 

	double [] learnedDistr_obsfeatures = new double[2*model.getNumObFeatures()]; 
	// learned distribution incrementally averaged over sessions
	double [] runAvg_learnedDistr_obsfeatures = new double[2*model.getNumObFeatures()]; 
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

	foreach(idxs; 0..num_sessions) {
		////////////////////////////////// Demonstration  /////////////////////////////


		for(int i = 0; i < num_trajs; i++) {

			temp_traj = simulate(model, policy, initial, size_traj);
			GT_trajs ~= temp_traj;
			obs_traj.length = 0; 

			foreach(e_sar;temp_traj) {

				// give specific prob value to each obs feature 
				obs_fv = model.obsFeatures(e_sar.s,e_sar.a,e_sar.s,e_sar.a);

				//// add meaningless noise: replace moveforward with turning ////

				// single corrupted sa pair with one shared feature
				//if (cast(MoveForwardAction)e_sar.a && (cast(BoydState)e_sar.s).getLocation()[1]==0) {
				
				// single corrupted sa pair with two shared features
				//if (cast(MoveForwardAction)e_sar.a && (cast(BoydState)e_sar.s).getLocation()[1]==0
				
				// multiple corrupted sa pairs with two shared features
				//	&& (cast(BoydState)e_sar.s).getLocation()[0]==0) {
				if ( cast(MoveForwardAction)e_sar.a &&  ( (cast(BoydState)e_sar.s).getLocation()[1]==0
					|| (cast(BoydState)e_sar.s).getLocation()[0]==0) ) {
					// save to list of unseen combinations of tau values
					if (! noiseCorrupted_obs_fvs.canFind(obs_fv)) noiseCorrupted_obs_fvs ~= obs_fv;

					// introduce faulty input
					e_sar.a = new TurnLeftAction();
					obs_fv = model.obsFeatures(e_sar.s,e_sar.a,e_sar.s,new TurnLeftAction());
				}

				obs_traj ~= sac(e_sar.s,e_sar.a,cum_prob_fvs[countUntil(possible_obs_fvs,obs_fv)]);
			}
			obs_trajs ~= obs_traj;
		}

		debug{
			writeln("created observed  trajectories ");
			//exit(0);
		}

		////////////////////////////////// Session Starts /////////////////////////////
		double avg_cum_diff1, avg_cum_diff2;
		writeln("numSessionsSoFar ",numSessionsSoFar);
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

	writeln("BEGPARSING");


	writeln("ENDPARSING");
	
	return 0;
}

public double [] runAvgSessionLearnObsModel(int num_trials_perSession, Model model, 
	double [] trueDistr_obsfeatures, int  [][] noiseCorrupted_obs_fvs, sar [][] GT_trajs,
	sac [][] obs_trajs, double[StateAction][StateAction] trueObsMod,
	ref double numSessionsSoFar, ref double [] runAvg_learnedDistr_obsfeatures, 
	ref double avg_cum_diff1, ref double avg_cum_diff2, ref double [] [] arr_arr_cum_diff1) {
		//////// For average over 10 runs with same trueObsFeatDistr and observations /////// 

	
	writeln("numSessionsSoFar ",numSessionsSoFar);
	double [] arr_cum_diff1, arr_cum_diff2;
	// array of outputs from trials within same session
	double [] learnedDistr_obsfeatures;
	double [][] arr_learnedDistr_obsfeatures;
	arr_learnedDistr_obsfeatures.length = 0;
	arr_cum_diff1.length = 0;
	arr_cum_diff2.length = 0;
	numSessionsSoFar += 1;  

	foreach(tr; 0..num_trials_perSession) {

		auto estimateObsMod = new MaxEntUnknownObsMod(100,new ValueIteration(), 2000, .00001, .1, .1);
		double opt_val_Obj;
		learnedDistr_obsfeatures = estimateObsMod.solve2(model, obs_trajs, opt_val_Obj);

		foreach (i; 0..(learnedDistr_obsfeatures.length/2)) {
			// separately for each tau and taubar
			learnedDistr_obsfeatures[i] = learnedDistr_obsfeatures[i]/(learnedDistr_obsfeatures[i]
				+learnedDistr_obsfeatures[i+model.getNumObFeatures()]);
			learnedDistr_obsfeatures[i+model.getNumObFeatures()] = learnedDistr_obsfeatures[i]/(learnedDistr_obsfeatures[i]
				+learnedDistr_obsfeatures[i+model.getNumObFeatures()]);
		}
		writeln("learnedDistr_obsfeatures ",learnedDistr_obsfeatures);
		//exit(0);

		// update the incrementally learned feature distribution 
		// local substitute variable for incremental runing average
		double [] temp_runAvg_learnedDistr_obsfeatures = new double[learnedDistr_obsfeatures.length];
		//writeln("runAvg_learnedDistr_obsfeatures ",runAvg_learnedDistr_obsfeatures);
		//writeln("numSessionsSoFar ",numSessionsSoFar);

		temp_runAvg_learnedDistr_obsfeatures[] = (runAvg_learnedDistr_obsfeatures[]*(numSessionsSoFar-1) + learnedDistr_obsfeatures[]);
		temp_runAvg_learnedDistr_obsfeatures[] /= numSessionsSoFar; 

		// for averaging over trials within session
		arr_learnedDistr_obsfeatures ~= temp_runAvg_learnedDistr_obsfeatures;

		// verify if this step is needed
		//temp_runAvg_learnedDistr_obsfeatures = learnedDistr_obsfeatures;

		writeln("true distr obsfeatures ",trueDistr_obsfeatures); 
		//writeln("cum_prob_fvs ",cum_prob_fvs); 
		writeln(); 
		writeln("temp_runAvg_learnedDistr_obsfeatures: ", temp_runAvg_learnedDistr_obsfeatures); 
		writeln(); 
		double[StateAction][StateAction] learnedObsMod;
		learnedObsMod = createObsModel(model, temp_runAvg_learnedDistr_obsfeatures); 

		// Metric 1a: estimation of the observation likelihood of s-a pairs misidentified due to noise 
		// They happened in input but not in output of perception pipeline 
		double diffprod, cum_diff1, cum_diff2, cum_diff1a, prod;
		writeln("Divide by 0 check: number of s-a pairs with noise in current session ",cast(double)noiseCorrupted_obs_fvs.length);
		cum_diff1a = 0.0;
		foreach (obs_fv2; noiseCorrupted_obs_fvs) {

			prod = 1.0;
			foreach(i,f;obs_fv2) {
				if (f==1) prod *= trueDistr_obsfeatures[i];
				else prod *= trueDistr_obsfeatures[i+model.getNumObFeatures()];
			} 
			diffprod = prod;

			prod = 1.0;
			foreach(i,f;obs_fv2) {
				if (f==1) prod *= temp_runAvg_learnedDistr_obsfeatures[i];
				else prod *= temp_runAvg_learnedDistr_obsfeatures[i+model.getNumObFeatures()];
			}
			diffprod -= prod;
			//writeln("diff prod for unseen combination ",obs_fv2,": ",diffprod);
			cum_diff1a += diffprod;
		}
		if (cast(double)noiseCorrupted_obs_fvs.length == 0) {
			cum_diff1a = -double.max;
		} else {
			cum_diff1a= cum_diff1a/cast(double)noiseCorrupted_obs_fvs.length;				
		}
		//arr_cum_diff1 ~= cum_diff1a;

		// Metric 1b: 
		// Cumulative diff for P(obs-sa | GT-sa) distributions for only those GT-sa pairs that got corrupted in input to perception pipeline
		StateAction[] corrupted_GT_SAs;
		corrupted_GT_SAs.length = 0;
		bool corrupted;
		foreach(i,GT_traj;GT_trajs) {
			sac[] ob_traj = obs_trajs[i];
			foreach(j,e_sar;GT_traj) {
				
				sac e_sac = ob_traj[j];
				if ((e_sar.s != e_sac.s) || (e_sar.a != e_sac.a)) {

					StateAction corrupted_sa = new StateAction(e_sar.s,e_sar.a);
					// not demonstrated
					if (! corrupted_GT_SAs.canFind(corrupted_sa)) corrupted_GT_SAs ~= corrupted_sa;
					//writeln("corrupted_GT_SAs ",corrupted_GT_SAs);
				}
			}
		}
		cum_diff1 = 0.0;
		foreach(sa;corrupted_GT_SAs){
			cum_diff1 += normedDiff_SA_Distr(trueObsMod[sa], learnedObsMod[sa]);
		}
		if (cast(double)corrupted_GT_SAs.length == 0) {
			cum_diff1 = -double.max;
		} else {
			cum_diff1 = cum_diff1/cast(double)corrupted_GT_SAs.length;
		}
		arr_cum_diff1 ~= cum_diff1;
		writeln("cumulative diff for observation likelihood of s-a pairs misidentified due to noise ",cum_diff1);
		//exit(0);

		// Metric 2: estimation of the observation likelihood of s-a pairs never present in input of perception pipeline 
		// Cumulative diff for P(obs-sa | GT-sa) distributions for GT-sa pairs that never occured in input to perception pipeline
		StateAction[] unseen_GT_SAs;
		unseen_GT_SAs.length = 0;
		bool inDemonsInpNeuralNet;
		foreach(s;model.S()) {
			foreach(a;model.A(s)){
				inDemonsInpNeuralNet = false;
				foreach(GT_traj;GT_trajs) {
					foreach(e_sar;GT_traj) {

						if (e_sar.s.opEquals(s) && e_sar.a.opEquals(a)) {
							inDemonsInpNeuralNet = true;
						} 
					}
				}

				if (inDemonsInpNeuralNet == false) {
					StateAction curr_gt_sa = new StateAction(s,a);
					//writeln("e_sar",e_sar);
					//writeln("s:",s," a:",a);
					//writeln("unseen_GT_SAs ",unseen_GT_SAs);
					// not demonstrated
					if (! unseen_GT_SAs.canFind(curr_gt_sa)) unseen_GT_SAs ~= curr_gt_sa;
				}
			}
		} 
		cum_diff2 = 0.0;
		foreach(sa;unseen_GT_SAs){
			cum_diff2 += normedDiff_SA_Distr(trueObsMod[sa], learnedObsMod[sa]);
		}
		if (cast(double)unseen_GT_SAs.length == 0) {
			cum_diff2 = -double.max;
		} else {
			cum_diff2 = cum_diff2/cast(double)unseen_GT_SAs.length;
		}
		
		//writeln("total sa pairs with noise ",cast(double)noiseCorrupted_obs_fvs.length);
		writeln("Divide by 0 check: number of sa pairs:",trueObsMod.length,"\n number of unseen GT sa pairs:",unseen_GT_SAs.length);
		writeln("cumulative diff for P(obs-sa | GT-sa) distributions for GT-sa pairs that never occured in input to perception pipeline ",cum_diff2);
		arr_cum_diff2 ~= cum_diff2;

	}

	avg_cum_diff1 = (sum(arr_cum_diff1)+0.000001)/cast(double)num_trials_perSession;
	avg_cum_diff2 = (sum(arr_cum_diff2)+0.000001)/cast(double)num_trials_perSession;

	//writeln(arr_cum_diff1,arr_cum_diff2);
	writeln("average cum_diff1 for ",num_trials_perSession," trials of this session ",avg_cum_diff1);
	writeln("average cum_diff2 for ",num_trials_perSession," trials of this session ",avg_cum_diff2);
	writeln("numSessionsSoFar ",numSessionsSoFar);


	// average learned distribution from trials within session 
	foreach(i; 0 .. arr_learnedDistr_obsfeatures[0].length ) {
		learnedDistr_obsfeatures[i] = sum(arr_learnedDistr_obsfeatures.transversal(i))/cast(double)(arr_learnedDistr_obsfeatures.length);
	}
	// only for debugging 
	debug {
		arr_arr_cum_diff1 ~= arr_cum_diff1;
	}

	return learnedDistr_obsfeatures;
}

public double[StateAction][StateAction] createObsModel(Model model, double [] featureWeights) {

	double p_success, tot_mass_obsSA_wd_sharedActvFeat, temp_total_mass;

	int [] features;
	double[StateAction][StateAction] returnedObModel;
	double[StateAction] tempdict_gt_sa;
	int totalSA_pairs, tot_obsSA_wo_sharedActvFeat;

	totalSA_pairs = 0;
	foreach(s; model.S()) { // for each state
		foreach(a; model.A(s)){
			totalSA_pairs += 1;
		}
	}
	int total_keys_obsMod = 0;

	foreach(s; model.S()) { // for each state
		foreach(a; model.A(s)){
			StateAction curr_gt_sa = new StateAction(s,a);

			tot_mass_obsSA_wd_sharedActvFeat = 0.0;
			tot_obsSA_wo_sharedActvFeat = 0;

			foreach(obs_s; model.S()) { // for each obs state
				foreach(obs_a; model.A(s)){ 
					// use p_success of the features shared among GT and obs  
					

					p_success = 1.0;
					features = model.obsFeatures(s,a,obs_s,obs_a);

					// P(obs s, obs a | s, a) = 
					foreach(i,f;features) {
						//if (f == 1) p_success *= featureWeights[i];
						//else p_success *= featureWeights[i+model.getNumObFeatures()];
						if (f==0) p_success *= featureWeights[i+model.getNumObFeatures()];
					} 
					
					// if no features activated (p_success == 1.0) 
					// or features could not capture the physical description in that s-a 
					if (p_success == 1.0 || p_success == 0) tot_obsSA_wo_sharedActvFeat += 1;
					else  tot_mass_obsSA_wd_sharedActvFeat += p_success;

					if (p_success < 0) p_success = 0;
					if (p_success > 1) p_success = 1;

					returnedObModel[curr_gt_sa][new StateAction(obs_s,obs_a)] = p_success;
				}
			}
			
			if (tot_obsSA_wo_sharedActvFeat == totalSA_pairs) {
				// if none of the features were activated, then features could not capture this s-a pair 
				// Assign uniform distribution and tot_obsSA_wo_sharedActvFeat = 0
				foreach(obs_s; model.S()) { // for each obs state
					foreach(obs_a; model.A(s)){ // for each obs action
						returnedObModel[curr_gt_sa][new StateAction(obs_s,obs_a)] = 1.0/cast(double)totalSA_pairs;
						
					}
				}		
				total_keys_obsMod += 1;
				debug {
					// writeln("features can't capture this s-a pair ");
				}
			} else {
				if (tot_obsSA_wo_sharedActvFeat > 0 ) {
					// those observations current set of features couldn't capture 
					// distribute the remaining mass  
					foreach(obs_s; model.S()) { // for each obs state
						foreach(obs_a; model.A(s)){ // for each obs action
							p_success = returnedObModel[curr_gt_sa][new StateAction(obs_s,obs_a)];
							if (p_success == 1.0 || p_success == 0) {
								returnedObModel[curr_gt_sa][new StateAction(obs_s,obs_a)] = 
								(1.0-tot_mass_obsSA_wd_sharedActvFeat)/cast(double)(tot_obsSA_wo_sharedActvFeat); 
							} 
						} 
					}
					total_keys_obsMod += 1;
				} else { } //writeln("createObsModel: tot_obsSA_wo_sharedActvFeat == 0 case"); 
			}

			Distr!StateAction.normalize(returnedObModel[curr_gt_sa]);

			debug {
				// writeln (returnedObModel[new StateAction(s,a)]); 
				// test if you can sample from the distribution 
				tempdict_gt_sa = returnedObModel[curr_gt_sa];
				StateAction sampled_obs_sa = Distr!StateAction.sample(tempdict_gt_sa);
				// writeln("test sampling observation ",sampled_obs_sa);
			}
		}
	}

	return returnedObModel;

} 

double normedDiff_SA_Distr (double[StateAction] distr1, double[StateAction] distr2) {

	double[] sorted_arr1, sorted_arr2;

	foreach (sa1,v1;distr1) {
		foreach (sa2,v2;distr2) {
			if (sa2 == sa1) {
				sorted_arr1 ~= v1;
				sorted_arr2 ~= v2;
			}
		}
	}

	double [] diff;
	diff.length = sorted_arr1.length;
	diff[] = sorted_arr1[] - sorted_arr2[];
	return l1norm(diff)/l1norm(sorted_arr1);

}

string feature_vector_to_string(int [] features) {
	string returnval = "";
	foreach (f; features) {
		returnval ~= f == 0 ? "0" : "1";
	}
	return returnval;
}



int[][] define_events_obsMod(Model model, sac[][] samples, out int[string] eventMapping) {
	// vector of all the outputs of feature function, that got instantiated in samples
	// <s,a,c> c = score
	
	int[][] returnval;
	int[string] eventIds;
	

	foreach(sample; samples) {

		foreach(e_sac; sample) {

			foreach(s; model.S()) { // for each state action pair
				foreach(a; model.A(s)){

					auto features = model.obsFeatures(s,a,e_sac.s,e_sac.a);

					auto eID = feature_vector_to_string(features);

					if (! ( eID in eventIds)) {
						eventIds[eID] = cast(int)returnval.length;
						
						int[] feature_indices;
						foreach(i,f;features) {
							if (f==0) feature_indices ~= cast(int)i;
						} 
						// returnval [eventIds[eID]]
						returnval ~= feature_indices;
						debug {
							writeln("define events: eID  = ",eID," features=",features);
							writeln("returnval ",returnval);
						}
					}

				}
			}
		}
	}
	//exit(0);
	eventMapping = eventIds;
	
	return returnval;
}

double [] calc_log_success_rate_obsMod(sac[][] samples, int[string] eventMapping, Model model) {

	double [] returnval = new double[eventMapping.length];
	returnval[] = 0;
	
	int[string] totalCount;
	double[string] cumulativeScore;
	debug {
		//writeln("calc_log_success_rate: E len");
	}
	
	foreach(i,sample; samples) {

		foreach(e_sac; sample) {

			foreach(s; model.S()) { // for each state action pair
				foreach(a; model.A(s)){

					// P(obs s, obs a | s, a)

					auto features = model.obsFeatures(s,a,e_sac.s,e_sac.a);

					auto eID = feature_vector_to_string(features);

					totalCount[eID] += 1;
					cumulativeScore[eID] += e_sac.c;

				}
			}

		}
	
	}
	
	foreach (ID, num; eventMapping) {
		if (ID in cumulativeScore && ID in totalCount) {
			debug {
				writeln("ID in cumulativeScore ",cumulativeScore[ID]," && ID in totalCount ",totalCount[ID]);
				//exit(0);
			}
			returnval[num] = log(cast(double)(cumulativeScore[ID] ) / cast(double)(totalCount[ID] ) );
		}
		else {
			returnval[num] = 0;
		}
	}
	//exit(0);

	return returnval;
}


int [][] coarsest_partition(int[][] E) {

	int[][] S;
	int[][] Edup = E.dup;
	writeln("coarsest_partition ",Edup);
	// union of all members of E, i.e. all feature vectors that got instantiated in samples
	int [] trivial = set_type!(int).to_set(uniq(nWayUnion(Edup)));
	S ~= trivial;
	// writeln("coarsest_partition trivial S ",S);
	
	int [] empty_set;
	empty_set.length = 0;
	
	foreach (j, Ej; E) {
		foreach(i, Si; S) {
			auto intersection = setIntersection(Ej, Si);

			
			if (equal(intersection, empty_set) || equal(intersection, Si))
				continue; // Si is either a proper subset of Ej, or else has nothing in common
		
			
			S ~= set_type!(int).to_set(intersection);
			
			S[i] = set_type!(int).to_set(setDifference(Si, intersection));

		}	
		//writeln("S: ",S);

	}
	//exit(0);
	
	return S;
}

template set_type(Q) {
	Q [] to_set(R)(R si) if (isInputRange!(R) && is(ElementType!(R) : Q) ) {
		
		Q [] returnval;
		foreach(s; si) {
			returnval ~= s;
		}
		return returnval;
	}
}

double [] map_p_onto_features(double [] p, int [][] S, int num_obsfeatures) {

	double [] returnval = new double[2*num_obsfeatures];
	returnval[] = 1;
	    	
	foreach(i, si; S) {
		foreach(subsi; si) {
			// tau
			returnval[subsi] = pow(p[i+S.length], 1.0 / S[i].length); 
			//writeln("i:",i,",p[i]: ",p[i],",S[i].length:",S[i].length,",returnval[subsi]:",returnval[subsi]);
			// taubar
			returnval[subsi+num_obsfeatures] = pow(p[i], 1.0 / S[i].length); 
		}
	}
	
	return returnval;

}

class MaxEntUnknownObsMod : MaxEntIrl {

	private int[][] E; // E is a set of subsets, each subset contains the number of a feature
	private double[] Q; // Q is the success rate of each E
	private int[][] S; // the `coursest partition of the feature space induced by the Es
	private int[][] P; // M x N matrix, each row contains the vector of v's corresponding to a given P
	    	
    private double [] v;
    private double [] lambda;
    private sac[][] obsTraj_samples;
   
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	}

	
	public double [] solve2(Model model, sac[][] obsTraj_samples, out double opt_value) {
		
        // Compute feature expectations of agent = mu_E from samples
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.max_iterations = max_iter;
        param.epsilon = error;
        //param.min_step = 0.1;
        
        this.model = model;
        this.obsTraj_samples = obsTraj_samples;
        this.sample_length = cast(int)obsTraj_samples.length;
        
        int[string] eventMapping;
        E = define_events_obsMod(model, obsTraj_samples, eventMapping); 
        debug {
        	writeln("E: ", E);
        }
        
        Q = calc_log_success_rate_obsMod(obsTraj_samples, eventMapping, model);
        foreach (ref q; Q) {
        	q = exp(q);
        }
        
        debug {
        	writeln("Q: ", Q);
        }
        
        S = coarsest_partition(E);
        
        debug {
        	writeln("partitions S: ", S);
        }	
        
        // we need a lagrangian multiplier for each event, call it v
    	v = new double[E.length];
    	v[] = .01;
    	lambda = new double[S.length];
    	lambda[] = .0001;
    	
    	
        double * p = lbfgs_malloc(2*cast(int)S.length);
        scope(exit) {
        	lbfgs_free(p);
        } 
        
        foreach (i; 0..(2*S.length)) {
        	p[i] = uniform(0.05, .95);
	        //write(p[i]," "); 
        }

        //writeln("p ",to!string(*p));
        //exit(0);
        
        double finalValue;	
        double [] weights;	
        weights.length = 2*S.length;
        int ret;	
        foreach (i; 0..30) {

        	auto temp =this;
        	ret = lbfgs(cast(int)(2*S.length), p, &finalValue, &evaluate_maxent, &progress, &temp, &param);
	        foreach(j; 0..(2*S.length)) {
	        	weights[j] = p[j];
	        }
	        
	        debug {
	        	writeln("\n Penalty Method iteration: ", i, " - Weights: ", weights);
	        	writeln();
	        }
        	
        	v[] *= 2;
        	lambda[] *= 2;
        }
        debug {
        	writeln("LBFGS Result: ", ret);
        }	

        debug {
        	writeln("\nE: ", E,"\nQ: ",Q,"\nS: ",S,"\n learned p ",weights);
        }
        
        opt_value = finalValue;
                
     
        return map_p_onto_features(weights, S, model.getNumObFeatures());
	}


	override double evaluate(double [] p, out double [] g, double step) {
    	
    	double returnval = 0;
    	
    	foreach(pi; p) {
    		returnval += pi * log(pi);
    		if (pi <= 0)
    			returnval = double.infinity;
    	}
    	
    	
    	foreach(j,Ej; E) {
    		double mult = 1.0;
    		foreach(i,Si; S) {
    			auto intersection = setIntersection(Si, Ej);
    			
    			if (equal(intersection, Si)) {
    				mult *= p[i];
    			}
    		}
    		mult -= Q[j];
    		mult = mult*mult;
    		mult *= v[j] / 2;
    		
    		returnval += mult;
    	}	
    	
    	foreach(i,Si; S) {
    		returnval += (lambda[i] / 2) * (p[i] + p[i + S.length] - 1)*(p[i] + p[i + S.length] - 1);
    	
    	}
    	
    	
    	g.length = 2*S.length;
    	g[] = 0;
    	

    	foreach(i,Si; S) {
    		if (p[i] <= 0) {
    			g[i] = - double.infinity;
    			continue;
    		}
    		g[i] = log(p[i]) + 1;
    		foreach(j,Ej; E) {
	    		double mult = v[j];
	    		
	    		foreach (k, Sk; S) {
	    			if (k != i) {
		    			auto intersection = setIntersection(Sk, Ej);
		    			
		    			if (equal(intersection, Sk)) {
		    				mult *= p[k];
		    			}
		    		} 
    			} 
	    		double mult2 = 1.0;
	    		foreach(l,Sl; S) {
	    			auto intersection2 = setIntersection(Sl, Ej);
	    			
	    			if (equal(intersection2, Sl)) {
	    				mult2 *= p[l];
	    			}
	    		}
	    		mult2 -= Q[j];
	    		mult *= mult2;
	    		
    			g[i] += mult;
    		}
    		g[i] += lambda[i] * (p[i] + p[i + S.length] - 1);
    	}
    	
    	
    	foreach(i,Si; S) {
    		if (p[i + S.length] <= 0 ) {
    			g[i + S.length] = -double.infinity;
    			continue;
    		}    	
    		g[i + S.length] = log(p[i + S.length]) + 1 + lambda[i] * (p[i] + p[i + S.length] - 1);
    	} 	
    	return returnval;
    	
	}


}