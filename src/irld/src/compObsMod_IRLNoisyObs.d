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

import irl;
import std.datetime;

int main() {
	debug {
		writeln("here");
	}
	sac [][] SAC;
    sac [][] SACfull;
	LinearReward reward;
	Model model;
	double chanceNoise = 0.0;

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
	double[] trueWeights = reward_weights; 

    model.setReward(reward);
	reward.setParams(reward_weights);
    model.setGamma(0.99);

	double VI_threshold = 0.2; 
	int vi_duration_thresh_secs = 30; 
	TimedValueIteration vi = new TimedValueIteration(int.max,false,vi_duration_thresh_secs); 
	Agent policy; 
    double[State] V; 
    V = vi.solve(model, VI_threshold); 
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

	debug {
		writeln("starting "); 
	    auto starttime = Clock.currTime();
	} 

	int num_trajs = 1;
	int size_traj = 3;
	bool useHierDistr = 1;	
	// decide what values of features will help lbfgs dsitinguish events
	bool lbfgs_use_ones = 0;
	int num_sessions = 10;
	int num_trials_perSession = 1;
	bool allGTdata = 0;
	
	sar [][] GT_trajs;
	sac [][] obs_trajs;
	sar [] temp_traj;
	sac [] obs_traj;
	double [] trueDistr_obsfeatures;
	if (useHierDistr == 1)trueDistr_obsfeatures	 = new double[2*model.getNumObFeatures()];
	else trueDistr_obsfeatures = new double[model.getNumObFeatures()];

	double currp;
	double totalmass_untilNw = 0.0;
	foreach (i; 0 .. model.getNumObFeatures()) {
		if (useHierDistr == 1) currp = uniform(0.0,1);
		else {
			currp = uniform(0.0,1-totalmass_untilNw);
			totalmass_untilNw += currp;
		}
		trueDistr_obsfeatures[i] = currp;
		if (useHierDistr == 1) trueDistr_obsfeatures[i+model.getNumObFeatures()] = 1-currp;
	} 
	if (useHierDistr == 0) {
		trueDistr_obsfeatures[model.getNumObFeatures()-1] += 1-totalmass_untilNw;
	}

	double [] cum_prob_fvs; 
	int [][] possible_obs_fvs; 
	int [] obs_fv; 
	double prod; 
	foreach(s;model.S()) {
		foreach(a;model.A(s)){
			foreach(os;model.S()) {
				foreach(oa;model.A(os)){

					obs_fv = model.obsFeatures(s, a, os, oa);
					if (! possible_obs_fvs.canFind(obs_fv)) {
						possible_obs_fvs ~= obs_fv;
						prod = 1.0;
						foreach(i,f;obs_fv) { 
							if (lbfgs_use_ones == 1) { 
								if (f==1) prod *= trueDistr_obsfeatures[i];
							} else { 
								if (f==0) { 
									if (useHierDistr == 1) prod *= trueDistr_obsfeatures[i+model.getNumObFeatures()];
									else prod *= 1-trueDistr_obsfeatures[i];
								}
							}
						}
						cum_prob_fvs ~= prod;
					}

				}
			} 
		}
	}

	double[StateAction][StateAction] trueObsMod;
	trueObsMod = createObsModel(model, trueDistr_obsfeatures, lbfgs_use_ones, useHierDistr); 
	double [] learnedDistr_obsfeatures;
	if (useHierDistr == 1) learnedDistr_obsfeatures = new double[2*model.getNumObFeatures()]; 
	else learnedDistr_obsfeatures = new double[model.getNumObFeatures()]; 

	// learned distribution incrementally averaged over sessions 
	double [] runAvg_learnedDistr_obsfeatures;
	if (useHierDistr == 1) runAvg_learnedDistr_obsfeatures = new double[2*model.getNumObFeatures()]; 
	else runAvg_learnedDistr_obsfeatures = new double[model.getNumObFeatures()]; 
	runAvg_learnedDistr_obsfeatures[] = 0.0;
	double numSessionsSoFar = 0.0;
	MaxEntUnknownObsMod estimateObsMod = new MaxEntUnknownObsMod(100,new ValueIteration(), 2000, .00001, .1, .1);


    int num_Trajsofar = 0;
	int length_subtrajectory = 40;
	double conv_threshold_stddev_diff_moving_wdw;
	int use_ImpSampling = 0;
	if (use_ImpSampling == 1){
		// imp sampling
		conv_threshold_stddev_diff_moving_wdw = 0.0005; 
	} else {
		// gibbs sampling
		conv_threshold_stddev_diff_moving_wdw = 0.005; 
	}
	int restart_attempts = 1;
	int moving_window_length_muE = 3;
	double conv_threshold_gibbs = 0.015;
	double grad_descent_threshold = 0.0000001; // Not being used in current descent method 
	double gradient_descent_step_size = 0.00001;
	int descent_duration_thresh_secs = 3*60;
	double Ephi_thresh = 0.1;
	int nSamplesTrajSpace = 100;

	double LBA;
	double [] arr_LBA, arr_EVD; 
	arr_LBA.length = 0;
	arr_EVD.length = 0;
    double [] lastWeights;
    double [] featureExpecExpert;
    double [] featureExpecExpertfull;
    double [] foundWeightsGlbl;
    lastWeights.length = reward.dim();
    lastWeights[] = 0.0;
    featureExpecExpertfull.length = reward.dim();
    featureExpecExpertfull[] = 0.0;
    featureExpecExpert.length = reward.dim();
    featureExpecExpert[] = 0.0;
    foundWeightsGlbl.length = reward.dim();

	double diff_wrt_muE_wo_sc;
	double diff_wrt_muE_scores1;
	double last_val;
	size_t max_sample_length = length_subtrajectory;

	MaxEntIrlZiebartApproxNoisyObs irl = new MaxEntIrlZiebartApproxNoisyObs(restart_attempts, 
		new TimedValueIteration(int.max, false, vi_duration_thresh_secs), model.S(), 
		nSamplesTrajSpace, grad_descent_threshold, VI_threshold);

	foreach(idxs; 0..num_sessions) {
		////////////////////////////////// Demonstration  /////////////////////////////
		for(int i = 0; i < num_trajs; i++) {

			if (allGTdata==0) {
				temp_traj = simulate(model, policy, initial, size_traj);
				GT_trajs ~= temp_traj;
			} else {
				temp_traj.length = 0;
			}

			obs_traj.length = 0; 

			foreach(e_sar;temp_traj) {

				// give specific prob value to each obs feature 
				obs_fv = model.obsFeatures(e_sar.s,e_sar.a,e_sar.s,e_sar.a);

				//// add meaningless noise: ////
				StateAction newsa = model.noiseIntroduction(e_sar.s,e_sar.a);
				if ((newsa.s!=e_sar.s) || (newsa.a!=e_sar.a)) {
					obs_fv = model.obsFeatures(e_sar.s,e_sar.a,newsa.s,newsa.a);
				}

				obs_traj ~= sac(newsa.s,newsa.a,cum_prob_fvs[countUntil(possible_obs_fvs,obs_fv)]);
			}
			
			if (allGTdata==1) {
				foreach(s;model.S()){
					foreach(a;model.A(s)){
						temp_traj ~= sar(s,a,1.0);
						obs_traj ~= sac(s,a,1.0); 
					}
				}
				GT_trajs ~= temp_traj;
			} 

			obs_trajs ~= obs_traj;
		}

		debug{
			writeln("created observed  trajectories ");
			//exit(0);
		}
		double avg_cum_diff1, avg_cum_diff2, avg_cum_diff3;
		writeln("numSessionsSoFar ",numSessionsSoFar);
		learnedDistr_obsfeatures = 
		avg_singleSession_obsModelLearning(estimateObsMod, num_trials_perSession, model,
		trueDistr_obsfeatures, GT_trajs,
		obs_trajs, trueObsMod, numSessionsSoFar,  runAvg_learnedDistr_obsfeatures,
		avg_cum_diff1, avg_cum_diff2, lbfgs_use_ones, 
		all_sa_pairs, avg_cum_diff3, useHierDistr);

		// updating global variable for runing average
		runAvg_learnedDistr_obsfeatures[] = (runAvg_learnedDistr_obsfeatures[]*(numSessionsSoFar-1) + learnedDistr_obsfeatures[]);
		runAvg_learnedDistr_obsfeatures[] /= numSessionsSoFar; 
		double[StateAction][StateAction] obsModel = createObsModel(model, runAvg_learnedDistr_obsfeatures, lbfgs_use_ones, useHierDistr); 
		model.setObsMod(obsModel);	

		//////////////////// IRL under noisy Obs /////////////////////////////////////

		writeln("calling MaxEntIrlZiebartApproxNoisyObs.solve");
		debug {

			//writeln("verify if any s-a pair is not present in keys of obs model associative array.");
			foreach(s;model.S()) {
				foreach(a;model.A(s)) {
					StateAction gtsa = new StateAction(s,a);
					auto mem = (gtsa in obsModel);
					if (mem is null) {
						//writeln("s-a ",gtsa," not present ");
					} else {
						foreach(os;model.S()) {
							foreach(oa;model.A(os)){
								StateAction obsa = new StateAction(os,oa);
								auto mem2 = (obsa in obsModel[gtsa]);
							//	if (mem2 is null) writeln("s-a ",obsa," not present returnedObModel[gtsa] ");
							//	if ((os==new sortingState([0, 2, 3, 0 ])) && 
							//		(oa==new ClaimNewOnion())) 
							//		writeln(" [0, 2, 3, 0 ] - ClaimNewOnion found for key ",gtsa);
							}
						}
					}
				}
			}

		}
		//exit(0);

		policy = irl.solve(model, initial, obs_trajs, max_sample_length, 
			lastWeights, last_val, foundWeightsGlbl, featureExpecExpert, num_Trajsofar, 
			Ephi_thresh, gradient_descent_step_size, descent_duration_thresh_secs,
			trueWeights, conv_threshold_stddev_diff_moving_wdw, 
			moving_window_length_muE, use_ImpSampling, conv_threshold_gibbs,
			diff_wrt_muE_wo_sc, diff_wrt_muE_scores1); 

		policy = vi.createPolicy(model,vi.solve(model, 0.1));

	    reward.setParams(trueWeights); 
		Agent truePolicy = vi.createPolicy(model,vi.solve(model, 0.1)); 
	    reward.setParams(foundWeightsGlbl);	
		Agent learnedPolicy = vi.createPolicy(model,vi.solve(model, 0.1)); 
		LBA = irl.computeLBA(cast(MapAgent)learnedPolicy,cast(MapAgent)truePolicy); 
		arr_LBA ~= LBA; 

	    writeln("\n LBA",LBA,"ENDLBA"); 
	    writeln("\n DIFF1",diff_wrt_muE_wo_sc,"ENDDIFF1"); 
	    writeln("\n DIFF2",diff_wrt_muE_scores1,"ENDDIFF2"); 

		sar [] trajs;
		for(int i = 0; i < 2; i++) {
			trajs = simulate(model, policy, initial, 50);
		}

        //Compute average EVD
        double EVD = 0.0;
	    double trajval_trueweight, trajval_learnedweight;
	    double [][] fk_Xms_demonstration;
        fk_Xms_demonstration.length = obs_trajs.length;
        fk_Xms_demonstration = irl.calc_feature_expectations_per_sac_trajectory_gibbsSampling(
        	model, obs_trajs, foundWeightsGlbl, conv_threshold_gibbs);
        foreach (j; 0 .. fk_Xms_demonstration.length) {
            trajval_learnedweight = dotProduct(foundWeightsGlbl,fk_Xms_demonstration[j]);
            trajval_trueweight = dotProduct(trueWeights,fk_Xms_demonstration[j]);
            EVD += abs(trajval_trueweight - trajval_learnedweight)/(trajval_trueweight*cast(double)fk_Xms_demonstration.length);
        }
        writeln("\n EVD",EVD);
		arr_EVD ~= EVD;
	}
	
	/////////////////////////////////////////////// 
	
	File file1 = File("/home/saurabharora/Downloads/noisyObsRobustSamplingMeirl_LBA_data.csv", "a"); 
	string str_arr_LBA = to!string(arr_LBA);
	str_arr_LBA = str_arr_LBA[1 .. (str_arr_LBA.length-1)];
	file1.writeln(str_arr_LBA);
	file1.close(); 

	File file2 = File("/home/saurabharora/Downloads/noisyObsRobustSamplingMeirl_EVD_data.csv", "a"); 
	string str_arr_EVD = to!string(arr_EVD);
	str_arr_EVD = str_arr_EVD[1 .. (str_arr_EVD.length-1)];
	file2.writeln(str_arr_EVD);
	file2.close(); 

	writeln("BEGPARSING");	
	foreach (State s; model.S()) {
		foreach (Action a, double chance; policy.actions(s)) {

            sortingState ps = cast(sortingState)s;
            writeln( ps.toString(), " = ", a);
            string str_s="";
			if (ps._onion_location == 0) str_s=str_s~"Onconveyor,";
			if (ps._onion_location == 1) str_s=str_s~"Infront,";
			if (ps._onion_location == 2) str_s=str_s~"Inbin,";
			if (ps._onion_location == 3) str_s=str_s~"Picked/AtHomePose,";
			if (ps._onion_location == 4) str_s=str_s~"Placed,";

			if (ps._prediction == 0) str_s=str_s~"bad,";
			if (ps._prediction == 1) str_s=str_s~"good,";
			if (ps._prediction == 2) str_s=str_s~"unknown,";

			if (ps._EE_location == 0) str_s=str_s~"Onconveyor,";
			if (ps._EE_location == 1) str_s=str_s~"Infront,";
			if (ps._EE_location == 2) str_s=str_s~"Inbin,";
			if (ps._EE_location == 3) str_s=str_s~"Picked/AtHomePose,";

			if (ps._listIDs_status == 0) str_s=str_s~"Empty";
			if (ps._listIDs_status == 1) str_s=str_s~"NotEmpty"; 
			if (ps._listIDs_status == 2) str_s=str_s~"Unavailable";

            //writeln(str_s," = ", a);

		}
	}
	
	writeln("ENDPOLICY");

	debug {
		writeln("\nSimulation for learned policy:");
		sar [] trajs;
		for(int i = 0; i < 2; i++) {
			trajs = simulate(model, policy, initial, 50);
			//foreach (sar pair ; traj) {
			//	writeln(pair.s, " ", pair.a, " ", pair.r);
			//}
			//writeln(" ");
		}
	} 	

	writeln("ENDPARSING");

	debug {
	    auto endttime = Clock.currTime();
	    auto duration = endttime - starttime;
	    writeln("Runtime Duration ==> ", duration);
	}
	
	return 0;
}
