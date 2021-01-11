import std.stdio;
import mdp;
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

public double [] runAvgSessionLearnObsModel(int num_trials_perSession, Model model, 
	double [] trueDistr_obsfeatures, int  [][] noiseCorrupted_obs_fvs, sar [][] GT_trajs,
	sac [][] obs_trajs, double[StateAction][StateAction] trueObsMod,
	ref double numSessionsSoFar, ref double [] runAvg_learnedDistr_obsfeatures, 
	ref double avg_cum_diff1, ref double avg_cum_diff2, ref double [] [] arr_arr_cum_diff1) {

		//////// For averaging results over a predefined number of runs with same trueObsFeatDistr /////// 

	
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
		learnedDistr_obsfeatures[] = learnedDistr_obsfeatures[]/cast(double)(sum(learnedDistr_obsfeatures));

		// update the incrementally learned feature distribution 
		// local substitute variable for incremental runing average
		double [] temp_runAvg_learnedDistr_obsfeatures = new double[learnedDistr_obsfeatures.length];
		writeln(runAvg_learnedDistr_obsfeatures);
		writeln(learnedDistr_obsfeatures);
		writeln("numSessionsSoFar ",numSessionsSoFar);

		temp_runAvg_learnedDistr_obsfeatures[] = (runAvg_learnedDistr_obsfeatures[] + learnedDistr_obsfeatures[]);
		temp_runAvg_learnedDistr_obsfeatures[] /= numSessionsSoFar; 

		// for averaging over trials within session
		arr_learnedDistr_obsfeatures ~= temp_runAvg_learnedDistr_obsfeatures;

		// verify if this step is needed
		//temp_runAvg_learnedDistr_obsfeatures = learnedDistr_obsfeatures;

		writeln("true distr obsfeatures ",trueDistr_obsfeatures); 
		//writeln("cum_prob_fvs ",cum_prob_fvs); 
		writeln(); 
		writeln("learned distr obsfeatures: ", temp_runAvg_learnedDistr_obsfeatures); 
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
			} 
			diffprod = prod;

			prod = 1.0;
			foreach(i,f;obs_fv2) {
				if (f==1) prod *= temp_runAvg_learnedDistr_obsfeatures[i];
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
		arr_cum_diff1 ~= cum_diff1a;
		writeln("cumulative diff for observation likelihood of s-a pairs misidentified due to noise ",cum_diff1a);

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
		//arr_cum_diff1 ~= cum_diff1;
		//writeln("cumulative diff for observation likelihood of s-a pairs misidentified due to noise ",cum_diff1);
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
		writeln("Divide by 0 check: number of sa pairs:",trueObsMod.length," number of unseen GT sa pairs:",unseen_GT_SAs.length);
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


string feature_vector_to_string(int [] features) {
	string returnval = "";
	foreach (f; features) {
		returnval ~= f == 0 ? "0" : "1";
	}
	return returnval;
}

int[][] define_events(Model model, sac[][] samples, out int[string] eventMapping) {

	// create an associative array to hold the found events so far, 
	// events are distinguished by the set of observation features attached to their state/action
	// the easiest way I can think of is to convert the vector of features to a string ID
	
	// Note, later code (setIntersection) requires that the subsets be sorted in increasing order
	
	int[][] returnval;
	int[string] eventIds;
	
	foreach(sample; samples) {

		foreach(e_sac; sample) {
			auto feature_vector = model.obsFeatures(e_sac.s, e_sac.a);

			auto eID = feature_vector_to_string(feature_vector);
			debug {
				writeln("defin events: eID ",eID);
			}
			if (! ( eID in eventIds)) {
				eventIds[eID] = cast(int)returnval.length;
				
				int[] feature_numbers;
				foreach(i,f; feature_vector) {
					if (f != 0) {
						feature_numbers ~= cast(int)i;
					}
				}
				returnval ~= feature_numbers;
			}
		}
	
	}
	
	eventMapping = eventIds;
	
	return returnval;
}


int[][] define_events_obsMod(Model model, sac[][] samples, out int[string] eventMapping) {

	
	int[][] returnval;
	int[string] eventIds;
	

	foreach(sample; samples) {

		foreach(e_sac; sample) {
			auto feaures_obs_sa = model.obsFeatures(e_sac.s, e_sac.a);

			auto eID_prefix = feature_vector_to_string(feaures_obs_sa);
			debug {
				//writeln("define events: eID_prefix ",eID_prefix);
			}

			foreach(s; model.S()) { // for each state action pair
				foreach(a; model.A(s)){
					StateAction curr_gt_sa = new StateAction(s,a);

					auto feaures_GT_sa = model.obsFeatures(s,a);

					auto eID_suffix = feature_vector_to_string(feaures_GT_sa);

					auto eID = eID_prefix ~ eID_suffix;

					if (! ( eID in eventIds)) {
						debug {
							writeln("define events: eID  = eID_prefix ~ eID_suffix = ",eID);
						}
						eventIds[eID] = cast(int)returnval.length;
						
						int[] feature_numbers;
						foreach(i,f;feaures_obs_sa) {
							if (feaures_GT_sa[i] == feaures_obs_sa[i] && f == 1) feature_numbers ~= cast(int)i;
						} 
						returnval ~= feature_numbers;
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

			auto feaures_obs_sa = model.obsFeatures(e_sac.s, e_sac.a);

			auto eID_prefix = feature_vector_to_string(feaures_obs_sa);
			debug {
				//writeln("define events: eID_prefix ",eID_prefix);
			}

			foreach(s; model.S()) { // for each state action pair
				foreach(a; model.A(s)){
					StateAction curr_gt_sa = new StateAction(s,a);

					auto feaures_GT_sa = model.obsFeatures(s,a);

					auto eID_suffix = feature_vector_to_string(feaures_GT_sa);

					auto eID = eID_prefix ~ eID_suffix;

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

double [] calc_log_success_rate(sac[][] samples, int[string] eventMapping, Model model) {

	double [] returnval = new double[eventMapping.length];
	returnval[] = 0;
	
	int[string] totalCount;
	double[string] cumulativeScore;
	debug {
		//writeln("calc_log_success_rate: E len");
	}
	
	foreach(i,sample; samples) {

		foreach(e_sac; sample) {
			auto feature_vector = model.obsFeatures(e_sac.s, e_sac.a);

			auto eID = feature_vector_to_string(feature_vector);

			totalCount[eID] += 1;
			cumulativeScore[eID] += e_sac.c;

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

	return returnval;
}

int [][] coarsest_partition(int[][] E) {

	int[][] S;
	int[][] Edup = E.dup;
	int [] trivial = set_type!(int).to_set(uniq(nWayUnion(Edup)));
	S ~= trivial;
	
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
	}
	
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

	double [] returnval = new double[num_obsfeatures];
	returnval[] = 1;
	    	
	foreach(i, si; S) {
		foreach(subsi; si) {
			returnval[subsi] = pow(p[i], 1.0 / S[i].length); 
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
        }
        
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