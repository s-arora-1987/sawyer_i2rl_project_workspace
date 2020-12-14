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

byte [][] patrolToyMap() {
	return [[1,1]]; 
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
    BoydModelWdObsFeatures model = new BoydModelWdObsFeatures(null, map, null, 0, &patrolObsFeatures);
    State ts = model.S()[0];
    int numObFeatures = cast(int)(patrolObsFeatures(ts,model.A(ts)[0]).length);
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
	debug {
		writeln("number of states ",(model.S()).length);
		//writeln("number of actions ",(model.A()).length);
		int count = 0;
		//writeln("created trans function");
		foreach(s;model.S()) {
			foreach(a;model.A(s)){
				//writeln(model.T(s,a));
				//writeln("\n");
				count +=1;
			}
		}
		//writeln("# valid sa pairs ",count);
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

	sac [][] obs_trajs;
	int num_trajs = 1;
	double [] noise_range = [0.8,0.99];
	sar [] temp_traj;
	sac [] obs_traj;
	for(int i = 0; i < num_trajs; i++) {
		temp_traj = simulate(model, policy, initial, 10);
		obs_traj.length = 0; 
		foreach(e_sar;temp_traj) {
			obs_traj ~= sac(e_sar.s,e_sar.a,uniform(noise_range[0], noise_range[1]));
		}
		obs_trajs ~= obs_traj;
	}

	debug{
		writeln("observed trajectories ");
		foreach(obstraj;obs_trajs) writeln("\n",obstraj);
		//exit(0);
	}
	
	auto estimateObsMod = new MaxEntUnknownObsMod(100,new ValueIteration(), 2000, .00001, .1, .1);
	double opt_val_Obj;
	auto foundWeights = estimateObsMod.solve2(model, obs_trajs, opt_val_Obj);
	debug {
		writeln();
		writeln("learned p mapped to obs features : ", foundWeights);
	}

	writeln("BEGPARSING");


	writeln("ENDPARSING");
	
	return 0;
}

string feature_vector_to_string(int [] features) {
	string returnval = "";
	foreach (f; features) {
		returnval ~= f == 0 ? "0" : "1";
	}
	return returnval;
}


int [] patrolObsFeatures(State state, Action action) {
	
	//if (cast(TurnRightAction)action) { 
	//	return [0, 1, 1, 1];
	//} 

	//if (cast(TurnLeftAction)action) { 
	//	return [1, 0, 1, 1];
	//} 
	
	//if (cast(StopAction)action) {
	//	return [1, 1, 0, 1];
	//} 
	//return [1, 1, 1, 1];

	//if (cast(TurnLeftAction)action) { 
	//	return [1,0];
	//} 
	//return [1,1];

	if (cast(MoveForwardAction)action) { 
		return [1,0,0];
	} 
	if (cast(TurnLeftAction)action && (cast(BoydState)state).getLocation()[1]==0) { 
		return [0,1,0];
	} 
	return [0,0,1];

}

int[][] define_events(BoydModelWdObsFeatures model, sac[][] samples, out int[string] eventMapping) {

	// create an associative array to hold the found events so far, 
	// events are distinguished by the set of observation features attached to their state/action
	// the easiest way I can think of is to convert the vector of features to a string ID
	
	// Note, later code (setIntersection) requires that the subsets be sorted in increasing order
	
	int[][] returnval;
	int[string] eventIds;
	
	foreach(sample; samples) {

		foreach(e_sac; sample) {
			auto feature_vector = model.obFeatures(e_sac.s, e_sac.a);

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


double [] calc_log_success_rate(sac[][] samples, int[string] eventMapping, BoydModelWdObsFeatures model) {

	double [] returnval = new double[eventMapping.length];
	returnval[] = 0;
	
	int[string] totalCount;
	double[string] cumulativeScore;
	debug {
		//writeln("calc_log_success_rate: E len");
	}
	
	foreach(i,sample; samples) {

		foreach(e_sac; sample) {
			auto feature_vector = model.obFeatures(e_sac.s, e_sac.a);

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
	private double[] Q; // Q is the failure rate of each E
	private int[][] S; // the `coursest partition of the feature space induced by the Es
	private int[][] P; // M x N matrix, each row contains the vector of v's corresponding to a given P
	    	
    private double [] v;
    private double [] lambda;
    private sac[][] obsTraj_samples;
   
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	}

	
	public double [] solve2(BoydModelWdObsFeatures model, sac[][] obsTraj_samples, out double opt_value) {
		
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
        E = define_events(model, obsTraj_samples, eventMapping); 
        debug {
        	writeln("E: ", E);
        }
        
        Q = calc_log_success_rate(obsTraj_samples, eventMapping, model);
        foreach (ref q; Q) {
        	q = exp(q);
        }
        
        debug {
        	writeln("Q: ", Q);
        }
        
        S = coarsest_partition(E);
        
        debug {
        	writeln("S: ", S);
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
        //weights.length = 2*S.length;
        weights.length = S.length;
        int ret;	
        foreach (i; 0..30) {

        	auto temp =this;
        	//ret = lbfgs(cast(int)(2*S.length), p, &finalValue, &evaluate_maxent, &progress, &temp, &param);
        	ret = lbfgs(cast(int)(S.length), p, &finalValue, &evaluate_maxent, &progress, &temp, &param);
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
        	writeln("\nE: ", E,"\nQ: ",Q,"\nS: ",S,"\n learned p ",weights,
        		"\n cast(double)sum(Q) ",cast(double)sum(Q));
        	double [] normed_Q = new double[Q.length]; 
        	normed_Q[] = Q[]/cast(double)sum(Q);
        	double [] normed_p = new double[weights.length]; 
        	normed_p[] = weights[]/cast(double)sum(weights);
        	writeln("\nE: ", E,"\nQ: ",normed_Q,"\nS: ",S,"\n learned p ",normed_p);
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