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

alias std.string.indexOf indexOf;



int [] boyd2features(State state, Action action) {
	
	if (cast(TurnRightAction)action) { 
		return [0, 1, 1, 1];
	} 

	if (cast(TurnLeftAction)action) { 
		return [1, 0, 1, 1];
	} 
	
	if (cast(StopAction)action) {
		return [1, 1, 0, 1];
	} 
	return [1, 1, 1, 1];
}

int main() {


	sar [][][] SAR;
	string mapToUse;
	string buf;
	buf = readln();
	bool useSimpleFeatures;
	
	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	buf = readln();
	formattedRead(buf, "%s", &useSimpleFeatures);
	
	int curPatroller = 0;
	SAR.length = 1;
	
    while ((buf = readln()) != null) {
    	buf = strip(buf);
    
    	if (buf == "ENDTRAJ") {
    		curPatroller ++;
    		SAR.length = SAR.length + 1;
    		
    	} else {
    		sar [] newtraj;
    		
    		while (buf.indexOf(";") >= 0) {
    			string percept = buf[0..buf.indexOf(";")];
    			buf = buf[buf.indexOf(";") + 1 .. buf.length];
    			
    			string state;
    			string action;
    			double p;
    			
   				formattedRead(percept, "%s:%s:%s", &state, &action, &p);
   				
   				int x;
   				int y;
   				int z;
   				
				state = state[1..state.length];
   				formattedRead(state, "%s, %s, %s]", &x, &y, &z);
   				
   				Action a;
   				if (action == "MoveForwardAction") {
   					a = new MoveForwardAction();
   				} else if (action == "StopAction") {
   					a = new StopAction();
   				} else if (action == "TurnLeftAction") {
   					a = new TurnLeftAction();
   				} else if (action == "TurnAroundAction") {
   					a = new TurnAroundAction();
   				} else {
   					a = new TurnRightAction();
   				}
   				
   				
   				newtraj ~= sar(new BoydState([x, y, z]), a, p);

    		}
    		
    		SAR[curPatroller] ~= newtraj;
    		
    	}
    	
    }
	SAR.length = SAR.length - 1;
	
	byte [][] themap;
	
	if (mapToUse == "boyd2") {
		themap  = boyd2PatrollerMap();
		
	} else {
		themap = boydrightPatrollerMap();
		
	}
	
	BoydModel model;
	
	if (useSimpleFeatures) {
		model = new BoydModel(null, themap, null, 1, &simplefeatures);
	} else {
		model = new BoydModel(null, themap, null, 4, &boyd2features);	
	}
	
	double val;
	
	auto irl2 = new MaxEntUnknownTPenaltyMethod(100,new ValueIteration(), 2000, .00001, .1, .1);
//	irl = new MaxEntIrl(100,new ValueIteration(), 20);
	
	auto foundWeights = irl2.solve2(model, SAR[0], val);
	debug {
		writeln();
		writeln("Found Weights: ", foundWeights);
	}
	model.setT(model.createTransitionFunction(foundWeights, &stopErrorModel));
	
	
	foreach (s; model.S()) {
		foreach(a; model.A()) {
			auto s_primes = model.T(s, a);
			
			foreach(s_prime, pr_s_prime; s_primes) {
				writeln( (cast(BoydState)s).getLocation(), ":", a, ":", (cast(BoydState)s_prime).getLocation(), ":", pr_s_prime);
			}
		}
	}
	writeln("ENDT");
	auto foundWeights1 = foundWeights;
	
	foundWeights = irl2.solve2(model, SAR[1], val);
	debug {
		writeln();
		writeln("Found Weights: ", foundWeights);
	}
	model.setT(model.createTransitionFunction(foundWeights, &stopErrorModel));
	
	
	foreach (s; model.S()) {
		foreach(a; model.A()) {
			auto s_primes = model.T(s, a);
			
			foreach(s_prime, pr_s_prime; s_primes) {
				writeln( (cast(BoydState)s).getLocation(), ":", a, ":", (cast(BoydState)s_prime).getLocation(), ":", pr_s_prime);
			}
		}
	}
	writeln("ENDT");
	
	writeln("WEIGHTS: ", foundWeights1);	
	writeln("WEIGHTS: ", foundWeights);

	return 0;
}




class MaxEntUnknownTPenaltyMethod : MaxEntIrl {

	private int[][] E; // E is a set of subsets, each subset contains the number of a feature
	private double[] Q; // Q is the failure rate of each E
	private int[][] S; // the coursest partition of the feature space induced by the Es
	private int[][] P; // M x N matrix, each row contains the vector of v's corresponding to a given P
	    	
    private double [] v;
    private double [] lambda;
    	
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	}

	
	public double [] solve2(Model model, sar[][] true_samples, out double opt_value) {
		
        // Compute feature expectations of agent = mu_E from samples
        
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.max_iterations = max_iter;
        param.epsilon = error;
//        param.min_step = .00001; 
        
        this.model = model;
        this.true_samples = true_samples;
        this.sample_length = cast(int)true_samples.length;
        
        int[string] eventMapping;
        
        E = define_events(model, true_samples, eventMapping); 
        debug {
        	writeln("E: ", E);
        }
        
        Q = calc_log_success_rate(true_samples, eventMapping, model);
        foreach (ref q; Q) {
        	q = exp(q);
        }
        
        debug {
        	writeln("Q: ", Q);
        }
        
        S = coarsest_partition(E, model.numTFeatures());
        
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
        weights.length = 2*S.length;
        int ret;
        foreach (i; 0..30) {
//		  	opt_value = evaluate_nelder_mead ( init_weights.ptr, init_weights.length );
	
//	        opt_weights.length = init_weights.length;
//	        nelmin ( &evaluate_nelder_mead, init_weights.length, init_weights.ptr, p.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );
        
        	ret = lbfgs(cast(int)(2*S.length), p, &finalValue, &evaluate_maxent, &progress, &this, &param);
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
        
        opt_value = finalValue;
        
        // now we have all the p's, find the probability of each individual feature using S
        
        // Note that we might not have a weight for each feature

        return map_p_onto_features(weights, S, model.numTFeatures());
	}

	
	
	
	override double evaluate(double [] p, out double [] g, double step) {
    	
    	// format: [p_1^1, p_2^1, ... p_1^2 ...]

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


int[][] define_events(Model model, sar[][] samples, out int[string] eventMapping) {

	// create an associative array to hold the found events so far, events are distinguished by the set of transition features attached to their state/action
	// the easiest way I can think of is to convert the vector of features to a string ID
	
	// Note, later code (setIntersection) requires that the subsets be sorted in increasing order
	
	int[][] returnval;
	int[string] eventIds;
	
	foreach(sar; samples) {
		if (sar.length == 0)
			continue;
			
		auto feature_vector = model.TFeatures(sar[0].s, sar[0].a);
	
		auto eID = feature_vector_to_string(feature_vector);
		
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
	
	eventMapping = eventIds;
	
	return returnval;
}

string feature_vector_to_string(int [] features) {
	string returnval = "";
	foreach (f; features) {
		returnval ~= f == 0 ? "0" : "1";
	}
	return returnval;
}


double [] calc_log_success_rate(sar[][] samples, int[string] eventMapping, Model model) {

	double [] returnval = new double[eventMapping.length];
	returnval[] = 0;
	
	int[string] totalCount;
	int[string] successCount;
	
	
	foreach(i,sar; samples) {
		if (sar.length == 0)
			continue;
		
		if (i >= samples.length - 1)
			continue;
		
			
		auto feature_vector = model.TFeatures(sar[0].s, sar[0].a);
	
		auto eID = feature_vector_to_string(feature_vector);
	
		State sp = sar[0].a.apply(sar[0].s);
		
		totalCount[eID] += 1;
		
		// assume a successful move if we lose sight of the expert
		if (samples[i+1].length == 0 || sp == samples[i+1][0].s) {
			successCount[eID] += 1;
		}
		
	}
	
/*	writeln("totalCount: ", totalCount);
	writeln("failureCount: ", failureCount);
	writeln("eventMapping: ", eventMapping); */
	
	foreach (ID, num; eventMapping) {
		if (ID in successCount)
			returnval[num] = log(cast(double)(successCount[ID] + 1) / cast(double)(totalCount[ID] + 1) );
		else
			returnval[num] = 0;
	}
	
	
	// Place hardcoded failure rates in here that match the true ones for debugging

//	returnval[eventMapping["1"]] = log(0.824614);
	
/*	returnval[eventMapping["1010"]] = log(0.871465);
	returnval[eventMapping["0110"]] = log(0.824614);
	returnval[eventMapping["0101"]] = log(0.662962); 
	*/
	return returnval;
}

/*
 * Returns the finest partition of a set Ei union Ej (over all E's
 */ 
int [][] finest_partition(int[][] E, int num_features) {

	int[][] S;
	int[][] Edup = E.dup;
	int [] trivial = set_type!(int).to_set(uniq(nWayUnion(Edup)));
	S ~= trivial;
	
	int[][] returnval;
	
	foreach (i, Si; S) {
		foreach(j, Sij; Si) {
		returnval ~= [Sij];
		}
	
	}
	
	return returnval;
}

/*
 * Returns the coarsest partition of a set {0, .. num_features} induced by all of the subsets E
 */ 
int [][] coarsest_partition(int[][] E, int num_features) {

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

double [] map_onto_features(double [] w, int [][] S, int[][] E, int num_features) {
	    
    	
	auto v = w[0..E.length];
	auto lambda = w[E.length..$];
	
	// need to find the p's here
	
    auto p = new double[S.length];
	p[] = 0;

	foreach(i,Si; S) {
		foreach(j,Ej; E) {
		
			auto intersection = setIntersection(Si, Ej);
			
			if (equal(intersection, Si)) {
				p[i] += v[j];
			}
		}
		p[i] += lambda[i] - 1;
		p[i] = exp(p[i]);
	}
	
	return map_p_onto_features(p, S, num_features);

}

double [] map_p_onto_features(double [] p, int [][] S, int num_features) {

	double [] returnval = new double[num_features];
	returnval[] = 1;
	    	
	foreach(i, si; S) {
		foreach(subsi; si) {
			returnval[subsi] = pow(p[i], 1.0 / S[i].length); 
		}
	}
	
	return returnval;

}
