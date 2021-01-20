import mdp;
import std.math;
import std.algorithm;
import std.stdio;
import std.random;
import std.numeric;
import std.datetime;
import core.stdc.stdlib : exit;
import std.algorithm.iteration;
import std.datetime;


alias extern(C) double function(void *, const double *, double *, const int, const double) evaluate_callback;
alias extern(C) int function(void *, const double *, const double *, const double, const double, const double, const double, int, int, int) progress_callback;

alias extern(C) double function(double * x, int n) nm_callback;

extern(C) void nelmin ( nm_callback callback, int n, double * start, double * xmin, double *ynewlo, double reqmin, double * step, int konvge, int kcount,  int *icount, int *numres, int *ifault );
extern(C) void timestamp ( );

extern(C) struct  lbfgs_parameter_t {

    int m;
    double epsilon;
    int past;
    double delta;
    int max_iterations;
    int linesearch;
    int max_linesearch;
    double min_step;
    double max_step;
    double ftol;
    double wolfe;
    double gtol;
    double xtol;
    double orthantwise_c;
    int orthantwise_start;
    int orthantwise_end;
};

extern(C) void lbfgs_parameter_init(lbfgs_parameter_t *param);
extern(C) double * lbfgs_malloc(int n);
extern(C) void lbfgs_free(double *x);

extern(C) int lbfgs(int n, double *x,
    double *ptr_fx,
    evaluate_callback proc_evaluate,
    progress_callback proc_progress,
    void *instance,
    lbfgs_parameter_t *param
    );

extern(C) double evaluate_maxent(
    void *instance,
    const double *x,
    double *g,
    const int n,
    const double step
    ) {
    	
    	MaxEntIrl * irl = cast(MaxEntIrl*)instance;
    	
    	double [] w;
    	w.length = n;
    	w[0..n] = x[0..n];

    	for (int i = 0; i < n; i ++) {
    		if (! isNormal(w[i])) {
    				return 0.0/0.0;
    			}
    	}
    	double [] _mu;
    	double ans = irl.evaluate(w, _mu, cast(double)step);

    	debug {    	
	    	//writeln(ans, " w: ", w, " ",  _mu, " ", step);
    	}
    	
        for (int i = 0; i < n; i ++) {
        	g[i] = _mu[i];
        	
        }
        
        return ans;
        
    }


extern(C) int progress(
    void *instance,
    const double *x,
    const double *g,
    const double fx,
    const double xnorm,
    const double gnorm,
    const double step,
    int n,
    int k,
    int ls
    ) {
    	debug {
	    	writeln("Iteration ", k);
	    	writeln("  fx = ",fx,"  xnorm = ", xnorm,", gnorm = ",gnorm ,", step = ",step);
    	}
    	return 0;
    }

MaxEntIrl nelderMeadReference;


extern(C) double evaluate_nelder_mead(double * x, int n) {
    	
    	double [] w;
    	w.length = n;
    	w[0..n] = x[0..n];

    	for (int i = 0; i < n; i ++) {
    		if (! isNormal(w[i])) {
    			return 0.0/0.0;
    		}
    	}
    	double [] _mu;
    	
    	double ans = nelderMeadReference.evaluate(w, _mu, 0);
    	
    	debug(lbfgs) {    	
	    	writeln(ans, " w: ", w, " ",  _mu);
    	}    	
        
        return ans;
}



class IRL {

	protected int max_iter;
	protected MDPSolver solver;
	protected int n_samples;
	protected double error;
	protected double solverError;
	protected double[StateAction][] sa_freq;
		
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError=0.1) {
		this.max_iter = max_iter;
		this.solver = solver;
		this.n_samples = n_samples;
		this.error = error;
		this.solverError = solverError;
	}

	public abstract Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights);
		

            
    double [] feature_expectations(Model model, sar [][] samples) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		double [] f = ff.features(SAR.s, SAR.a);
        		returnval[] += f[];
        	}
        }
        returnval [] /= cast(double)samples.length;
        return returnval;
	}
    
    double [] feature_expectations2(Model model, sar [][] samples, int num) {
/*      Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        
        This is for the expert only! we assume each timestep has multiple possible states! */
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
/*    	foreach (State s; model.S()) {
    		foreach (Action a; model.A(s)) {
    			sa_freq[new StateAction(s, a)] = 0;
    		}
    	} */
    	
    	if (sa_freq.length < num + 1)
    		sa_freq.length = num + 1;
    	
        foreach (int i, sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		if (SAR.s !is null) {
	        		double [] f = ff.features(SAR.s, SAR.a);
	        		f[] = f[] * SAR.p;
	//        		writeln(SAR.s, " -> ", f);
	        		returnval[] = f[] + returnval[];
	        		StateAction key = new StateAction(SAR.s, SAR.a);
	        		sa_freq[num][key] = sa_freq[num].get(key, 0.0) + SAR.p;
        		}
        	} 
        }
/*        foreach (key, value; sa_freq) {
        	sa_freq[key] /= samples.length;
        	
        } */
//        returnval [] = returnval[];
        return returnval;
    }	
}

class MaxEntIrl : IRL {
	
	protected double qval_thresh;
	public Model model;
	protected double[State] initial;
	protected sar[][] true_samples;
	protected size_t sample_length;
	protected double [] mu_E;
	
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError);

		this.qval_thresh = qval_thresh;
	}

	
	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
		
        // Compute feature expectations of agent = mu_E from samples
        
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.max_iterations = max_iter;
//        param.epsilon = error;
        param.min_step = .00001; 
        
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
        this.true_samples = true_samples;
        this.sample_length = cast(int)true_samples.length;
        
        mu_E = feature_expectations2(model, true_samples, 0);
        
 //       writeln("True Samples ", mu_E, " L: ", sample_length);
        
        LinearReward r = cast(LinearReward)model.getReward();


        opt_weights.length = r.dim();
        
        double * x = lbfgs_malloc(cast(int)opt_weights.length);
        scope(exit) {
        	lbfgs_free(x);
        } 
        
        for (int i = 0; i < opt_weights.length; i ++) 
//       		x[i] = uniform(0.0, 1.0);
        	x[i] = init_weights[i];
//        	x[i] = 1;
        
        double finalValue;
        auto temp = this;
     	int ret = lbfgs(cast(int)opt_weights.length, x, &finalValue, &evaluate_maxent, &progress, &temp, &param);


        for (int i = 0; i < opt_weights.length; i ++)
        	opt_weights[i] = x[i];
  
        
 //       writeln(ret, ": ", finalValue, " -> ", opt_weights);
        r.setParams(opt_weights);
        
        opt_value = finalValue;
        
        return solver.createPolicy(model, solver.solve(model, solverError));
		
 
	}

	
	
	
	double evaluate(double [] w, out double [] g, double step) {
    	
    	LinearReward r = cast(LinearReward)model.getReward();
    	
    	r.setParams(w);
    	
        double[StateAction] Q_value = QValueSolve(model, qval_thresh);
        double sum = 0;
     
 /*       foreach (sa, v; Q_value) {
        	writeln("Q:", sa.s, " ", sa.a, " ", v); 
        }*/
        
        foreach (sa, count ; sa_freq[0] ) {
        	sum += Q_value[sa] * count;
        } 


        Agent agent = CreatePolicyFromQValue(model, Q_value);
            	
/*        foreach (State s; model.S()) {
        	writeln(s, " ", agent.actions(s));
        	
        } */
        
        double [] _mu;    	
    	if (n_samples > 0) {
    	
 /*   		What we need to do is increase the number of samples (starting at n_samples) each time we do an evalution without also increasing the interation.  In other words,
    			number of samples = n_samples + n_samples * alpha * (number of evalutions since iteration increase)
    		for some value alpha, possibly determined by the step size*/
    	
	    	sar [][] samples = generate_samples(model, agent, initial, this.n_samples, sample_length);

	        _mu = feature_expectations(model, samples);
        } else {
        
	    	double[StateAction] D = calcStateActionFreq(agent, initial, model, sample_length);
	    	
	        _mu = feature_expectations_exact(model, D); 
        }
        
        g.length = w.length;
	
        g[] = -( mu_E[] - _mu[] );

        return -sum;		
		
	}

    double [] feature_expectations_exact(Model model, double[StateAction] D) {

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



	double exponentiatedGradient(ref double [] w, double c) {
	/*
	The world reveals a set of features x in {0,1}n. In the online learning with an adversary literature, 
	the features are called “experts” and thought of as subpredictors, but this interpretation isn’t 
	necessary—you can just use feature values as experts (or maybe the feature value and the negation of 
	the feature value as two experts).
EG makes a prediction according to y’ = w . x (dot product).
The world reveals the truth y in [0,1].
EG updates the weights according to wi <- wie-2 c (y’ – y)xi. Here c is a small constant learning rate.
The weights are renormalized to sum to 1.
	*/
		double [] x;
		double y_prime;

    	LinearReward r = cast(LinearReward)model.getReward();
	
//		do {
		foreach (q; 0 .. 10) {
	        foreach (sar [] sar1; true_samples) {
	        	foreach (sar SAR; sar1) {

	        		/* ok lets try this:
	        		calculate feature expectations for the current weights
	        		x = subtract from mu_e
	        		normalize x by l2norm(x)
	        		
	        		ok so what the fuck is y?
	        		
	        		y is e^qvaluesoftmax(Sar.s, SAR.a)
	        		after we solve for the current weights
	        		
	        		*/
		    	
			    	x = r.features(SAR.s, SAR.a);
			    	
			    	
			    	y_prime = dotProduct(w, x);
			        
			        r.setParams(w);
	                double[StateAction] Q_value = QValueSoftMaxSolve(model, qval_thresh);
	                
		        	auto y = exp(Q_value[new StateAction(SAR.s, SAR.a)]);
		        	
			        double sumweights = 0;
		        	foreach (i, ref wi; w) { 
		        		wi = wi*exp(-2*c*(y_prime - y)*x[i]);
		        		sumweights += wi;
		        	}
			        
			        w[] /= sumweights;
			        
			        writeln(y_prime, ":",y," @ ", w, " -> ", x);
//			        writeln(w);
		
	        	}
	        }
	        c /= 10;
	     }   
//	    } while (l2norm(x) > .1)     
	
	
	    return y_prime;
	}

}

class MaxEntIrlZiebartExact : MaxEntIrl {
	
		
	Agent last_stochastic_policy;
	double [][] feature_expectations_per_trajectory;
	protected State [] observableStates;

	
	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1) {
		super(max_iter, solver, n_samples, error, solverError);
		this.observableStates = observableStates;
	}

	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {

        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
        this.true_samples = true_samples;
        
        
        feature_expectations_per_trajectory = calc_feature_expectations_per_trajectory(model, true_samples);
        
        debug {
 //       	writeln("FE's ", feature_expectations_per_trajectory);
        }	
        
        
        LinearReward ff = cast(LinearReward)model.getReward();

        mu_E.length = ff.dim();
        mu_E[] = 0;
        
        foreach(traj_fe; feature_expectations_per_trajectory) {
        	mu_E[] += traj_fe[] / true_samples.length;
        }
        
        /*
        foreach(traj; true_samples) {
        	double val = 1.0;
        	foreach (i; 0..traj.length - 1) {
        		auto transitions =  model.T(traj[i].s, traj[i].a);
        		
        		foreach (state, pr; transitions) {
        			if (traj[i+1].s == state) {
        				val *= pr;
        				break;
        			}
        		}
        	}
        	
        	pr_traj ~= val;
        	
        }
        */
   		debug {
   			writeln("True Samples ", mu_E, " L: ", sample_length);
   		}

		
        auto temp_init_weights = init_weights.dup;
        foreach(ref t; temp_init_weights)
                t = abs(t);
        temp_init_weights[] /= l1norm(temp_init_weights);

        debug {
        	writeln("Initial Weights ", temp_init_weights);
		}
        
        opt_weights = exponentiatedGradient(temp_init_weights, 1, error, sample_length);

        
        (cast(LinearReward)model.getReward()).setParams(opt_weights);
        
        return solver.createPolicy(model, solver.solve(model, solverError));        
        
	}
	
	
	double[][] calc_feature_expectations_per_trajectory(Model model, sar[][] trajs) {
		
		double [][] returnval;
        LinearReward ff = cast(LinearReward)model.getReward();
		
		this.sample_length = 0;
		
		foreach (traj; trajs) {
			
			double [] temp_fe = new double[ff.dim()];
			temp_fe[] = 0;
			foreach(i, SAR; traj) {
				if (SAR.s ! is null)
					temp_fe[] += ff.features(SAR.s, SAR.a)[];
				if (i > this.sample_length)
					this.sample_length = i;
			}
			
			returnval ~= temp_fe;
		}
		
		return returnval;
	} 
	
	
	double [] ExpectedEdgeFrequencyFeatures(double [] weights) {
		
		double [State] Z_s;
		double [Action][State] Z_a;
		
		foreach(s; model.S()) {
			if (model.is_terminal(s)) {
				Z_s[s] = 1.0;
			}
			
		}
		
		size_t N = this.sample_length;
		LinearReward ff = cast(LinearReward)model.getReward();
		
		foreach(q; 0..N) {
			foreach(i; model.S()) {
				if (model.is_terminal(i)) {
					double [] features = ff.features(i, new NullAction());
					features[] *= weights[]; 
					Z_a[i][new NullAction()] = exp(reduce!("a + b")(0.0, features)) * Z_s.get(i, 0.0) * model.A().length;
					
				} else {
					foreach(j; model.A(i)) {
						double sum = 0;
						foreach (k; model.S()) {
							double [] features = ff.features(i, j);
							features[] *= weights[]; 
							sum += model.T(i,j).get(k, 0.0) * exp(reduce!("a + b")(0.0, features)) * Z_s.get(k, 0.0);
						}
						
						Z_a[i][j] = sum; 
						
					}
				}
			}
			
			foreach(i; model.S()) {
				double sum = 0;
				if (model.is_terminal(i)) {
					sum += Z_a[i][new NullAction()];
				} else {	
					foreach(j; model.A(i)) {
						sum += Z_a[i][j];
					}
				}
				Z_s[i] = sum;
				
			}
			
		}
		
		double [Action][State] P;
		
		foreach(i; model.S()) {
			if (model.is_terminal(i)) {
				P[i][new NullAction()] = Z_a[i][new NullAction()] / Z_s[i];
			} else {
				foreach(j; model.A(i)) {
					P[i][j] = Z_a[i][j] / Z_s[i];
				}
			}
		} 
		last_stochastic_policy = new StochasticAgent(P);
		
		double [State][] D;
		
		D ~= initial.dup;
		
		foreach (t; 1..N) {
			double [State] temp;
			foreach (k; model.S()) {
				temp[k] = 0.0;
				foreach(i; model.S()) {
					if (model.is_terminal(i)) {
						if (i == k)
							temp[k] += D[t-1].get(k, 0.0) * P[k][new NullAction()];
					} else {
						foreach(j; model.A(i)) {
							temp[k] += D[t-1].get(i, 0.0) * P[i][j] * model.T(i, j).get(k, 0.0);
						}
					}
				}
			}
			D ~= temp;
		
		}
		double [] returnval = new double[ff.dim()];
		returnval[] = 0;
		
		double [State] Ds;
		foreach(t; 0..N) {
			foreach (i; observableStates) {
				if (t == 0)
					Ds[i] = 0.0;
				Ds[i] += D[t].get(i, 0.0);
			}
		}
		foreach (i; observableStates) {	
			if (model.is_terminal(i)) {
				returnval[] += Ds.get(i, 0.0) * P[i][new NullAction()] * ff.features(i, new NullAction())[];
			} else {
				foreach (j; model.A(i)) {
					returnval[] += Ds.get(i, 0.0) * P[i][j] * ff.features(i, j)[]; 
				}
			}
		}
		
		return returnval;
		
	}
	
	double [] exponentiatedGradient(double [] w, double c, double err, size_t max_sample_length) {
		import std.math;

		debug {
			writeln("max_sample_length: ", max_sample_length);
		}

		double [] y = mu_E.dup;
//		foreach (ref y1; y)
//			y1 = pow(y1, 1/2.0);
/*		auto y_norm = l1norm(y);
		if (y_norm != 0)
			y[] /= y_norm;*/
		y[] /= max_sample_length;
		
		debug {
			writeln("Y Normed: ", y);
		}

        foreach(ref t; w)
            t = abs(t);
        w[] /= l1norm(w);
	        	
		double diff;
		double lastdiff = double.max;
		double [] lastGradient = y.dup;

		do {
		
			double [] y_prime =  ExpectedEdgeFrequencyFeatures(w);
//			foreach (ref y1; y_prime)
//				y1 = pow(y1, 1/2.0);
			debug {
				//writeln("y_prime",y_prime);
			}

/*			auto y_prime_norm = l1norm(y_prime);
			if (y_prime_norm != 0)
				y_prime[] /= y_prime_norm;*/
			y_prime[] /= max_sample_length;
			
			debug {
				auto temp = y_prime.dup;
				temp[] -= y[];
				writeln("GRADIENT: ", temp);
			}

			double [] next_w = new double[w.length]; 
			foreach(i; 0..w.length) {
				next_w[i] = w[i] * exp(-2*c*(y_prime[i] - y[i]) );
			}
			
			double norm = l1norm(next_w);
			if (norm != 0)
				next_w[] /= norm;
			
			double [] test = w.dup;
			test[] -= next_w[];
			diff = l2norm(test);
			
			double [] gradient = y_prime.dup;
			gradient[] -= y[];
/*			double [] test = gradient.dup;
			test[] -= lastGradient[];
			diff = l2norm(test);*/
			
			debug {
				writeln("diff:",diff," err:",err," next_w:",next_w);
				
			}
			w = next_w;
			c /= 1.05;
/*			if (diff > lastdiff) {
				c /= 1.1;
			} else {
				c *= 1.05;
			}*/	
			lastdiff = diff;	
			lastGradient = gradient;
		
		} while (diff > err);
		
		return w;
	}

		
}

class MaxEntIrlZiebartApprox : MaxEntIrlZiebartExact {
	/*
	No occlusion yet.
	Computing FE for agent by sampling trajectories rather than 
	computing partition function.
	Using FE of single agent (without stacking weights of other agent)
	to do gradient descent using MaxEntIrlZiebartExact.
	exponentiatedGradient(double [] w, double c, double err, size_t max_sample_length)

	*/

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, 
		double error=0.1, double solverError =0.1) {
		super(max_iter, solver, observableStates, n_samples, error, solverError);
	}

	public Agent solve(Model model, double[State] initial, sar[][] true_samples,
	size_t sample_length, double [] init_weights, out double opt_value, 
	out double [] opt_weights, ref double [] featureExpecExpert, int num_Trajsofar,
	double Ephi_thresh, double step_size, int descent_dur_thresh_secs, double [] trueWeights) {

        this.model = model;
        this.initial = initial;
        this.true_samples = true_samples;
    	this.sample_length = cast(int)sample_length;
        
    	LinearReward r = cast(LinearReward)model.getReward();			
    	mu_E.length = r.dim();
        
        double num_Trajsofard = cast(double)num_Trajsofar;
        double [] mu_Eprev = new double[mu_E.length];

        foreach(i2, t; featureExpecExpert)
            mu_Eprev[i2] = t;

        debug {
        	//writeln("init_weights: ",init_weights);
        	//writeln("mu_Eprev: ",mu_Eprev);
        }

        double lastQValue = -double.max;
        bool hasConverged;
        double [] temp_opt_weights;
	    double [] last_temp_opt_weights = init_weights.dup;
        size_t max_sample_length = cast(int)sample_length;

        feature_expectations_per_trajectory = calc_feature_expectations_per_trajectory(model, true_samples);
        debug {
        	writeln("FE's ", feature_expectations_per_trajectory);
        }	

        mu_E[] = 0;        
        foreach(traj_fe; feature_expectations_per_trajectory) {
        	//mu_E[] += traj_fe[];// 
        	mu_E[] += traj_fe[]/ true_samples.length;
        }        
        debug {
        	writeln("mu_E: ", mu_E);
        	//exit(0);
        }
    	//writeln("mu_E: ", mu_E);
        //computing new target mu_E specific to current iteration
        foreach (int i, double val; mu_E)
           mu_E[i] = (val*true_samples.length+mu_Eprev[i]*num_Trajsofard)/(num_Trajsofard+true_samples.length);
           //updating muE doesn't give better learning curve
        debug {
        	writeln("mu_E after update: ", mu_E);
        }

        
        opt_value=-double.max;
        double opt_value_grad_val = double.max;
        opt_weights.length = init_weights.length;

        int iterations=0;
        double grad_val;
         //multiple restarts and pick the answer with best likelihood
        do {
        	temp_opt_weights = init_weights.dup;
	        //temp_opt_weights = SingleAgentExponentiatedGradient(temp_opt_weights.dup, step_size, 
	        //	error, max_sample_length,Ephi_thresh, grad_val, descent_dur_thresh_secs);

	        temp_opt_weights = singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent(temp_opt_weights.dup, 0.25, 
	        	error, max_sample_length, Ephi_thresh, trueWeights, true, 1);

			//double [] singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent(double [] w,
			//	double nu, double err, size_t max_iter, double Ephi_thresh,
			//	bool usePathLengthBounds = true, size_t moving_average_length = 5) {

			    //compare final gradients
			//	if (grad_val < opt_value_grad_val && false) {
			//		opt_value = grad_val;
			      //  auto i=0;
			      //  foreach(j, ref o2; opt_weights) {
			    		//o2 = temp_opt_weights[i++];     	
			      //  }
			      //  debug {
			      //  	writeln("grad val (", iterations, ") = ", grad_val, " for weights: ", temp_opt_weights);		        	
			      //  }
			//	}

        	// calculate Q value
        	double newQValue = SingleAgentcalcQ(temp_opt_weights,Ephi_thresh);
        	if ((newQValue > opt_value) ) {
        		opt_value = newQValue;
		        auto i=0;
		        foreach(j, ref o2; opt_weights) {
		    		o2 = temp_opt_weights[i++];     	
		        }
		        debug {
		        	writeln("Q(", iterations, ") = ", newQValue, " for weights: ", temp_opt_weights);		        	
		        }
        	}

	        iterations ++;
	    } while (iterations < max_iter);
        
	    
     	Agent returnval;
        r.setParams(opt_weights);        
        returnval = solver.createPolicy(model, solver.solve(model, solverError));

        auto i = 0;
        mu_E[] /= max_sample_length;
        foreach(j, ref o2; featureExpecExpert) {
        	o2 = mu_E[i++];
        }
        //featureExpecExpertfull = featureExpecExpert.dup;
        return returnval;
	}	

	double [] SingleAgentExpectedEdgeFrequencyFeatures(double [] w, double threshold) {
		// approximate feature expectations to avoid computing partition function

		debug {
			//writeln("SingleAgentExpectedEdgeFrequencyFeatures started");
		}
		LinearReward r = cast(LinearReward)this.model.getReward();
        r.setParams(w);		

		Agent policy = this.solver.createPolicy(this.model, this.solver.solve(this.model, this.solverError));

		debug {
			//writeln("SingleAgentExpectedEdgeFrequencyFeatures computed policy in ");
		}

		//Agent [] policies = getPoliciesFor(w);

        double [] returnval = new double[w.length];
        returnval[] = 0;
        debug {
        	//writeln("SingleAgentExpectedEdgeFrequencyFeatures");
        }

 		//double threshold = 0.5;//for patrolling task
 		//threshold = 0.5;//sorting task both behaviors
 		////threshold = 2.0;//for learning weights for individual sorting behaviors
        if (this.n_samples > 0) { 

               double [] total = returnval.dup;
               double [] last_avg = returnval.dup;
               size_t repeats = 0;
               while(true) {

					debug {
						//writeln("SingleAgentExpectedEdgeFrequencyFeatures started sampling traj space");
					}
					sar [][] samples = generate_samples(this.model, policy, this.initial, 
						this.n_samples, this.sample_length);
					debug {
						//writeln("sampled traj space");
					}

					// cumulative feature count for samples
					total[] += feature_expectations(this.model, samples)[];
					debug {
						//writeln("computed FEs");
					}

					repeats ++;

					double [] new_avg = total.dup;
					// running average
					new_avg[] /= repeats;

					double max_diff = -double.max;

					foreach(i, k; new_avg) {
						// change in running average
						auto tempdiff = abs(k - last_avg[i]);
						if (tempdiff > max_diff) {
							max_diff = tempdiff;
						}
					}// end foreach 
					debug {
						//writeln("E[phi]/ mu: max_diff for ", repeats," repeat is ", max_diff);
					}

					if (max_diff < threshold ) {
						debug {
							//writeln("mu Converged after ", repeats, " repeats, ", n_samples * repeats, " simulations");
							//writeln("mu Converged after ", repeats, " repeats, ", n_samples * repeats, " simulations");
						}
						break;
					}// end if
                       
                    last_avg = new_avg;             
                       
               }
               // total/repeats , not last_avg/repeats
               returnval[] = total[] / repeats;
               
        } else {
//             auto Ds = calcStateFreq(policy, initial, model, sample_length);
               throw new Exception("Not Supported with n_samples < 0");
                       
        }
         
        return returnval;

	}

	double SingleAgentcalcQ(double [] weights, double Ephi_thresh) {
		auto features = SingleAgentExpectedEdgeFrequencyFeatures(weights,Ephi_thresh);
		
		features[] *= weights[];
				
		double returnval = -1 * reduce!("a + b")(0.0, features);
		
		debug {
			//writeln("SingleAgentcalcQ");
			//writeln("Q: log(Z) ", returnval, " for Features ", features);
			
		}
		
		double [] expectation = mu_E.dup;
		expectation[] *= weights[];
		
		returnval += reduce!("a + b")(0.0, expectation);
		
				
		return returnval;
		
		
	}	

	double [] SingleAgentExponentiatedGradient(double [] w, double c, double err, 
		size_t max_sample_length, double Ephi_thresh, out double gradient_val, int dur_thresh_secs) {
		import std.math;

		debug {
			//writeln("max_sample_length: ", max_sample_length);
			//writeln("SingleAgentExponentiatedGradient mu_E: ",mu_E);
		}
		
		double [] y = mu_E.dup;
//		foreach (ref y1; y)
//			y1 = pow(y1, 1/2.0);
/*		auto y_norm = l1norm(y);
		if (y_norm != 0)
			y[] /= y_norm;*/
		//y[] /= max_sample_length;
		
        //foreach(ref t; w)
        //    t = abs(t);
        w[] /= l1norm(w);

		debug {
			//writeln("initial weights: ", w);
			//writeln("Y : ", y);
		}
	        	
		double diff;
		double lastdiff = double.max;
		double [] lastGradient = y.dup;
		double [] y_prime;
		double [] gradient;

	    auto stattime = Clock.currTime();
	    auto endttime = Clock.currTime();
	    auto duration = dur!"seconds"(1);

		do {
		
			y_prime =  SingleAgentExpectedEdgeFrequencyFeatures(w,Ephi_thresh);

//			foreach (ref y1; y_prime)
//				y1 = pow(y1, 1/2.0);
			debug {
				//writeln("y_prime",y_prime);
			}
			
			double [] next_w = new double[w.length]; 
			foreach(i; 0..w.length) {
				//next_w[i] = w[i] * exp(-2*c*(y_prime[i] - y[i]) );
				next_w[i] = w[i] * exp(-2*c*(y_prime[i] - y[i]) );
			}

			debug {
				//writeln("next_w: ", next_w);
			}
			
			double norm = l1norm(next_w);
			if (norm != 0)
				next_w[] /= norm;
			
			double [] test = w.dup;
			test[] -= next_w[];
			diff = l2norm(test);
			
			gradient = y_prime.dup;
			gradient[] -= y[];
/*			double [] test = gradient.dup;
			test[] -= lastGradient[];
			diff = l2norm(test);*/
			
			debug {
				//writeln("normed (lhs - rhs) of constraint: ", l1norm(gradient));
				//writeln("diff:",diff," err:",err," next_w:",next_w);
				//writeln("likelihood:",SingleAgentcalcQ(next_w,Ephi_thresh));
			}
			w = next_w;
			
			//c /= 1.025;

			if (diff > lastdiff) {
				c /= 1.05;
			} else {
				c *= 1.025;
			}	

			lastdiff = diff;	
			lastGradient = gradient;
		    
		    endttime = Clock.currTime();
		    duration = endttime - stattime;
		
		} while (diff > err && duration < dur!"seconds"(dur_thresh_secs));

		debug {
			writeln("descent converged");
			writeln("diff < err  :  ",(diff < err ));
			//writeln("E[phi] / mu for learned weights ", y_prime);
		}
		gradient_val = l1norm(gradient);
		return w;
	}

    double computeLBA(MapAgent agent1, MapAgent agent2) {
        double totalsuccess = 0.0;
        double totalstates = 0.0;
        // Mapagent is a deterministic policy by definition
        foreach (s; this.model.S()) {
            if (s in agent1.getPolicy()) {
                // check key existence 
                //writeln("number of actions in current state in learned policy",(agent1.policy[s]))

                Action action = agent1.sample(s);

                if (s in agent2.getPolicy()) {
                    totalstates += 1;
                    if (agent2.sample(s) == action) {
                        //writeln("found a matching action");
                        totalsuccess += 1;
                    } //else writeln("for state {},  action {} neq action {} ".format(ss2,action,truePol[ss2]));                    
                } else writeln("state ",s," not found in agent2");
            } else writeln("state ",s," not found in agent1");
        }

        //print("totalstates, totalsuccess: "+str(totalstates)+", "+str(totalsuccess))
        if (totalstates == 0.0) {
            writeln("Error: states in two policies are different");
            return 0;
        }
        double lba = (totalsuccess) / (totalstates);
        return lba;
    }

	double [] singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent(double [] w,
		double nu, double err, size_t max_iter, double Ephi_thresh, double [] trueWeights,
		bool usePathLengthBounds = true, size_t moving_average_length = 5) {
		//////////////////////////////////////////////////////
	    // nu = 0.01;
	    // nu = 0.0075;
	    // without adaptive nu worked for sorting mdp tasks without
	    // prediction scores !!
		//////////////////////////////////////////////////////
		  
		usePathLengthBounds = false;
		double diff;
		double lastdiff = double.max;
		err = 1;
		moving_average_length = 6;
	    
		double [] expert_features = mu_E.dup;
		//expert_features[] *= (1-this.model.gamma);	    
	    //foreach (ref e ; err_moving_averages) {
	    //   e = double.max;
	    //}
		//writeln("normalized mu_E ",expert_features);
	    double [] beta = new double[mu_E.length];
	    beta[] = - log(beta.length );
	    
	    double [] z_prev = new double [beta.length];
	    z_prev[] = 0;
	    double [] w_prev = new double [beta.length];
	    w_prev[] = w[];

	    size_t t = 0;
	    size_t iterations = 0;
	    double[][] moving_average_data;
	    size_t moving_average_counter = 0;
	    double [] err_moving_averages = new double[moving_average_length];
	    foreach (ref e ; err_moving_averages) {
	       e = double.max;
	    }
	    double err_diff = double.infinity;

	    //writeln("starting singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent");
		LinearReward r = cast(LinearReward)this.model.getReward();
        r.setParams(trueWeights);		
		Agent truePolicy = this.solver.createPolicy(this.model, this.solver.solve(this.model, this.solverError));

	    //while (iterations < max_iter && (err_diff > err || iterations < moving_average_length)) {

	    while ((err_diff > err )) {

	        double [] m_t = z_prev.dup;

	        if (! usePathLengthBounds && iterations > 0)
	            m_t[] /= iterations;

	        double [] weights = new double[beta.length];
	        foreach (i ; 0 .. (beta.length)) {
	            weights[i] = exp(beta[i] - nu*m_t[i]);
	        }

			debug {

				//writeln("likelihood:",SingleAgentcalcQ(weights,Ephi_thresh));
			}

	        // allow for negative weights by interpreting the second half
	        // of the weight vector as negative values
	        //double [] actual_weights = new double[beta.length / 2];
	        //foreach(i; 0 .. actual_weights.length) {
	        //    actual_weights[i] = weights[i] - weights[i + actual_weights.length];
	        //}

			double [] actual_weights = weights.dup;

	        double [] z_t = SingleAgentExpectedEdgeFrequencyFeatures(actual_weights,Ephi_thresh);
			////import std.stdio;        
			////writeln(t, ": ", z_t, " => ", expert_features[t], " w: ", weights, " actual_w: ", actual_weights);
					//writeln("z_t from ff ",z_t);

	        z_t[] -= expert_features[];

			diff = l1norm(z_t);

			debug {
				//writeln("learned weights ",actual_weights);
				//writeln(" (lhs - rhs) of constraint: ", (z_t));
				writeln("normed (lhs - rhs) of constraint: ", l1norm(z_t));
		        r.setParams(actual_weights);		
				Agent learnedPolicy = this.solver.createPolicy(this.model, this.solver.solve(this.model, this.solverError));
				writeln("lba w.r.t trueWeights ",this.computeLBA(cast(MapAgent)learnedPolicy,cast(MapAgent)truePolicy));
			}
	            
	        if (usePathLengthBounds) {
	            z_prev = z_t;
	        } else {
	            z_prev[] += z_t[];
	        }

	        foreach(i; 0..(beta.length)) {
	            beta[i] = beta[i] - nu*z_t[i] - nu*nu*(z_t[i] - m_t[i])*(z_t[i] - m_t[i]);
	        }

	        err_moving_averages[moving_average_counter] = diff;
            //writeln("err_moving_averages\n ",err_moving_averages);
            moving_average_counter ++;
	        
	        t ++;
	        //t %= moving_average_length;
	        //t %= expert_features.length;
	        iterations ++;
//	        if (t == 0) {
            //writeln("err_moving_averages\n ",err_moving_averages);
            //nu /= 1.01;
            //err_moving_averages[moving_average_counter] = this.abs_average(moving_average_data);

            //moving_average_counter ++;
            moving_average_counter %= moving_average_length;
            //moving_average_data.length = 0;

		    double sum = 0;
		    foreach (entry; err_moving_averages) {
		        sum += abs(entry);
		    }
		    sum /= err_moving_averages.length;
		    err_diff = sum;

            err_diff = this.stddev(err_moving_averages);
            //writeln("err_moving_averages\n ",err_moving_averages);


            //writeln("err_diff ",err_diff);
	//            writeln(abs_diff_average(err_moving_averages));

//	        } 

	        //moving_average_data ~= z_t.dup;

	        w_prev = actual_weights;

			//if (diff > lastdiff) {
			//	nu /= 1.05;
			//} else {
			//	nu *= 1.025;
			//}	
	        lastdiff = diff;

	    } 
	    
	    //writeln("iterations >= max_iter ",(iterations >= max_iter)," err_diff < err ",
	    //	(err_diff < err)," iterations > moving_average_length ",(iterations > moving_average_length));
	    
	    return w_prev;
	}


	double abs_average(double [][] data) {

	    if (data.length == 0)
	        return double.infinity;
	        
	    double [] sum = new double[data[0].length];
	    sum[] = 0;
	    foreach (entry; data) {
	        sum [] += array_abs(entry)[];
	    }
	    sum [] /= data.length;
	    
	    double returnval = 0;
	    foreach (s ; sum) {
	        returnval += s;
	    }
	    return returnval / sum.length;

	}

	double [] array_abs(double [] data) {

	    double [] returnval = data.dup;

	    foreach (ref a; returnval) {
	        a = abs(a);
	    }
	        
	    return returnval;
	}

	double stddev(double [] data) {
	    auto n = data.length;
	    auto avg = reduce!((a, b) => a + b / n)(0.0, data);
	    auto var = reduce!((a, b) => a + pow(b - avg, 2) / n)(0.0, data);

	    return sqrt(var);         
	}

}


class MaxEntIrlZiebartApproxNoisyObs : MaxEntIrlZiebartApprox {
	/*
	Sampling (Imp Sampling or Gibbs sampling) to compute mu_E 
	by using observation model based on prediction score of s-a pairs 
	*/

	double Qsolve_qval_thresh;
	ulong QSolve_max_iter;

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, 
		double error=0.1, double solverError =0.1, double Qsolve_qval_thresh = 0.01, ulong QSolve_max_iter = 100) {
		super(max_iter, solver, observableStates, n_samples, error, solverError);
		this.Qsolve_qval_thresh = Qsolve_qval_thresh;
		this.QSolve_max_iter = QSolve_max_iter;
	}

	public Agent solve(Model model, double[State] initial, sac[][] noisy_samples,
	size_t sample_length, double [] init_weights, out double opt_value, 
	out double [] opt_weights, ref double [] featureExpecExpert, int num_Trajsofar,
	double Ephi_thresh, double step_size, int descent_dur_thresh_secs, double [] trueWeights,
	double conv_threshold_stddev_diff_moving_wdw, size_t moving_window_length_muE,
	int use_ImpSampling, double conv_threshold_gibbs, out double diff_wrt_muE_wo_sc, 
	out double diff_wrt_muE_scores1) {

        this.model = model;
        this.initial = initial;
        this.true_samples = true_samples;
    	this.sample_length = cast(int)sample_length;
        
    	LinearReward r = cast(LinearReward)model.getReward();			
    	mu_E.length = r.dim();
        
        double num_Trajsofard = cast(double)num_Trajsofar;
        double [] mu_Eprev = new double[mu_E.length];

        foreach(i2, t; featureExpecExpert)
            mu_Eprev[i2] = t;

        debug {
        	//writeln("init_weights: ",init_weights);
        	//writeln("mu_Eprev: ",mu_Eprev);
        }

        double lastQValue = -double.max;
        bool hasConverged;
        double [] temp_opt_weights = init_weights.dup;
	    double [] last_temp_opt_weights = init_weights.dup;
        size_t max_sample_length = cast(int)sample_length;

        opt_value=-double.max;
        double opt_value_grad_val = double.max;
        opt_weights.length = init_weights.length;

        int iterations=0;
        double grad_val;
        //multiple restarts and pick the answer with best likelihood
        double diff_muE_norm;
        // not used
        double conv_threshold_abs_diff_moving_wdw = 0.01;
        double [] last_muE, tempdiff;
        last_muE.length = r.dim();
        tempdiff.length = r.dim();
        mu_E[] = 0;

        // moving window to track the change in muE
		size_t moving_window_counter_muE = 0;
		double [] err_moving_window_muE = new double[moving_window_length_muE];
		foreach (ref e ; err_moving_window_muE) {
		   e = double.max;
		}
		double stdev_moving_window_diff_muE_norm = double.infinity;
		double [][] feature_expectations_per_trajectory_wo_sc;
		double [][] feature_expectations_per_trajectory_scores1;

		debug {
	        double [] mu_E_wo_sampling = new double[mu_E.length];
	        sar [][] trajs_wo_scores;
	        foreach(traj; noisy_samples) {
	        	sar [] traj_wo_scores;
		        foreach(e_sac; traj) {
		        	traj_wo_scores ~= sar(e_sac.s,e_sac.a,1.0);
		        }
		        trajs_wo_scores ~= traj_wo_scores;
	        }
			feature_expectations_per_trajectory_wo_sc = 
			calc_feature_expectations_per_trajectory(model, trajs_wo_scores);
			mu_E_wo_sampling[] = 0.0;
			foreach(double [] traj_fe; feature_expectations_per_trajectory_wo_sc) {
				// no division by size of dataset
				mu_E_wo_sampling[] += traj_fe[]/cast(double)noisy_samples.length;
			} 
			double [] temp_diff_wrt_muE_wo_sc = new double[mu_E.length];

	        double [] mu_E_scores1 = new double[mu_E.length];
	        sac [][] trajs_scores1;
			double [] temp_diff_wrt_muE_scores1 = new double[mu_E.length];
        	double [] mu_E_copy = new double[mu_E.length];
		}
		double [] leaned_muE = new double[mu_E.length];

        do {
        	temp_opt_weights = init_weights.dup;

	        last_muE[] = 0.01; 
	        writeln("\n random restart begin \n");
		    auto starttime = Clock.currTime();

        	do {

		        leaned_muE = SingleAgentExpectedEdgeFrequencyFeatures(
		        	temp_opt_weights.dup, Ephi_thresh);

        		mu_E[] = 0.0;

				if (use_ImpSampling == 1) {
					// Use imp sampling and learned weights to calculate muE 
					feature_expectations_per_trajectory = 
					calc_feature_expectations_per_sac_trajectory_impSampling(model, 
					noisy_samples, temp_opt_weights.dup);
				} else {
					// Use MCMC sampling and learned weights to calculate muE 
					feature_expectations_per_trajectory = 
					calc_feature_expectations_per_sac_trajectory_gibbsSampling(model, 
					noisy_samples, temp_opt_weights.dup, conv_threshold_gibbs);
				}

		        foreach(traj_fe; feature_expectations_per_trajectory) {
		        	// no division by size of dataset
		        	mu_E[] += traj_fe[];
		        } 

				tempdiff[] = (last_muE[] - mu_E[])/sum(last_muE);
				last_muE[] = mu_E[];
				diff_muE_norm = l1norm(tempdiff);
				//writeln("diff mu_E - mu_E last iteration  ",diff_muE_norm);

				err_moving_window_muE[moving_window_counter_muE] = diff_muE_norm;
				moving_window_counter_muE ++;
				moving_window_counter_muE %= moving_window_length_muE;
				stdev_moving_window_diff_muE_norm = this.stddev(err_moving_window_muE);
				writeln("stdev_moving_window_diff_muE_norm ",stdev_moving_window_diff_muE_norm);

		        debug {

		        	mu_E_copy = mu_E.dup;
		        	// Normalize all muE's to get realtive preferences
		        	//mu_E_copy[] = mu_E_copy[]/sum(mu_E_copy);
		        	//mu_E_wo_sampling[] = mu_E_wo_sampling[]/sum(mu_E_wo_sampling);

		        	temp_diff_wrt_muE_wo_sc[] = mu_E_wo_sampling[]-mu_E_copy[];
		        	//writeln("diff w.r.t hat-phi without imp sampling ",l1norm(temp_diff_wrt_muE_wo_sc));

		        	// For current weights, compute norm diff w.r.t. muE with ALL scores 1
		        	trajs_scores1.length = 0;
			        foreach(traj; noisy_samples) {
			        	sac [] traj_scores1;
				        foreach(e_sac; traj) {
				        	traj_scores1 ~= sac(e_sac.s,e_sac.a,1.0);
				        }
				        trajs_scores1 ~= traj_scores1;
			        }

					if (use_ImpSampling == 1) {
						// Use imp sampling and learned weights to calculate muE 
						feature_expectations_per_trajectory = 
						calc_feature_expectations_per_sac_trajectory_impSampling(model, 
						noisy_samples, temp_opt_weights.dup);
					} else {
						// Use MCMC sampling and learned weights to calculate muE 
						feature_expectations_per_trajectory = 
						calc_feature_expectations_per_sac_trajectory_gibbsSampling(model, 
						noisy_samples, temp_opt_weights.dup, conv_threshold_gibbs);
					}
					
					mu_E_scores1[] = 0.0;
					foreach(double [] traj_fe; feature_expectations_per_trajectory_scores1) {
						// no division by size of dataset
						mu_E_scores1[] += traj_fe[]; 
					} 

					//mu_E_scores1[] = mu_E_scores1[]/sum(mu_E_scores1);
		        	temp_diff_wrt_muE_scores1[] = mu_E_scores1[]-mu_E_copy[];
		        	//writeln("diff w.r.t hat-phi with ALL scores 1 ",l1norm(temp_diff_wrt_muE_scores1));

					//tempdiff[] = leaned_muE[] - mu_E[];
					//diff_muE_norm = l1norm(tempdiff);
					//writeln("diff mu_E - learnedmuE  ",diff_muE_norm);
		        }

		        // weights with lowest standard deviation in  (muE-Ephi])
		        //temp_opt_weights = singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent(
		        //	temp_opt_weights.dup, 0.0075, 
		        //  error, max_sample_length, Ephi_thresh, trueWeights, true, 1);

		        // Adaptive nu with fixed max iterations, weights with lowest (muE-Ephi])
		        temp_opt_weights = singleTaskUncAdapExpoStochGradDescReturnBestInMaxIter(
		        	temp_opt_weights.dup, 0.0075, 
		        	error, Ephi_thresh, trueWeights, true, 1, 15);

		        //exit(0);

        	} while (stdev_moving_window_diff_muE_norm > conv_threshold_stddev_diff_moving_wdw);
        	//} while ((diff_muE_norm > conv_threshold_abs_diff_moving_wdw) && 
        		//(stdev_moving_window_diff_muE_norm > conv_threshold_stddev_diff_moving_wdw));

	        writeln("\n random restart ends in ",(Clock.currTime()-starttime)," time \n");
	        //exit(0);

        	// calculate Q value
        	double newQValue = SingleAgentcalcQ(temp_opt_weights,Ephi_thresh);
        	if ((newQValue > opt_value) ) {
        		opt_value = newQValue;
		        auto i=0;
		        foreach(j, ref o2; opt_weights) {
		    		o2 = temp_opt_weights[i++];     	
		        }
		        debug {
		        	writeln("Q(", iterations, ") = ", newQValue, " for weights: ", temp_opt_weights);		        	
		        }
        	}

	        iterations ++;

	    } while (iterations < max_iter); //max_iter is number of random restarts 
        
        debug {
        	diff_wrt_muE_wo_sc = l1norm(temp_diff_wrt_muE_wo_sc);
        	diff_wrt_muE_scores1 = l1norm(temp_diff_wrt_muE_scores1);
        }
	    
     	Agent returnval;
        r.setParams(opt_weights);        
        returnval = solver.createPolicy(model, solver.solve(model, solverError));

        auto i = 0;
        mu_E[] /= max_sample_length;
        foreach(j, ref o2; featureExpecExpert) {
        	o2 = mu_E[i++];
        }
        //featureExpecExpertfull = featureExpecExpert.dup;
        return returnval;
	}	

	double[][] calc_feature_expectations_per_sac_trajectory_gibbsSampling(Model model, 
		sac[][] trajs, double[] weights, double conv_threshold_gibbs) {
		// Robust IRL
		debug {
			//writeln("calc_feature_expectations_per_sac_trajectory_gibbsSampling ");
		}

        LinearReward rw = cast(LinearReward)model.getReward();
        rw.setParams(weights);        
		double[StateAction] Q_value = QValueSoftMaxSolve(model, this.Qsolve_qval_thresh, this.QSolve_max_iter);        
		Agent stochPolicy = CreateStochasticPolicyFromQValue(model, Q_value);
		double[Action][State] stochPolicyMatrix = (cast(StochasticAgent)stochPolicy).getPolicy();
		debug {
			//writeln("stoch policy computed ");
		}
		// Unlike domain used by Shervin, our domain of observations is same as ground truths 		
		// The transition probabilities P(sa|MB(sa)) are computed based on ground truths 
		// of previous and next states. However, inputs are only observations. 
		// Therefore, we need an initial set of random ground truth trajectories
		// simulated using learned policy. 

		sar[][] GT_trajs;
		sac[] obs_traj;
		foreach(i; 0..trajs.length) {
			obs_traj = trajs[i].dup;
			sar[] GT_traj = simulate(model, stochPolicy, initial, obs_traj.length); 
			GT_trajs ~= GT_traj;
		}

		// Then the algorithm is as follows. 
		// 
		// 1) Until the relative diff in hat-phi or muE stabilizes, do following
		// 2) For each GT_traj, 
		// 2 a) For each timestep and corresponding s-a-c triplet in observed traj, 
		// 2 b a) Create a distribution over all possible GT s-a pairs, dict_gt_sa
		// by iterating over states and allowed actions
		// dict_gt_sa[s-a] = P(sa|MB(sa)) = P(s|prev-s,prev-a) * P(a|s) * P(next-s|s,a) * P(obs-s-a|s,a) 
		// = P(s|prev-s,prev-a) * stochPolicyMatrix[s][a] * P(next-s|s,a) * P[obs-s-a|s,a] 
		// for first timestep P(s|prev-s,prev-a)  = 1, and for last, P(next-s|s,a) = 1
		// 2 b b) Sample GT s-a pair using dict_gt_sa for the chosen timestep
		// 3) Compute new muE and relative diff

		debug {
			//writeln("MCMC started ");
		}

		double [] muE_sampled = new double[rw.dim()];
		muE_sampled[] = double.max;
		double [] last_muE_sampled = new double[rw.dim()];
		double [] diff_muE_sampled = new double[rw.dim()];
		diff_muE_sampled[] = double.max;
		//double conv_threshold_gibbs = 1;
		double [][] fe_per_gt_traj;
		fe_per_gt_traj.length = trajs.length;
		// dictionary for distribution over grround truth s-a pairs
		double[StateAction] dict_gt_sa;
		double[State] dict_P_s_prevs_preva, dict_P_nexts_s_a;
		double P_s_prevs_preva, P_nexts_s_a, P_obssa_GTsa, total_mass;
		sar[] GT_traj;
		int iter_count = 0;
		double perce_diff_wrt_last_muE_sampled = double.max;

		while (perce_diff_wrt_last_muE_sampled>conv_threshold_gibbs) {

			// update GT_trajs by smapling every node
			foreach(int i, sar[] traj; GT_trajs) {
				obs_traj = trajs[i].dup;
				debug {
					//writeln("traj length ", traj.length);
					//writeln("obs_traj length ", obs_traj.length);
				}

				foreach(int j, sar gt_sar; traj) {
					// because GT_traj changes with every j increment
					GT_traj = GT_trajs[i].dup;
					sac e_sac = obs_traj[j];

					StateAction temp_sa = new StateAction(e_sac.s,e_sac.a);

					// define distribution for current node
					foreach(s;model.S()) {
						foreach(a; model.A(s)) {

							// as action.apply are stochastic, so is model.T
							// call it only once and save it
							if (j!=0) dict_P_s_prevs_preva = model.T(GT_traj[j-1].s,GT_traj[j-1].a);
							if (j!=traj.length-1) dict_P_nexts_s_a = model.T(s,a);

							if (j==0) {
								P_s_prevs_preva = 1.0;
							} else if ((s in dict_P_s_prevs_preva) !is null) {
								// if current GT state is possible using previous GT s-a
								P_s_prevs_preva = dict_P_s_prevs_preva[s];
							} else P_s_prevs_preva = 0.0;

							debug {
								//writeln("P_s_prevs_preva ",P_s_prevs_preva); 
							}

							if (j==traj.length-1) {
								P_nexts_s_a = 1.0;
							} else if ((GT_traj[j+1].s in dict_P_nexts_s_a) !is null) {
								// if next GT state is possible using GT s-a
								P_nexts_s_a = dict_P_nexts_s_a[GT_traj[j+1].s];
							} else P_nexts_s_a = 0.0;

							debug {
								//writeln("MCMC 3",new StateAction(s, a));
								//writeln("(new StateAction(s, a)) in model.obsMod ",
								//	(new StateAction(s, a)) in model.obsMod);
								//writeln(new StateAction(e_sac.s,e_sac.a));
								//writeln((new StateAction(e_sac.s,e_sac.a)) in model.obsMod[new StateAction(s, a)]);
							}

							P_obssa_GTsa = model.obsMod[new StateAction(s, a)][new StateAction(e_sac.s,e_sac.a)];

							dict_gt_sa[new StateAction(s, a)] = P_s_prevs_preva 
							* stochPolicyMatrix[s][a] * P_nexts_s_a * P_obssa_GTsa;

						}
					}

					total_mass = 0;
					foreach (key, val ; dict_gt_sa) {
						total_mass += val;						
					} 
					// It is possible and valid that dict_gt_sa has all values 0 
					// Skip the case where all values in dict_gt_sa are zero
					if (total_mass != 0) {
						// sample and replace GT s-a 
						Distr!StateAction.normalize(dict_gt_sa);
						debug {
							//writeln("\ni,j - ",i," ",j," before sampling GT s-a  ");
						}

						StateAction sampled_GT_sa = Distr!StateAction.sample(dict_gt_sa);
						sar sampled_GT_sar = sar(sampled_GT_sa.s,sampled_GT_sa.a,-1.0);
						GT_trajs[i][j] = sampled_GT_sar;
						debug {
							//writeln("\ni,j - ",i," ",j," sampled and replaced GT s-a  ");
						}
					}
					// rehash the distribution for next timestep
					dict_gt_sa.rehash;
				}
			}

			last_muE_sampled = muE_sampled.dup;
			// update muE 
			fe_per_gt_traj = calc_feature_expectations_per_trajectory(model, GT_trajs);
			muE_sampled[] = 0.0;
			foreach(fe;fe_per_gt_traj) muE_sampled[] += fe[];
			diff_muE_sampled[] = last_muE_sampled[] - muE_sampled[];

			if (cast(double)l1norm(last_muE_sampled) >= double.max) {
				perce_diff_wrt_last_muE_sampled = double.max;
			} else {				
	 			perce_diff_wrt_last_muE_sampled = 
				cast(double)l1norm(diff_muE_sampled)/cast(double)l1norm(last_muE_sampled);
			}

			debug {
				//writeln("\niter_count - ",iter_count,
				//"cast(double)l1norm(last_muE_sampled) ", cast(double)l1norm(last_muE_sampled),
				//"cast(double)l1norm(diff_muE_sampled) ",cast(double)l1norm(diff_muE_sampled),
				//" perce_diff_wrt_last_muE_sampled ",perce_diff_wrt_last_muE_sampled);
			}
			iter_count += 1;

		}
		//exit(0);

		debug {
			//writeln("MCMC finished ");
		}

		return fe_per_gt_traj;
	} 

	double[][] calc_feature_expectations_per_sac_trajectory_impSampling(Model model, sac[][] trajs, double[] weights) {
		
		double [][] returnval;
        LinearReward rw = cast(LinearReward)model.getReward();
        rw.setParams(weights);        
		double[StateAction] Q_value = QValueSoftMaxSolve(model, this.Qsolve_qval_thresh, this.QSolve_max_iter);        
		Agent stochPolicy = CreateStochasticPolicyFromQValue(model, Q_value);
		double[Action][State] stochPolicyMatrix = (cast(StochasticAgent)stochPolicy).getPolicy();
		double sum_P_X_X_P_theta_X  = 0;
		double [] P_theta_Xs = new double[0];
		double [] P_X_Xs = new double[0];
		
		//writeln("init P_theta_Xs ",P_theta_Xs);
		//writeln("init P_X_Xs ",P_X_Xs);

		foreach (traj; trajs) {
			
			double [] temp_fe = new double[rw.dim()];
			temp_fe[] = 0;
	        // likelihood of trajectory 
			double P_theta_X = 1;
			// prediction score or confidence of trajectory
			double P_X_X = 1;
			foreach(i, e_sac; traj) {
				if (e_sac.s ! is null) {
					temp_fe[] += rw.features(e_sac.s, e_sac.a)[];
			        // if next state exists in current trajectory 
			        if ((i+1) < traj.length) {
				        double[State] T = model.T(e_sac.s, e_sac.a); 
				        
				        if (traj[i+1].s in T) {
				        	//writeln("found next state in T ",T,": ",T[traj[i+1].s]);
					        P_theta_X *= T[traj[i+1].s];
				        }
			        } 
			        //writeln("pi_E(s) ",stochPolicyMatrix[e_sac.s]," pi_E(s,a) ",stochPolicyMatrix[e_sac.s][e_sac.a]);
			        P_theta_X *= stochPolicyMatrix[e_sac.s][e_sac.a];
			        P_X_X *= e_sac.c; 
			    }

				//writeln("P_X_X",P_X_X," P_theta_X ",P_theta_X);
			}
			//writeln("P_X_X ",P_X_X," P_theta_X ",P_theta_X);
			sum_P_X_X_P_theta_X += P_X_X*P_theta_X;
			temp_fe[] = temp_fe[]*P_theta_X; 

			P_theta_Xs ~= P_theta_X; 
			P_X_Xs ~= P_X_X; 
			returnval ~= temp_fe; 
		}
		debug {
			//writeln("P_theta_Xs ",P_theta_Xs);
			//writeln("P_X_Xs ",P_X_Xs);
		}

		foreach (double [] traj_fe; returnval) {
			traj_fe[] /= sum_P_X_X_P_theta_X;
			//writeln(traj_fe[]);
		}
		//exit(0);

		return returnval;
	} 

	double [] singleTaskUncAdapExpoStochGradDescReturnBestInMaxIter(double [] w,
		double nu, double err, double Ephi_thresh, double [] trueWeights,
		bool usePathLengthBounds = true, size_t moving_average_length = 5, int max_iter=100) {
		  
		usePathLengthBounds = false;
		double diff;
		double lastdiff = double.max;
		err = 1;
		moving_average_length = 3;
	    
		double [] expert_features = mu_E.dup;
		//expert_features[] *= (1-this.model.gamma);	    
	    //foreach (ref e ; err_moving_averages) {
	    //   e = double.max;
	    //}
		//writeln("normalized mu_E ",expert_features);
	    double [] beta = new double[mu_E.length];
	    beta[] = - log(beta.length );
	    
	    double [] z_prev = new double [beta.length];
	    z_prev[] = 0;
	    double [] w_prev = new double [beta.length];
	    w_prev[] = w[];

	    size_t t = 0;
	    int iterations = 0;
	    double[][] moving_average_data;
	    size_t moving_average_counter = 0;
	    double [] err_moving_averages = new double[moving_average_length];
	    foreach (ref e ; err_moving_averages) {
	       e = double.max;
	    }
	    double err_diff = double.infinity;

	    //writeln("starting singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent");
		LinearReward r = cast(LinearReward)this.model.getReward();
        r.setParams(trueWeights);		
		Agent truePolicy = this.solver.createPolicy(this.model, this.solver.solve(this.model, this.solverError));
		double [] result = new double [beta.length];
		double best_diff = lastdiff;

	    while ((iterations < max_iter)) {

	        double [] m_t = z_prev.dup;

	        if (! usePathLengthBounds && iterations > 0)
	            m_t[] /= iterations;

	        double [] weights = new double[beta.length];
	        foreach (i ; 0 .. (beta.length)) {
	            weights[i] = exp(beta[i] - nu*m_t[i]);
	        }

			debug {

				//writeln("likelihood:",SingleAgentcalcQ(weights,Ephi_thresh));
			}


			double [] actual_weights = weights.dup;

	        double [] z_t = SingleAgentExpectedEdgeFrequencyFeatures(actual_weights,Ephi_thresh);

	        z_t[] -= expert_features[];

			diff = l1norm(z_t);

			if (diff > lastdiff) {
				nu /= 1.05;
			} else {
				best_diff = diff;
				result = actual_weights;
				nu *= 1.025;
			}	

			debug {
				//writeln("learned weights ",actual_weights);
				//writeln(" (lhs - rhs) of constraint: ", (z_t));
				//if (iterations==0) {
				//	writeln("normed (lhs - rhs) of constraint: ", l1norm(z_t));
				//}
				//writeln("normed (lhs - rhs) of constraint: ", l1norm(z_t));
		  //      r.setParams(actual_weights);		
				//Agent learnedPolicy = this.solver.createPolicy(this.model, this.solver.solve(this.model, this.solverError));
				//writeln("lba w.r.t trueWeights ",this.computeLBA(cast(MapAgent)learnedPolicy,cast(MapAgent)truePolicy));
			}
	            
	        if (usePathLengthBounds) {
	            z_prev = z_t;
	        } else {
	            z_prev[] += z_t[];
	        }

	        foreach(i; 0..(beta.length)) {
	            beta[i] = beta[i] - nu*z_t[i] - nu*nu*(z_t[i] - m_t[i])*(z_t[i] - m_t[i]);
	        }

	        err_moving_averages[moving_average_counter] = diff;
            //writeln("err_moving_averages\n ",err_moving_averages);
            moving_average_counter ++;
	        
	        t ++;
	        iterations ++;
            moving_average_counter %= moving_average_length;

            err_diff = this.stddev(err_moving_averages);


	        w_prev = actual_weights;
	        lastdiff = diff;

	    } 
	    
	    //writeln("iterations >= max_iter ",(iterations >= max_iter)," err_diff < err ",
	    //	(err_diff < err)," iterations > moving_average_length ",(iterations > moving_average_length));
		debug {
			//writeln("best diff in maxent constraint:",best_diff);
	  //      r.setParams(result);		
			//Agent learnedPolicy = this.solver.createPolicy(this.model, this.solver.solve(this.model, this.solverError));
			//writeln("lba w.r.t trueWeights ",this.computeLBA(cast(MapAgent)learnedPolicy,cast(MapAgent)truePolicy));
		}
	    
	    return result;
	}

}

class LatentMaxEntIrlZiebartExact : MaxEntIrlZiebartExact {

	size_t[] y_from_trajectory;	
	double [] pr_traj;
	double lastQValue;

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1) {
		super(max_iter, solver, observableStates, n_samples, error, solverError);
	}
	
	public void setYAndZ(size_t[] y_from_trajectory, double [] pr_traj, double [][] feature_expectations_per_trajectory, size_t sample_length) {
		this.y_from_trajectory = y_from_trajectory;
		this.feature_expectations_per_trajectory = feature_expectations_per_trajectory;
		this.pr_traj = pr_traj;
		this.sample_length = sample_length;
	}
	
	public void getYAndZ(out size_t[] y_from_trajectory, out double [] pr_traj, out double [][] feature_expectations_per_trajectory, out size_t sample_length) {
		y_from_trajectory = this.y_from_trajectory;
		feature_expectations_per_trajectory = this.feature_expectations_per_trajectory;
		pr_traj = this.pr_traj;
		sample_length = this.sample_length;
	}
	
	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
		
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
        this.true_samples = true_samples;
        
        LinearReward r = cast(LinearReward)model.getReward();

        mu_E.length = r.dim();
        mu_E[] = 0;
        
        if (y_from_trajectory.length == 0)
        	calc_y_and_z(true_samples);		
        
        // reset observableStates so that we consider all features
        this.observableStates = model.S();
        	
/*        debug {	
        	writeln("FE's ", feature_expectations_per_trajectory);
        }*/	
		
		foreach(ref t; init_weights)
            t = abs(t);
        init_weights[] /= l1norm(init_weights);

        double ignored;
        double [] traj_distr = getTrajectoryDistribution(init_weights, ignored);
        
              
        lastQValue = -double.max;
        bool hasConverged;
        opt_weights = init_weights.dup;
        
        mu_E.length = init_weights.length;
        
        
        auto iterations = 0;
        // loop until convergence
        do {
	        mu_E = calc_E_step(traj_distr);

	        debug(LMEExact) {
	        	writeln("mu_E: ", mu_E);
	        }
	        
	        
            auto temp_init_weights = init_weights.dup;
	        
	        debug {
	        	writeln("Initial Weights ", temp_init_weights);
			}
	        
	        opt_weights = exponentiatedGradient(temp_init_weights, 1, error, sample_length);

	        
	        r.setParams(opt_weights);

	        double normalizer;
	    	traj_distr = getTrajectoryDistribution(opt_weights, normalizer);

        	// calculate Q value
        	double newQValue = calcQ(opt_weights, normalizer);
	        debug(LMEExact) {
	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", opt_weights);
	        	
	        }
	        hasConverged = (abs(newQValue - lastQValue) <= error) || l2norm(opt_weights) / opt_weights.length > 70;
	        
	        lastQValue = newQValue;
	        
	        
	        
	        iterations ++;
	    } while (! hasConverged && iterations < max_iter);
        
//        writeln("EM Iterations, ", iterations);
        (cast(LinearReward)model.getReward()).setParams(opt_weights);
        
        opt_value = -lastQValue;
		
        return solver.createPolicy(model, solver.solve(model, solverError));
				
	}

	public double calcQ(double [] new_weights, double normalizer) {
		
		double returnval = -log(normalizer);
		
		double [] expectation = mu_E.dup;
		expectation[] *= new_weights[];
		
		returnval += reduce!("a + b")(0.0, expectation);
		
				
		return returnval;
		
	}
	
	double[] getTrajectoryDistribution(double [] weights, out double norm) {
		
		double [] returnval;
		norm = 0;
		
		foreach(i, traj_fe; feature_expectations_per_trajectory) {
			double [] weighted_fe = weights.dup;
			weighted_fe[] *= traj_fe[];
			
			double temp = exp(reduce!("a + b")(0.0, weighted_fe));
			
			norm += temp;
			
			temp *= pr_traj[i];
			
			returnval ~= temp;
			
		}
		
		foreach (ref r; returnval) {
			r /= norm;
		}
		
		return returnval;
	}
	
	    
    double pr_z_y(size_t trajectory_number, double [] traj_distr, double [] denominators) {
    	
    	double numerator = traj_distr[trajectory_number];
    	
    	double denominator = denominators[y_from_trajectory[trajectory_number]];
    	
    	return numerator / denominator;
    }
	
	double [] calc_E_step(double [] traj_distr) {
		
    	double temp [];
    	temp.length = mu_E.length;
    	temp[] = 0;

    	double [] pr_z_denom = pr_z_denominator(traj_distr);
    	
    	foreach(i, pr_zeta; traj_distr) {
   			temp[] += ( pr_z_y(i, traj_distr, pr_z_denom) * feature_expectations_per_trajectory[i][] ) / true_samples.length;
    	}
    	
    	return temp;
	}
	
	void calc_y_and_z(sar[][] true_samples) {
		

		
		// actually, I don't think we should do this, we want repeats to be given higher counts
		/*
		sar [][] Y;
		
		// first check that each sample is not a duplicate
		foreach(traj; true_samples) {
			bool is_new = true;
			foreach(y; Y) {
				
				if (y.length != traj.length)
					break;
				
				bool all_found = true;	
				foreach(i, SAR; y) {
					if (SAR.s != traj[i].s || SAR.a != traj[i].a) {
						all_found = false;
						break;
					}
				}
				
				if (all_found) {
					is_new = false;
					break;
				}
				
			}
			
			if (is_new)
				Y ~= traj.dup;
		}
		*/
		
		// generate one complete trajectory, depth first
		// record Y for this trajectory
		// get features 
		// add to complete list
        LinearReward ff = cast(LinearReward)model.getReward();

        this.sample_length = 0;

		foreach(i, traj; true_samples) {
			
			if (traj.length > this.sample_length)
				this.sample_length = traj.length;
			
			double[State][Action][] storage_stack;
			double[] pr_working_stack;
			double[][] fe_working_stack;
			sar[] traj_working_stack;
			
			storage_stack.length = traj.length;
			
			bool is_visible(State s) {
				foreach(s2; observableStates) {
					if (s2.samePlaceAs(s))
						return true;
				}
				return false;
				
			}
			
			
			// is the first position empty?  If so, intiialize with initial
			if (traj[0].s is null) {
				// add all states in initial, but only if they're not visible
				
				foreach(i_s, pr; initial) {
					
					if (! is_visible(i_s)) {
						if (model.is_terminal(i_s)) {
							storage_stack[0][new NullAction()][i_s] = pr;
						} else {
							foreach(a; model.A(i_s)) {
								storage_stack[0][a][i_s] = pr;
							}	
						}
					}
					
					
				}
				
			} else {
				storage_stack[0][traj[0].a][traj[0].s] = traj[0].p;
			}
			
			sar[] hist;
			long cursor = 0;

			outer_loop: while (true) {
				
				/*
				
				when you push onto the working stacks, check if the next position's storage stack is empty, if so generate entries for it

				each step, check if the pr_traj is zero, if so pop the current working stack and continue
				
				if current pointer is past the end, we have a complete traj, record to output, backup, and pop working stacks
				
				if the current pointer storage stack is empty then backup, popping working stacks as we go until we find a 
				non-empty storage stack and pop or we 	go past the beginning, in which case we're done.
					
				
				idea is to build up the working stacks until we get to the end
				only pop working stacks on backup
				pop storage stack onto working stack
				*/
				
				
				while (cursor < traj.length) {
					
					// backtrack until we find some stack entries
					while (storage_stack[cursor].length == 0) {
//						writeln(cursor);
						cursor --;
						if (cursor < 0) {
							// we're done!
							break outer_loop;
						}
						pr_working_stack.length = pr_working_stack.length - 1;
						fe_working_stack.length = fe_working_stack.length - 1;
						traj_working_stack.length = traj_working_stack.length - 1;		

						
					}
					
					// pop entry off the storage stack or observed traj
					
					// place into working stack
					
					auto first_action = storage_stack[cursor].keys()[0];
					auto first_state = storage_stack[cursor][first_action].keys()[0];
					
					storage_stack[cursor][first_action].remove(first_state);
					if (storage_stack[cursor][first_action].length == 0) {
						storage_stack[cursor].remove(first_action);
					}
					
					auto SAR = sar(first_state, first_action, 1.0);
					traj_working_stack ~= SAR;
					
					
					// update fe and pr
					
					double [] features = ff.features(traj_working_stack[$-1].s, traj_working_stack[$-1].a);
					
					if (cursor == 0) {
						fe_working_stack ~= features;
					} else {
						fe_working_stack ~= fe_working_stack[$-1].dup;
						fe_working_stack[$-1][] += features[];
					}						
					
					double val = 0;
					if (cursor >= 1) {
		        		auto transitions =  model.T(traj_working_stack[$-2].s, traj_working_stack[$-2].a);
		        		
		        		foreach (state, pr; transitions) {
		        			if (traj_working_stack[$-1].s == state) {
		        				val = pr_working_stack[$-1] * pr;
		        				break;
		        			}
		        		}
	        		} else {
	        			val = 1.0; // Should this be initial(s)?
	        		}
	        		
/*	        		if (val == 0) { // prevent noisy trajectories from breaking the feature expectations
	        			val = .0001;
	        		}
	*/        		
	        		pr_working_stack ~= val;
		        	
					// is pr 0? if so, pop working stack and continue
					if (val == 0) {
						// no point in continuing this trajectory
						
						pr_working_stack.length = pr_working_stack.length - 1;
						fe_working_stack.length = fe_working_stack.length - 1;
						traj_working_stack.length = traj_working_stack.length - 1;						 
						
						continue;
						
					}
					
					
					// is the next timestep hidden? if so, generate possible states using the current one and place in storage
					
					if (cursor < traj.length - 1) {
						while (cursor < traj.length - 1 && storage_stack[cursor + 1].length == 0) {

							if (traj[cursor + 1].s ! is null && storage_stack[cursor + 1].length == 0) {
								storage_stack[cursor + 1][traj[cursor + 1].a][traj[cursor + 1].s] = traj[cursor + 1].p;
								
							} else {
								foreach(State newS, double newP; model.T(traj_working_stack[cursor].s, traj_working_stack[cursor].a)) {
									
									if (! is_visible(newS)) {
										if (model.is_terminal(newS)) {
											storage_stack[cursor + 1][new NullAction()][newS] = newP;	
										} else {	
											foreach (Action action; model.A(newS)) {
												storage_stack[cursor + 1][action][newS] = newP;
											}
										}
									}
								
								}
								if ( storage_stack[cursor + 1].length == 0) {
									// Observation error, no possible way the agent wasn't observed (according to our model)
									
									// add every state with equal probability, hopefully not screwing up the feature expectations too much
/*									foreach(tempState; model.S()) {
										storage_stack[cursor + 1][new NullAction()][tempState] = 1.0/model.S().length;
									}*/
									
									auto tempTraj = traj[0..cursor+1];
																		
									if (cursor < traj.length - 2)
										tempTraj ~= traj[cursor + 2 .. $];
									
									true_samples[i] = tempTraj;
									traj = tempTraj;
																		
									this.sample_length = 0;
									foreach(tt; true_samples) {
										if (tt.length > this.sample_length)
											this.sample_length = tt.length;
										
									}
									
								}

							}
						
						} 
						
					}
					debug {
					writeln("traj_working_stack");

					writeln(traj_working_stack);
					writeln(fe_working_stack);
					writeln(pr_working_stack);
					}/* */
					cursor ++;
					
				}
				
				// add trajectory to feature and pr vector 
	        	
	        	y_from_trajectory ~= i;
	        	feature_expectations_per_trajectory ~= fe_working_stack[$-1];
	        	pr_traj ~= pr_working_stack[$-1];
	        	
				// pop working stacks
				pr_working_stack.length = pr_working_stack.length - 1;
				fe_working_stack.length = fe_working_stack.length - 1;
				traj_working_stack.length = traj_working_stack.length - 1;
				
				cursor --;
				 
			}
		}
		
/*		writeln(y_from_trajectory);
		writeln(feature_expectations_per_trajectory);*/
		
	}


    double [] pr_z_denominator(double [] traj_distr) {
    	double [] denominator = new double[true_samples.length];
    	
//    	double norm = 0;

    	denominator[] = 0;		
		foreach (i, pr; traj_distr) {
			denominator[y_from_trajectory[i]] += pr;
		}
    	
//		foreach (d; denominator)
//			norm += d;
			
//		denominator[] /= norm;
		
		return denominator;    	
    }
    
    
    	
}

class LatentMaxEntIrlZiebartExactMultipleAgents : LatentMaxEntIrlZiebartExact {

	Agent [] last_stochastic_policies;
	Model [] models;
	double[State][] initials;
	sar[][][] all_true_samples;
	double[JointStateAction][] e_step_samples;
	double[JointStateAction][] e_step_samples_full;
	double[Action][] equilibrium;
	int interactionLength;
	int [] sample_lengths;
	size_t[] weight_map;
	double [][] init_weights;
 	bool delegate(State, State) is_interacting;
	size_t iterations = 0;

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1, bool delegate(State, State) is_interacting = null) {
		super(max_iter, solver, observableStates, n_samples, error, solverError);
		this.is_interacting = is_interacting;
	}


	public Agent [] solve(Model [] models, double[State][] initials, sar[][][] true_samples, size_t[] sample_lengths, double [][] all_init_weights, double[Action][] NE, int interactionLength, out double opt_value, out double [][] opt_weights) {
		
        this.models = models;
        this.initials = initials;
        this.all_true_samples = true_samples;
        foreach(sample_length; sample_lengths)
        	this.sample_lengths ~= cast(int)sample_length;
        this.equilibrium = NE;
        this.interactionLength = interactionLength;        
        this.init_weights = all_init_weights;
        
        foreach(model; models) {
        	LinearReward r = cast(LinearReward)model.getReward();
			weight_map ~= mu_E.length;
			
        	mu_E.length += r.dim();
        }	
        
        mu_E[] = 0;
        
        if (y_from_trajectory.length == 0)
        	calc_y_and_z(true_samples);		
        
        // reset observableStates so that we consider all features
//        this.observableStates = model.S(); Not necessary anymore
        	
/*        debug {	
        	writeln("FE's ", feature_expectations_per_trajectory);
        }*/	
        
        double [] init_weights;
		init_weights.length = mu_E.length;
		
		foreach (i, init_weight; all_init_weights)
			foreach(i2, t; init_weight)
	            init_weights[i2 + weight_map[i]] = abs(t);
	            
        init_weights[] /= l1norm(init_weights);

        double ignored;
        double [] traj_distr = getTrajectoryDistribution(init_weights, ignored);
        
              
        lastQValue = -double.max;
        bool hasConverged;
        double [] temp_opt_weights = init_weights.dup;
                
        size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;
				
        // loop until convergence
        do {
	        mu_E = calc_E_step(traj_distr);

	        debug {
	        	writeln("mu_E: ", mu_E);
	        }
	        
	        
	        temp_opt_weights = exponentiatedGradient(temp_opt_weights, 0.33, error, max_sample_length);

	        double normalizer;
	    	traj_distr = getTrajectoryDistribution(temp_opt_weights, normalizer);


        	// calculate Q value
        	double newQValue = calcQ(temp_opt_weights, normalizer);
	        debug {
	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", temp_opt_weights);
	        	
	        }
	        hasConverged = (abs(newQValue - lastQValue) <= error);
	        
	        lastQValue = newQValue;
	        
	        
	        iterations ++;	        
	    } while (! hasConverged && iterations < max_iter);
        
        // copy the flat vector of weights into the structured array
        
        opt_value = -lastQValue;
		
        opt_weights.length = all_init_weights.length;
        foreach(k, ref o; opt_weights) {
        	o.length = all_init_weights[k].length;
        }
        auto i = 0;
     	Agent [] returnval = new Agent[models.length];
        foreach(j, ref o; opt_weights) {
        	foreach(ref o2; o) {
        		o2 = temp_opt_weights[i++];
        	}
     	
	        LinearReward r = cast(LinearReward)models[j].getReward();
	        r.setParams(o);
        
            returnval[j] = solver.createPolicy(models[j], solver.solve(models[j], solverError));
             
        }
        
        return returnval;
	}
	
	Agent [] getPoliciesFor(double [] w) {

		// return the feature expectations
		
		double [][] weights;
    	weights.length = init_weights.length;
        foreach(k, ref o; weights) {
        	o.length = init_weights[k].length;
        }
        int i = 0;
        foreach(ref o; weights) {
        	foreach(ref o2; o) {
        		o2 = w[i++];
        	}
 //       	o[] /= l1norm(o);
     	}
    	
    	Agent [] policies;
    	
    	foreach (j, model; models) {
	    	
	    	LinearReward r = cast(LinearReward)model.getReward();
	    	
	    	auto mag_weights = weights[j].dup;
//	    	if (l2norm(mu_E) > 0)
//		    	mag_weights[] *= l2norm(mu_E[weight_map[j] .. weight_map[j] + r.dim()]);
//				mag_weights[] *= sample_lengths[j] / (((max_iter +1) - cast(double)iterations) *((max_iter +1) - cast(double)iterations) );
	    	debug {
//	    		writeln("Mag_weights: ", mag_weights);
	    	}
	    	
	    	r.setParams(mag_weights);
	/*    	r.setParams(weights);*/
	    	
	    	double[StateAction] Q_value = QValueSoftMaxSolve(model, qval_thresh, this.sample_length);        
	        policies ~= CreateStochasticPolicyFromQValue(model, Q_value);
//	    	double[StateAction] Q_value = QValueSolve(model, qval_thresh);		
//			policies ~= CreatePolicyFromQValue(model, Q_value);
        
        }

        last_stochastic_policies = policies; 
       
        return policies;
	}
	
	override double [] ExpectedEdgeFrequencyFeatures(double [] w) {

		Agent [] policies = getPoliciesFor(w);

        double [] returnval = new double[w.length];
        returnval[] = 0;
 
        if (this.n_samples > 0) { 

               double [] total = returnval.dup;
               double [] last_avg = returnval.dup;
               size_t repeats = 0;
               while(true) {
               
                       sar [][][] samples = generate_samples_interaction(models, policies, initials, this.n_samples, sample_lengths, equilibrium, this.interactionLength);
                       foreach (j, model; models) {
                               LinearReward r = cast(LinearReward)model.getReward();
                               auto temp_fe = feature_expectations(model, samples[j]);                             
                                   
                                   total[weight_map[j] .. weight_map[j] + r.dim()] += temp_fe[];
                       }

                       repeats ++;
                       
                       double [] new_avg = total.dup;
                       new_avg[] /= repeats;
                       
                       double max_diff = -double.max;
                       
                       foreach(i2, k; new_avg) {
                               auto tempdiff = abs(k - last_avg[i2]);
                               if (tempdiff > max_diff)
                                       max_diff = tempdiff;
                               
                       } 
                       
                       if (max_diff < 0.5) {
                               debug {
//                                     writeln("mu Converged after ", repeats, " repeats, ", n_samples * repeats, " simulations");
                               }       
                               break;
                               
                       }         
                       
                       last_avg = new_avg;             
                       
               }
               returnval[] = total[] / repeats;
               
        } else {
//             auto Ds = calcStateFreq(policy, initial, model, sample_length);
               throw new Exception("Not Supported");
                       
        }
         
        return returnval;

	}
	
	double [] getFeaturesFor(Agent [] policies, size_t timestep) {
		double [] returnval;
		foreach (i, policy; policies) {
	    	LinearReward ff = cast(LinearReward)models[i].getReward();

			returnval.length += ff.dim();
			
			double [] totalFeatures = new double[ff.dim()];
			totalFeatures[] = 0;
			
			foreach(jsa, jsa_prob; e_step_samples[timestep]) {
				State s = (i == 0) ? jsa.s : jsa.s2;
				State other_agent_state = (i == 0) ? jsa.s2 : jsa.s;
				
				
				if (is_interacting(s, other_agent_state)) {
					foreach(action, prob; equilibrium[i]) {
						double [] f = ff.features(s, action);
						f[] *= prob * jsa_prob * (1.0/ interactionLength);
						totalFeatures[] += f[];
					}
				} else {
					
					foreach(action, prob; policy.actions(s)) {
						double [] f = ff.features(s, action);
						f[] *= prob * jsa_prob;
						totalFeatures[] += f[];
					}
				}
			}
			
			returnval[weight_map[i]..weight_map[i]+ff.dim()] = totalFeatures[];
			
		}
		
		
		return returnval;
		
	}
	
	double [][] full_feature_expectations(Model [] models, double[JointStateAction][] sample) {
		// returns the full joint feature expectations for the given distribition over joint trajectories
		
		double[][] returnval;
		
    	LinearReward [] ff;
    	
    	foreach (model; models)
    	  ff ~= cast(LinearReward)model.getReward();		
		
		foreach (timestep, jsadistr; sample) {
			double [] features = new double[mu_E.length];
			features[] = 0;
			
			foreach(jsa, prob; jsadistr) {
				features[weight_map[0] .. weight_map[0] + ff[0].dim()] += prob * ff[0].features(jsa.s, jsa.a)[] * pow(0.99,timestep);
				features[weight_map[1] .. weight_map[1] + ff[1].dim()] += prob * ff[1].features(jsa.s2, jsa.a2)[]* pow(0.99,timestep);
			}
			returnval ~= features;
		}
		
		
		return returnval;
	}
		
    override double [] feature_expectations(Model model, sar [][] samples) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		double [] f = ff.features(SAR.s, SAR.a);
        		returnval[] += f[];
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}	
    	
	void calc_y_and_z(sar[][][] true_samples, size_t agent_num = 0, sar [] other_trajectory = null, double [] other_features = null, double other_pr = 0, size_t cur_y = 0) {
		throw new Exception("Do not use, does not correctly handle interaction lengths");
		// generate one complete trajectory, depth first
		// record Y for this trajectory
		// get features 
		// add to complete list
        LinearReward ff = cast(LinearReward)models[agent_num].getReward();

        this.sample_length = 0;

		foreach(i, traj; true_samples[agent_num]) {
			
			if (agent_num > 0 && i != cur_y) // we're in a recursive portion, only consider one trajectory
				continue;
			
			if (traj.length > this.sample_length)
				this.sample_length = traj.length;
			
			double[State][Action][] storage_stack;
			double[] pr_working_stack;
			double[][] fe_working_stack;
			sar[] traj_working_stack;
			
			storage_stack.length = traj.length;
			
			bool is_visible(State s) {
				foreach(s2; observableStates) {
					if (s2.samePlaceAs(s))
						return true;
				}
				return false;
				
			}
			
			// is the first position empty?  If so, intiialize with initial
			if (traj[0].s is null) {
				// add all states in initial, but only if they're not visible
				
				foreach(i_s, pr; initial) {
					
					if (! is_visible(i_s)) {
						if (models[agent_num].is_terminal(i_s)) {
							storage_stack[0][new NullAction()][i_s] = pr;
						} else {
							foreach(a; models[agent_num].A(i_s)) {
								storage_stack[0][a][i_s] = pr;
							}	
						}
					}
					
					
				}
				
			} else {
				storage_stack[0][traj[0].a][traj[0].s] = traj[0].p;
			}
			
			sar[] hist;
			long cursor = 0;

			outer_loop: while (true) {
				
				/*
				
				when you push onto the working stacks, check if the next position's storage stack is empty, if so generate entries for it

				each step, check if the pr_traj is zero, if so pop the current working stack and continue
				
				if current pointer is past the end, we have a complete traj, record to output, backup, and pop working stacks
				
				if the current pointer storage stack is empty then backup, popping working stacks as we go until we find a 
				non-empty storage stack and pop or we 	go past the beginning, in which case we're done.
					
				
				idea is to build up the working stacks until we get to the end
				only pop working stacks on backup
				pop storage stack onto working stack
				*/
				
				while (cursor < traj.length) {
					// backtrack until we find some stack entries
					while (storage_stack[cursor].length == 0) {
//						writeln(cursor);
						cursor --;
						if (cursor < 0) {
							// we're done!
							break outer_loop;
						}
						pr_working_stack.length = pr_working_stack.length - 1;
						fe_working_stack.length = fe_working_stack.length - 1;
						traj_working_stack.length = traj_working_stack.length - 1;		

						
					}
					
					// pop entry off the storage stack or observed traj
					
					// place into working stack
					
					auto first_action = storage_stack[cursor].keys()[0];
					auto first_state = storage_stack[cursor][first_action].keys()[0];
					
					storage_stack[cursor][first_action].remove(first_state);
					if (storage_stack[cursor][first_action].length == 0) {
						storage_stack[cursor].remove(first_action);
					}
					
					auto SAR = sar(first_state, first_action, 1.0);
					traj_working_stack ~= SAR;
					
					
					// update fe and pr
					
					double [] features = ff.features(traj_working_stack[$-1].s, traj_working_stack[$-1].a);
					
					if (cursor == 0) {
						fe_working_stack ~= features;
					} else {
						fe_working_stack ~= fe_working_stack[$-1].dup;
						fe_working_stack[$-1][] += features[];
					}						
					
					double val = 0;
					if (cursor >= 1) {
		        		auto transitions =  models[agent_num].T(traj_working_stack[$-2].s, traj_working_stack[$-2].a);
		        		
		        		foreach (state, pr; transitions) {
		        			if (traj_working_stack[$-1].s == state) {
		        				val = pr_working_stack[$-1] * pr;
		        				break;
		        			}
		        		}
	        		} else {
	        			val = 1.0; // Should this be initial(s)?
	        		}
	        		
	        		if (other_trajectory !is null) {
	        			// check if this timestep's state interacts with the other agent
	        			// if so, check if the given action is the one specified by the NE for BOTH agents
	        			// if either of them is wrong, val = 0
	        			
	        			if (is_interacting(SAR.s, other_trajectory[cursor].s)) {
	        				if (equilibrium[0].get(other_trajectory[cursor].a, -1) <= 0 ||
	        					equilibrium[1].get(SAR.a, -1) <= 0)
	        					val = 0;
	        			} 
	        			
	        			
	        		}
	        		
	        		if (val == 0) { // prevent noisy trajectories from breaking the feature expectations
	        			val = .0001;
	        		}
	        		
	        		pr_working_stack ~= val;
		        	
					// is pr 0? if so, pop working stack and continue
					if (val == 0) {
						// no point in continuing this trajectory
						
						pr_working_stack.length = pr_working_stack.length - 1;
						fe_working_stack.length = fe_working_stack.length - 1;
						traj_working_stack.length = traj_working_stack.length - 1;						 
						
						continue;
						
					}
					
					
					// is the next timestep hidden? if so, generate possible states using the current one and place in storage
					
					if (cursor < traj.length - 1) {
						while (cursor < traj.length - 1 && storage_stack[cursor + 1].length == 0) {

							if (traj[cursor + 1].s ! is null && storage_stack[cursor + 1].length == 0) {
								storage_stack[cursor + 1][traj[cursor + 1].a][traj[cursor + 1].s] = traj[cursor + 1].p;
								
							} else {
								foreach(State newS, double newP; models[agent_num].T(traj_working_stack[cursor].s, traj_working_stack[cursor].a)) {
									
									if (! is_visible(newS)) {
										if (models[agent_num].is_terminal(newS)) {
											storage_stack[cursor + 1][new NullAction()][newS] = newP;	
										} else {	
											foreach (Action action; models[agent_num].A(newS)) {
												storage_stack[cursor + 1][action][newS] = newP;
											}
										}
									}
								
								}
								if ( storage_stack[cursor + 1].length == 0) {
									// Observation error, no possible way the agent wasn't observed (according to our model)
									
									// add every state with equal probability, hopefully not screwing up the feature expectations too much
/*									foreach(tempState; model.S()) {
										storage_stack[cursor + 1][new NullAction()][tempState] = 1.0/model.S().length;
									}*/
									
									auto tempTraj = traj[0..cursor+1];
																		
									if (cursor < traj.length - 2)
										tempTraj ~= traj[cursor + 2 .. $];
									
									true_samples[agent_num][i] = tempTraj;
									traj = tempTraj;
																		
									this.sample_length = 0;
									foreach(tt; true_samples) {
										if (tt.length > this.sample_length)
											this.sample_length = tt.length;
										
									}
									
								}

							}
						
						} 
						
					}
					/*
					writeln(traj_working_stack);
					writeln(fe_working_stack);
					writeln(pr_working_stack);*/
					cursor ++;
					
				}
				
				
				// now do the same for the other trajectory
				if (other_trajectory is null) {
					calc_y_and_z(true_samples, agent_num + 1, traj_working_stack, fe_working_stack[$-1], pr_working_stack[$-1], i);
				} else {
				
					// add trajectory to feature and pr vector 
		        	
		        	y_from_trajectory ~= cur_y;
		        	
		        	double [] full_fe = new double[other_features.length + fe_working_stack[$-1].length];
		        	full_fe[0..other_features.length] = other_features[];
		        	full_fe[other_features.length .. $] = fe_working_stack[$-1][];
		        	feature_expectations_per_trajectory ~= full_fe;
		        	
		        	pr_traj ~= pr_working_stack[$-1] * other_pr;
	        	}
				
				// pop working stacks
				pr_working_stack.length = pr_working_stack.length - 1;
				fe_working_stack.length = fe_working_stack.length - 1;
				traj_working_stack.length = traj_working_stack.length - 1;
				
				cursor --;
				 
			}
		}
		
/*		writeln(y_from_trajectory);
		writeln(feature_expectations_per_trajectory);*/
		
	}	
	
	override double [] calc_E_step(double [] traj_distr) {
		
    	double temp [];
    	temp.length = mu_E.length;
    	temp[] = 0;

    	double [] pr_z_denom = pr_z_denominator(traj_distr);
    	
    	foreach(i, pr_zeta; traj_distr) {
   			temp[] += ( pr_z_y(i, traj_distr, pr_z_denom) * feature_expectations_per_trajectory[i][] ) / all_true_samples[0].length;
    	}
    	
    	return temp;
	}	

    override double [] pr_z_denominator(double [] traj_distr) {
    	double [] denominator = new double[all_true_samples[0].length];
    	
    	denominator[] = 0;		
		foreach (i, pr; traj_distr) {
			denominator[y_from_trajectory[i]] += pr;
		}
		
		return denominator;    	
    }	
    
	
	double [] unconstrainedAdaptiveExponentiatedGradient( double [][] expert_features, double nu, size_t iter_limit) {
				
		double [] beta = new double[expert_features[0].length];
		beta[] = - log(beta.length);
		
		double diff;
		size_t iterations = 0;
		double[] zs = new double[beta.length];
		zs[] = 0;
		size_t z_count = 0;
		double [] last_w = new double[beta.length];
		last_w[] = 0;
		
		size_t t = 0;
		size_t block_counter = 0;
		int block_size = 10;
		
		Agent  [] policies;
		double gamma = 0.9995;
		
		while(iterations < iter_limit) {
			
			double [] m = zs.dup;
			if (z_count > 0)
				m[] /= z_count;
			
//			writeln("Beta, m ", beta, " : ", m); 

			double [] w = new double[beta.length];
			foreach(i; 0..w.length) {
				w[i] = exp(beta[i] - nu*m[i]);
			}
			
			// allow negative weights by subtracting the average
			double max = 0;
			foreach(w_i; w)
				if (w_i > max)
					max = w_i;
			w[] -= max / 2;	

			// use function to get our expected feature vector
			
			if (block_counter > block_size || policies.length == 0) {
				policies = getPoliciesFor(w);
				block_counter = 0;
			}
			
//			auto r = uniform(0.0, .999);		
//			size_t timestep = cast(size_t)(r * expert_features.length);
			
			double [] z = getFeaturesFor(policies, t);
			z[] -= expert_features[t][];
//			zs[] *= gamma*zs[];
//			zs[] += (1.0 - gamma)*z[];
			zs[] += z[];
			z_count ++;
			debug {
//				writeln("GRADIENT: ", z);
//				writeln(/*z[0], " - ",*/ w[0]);
			}
						
			foreach(i; 0..beta.length) {
				beta[i] = beta[i] - nu*z[i] - nu*nu*(z[i] - m[i])*(z[i] - m[i]);
			}			
			
			double [] test = w.dup;
			test[] -= last_w[];
			diff = l2norm(test);

			debug {
//				writeln(z_count, " : ", diff, " -> ", w);
			}
/*			double max = -double.infinity;
			foreach(z_i; z) {
				if (abs(z_i) > max)
					max = abs(z_i);
			}*/
			
			
			last_w = w;
			block_counter ++;
			t = (t + 1) % expert_features.length ;
			if (t == 0) {
				iterations ++;
				nu /= 1.04;
			}
			if(iterations >= iter_limit) {
				return w;
			} 
		} 
		
		return null;
	}    
}

class RunningAverageConvergenceThreshold {
	
	double [][] storage;
	size_t total_count;
	size_t front;
	
	double [] last;
	double threshold;
	
	public this(double threshold, size_t length) {
		storage.length = length;
		total_count = 0;
		front = 0;
		this.threshold = threshold;
		last = null;
	}
	
	bool hasAllConverged(double [] newData) {
		
		bool returnval = true;
		
		if (last !is null) {
			last[] -= newData[];
			
			storage[front] = last;

			front = (front + 1) % storage.length;
			
			total_count ++;
			
			if (total_count < storage.length) // not enough data points yet, we haven't converged
				return false;
				
			foreach(i; 0 .. storage[0].length) {
				double total = 0;
				foreach ( s; storage) {
					total += abs(s[i]) / storage.length;
				}
				
				if (total > threshold) {
					returnval = false;
					break;
				}	
			}
		} else {
			returnval = false;
		}	
		
		last = newData.dup;	
		
		return returnval;

	}
	
	
}
	
	
class LatentMaxEntIrlZiebartApproxMultipleAgents : LatentMaxEntIrlZiebartExactMultipleAgents {

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, 
		double error=0.1, double solverError =0.1, bool delegate(State, State) is_interacting = null) {
		super(max_iter, solver, observableStates, n_samples, error, solverError, is_interacting);
	}

	override public Agent [] solve(Model [] models, double[State][] initials, sar[][][] true_samples, 
		size_t[] sample_lengths, double [][] all_init_weights, double[Action][] NE, int interactionLength, 
		out double opt_value, out double [][] opt_weights) {
        this.models = models;
        this.initials = initials;
        this.all_true_samples = true_samples;
        foreach(sample_length; sample_lengths)
            this.sample_lengths ~= cast(int)sample_length;
        this.equilibrium = NE;
        this.interactionLength = interactionLength;
        this.init_weights = all_init_weights;

        foreach(model; models) {
            LinearReward r = cast(LinearReward)model.getReward();
            weight_map ~= mu_E.length;

            mu_E.length += r.dim();
        }



        /*        debug {
            writeln("FE's ", feature_expectations_per_trajectory);
        }*/

        double [] init_weights;
        init_weights.length = mu_E.length;

        foreach (i, init_weight; all_init_weights)
            foreach(i2, t; init_weight)
                init_weights[i2 + weight_map[i]] = t;

        //        init_weights[] /= l1norm(init_weights);


        lastQValue = -double.max;
        bool hasConverged;
        double [] temp_opt_weights = init_weights.dup;
        double [] last_temp_opt_weights = init_weights.dup;

        size_t max_sample_length = 0;
        foreach(sl; sample_lengths)
            if (sl > max_sample_length)
                max_sample_length = sl;

        // loop until convergence
        do {
        last_stochastic_policies = getPoliciesFor(temp_opt_weights);

        mu_E[] = 0;
        double [][] expert_features = calc_E_step(true_samples);

        //hack for compairons with I2RL because other code not working
        foreach (muE_step; expert_features) {
            mu_E[] += muE_step[];
        }

        debug {
                writeln("mu_E: ", expert_features);
            }

            temp_opt_weights = exponentiatedGradient(temp_opt_weights.dup, 0.25, error, max_sample_length);
        //	        temp_opt_weights = unconstrainedAdaptiveExponentiatedGradient(expert_features, .05, 10000 / max_sample_length);//.05, 10000 / max_sample_length);

        //			need to approximate the Q value now, similar to how the dual is approximated
        //		This is done by integrating the objective with respect to the weights, it should give the original dual


            // calculate Q value
            double newQValue = calcQ(temp_opt_weights);
        double [] test = temp_opt_weights.dup;
        test[] -= last_temp_opt_weights[];

            debug {
            writeln(" w diff lme em :",l2norm(test));
                writeln("Q(", iterations, ") = ", newQValue, " For weights: ", temp_opt_weights);

            }
            hasConverged = (abs(newQValue - lastQValue) <= error);
            last_temp_opt_weights = temp_opt_weights.dup;
            lastQValue = newQValue;

            iterations ++;
        } while (! hasConverged && iterations < max_iter);

        // copy the flat vector of weights into the structured array

        opt_value = -lastQValue;

        opt_weights.length = all_init_weights.length;
        foreach(k, ref o; opt_weights) {
            o.length = all_init_weights[k].length;
        }
        auto i = 0;
        Agent [] returnval = new Agent[models.length];
        foreach(j, ref o; opt_weights) {
            foreach(ref o2; o) {
                o2 = temp_opt_weights[i++];
            }

            LinearReward r = cast(LinearReward)models[j].getReward();
            r.setParams(o);

            returnval[j] = solver.createPolicy(models[j], solver.solve(models[j], solverError));

        }

        return returnval;
        }

	public Agent [] solve2(Model [] models, double[State][] initials, sar[][][] true_samples,
	size_t[] sample_lengths, double [][] all_init_weights, double[Action][] NE, int interactionLength,
	out double opt_value, out double [][] opt_weights, ref double [][] featureExpecExpert,
	int num_Trajsofar, sar[][][] fullsamples, ref double [][] featureExpecExpertfull) {

        this.models = models;
        this.initials = initials;
        this.all_true_samples = true_samples;
        foreach(sample_length; sample_lengths)
        	this.sample_lengths ~= cast(int)sample_length;
        this.equilibrium = NE;
        this.interactionLength = interactionLength;
        this.init_weights = all_init_weights;
        
        foreach(model; models) {
        	LinearReward r = cast(LinearReward)model.getReward();
			weight_map ~= mu_E.length;
			
        	mu_E.length += r.dim();
        }	

/*        debug {	
        	writeln("FE's ", feature_expectations_per_trajectory);
        }*/	
        
        double [] init_weights;
		init_weights.length = mu_E.length;
		
		foreach (i, init_weight; all_init_weights)
			foreach(i2, t; init_weight)
	            init_weights[i2 + weight_map[i]] = t;
	            
//        init_weights[] /= l1norm(init_weights);

        //Convert double [][] featureExpecExpert to double [] mu_Eprev
        double num_Trajsofard = cast(double)num_Trajsofar;
        double [] mu_Eprev = new double[mu_E.length];
        foreach (i, arr; featureExpecExpert)
            foreach(i2, t; arr)
                mu_Eprev[i2 + weight_map[i]] = t;

        debug {
        writeln("mu_Eprev: ",mu_Eprev);
        }



        lastQValue = -double.max;
        bool hasConverged;
        double [] temp_opt_weights = init_weights.dup;
	    double [] last_temp_opt_weights = init_weights.dup;


        size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;
				
        // loop until convergence
        do {
            last_stochastic_policies = getPoliciesFor(temp_opt_weights);

            mu_E[] = 0;
            double [][] expert_features = calc_E_step(true_samples);

            //hack for compairons with I2RL because other code not working
            foreach (muE_onestep; expert_features) {
                mu_E[] += muE_onestep[];
            }

            debug {
	        	writeln("mu_E: ", mu_E);
	        }

            //computing new target mu_E specific to current iteration
            foreach (int i, double val; mu_E)
               mu_E[i] = (val+mu_Eprev[i]*num_Trajsofard)/(num_Trajsofard+1);//val;
               //updating muE doesn't give better learning curve

            debug {
	        	writeln("mu_E: ", mu_E);
	        }

	        temp_opt_weights = exponentiatedGradient(temp_opt_weights.dup, 0.25, error, max_sample_length);
//	        temp_opt_weights = unconstrainedAdaptiveExponentiatedGradient(expert_features, .05, 10000 / max_sample_length);//.05, 10000 / max_sample_length);

//			need to approximate the Q value now, similar to how the dual is approximated
	//		This is done by integrating the objective with respect to the weights, it should give the original dual 


        	// calculate Q value
        	double newQValue = calcQ(temp_opt_weights);
		    double [] test = temp_opt_weights.dup;
		    test[] -= last_temp_opt_weights[];

	        debug {
				writeln(" w diff lme em :",l2norm(test));
	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", temp_opt_weights);
	        	
	        }
	        hasConverged = (abs(newQValue - lastQValue) <= error);
	        last_temp_opt_weights = temp_opt_weights.dup;
	        lastQValue = newQValue;

	        iterations ++;
	        //writeln("hasConverged",hasConverged,"iterations",iterations);
	    } while (! hasConverged && iterations < max_iter);
        
        // copy the flat vector of weights into the structured array
        
        opt_value = -lastQValue;

        //sar [][] working_sample;
        //
        //foreach(agent_num, agent_traj; fullsamples) {
        //    working_sample ~= agent_traj[0].dup;
        //}
        //
        //JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);
        //
        //e_step_samples_full.length = curJSA.length;
        //foreach(int step, JointStateAction jsa; curJSA) {
        //    e_step_samples_full[step][jsa] = 1;
        //}
        //
        //// normalize e_step_samples_full
        //foreach (ref timestep; e_step_samples_full) {
    		//double total = 0;
    		//foreach(jsa, val; timestep) {
    		//	total += val;
    		//}
    		//foreach(jsa, ref val; timestep) {
    		//	val /= total;
    		//}
        //}
        //writeln("e_step_samples_full done",e_step_samples_full);
        //
        //double [][] expert_featuresfull =
        //LatentMaxEntIrlZiebartExactMultipleAgents.full_feature_expectations(models, e_step_samples_full);
        //
        double [] mu_E_full = new double[mu_E.length];
        //mu_E_full[] = 0;
        //foreach (mu_E_full_onestep; expert_featuresfull) {
        //    mu_E_full[] += mu_E_full_onestep[];
        //}
        //
        //writeln("mu_E_full done");
        //
        auto i = 0;
        // save feature expectations for learned policy
        mu_E_full =  ExpectedEdgeFrequencyFeatures(temp_opt_weights.dup);
        mu_E_full[] /= max_sample_length;
        foreach(j, ref o; featureExpecExpertfull) {
            foreach(ref o2; o) {
                o2 = mu_E_full[i++];
            }
        }

        opt_weights.length = all_init_weights.length;
        foreach(k, ref o; opt_weights) {
        	o.length = all_init_weights[k].length;
        }

     	Agent [] returnval = new Agent[models.length];
        foreach(j, ref o; opt_weights) {
	        i = 0;
        	foreach(ref o2; o) {
        		o2 = temp_opt_weights[i++];
        	}
     	
	        LinearReward r = cast(LinearReward)models[j].getReward();
	        r.setParams(o);
        
            returnval[j] = solver.createPolicy(models[j], solver.solve(models[j], solverError));
            //writeln("returnval[j] = solver.createPolicy done");

        }

        mu_E[] /= max_sample_length;
        foreach(j, ref o; featureExpecExpert) {
	        i = 0;
			foreach(ref o2; o) {
			 o2 = mu_E[i++];
			}
        }
        //featureExpecExpertfull = featureExpecExpert.dup;
        return returnval;
	}	
	



	double calcQ(double [] weights) {
		auto features = ExpectedEdgeFrequencyFeatures(weights);
		
		features[] *= weights[];
				
		double returnval = -1 * reduce!("a + b")(0.0, features);
		
		debug {
//			writeln("Q: log(Z) ", returnval, " for Features ", features);
			
		}
		
		double [] expectation = mu_E.dup;
		expectation[] *= weights[];
		
		returnval += reduce!("a + b")(0.0, expectation);
		
				
		return returnval;
		
		
	}

    double [][] calc_E_step(sar[][][] true_samples) {
		return calc_E_step_internal(true_samples);
	}
	    	
    double [][] calc_E_step_internal(sar[][][] true_samples, int max_repeats = 250) {
    	

    	// using the last policy we have and the list of Y's, generate a new set of expert feature expectations, using gibbs sampling
    	// to generate trajectory samples, 
    	
    	double [] returnval = new double[mu_E.length];
    	returnval[] = 0;
    	
    	size_t repeats = 0;
    	double [] last_avg = returnval.dup;
    	last_avg[] = 0;
    	
    	e_step_samples.length = 0;
    	
    	/*
    	import boydmdp;
    	
    	Reward savedReward1 = models[0].getReward();
    	Reward savedReward2 = models[1].getReward();
    	  
    	
    	LinearReward reward = new Boyd2RewardGroupedFeatures(models[0]);
		double [6]  reward_weights = [1, -1, -1, -1, 0.5, -1];
		reward.setParams(reward_weights);
		
		models[0].setReward(reward);
		models[1].setReward(reward);
		
			
		ValueIteration vi = new ValueIteration();
		Agent [] policies;
		
		foreach(model; models) {
			double[State] V = vi.solve(model, .1);
	
			policies ~= vi.createPolicy(model, V);
			
		}
		last_stochastic_policies = policies;

		models[0].setReward(savedReward1);
		models[1].setReward(savedReward2);
    	*/
    	
    	while(true) {
    		
			double [] temp = new double[mu_E.length];
    		temp[] = 0;
	   		foreach(i, sample; true_samples[0]) {
	    		
		    	RunningAverageConvergenceThreshold convergenceTest = new RunningAverageConvergenceThreshold(0.005, 20);
	    		
	    		temp[] += gibbs_sampler(true_samples, models, last_stochastic_policies, i, cast(size_t)(4 * sample.length * .1), convergenceTest)[];
	    	}
	    	
	    	repeats ++;
	    	temp[] /= true_samples[0].length;
	    	
	    	returnval[] += temp[];
	    	
	    	double [] new_avg = returnval.dup;
	    	new_avg[] /= repeats;
	    	
	    	
	    	double max_diff = -double.max;
	    	
	    	foreach(i, k; new_avg) {
	    		auto tempdiff = abs(k - last_avg[i]);
	    		if (tempdiff > max_diff)
	    			max_diff = tempdiff;
	    		
	    	}
			debug {
		    	writeln(repeats, " repeats, ", max_diff, "max_diff ");
			}
	    	if (max_diff < 0.75 || repeats > max_repeats) {
	    		debug {
	    			writeln("Converged after ", repeats, " repeats, ", new_avg);
	    		}	
	    		break;
	    		
	    	}
//	    	if (repeats > 30) 
//	    		break;
	    	
	    	last_avg = new_avg;
	    	
    	}
    	
    	// normalize e_step_samples
    	
    	foreach (ref timestep; e_step_samples) {
    		double total = 0;
    		foreach(jsa, val; timestep) {
    			total += val;
    		}
    		foreach(jsa, ref val; timestep) {
    			val /= total;
    		}
    	}
    	return full_feature_expectations(models, e_step_samples);
    }
    	
	double Tin(Model model, State si, Action ai, State si_prime, bool interacting) {
		if (si_prime is null || si is null || ai is null)
			return 1.0;
			
		double transProb = model.T(si, ai).get(si_prime, 0.00000001);
/*		if (transProb == 0) {
			// this causes problems when used with agents that don't exactly match the transition model
			// (such as the robots)
			// so what we will do is return a slight non-zero transition probability related to how close si_prime is to the intended next state
			import boydmdp;
			
			BoydState intendedState = cast(BoydState)ai.apply(si);
			BoydState bs_s_prime = cast(BoydState)si_prime;
			
			transProb = 0.0001 * (1.0 / intendedState.distanceTo(bs_s_prime));
		}*/
		
		double interactionProb = 1.0/interactionLength; 
		
		if (! interacting)
			return transProb;
			
		if (si == si_prime) {
			return (1.0 - interactionProb) + interactionProb*transProb;
			
		} else {
			return interactionProb*transProb;
			
		}
		
	}
	
	JointStateAction [] workingSampleToJSAArray(sar[][] working_sample) {
		JointStateAction [] returnval;
		
		foreach(i;0..working_sample[0].length) {
			JointStateAction temp = new JointStateAction(working_sample[0][i].s, working_sample[0][i].a, working_sample[1][i].s, working_sample[1][i].a);
			returnval ~= temp;
		}
		
		
		return returnval;
	}
	
    double [] gibbs_sampler(sar [][][] sample, Model [] models, Agent [] policies, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest) {
    	
    	// when samping a state, we need to also consider that only unobservable states can be selected from
    	// if this results in no feasible samples, then delete the problem part of the sample and continue
		// convergenceTest.threshold /= 100;
    	
    	double[State] occluded_states;
    	
    	foreach(s; models[0].S()) {
    		occluded_states[s] = 1.0;
    	}
    	
    	foreach(s; observableStates) {
    		occluded_states.remove(s);
    	}
    	
    	if (occluded_states.length != 0) {
	    	Distr!State.normalize(occluded_states);
	    	occluded_states.rehash;
	    			
    	}
    	
    	LinearReward [] ff;
    	
    	foreach (model; models)
    	  ff ~= cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[mu_E.length];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		size_t[] missing_timesteps_map;
		
		
		double [] min_features = new double[mu_E.length];
		min_features[] = 0;
		
		foreach(agent_num, agent_traj; sample) {
			missing_timesteps_map ~= missing_timesteps.length;
			foreach(i, SAR; agent_traj[Y_num]) {
				if (SAR.s is null) {
					missing_timesteps ~= i;
				} else {
					min_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(SAR.s, SAR.a)[];
				}
			}
		}
		size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;
				
		e_step_samples.length = max_sample_length; 
		
		M = cast(size_t)(0.1 * missing_timesteps.length * 2); 

		sar[][] working_sample;
		
		foreach(agent_num, agent_traj; sample) {
			working_sample ~= agent_traj[Y_num].dup;
		}
		
		if (missing_timesteps.length == 0) {
			// Z is empty, don't need sampling
			
			JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);

			foreach(timestep_1, jsa; curJSA) {
				e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
			}
			
			return min_features;
			
		}
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(i, idx; missing_timesteps) {
			size_t agent_num = int.max;
			foreach (j, agent_idx; missing_timesteps_map)
				if (i >= agent_idx)
					agent_num = j;
					
/*			if (idx > 0)
				working_sample[agent_num][idx].s = Distr!State.sample(models[agent_num].T(working_sample[agent_num][idx-1].s, working_sample[agent_num][idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = this.initial.dup;
				foreach(s, ref pr; initial_occluded)
					pr *= occluded_states.get(s, 0);
				
				// guard against empty initial distr
				double total = 0;
		
				foreach (key, val ; initial_occluded) {
					total += val;
				} 	
				
				if (total == 0) {
					// no valid starting states, just use the occluded ones
					
					if (occluded_states.length == 0) { 
						// full observability, but still missing some observations.  Fill with the initial state distribution
						occluded_states = initials[agent_num].dup;
					}
				
					initial_occluded = occluded_states.dup;
					
					
				}
				
				Distr!State.normalize(initial_occluded);	
					 
				working_sample[agent_num][idx].s = Distr!State.sample(initial_occluded);
			}	*/
			
			if (occluded_states.length == 0) { 
				// full observability, but still missing some observations.  Fill with the initial state distribution
				occluded_states = initials[agent_num].dup;
			}
			
			working_sample[agent_num][idx].s = Distr!State.sample(occluded_states);
			working_sample[agent_num][idx].a = policies[agent_num].sample(working_sample[agent_num][idx].s);
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][idx].s, working_sample[agent_num][idx].a)[];
		}
		
		// then loop for N times, generating a new sample, calculating feature expectations, saving them for later
		
		JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);
		
		double[] total_features = min_features.dup;
		total_features[] = 0;
		
		
/*		if (N < 10000)
			N = 10000; 
		
		if (N > 400000)
			N = 400000;*/
		
		size_t i = 0;
		size_t j1 = 0;

		size_t r_sequential = 0;
		
//		foreach(i; 0..(N + M)) {
		while (true) {
			
			auto r = uniform(0.0, .999);
			
			size_t timestep = missing_timesteps[cast(size_t)(r * missing_timesteps.length)];
			// r_sequential = (r_sequential+1) % missing_timesteps.length;

			// size_t timestep = missing_timesteps[r_sequential];

			size_t agent_num = int.max;
			foreach (j, agent_idx; missing_timesteps_map)
				if (timestep >= agent_idx)
					agent_num = j;
			
			auto other_agent_num = agent_num + 1;
			if (other_agent_num >= models.length)
				other_agent_num = 0;
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] -= ff[agent_num].features(working_sample[agent_num][timestep].s, working_sample[agent_num][timestep].a)[];
			
			if (uniform(0.0, .999) < 0.5) {
				// sample the state from T(s, a, s') T(s'', a', s''') Pr(a|s)
                debug {
                    writeln("gibbs sampling: sample the state from T(s, a, s') T(s'', a', s''') Pr(a|s) ");
                }
				double[State] state_distr = occluded_states.dup;
				
				foreach(s, ref v; state_distr) {

					// if this agent is interacting with any others, use it's nash equilbrium, otherwise use it's policy
					
					bool interacting = is_interacting(working_sample[other_agent_num][timestep].s, s);
					
					if (interacting) {
						v *= equilibrium[agent_num].get(working_sample[agent_num][timestep].a, 0.00000001);
						v *= equilibrium[other_agent_num].get(working_sample[other_agent_num][timestep].a, 0.00000001);
					} else {
						v *= policies[agent_num].actions(s).get(working_sample[agent_num][timestep].a, 0.00000001);
						v *= policies[other_agent_num].actions(working_sample[other_agent_num][timestep].s).get(working_sample[other_agent_num][timestep].a, 0.00000001);
					}
						
					if (timestep > 0) {
						v *= Tin(models[agent_num], working_sample[agent_num][timestep-1].s, working_sample[agent_num][timestep-1].a, s, interacting); 						
					}
					if (timestep < working_sample.length - 1) {
						v *= Tin(models[agent_num], s, working_sample[agent_num][timestep].a, working_sample[agent_num][timestep+1].s, interacting); 
						v *= Tin(models[other_agent_num], working_sample[other_agent_num][timestep].s, working_sample[other_agent_num][timestep].a, working_sample[other_agent_num][timestep+1].s, interacting); 
					}
				}
				
				try {
					Distr!State.normalize(state_distr);
				} catch (Exception e) {
					state_distr = occluded_states.dup;
				}

				working_sample[agent_num][timestep].s = Distr!State.sample(state_distr);
				
				if (agent_num == 0) 
					curJSA[timestep].s = working_sample[agent_num][timestep].s;
				else
					curJSA[timestep].s2 = working_sample[agent_num][timestep].s;
				
				
			} else {
				// sample the action from T(s, a, s') Pr(a|s)
                debug {
                    writeln("gibbs sampling: sample the action from T(s, a, s') Pr(a|s)");
                }

				// if this agent is interacting with any others, use it's nash equilbrium, otherwise use it's policy
				
				bool interacting = false;
				foreach(agent_id, SAR; working_sample) {
					if (agent_id == agent_num)
						continue;
						
					if (is_interacting(SAR[timestep].s, working_sample[agent_num][timestep].s)) { 
						interacting = true;
						break;
					}		
				}
				
				double[Action] action_distr;
				if (interacting)
					action_distr = equilibrium[agent_num].dup;
				else
					action_distr = policies[agent_num].actions(working_sample[agent_num][timestep].s).dup;
				
				foreach(a, ref v; action_distr) {
					
					if (timestep < working_sample.length - 1) {
						v *= Tin(models[agent_num], working_sample[agent_num][timestep].s, a, working_sample[agent_num][timestep+1].s, interacting); 
					}
				}
				
				Distr!Action.normalize(action_distr);
				
				working_sample[agent_num][timestep].a = Distr!Action.sample(action_distr);
				
				if (agent_num == 0) 
					curJSA[timestep].a = working_sample[agent_num][timestep].a;
				else
					curJSA[timestep].a2 = working_sample[agent_num][timestep].a;
				
			}
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][timestep].s, working_sample[agent_num][timestep].a)[];
			
			
			if (j1 > M) {
				total_features[] += cur_features[];
				foreach(timestep_1, jsa; curJSA) {
					e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
				}
				
				i ++;
				auto temp = total_features.dup;
				temp[] /= i;
				debug {
					
//					writeln(i, " : ", cur_features, " -> ", temp);
				}			
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					break;
				}
				j1 = 0;
			}	
			
			j1 ++;
		}
		
		
		// now average the feature expectations
		total_features[] /= i;
		

		return total_features;
    	
    }

	// blocked gibbs sampler, with a block being one state/action pair of a given agent's timestep
	// sampling goes forward to backward through the trajectories
    double [] blocked_gibbs_sampler(sar [][][] sample, Model [] models, Agent [] policies, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest) {
    	
    	// when samping a state, we need to also consider that only unobservable states can be selected from
    	// if this results in no feasible samples, then delete the problem part of the sample and continue
//    	convergenceTest.threshold /= 100;
    	M /= 2;
    	
    	
    	double[State] occluded_states;
    	
    	foreach(s; models[0].S()) {
    		occluded_states[s] = 1.0;
    	}
    	
    	foreach(s; observableStates) {
    		occluded_states.remove(s);
    	}
    	
    	if (occluded_states.length != 0) {
	    	Distr!State.normalize(occluded_states);
	    	occluded_states.rehash;
	    			
    	}
    	
    	LinearReward [] ff;
    	
    	foreach (model; models)
    	  ff ~= cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[mu_E.length];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		size_t[] missing_timesteps_map;
		
		
		double [] min_features = new double[mu_E.length];
		min_features[] = 0;
		
		size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;

		e_step_samples.length = max_sample_length; 

		foreach(i; 0 .. max_sample_length) {
			foreach(agent_num, agent_traj; sample) {
				
				if (i < agent_traj[Y_num].length) {
					if (agent_traj[Y_num][i].s is null) {
						missing_timesteps ~= i;
						missing_timesteps_map ~= agent_num;
					} else {
						min_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(agent_traj[Y_num][i].s, agent_traj[Y_num][i].a)[];
					}
				}
				
			}
		}
		
		sar[][] working_sample;
		
		foreach(agent_num, agent_traj; sample) {
			working_sample ~= agent_traj[Y_num].dup;
		}
		
		
		if (missing_timesteps.length == 0) {
			// Z is empty, don't need sampling

			JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);

			foreach(timestep_1, jsa; curJSA) {
				e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
			}
			
			return min_features;
			
		} 
		M = cast(size_t)(0.1 * missing_timesteps.length);
		
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(i, idx; missing_timesteps) {
			size_t agent_num = missing_timesteps_map[i];
					
/*			if (idx > 0)
				working_sample[agent_num][idx].s = Distr!State.sample(models[agent_num].T(working_sample[agent_num][idx-1].s, working_sample[agent_num][idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = this.initial.dup;
				foreach(s, ref pr; initial_occluded)
					pr *= occluded_states.get(s, 0);
				
				// guard against empty initial distr
				double total = 0;
		
				foreach (key, val ; initial_occluded) {
					total += val;
				} 	
				
				if (total == 0) {
					// no valid starting states, just use the occluded ones
										
					if (occluded_states.length == 0) { 
						// full observability, but still missing some observations.  Fill with the initial state distribution
						occluded_states = initials[agent_num].dup;
					}
					
					initial_occluded = occluded_states.dup;
					
				}
				
				Distr!State.normalize(initial_occluded);	
					 
				working_sample[agent_num][idx].s = Distr!State.sample(initial_occluded);
			}*/	

			if (occluded_states.length == 0) { 
				// full observability, but still missing some observations.  Fill with the initial state distribution
				occluded_states = initials[agent_num].dup;
			}
				
			working_sample[agent_num][idx].s = Distr!State.sample(occluded_states);
			working_sample[agent_num][idx].a = policies[agent_num].sample(working_sample[agent_num][idx].s);
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][idx].s, working_sample[agent_num][idx].a)[];
		}
		
		// then loop for N times, generating a new sample, calculating feature expectations, saving them for later
		 
		JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);
 
		double[] total_features = min_features.dup;
		total_features[] = 0;
		
		
/*		if (N < 10000)
			N = 10000; 
		
		if (N > 400000)
			N = 400000;*/
		
		size_t i = 0;
		size_t j = 0;

		int step = 1;
		long timestep_idx = -1;
				
//		foreach(i; 0..(N + M)) {
		while (true) {
						
			timestep_idx += step;
			
			if (timestep_idx >= missing_timesteps.length) {
				step = -1;
				timestep_idx = missing_timesteps.length - 1;
			}
			
			if (timestep_idx < 0) {
				step = 1;
				timestep_idx = 0;
			}
			

/*			auto r = uniform(0.0, .999);
			timestep_idx = cast(size_t)(r * missing_timesteps.length);*/
			
			auto timestep = missing_timesteps[timestep_idx];			
			
			auto agent_num = missing_timesteps_map[timestep_idx];
						
			auto other_agent_num = agent_num + 1;
			if (other_agent_num >= models.length)
				other_agent_num = 0;
			
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] -= ff[agent_num].features(working_sample[agent_num][timestep].s, working_sample[agent_num][timestep].a)[];
			
			// sample the state from T(s, a, s') T(s'', a', s''') Pr(a|s)
						
			double[StateAction] state_action_distr;
			
			foreach(s, s_v; occluded_states) {
				
				foreach (a; models[agent_num].A(s)) {
					StateAction sa = new StateAction(s, a);
					
					double v = 1.0;
					
					
					// if this agent is interacting with any others, use it's nash equilbrium, otherwise use it's policy
					bool interacting = is_interacting(working_sample[other_agent_num][timestep].s, s);
					
					if (timestep > 0) {
						v *= Tin(models[agent_num], working_sample[agent_num][timestep-1].s, working_sample[agent_num][timestep-1].a, s, interacting);
					}
					
					if (timestep < working_sample.length - 1) {
						v *= Tin(models[agent_num], s, a, working_sample[agent_num][timestep+1].s, interacting);
						v *= Tin(models[other_agent_num], working_sample[other_agent_num][timestep].s, working_sample[other_agent_num][timestep].a, working_sample[other_agent_num][timestep+1].s, interacting); 

					}
						
					if (interacting) {
						v *= equilibrium[agent_num].get(a, 0.00000001);
						v *= equilibrium[other_agent_num].get(working_sample[other_agent_num][timestep].a, 0.00000001);
					} else {
						v *= policies[agent_num].actions(s).get(a, 0.00000001);
						v *= policies[other_agent_num].actions(working_sample[other_agent_num][timestep].s).get(working_sample[other_agent_num][timestep].a, 0.00000001);
					}	
	
					
					state_action_distr[sa] = v;	
				}	
			}
			
			try {
				Distr!StateAction.normalize(state_action_distr);

				auto sa = Distr!StateAction.sample(state_action_distr);
				
				working_sample[agent_num][timestep].s = sa.s;
				working_sample[agent_num][timestep].a = sa.a;
				
				if (agent_num == 0) { 
					curJSA[timestep].s = sa.s;
					curJSA[timestep].a = sa.a;
				} else {
					curJSA[timestep].s2 = sa.s;
					curJSA[timestep].a2 = sa.a;					
				}	
				

			} catch (Exception e) {
				
			}
			
			
			

			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][timestep].s, working_sample[agent_num][timestep].a)[];
			
			
			if (j > M) {
				total_features[] += cur_features[];
				foreach(timestep_1, jsa; curJSA) {
					e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
				}
				i ++;	
				
				auto temp = total_features.dup;
				temp[] /= i;
				debug {
					
//					writeln(i, " : ", cur_features, " -> ", temp);
				}
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					break;
				}
				j = 0;	
			}	
			
			j++;
			
		}
		
		
		// now average the feature expectations
		total_features[] /= i;
		
		debug {		
//			writeln(i, " : ", total_features);
		}
		return total_features;
    	
    }


}


class LatentMaxEntIrlZiebartApproxMultipleAgentsBlockedGibbs : LatentMaxEntIrlZiebartApproxMultipleAgents {

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1, bool delegate(State, State) is_interacting = null) {
		super(max_iter, solver, observableStates, n_samples, error, solverError, is_interacting);
	}
	
	override public Agent [] solve2(Model [] models, double[State][] initials, sar[][][] true_samples,
	size_t[] sample_lengths, double [][] all_init_weights, double[Action][] NE, int interactionLength,
	out double opt_value, out double [][] opt_weights, ref double [][] featureExpecExpert,
	int num_Trajsofar, sar[][][] fullsamples, ref double [][] featureExpecExpertfull) {
        this.models = models;
        this.initials = initials;
        this.all_true_samples = true_samples;
        foreach(sample_length; sample_lengths)
        	this.sample_lengths ~= cast(int)sample_length;
        this.equilibrium = NE;
        this.interactionLength = interactionLength;
        this.init_weights = all_init_weights;

        foreach(model; models) {
        	LinearReward r = cast(LinearReward)model.getReward();
			weight_map ~= mu_E.length;

        	mu_E.length += r.dim();
        }



/*        debug {
        	writeln("FE's ", feature_expectations_per_trajectory);
        }*/

        double [] init_weights;
		init_weights.length = mu_E.length;

		foreach (i, init_weight; all_init_weights)
			foreach(i2, t; init_weight)
	            init_weights[i2 + weight_map[i]] = t;

//        init_weights[] /= l1norm(init_weights);

        //Convert double [][] featureExpecExpert to double [] mu_Eprev
        double num_Trajsofard = cast(double)num_Trajsofar;
        double [] mu_Eprev = new double[mu_E.length];
        foreach (i, arr; featureExpecExpert)
            foreach(i2, t; arr)
                mu_Eprev[i2 + weight_map[i]] = t;

        debug {
        writeln("mu_Eprev: ",mu_Eprev);
        }



        lastQValue = -double.max;
        bool hasConverged;
        double [] temp_opt_weights = init_weights.dup;
	    double [] last_temp_opt_weights = init_weights.dup;


        size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;

        // loop until convergence
        do {
            last_stochastic_policies = getPoliciesFor(temp_opt_weights);

            mu_E[] = 0;
            double [][] expert_features = calc_E_step(true_samples);

            //hack for compairons with I2RL because other code not working
            foreach (muE_onestep; expert_features) {
                mu_E[] += muE_onestep[];
            }

            debug {
	        	writeln("mu_E: ", mu_E);
	        }

            //computing new target mu_E specific to current iteration
            foreach (int i, double val; mu_E)
               mu_E[i] = (val+mu_Eprev[i]*num_Trajsofard)/(num_Trajsofard+1);//val;
               //updating muE doesn't give better learning curve

            debug {
	        	writeln("mu_E: ", mu_E);
	        }

			temp_opt_weights = exponentiatedGradient(temp_opt_weights.dup, 0.25, error, max_sample_length);
	        //temp_opt_weights = unconstrainedAdaptiveExponentiatedGradient(expert_features, .05, 10000 / max_sample_length);//.05, 10000 / max_sample_length);

//			need to approximate the Q value now, similar to how the dual is approximated
	//		This is done by integrating the objective with respect to the weights, it should give the original dual


        	// calculate Q value
        	double newQValue = calcQ(temp_opt_weights);
		    double [] test = temp_opt_weights.dup;
		    test[] -= last_temp_opt_weights[];

	        debug {
				writeln(" w diff lme em :",l2norm(test));
	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", temp_opt_weights);

	        }
	        hasConverged = (abs(newQValue - lastQValue) <= error);
	        last_temp_opt_weights = temp_opt_weights.dup;
	        lastQValue = newQValue;

	        iterations ++;
	        //writeln("hasConverged",hasConverged,"iterations",iterations);
	    } while (! hasConverged && iterations < max_iter);

        // copy the flat vector of weights into the structured array

        opt_value = -lastQValue;

        //sar [][] working_sample;
        //
        //foreach(agent_num, agent_traj; fullsamples) {
        //    working_sample ~= agent_traj[0].dup;
        //}
        //
        //JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);
        //
        //e_step_samples_full.length = curJSA.length;
        //foreach(int step, JointStateAction jsa; curJSA) {
        //    e_step_samples_full[step][jsa] = 1;
        //}
        //
        //// normalize e_step_samples_full
        //foreach (ref timestep; e_step_samples_full) {
    		//double total = 0;
    		//foreach(jsa, val; timestep) {
    		//	total += val;
    		//}
    		//foreach(jsa, ref val; timestep) {
    		//	val /= total;
    		//}
        //}
        //writeln("e_step_samples_full done",e_step_samples_full);
        //
        //double [][] expert_featuresfull =
        //LatentMaxEntIrlZiebartExactMultipleAgents.full_feature_expectations(models, e_step_samples_full);
        //
        //double [] mu_E_full = new double[mu_E.length];
        //mu_E_full[] = 0;
        //foreach (mu_E_full_onestep; expert_featuresfull) {
        //    mu_E_full[] += mu_E_full_onestep[];
        //}
        //
        //writeln("mu_E_full done");
        //
        auto i = 0;
        //
        //foreach(j, ref o; featureExpecExpertfull) {
        // foreach(ref o2; o) {
        //    o2 = mu_E_full[i++];
        // }
        //}

        opt_weights.length = all_init_weights.length;
        foreach(k, ref o; opt_weights) {
        	o.length = all_init_weights[k].length;
        }
        i = 0;
     	Agent [] returnval = new Agent[models.length];
        foreach(j, ref o; opt_weights) {
        	foreach(ref o2; o) {
        		o2 = temp_opt_weights[i++];
        	}

	        LinearReward r = cast(LinearReward)models[j].getReward();
	        r.setParams(o);

            returnval[j] = solver.createPolicy(models[j], solver.solve(models[j], solverError));
            //writeln("returnval[j] = solver.createPolicy done");

        }

        i = 0;

        foreach(j, ref o; featureExpecExpert) {
          foreach(ref o2; o) {
             o2 = mu_E[i++];
          }
        }
        featureExpecExpertfull = featureExpecExpert.dup;
        return returnval;
	}

	// blocked gibbs sampler, with a block being one state/action pair of a given agent's timestep
	// sampling goes forward to backward through the trajectories
    override double [] gibbs_sampler(sar [][][] sample, Model [] models, Agent [] policies, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest) {
    	
    	// when samping a state, we need to also consider that only unobservable states can be selected from
    	// if this results in no feasible samples, then delete the problem part of the sample and continue
//    	convergenceTest.threshold /= 100;
    	M /= 2;
    	
    	
    	double[State] occluded_states;
    	
    	foreach(s; models[0].S()) {
    		occluded_states[s] = 1.0;
    	}
    	
    	foreach(s; observableStates) {
    		occluded_states.remove(s);
    	}
    	
    	if (occluded_states.length != 0) {
	    	Distr!State.normalize(occluded_states);
	    	occluded_states.rehash;
	    			
    	}
    	
    	LinearReward [] ff;
    	
    	foreach (model; models)
    	  ff ~= cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[mu_E.length];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		size_t[] missing_timesteps_map;
		
		
		double [] min_features = new double[mu_E.length];
		min_features[] = 0;
		
		size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;

		e_step_samples.length = max_sample_length; 

		foreach(i; 0 .. max_sample_length) {
			foreach(agent_num, agent_traj; sample) {
				
				if (i < agent_traj[Y_num].length) {
					if (agent_traj[Y_num][i].s is null) {
						missing_timesteps ~= i;
						missing_timesteps_map ~= agent_num;
					} else {
						min_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(agent_traj[Y_num][i].s, agent_traj[Y_num][i].a)[];
					}
				}
				
			}
		}
		
		sar[][] working_sample;
		
		foreach(agent_num, agent_traj; sample) {
			working_sample ~= agent_traj[Y_num].dup;
		}
		
		
		if (missing_timesteps.length == 0) {
			// Z is empty, don't need sampling

			JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);

			foreach(timestep_1, jsa; curJSA) {
				e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
			}
			
			return min_features;
			
		} 
		M = cast(size_t)(0.1 * missing_timesteps.length);
		
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(i, idx; missing_timesteps) {
			size_t agent_num = missing_timesteps_map[i];
					
/*			if (idx > 0)
				working_sample[agent_num][idx].s = Distr!State.sample(models[agent_num].T(working_sample[agent_num][idx-1].s, working_sample[agent_num][idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = this.initial.dup;
				foreach(s, ref pr; initial_occluded)
					pr *= occluded_states.get(s, 0);
				
				// guard against empty initial distr
				double total = 0;
		
				foreach (key, val ; initial_occluded) {
					total += val;
				} 	
				
				if (total == 0) {
					// no valid starting states, just use the occluded ones
										
					if (occluded_states.length == 0) { 
						// full observability, but still missing some observations.  Fill with the initial state distribution
						occluded_states = initials[agent_num].dup;
					}
					
					initial_occluded = occluded_states.dup;
					
				}
				
				Distr!State.normalize(initial_occluded);	
					 
				working_sample[agent_num][idx].s = Distr!State.sample(initial_occluded);
			}*/	

			if (occluded_states.length == 0) { 
				// full observability, but still missing some observations.  Fill with the initial state distribution
				occluded_states = initials[agent_num].dup;
			}
				
			working_sample[agent_num][idx].s = Distr!State.sample(occluded_states);
			working_sample[agent_num][idx].a = policies[agent_num].sample(working_sample[agent_num][idx].s);
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][idx].s, working_sample[agent_num][idx].a)[];
		}
		
		// then loop for N times, generating a new sample, calculating feature expectations, saving them for later
		 
		JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);
 
		double[] total_features = min_features.dup;
		total_features[] = 0;
		
		
/*		if (N < 10000)
			N = 10000; 
		
		if (N > 400000)
			N = 400000;*/
		
		size_t i = 0;
		size_t j = 0;

		int step = 1;
		long timestep_idx = -1;
				
//		foreach(i; 0..(N + M)) {
		while (true) {
						
			timestep_idx += step;
			
			if (timestep_idx >= missing_timesteps.length) {
				step = -1;
				timestep_idx = missing_timesteps.length - 1;
			}
			
			if (timestep_idx < 0) {
				step = 1;
				timestep_idx = 0;
			}
			

/*			auto r = uniform(0.0, .999);
			timestep_idx = cast(size_t)(r * missing_timesteps.length);*/
			
			auto timestep = missing_timesteps[timestep_idx];			
			
			auto agent_num = missing_timesteps_map[timestep_idx];
						
			auto other_agent_num = agent_num + 1;
			if (other_agent_num >= models.length)
				other_agent_num = 0;
			
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] -= ff[agent_num].features(working_sample[agent_num][timestep].s, working_sample[agent_num][timestep].a)[];
			
			// sample the state from T(s, a, s') T(s'', a', s''') Pr(a|s)
						
			double[StateAction] state_action_distr;
			
			foreach(s, s_v; occluded_states) {
				
				foreach (a; models[agent_num].A(s)) {
					StateAction sa = new StateAction(s, a);
					
					double v = 1.0;
					
					
					// if this agent is interacting with any others, use it's nash equilbrium, otherwise use it's policy
					bool interacting = is_interacting(working_sample[other_agent_num][timestep].s, s);
					
					if (timestep > 0) {
						v *= Tin(models[agent_num], working_sample[agent_num][timestep-1].s, working_sample[agent_num][timestep-1].a, s, interacting);
					}
					
					if (timestep < working_sample.length - 1) {
						v *= Tin(models[agent_num], s, a, working_sample[agent_num][timestep+1].s, interacting);
						v *= Tin(models[other_agent_num], working_sample[other_agent_num][timestep].s, working_sample[other_agent_num][timestep].a, working_sample[other_agent_num][timestep+1].s, interacting); 

					}
						
					if (interacting) {
						v *= equilibrium[agent_num].get(a, 0.00000001);
						v *= equilibrium[other_agent_num].get(working_sample[other_agent_num][timestep].a, 0.00000001);
					} else {
						v *= policies[agent_num].actions(s).get(a, 0.00000001);
						v *= policies[other_agent_num].actions(working_sample[other_agent_num][timestep].s).get(working_sample[other_agent_num][timestep].a, 0.00000001);
					}	
	
					
					state_action_distr[sa] = v;	
				}	
			}
			
			try {
				Distr!StateAction.normalize(state_action_distr);

				auto sa = Distr!StateAction.sample(state_action_distr);
				
				working_sample[agent_num][timestep].s = sa.s;
				working_sample[agent_num][timestep].a = sa.a;
				
				if (agent_num == 0) { 
					curJSA[timestep].s = sa.s;
					curJSA[timestep].a = sa.a;
				} else {
					curJSA[timestep].s2 = sa.s;
					curJSA[timestep].a2 = sa.a;					
				}	
				

			} catch (Exception e) {
				
			}
			
			
			

			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][timestep].s, working_sample[agent_num][timestep].a)[];
			
			
			if (j > M) {
				total_features[] += cur_features[];
				foreach(timestep_1, jsa; curJSA) {
					e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
				}
				i ++;	
				
				auto temp = total_features.dup;
				temp[] /= i;
				debug {
					
//					writeln(i, " : ", cur_features, " -> ", temp);
				}
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					break;
				}
				j = 0;	
			}	
			
			j++;
			
		}
		
		
		// now average the feature expectations
		total_features[] /= i;
		
		debug {		
//			writeln(i, " : ", total_features);
		}
		return total_features;
    	
    }

}

class LatentMaxEntIrlZiebartApproxMultipleAgentsTimestepBlockedGibbs : LatentMaxEntIrlZiebartApproxMultipleAgentsBlockedGibbs {

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1, bool delegate(State, State) is_interacting = null) {
		super(max_iter, solver, observableStates, n_samples, error, solverError, is_interacting);
	}
	
	// blocked gibbs sampler, with a block being one two agent timestep
	// sampling goes forward to backward through the trajectories
    override double [] gibbs_sampler(sar [][][] sample, Model [] models, Agent [] policies, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest) {
    	
    	// when samping a state, we need to also consider that only unobservable states can be selected from
    	// if this results in no feasible samples, then delete the problem part of the sample and continue
    	M /= 4;
    	
    	double[State] occluded_states;
    	
    	foreach(s; models[0].S()) {
    		occluded_states[s] = 1.0;
    	}
    	
    	foreach(s; observableStates) {
    		occluded_states.remove(s);
    	}
    	
    	if (occluded_states.length != 0) {
	    	Distr!State.normalize(occluded_states);
	    	occluded_states.rehash;
	    			
    	}
    	
    	LinearReward [] ff;
    	
    	foreach (model; models)
    	  ff ~= cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[mu_E.length];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		size_t[] missing_timesteps_map;
		
		
		double [] min_features = new double[mu_E.length];
		min_features[] = 0;
		
		size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;

		e_step_samples.length = max_sample_length; 

		foreach(i; 0 .. max_sample_length) {
			foreach(agent_num, agent_traj; sample) {
				
				if (i < agent_traj[Y_num].length) {
					if (agent_traj[Y_num][i].s is null) {
						missing_timesteps ~= i;
						missing_timesteps_map ~= agent_num;
					} else {
						min_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(agent_traj[Y_num][i].s, agent_traj[Y_num][i].a)[];
					}
				}
				
			}
		}
		
		
		sar[][] working_sample;
		
		foreach(agent_num, agent_traj; sample) {
			working_sample ~= agent_traj[Y_num].dup;
		}
				
		if (missing_timesteps.length == 0) {
			// Z is empty, don't need sampling

			JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);

			foreach(timestep_1, jsa; curJSA) {
				e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
			}
						
			return min_features;
			
		}

		
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(i, idx; missing_timesteps) {
			size_t agent_num = missing_timesteps_map[i];
					
			if (idx > 0)
				working_sample[agent_num][idx].s = Distr!State.sample(models[agent_num].T(working_sample[agent_num][idx-1].s, working_sample[agent_num][idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = this.initial.dup;
				foreach(s, ref pr; initial_occluded)
					pr *= occluded_states.get(s, 0);
				
				// guard against empty initial distr
				double total = 0;
		
				foreach (key, val ; initial_occluded) {
					total += val;
				} 	
				
				if (total == 0) {
					// no valid starting states, just use the occluded ones
					
					if (occluded_states.length == 0) { 
						// full observability, but still missing some observations.  Fill with the initial state distribution
						occluded_states = initials[agent_num].dup;
					}
					
					initial_occluded = occluded_states.dup;
					
				}
				
				Distr!State.normalize(initial_occluded);	
					 
				working_sample[agent_num][idx].s = Distr!State.sample(initial_occluded);
			}	
				
			working_sample[agent_num][idx].a = policies[agent_num].sample(working_sample[agent_num][idx].s);
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][idx].s, working_sample[agent_num][idx].a)[];
		}
		
		// then loop for N times, generating a new sample, calculating feature expectations, saving them for later
		 
		double[] total_features = min_features.dup;
		total_features[] = 0;
		
		JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);

		
/*		if (N < 10000)
			N = 10000; 
		
		if (N > 400000)
			N = 400000;*/

		
		double [JointStateAction] getJSADistr(size_t timestep) {
			double [JointStateAction] returnval;
			
			
     		State [] S0List;
     		State [] S1List;
     		
     		if (sample[0][Y_num].length > timestep && sample[0][Y_num][timestep].s !is null) {
     			S0List = [sample[0][Y_num][timestep].s];
     		} else 
     			S0List = occluded_states.keys;
     		if (sample[1][Y_num].length > timestep && sample[1][Y_num][timestep].s !is null) {
     			S1List = [sample[1][Y_num][timestep].s];
     		} else 
     			S1List = occluded_states.keys;
     		
    		foreach (s; S0List) {
	     		auto A0List = models[0].A(s);
	     		if (sample[0][Y_num].length > timestep && sample[0][Y_num][timestep].a !is null) {
	     			A0List = [sample[0][Y_num][timestep].a];
	     		}
    			foreach(a; A0List) {
		    		foreach (s2; S1List) {
			     		auto A1List = models[1].A(s2);
			     		if (sample[1][Y_num].length > timestep && sample[1][Y_num][timestep].a !is null) {
			     			A1List = [sample[1][Y_num][timestep].a];
			     		}
		    			foreach(a2; A1List) {		    				

		    				double temp_prob = 1.0;
		    				
		    				bool interacting = is_interacting(s, s2);
		    				
							if (interacting)
								temp_prob *= equilibrium[0].get(a, 0.00000001) * equilibrium[1].get(a2, 0.00000001);
							else
								temp_prob *= last_stochastic_policies[0].actions(s).get(a, 0.00000001) * last_stochastic_policies[1].actions(s2).get(a2, 0.00000001);
							
							
							if (timestep > 0) {
								temp_prob *= Tin(models[0], working_sample[0][timestep-1].s, working_sample[0][timestep-1].a, s, interacting); 						
								temp_prob *= Tin(models[1], working_sample[1][timestep-1].s, working_sample[1][timestep-1].a, s2, interacting); 						
							}
							
							if (timestep < max_sample_length - 1) {
								temp_prob *= Tin(models[0], s, a, working_sample[0][timestep+1].s, interacting); 
								temp_prob *= Tin(models[1], s2, a2, working_sample[1][timestep+1].s, interacting); 
							}
		    				
		    				returnval[new JointStateAction(s, a, s2, a2)] = temp_prob;
		    			}
		    		}
		    	}
    		}			
			return returnval;
			
		} 
				
		size_t i = 0;
		size_t j = 0;

		int step = 1;
		long timestep = -1;
				
//		foreach(i; 0..(N + M)) {
		while (true) {
			
			timestep += step;
			
			if (timestep >= max_sample_length) {
				step = -1;
				timestep = max_sample_length - 1;
			}
			
			if (timestep < 0) {
				step = 1;
				timestep = 0;
			}
			
			auto joint_state_action_distr = getJSADistr(timestep);
			
			foreach(agent_num; 0 .. models.length)
				cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] -= ff[agent_num].features(working_sample[agent_num][timestep].s, working_sample[agent_num][timestep].a)[];
			
			
			try {
				Distr!JointStateAction.normalize(joint_state_action_distr);

				auto jsa = Distr!JointStateAction.sample(joint_state_action_distr);
				
				working_sample[0][timestep].s = jsa.s;
				working_sample[0][timestep].a = jsa.a;
				working_sample[1][timestep].s = jsa.s2;
				working_sample[1][timestep].a = jsa.a2;
				
				
				curJSA[timestep].s = jsa.s;
				curJSA[timestep].a = jsa.a;
				curJSA[timestep].s2 = jsa.s2;
				curJSA[timestep].a2 = jsa.a2;					
				

			} catch (Exception e) {
				
			}
			
			
			

			
			foreach(agent_num; 0 .. models.length)
				cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][timestep].s, working_sample[agent_num][timestep].a)[];
			
			
			if (j > M) {
				total_features[] += cur_features[];
				foreach(timestep_1, jsa; curJSA) {
					e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
				}
								
				i ++;
				auto temp = total_features.dup;
				temp[] /= i;
				debug {
					
					writeln(i, " : ", cur_features, " -> ", temp);
				}
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					break;
				}
				j = 0;
			}	
			
			j ++;	

			
		}
		
		
		// now average the feature expectations
		total_features[] /= i;
		

		return total_features;
    	
    }

}

class LatentMaxEntIrlZiebartApproxMultipleAgentsMultiTimestepBlockedGibbs : LatentMaxEntIrlZiebartApproxMultipleAgentsTimestepBlockedGibbs {

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1, bool delegate(State, State) is_interacting = null) {
		super(max_iter, solver, observableStates, n_samples, error, solverError, is_interacting);
	}

	// blocked gibbs sampler, with a block being one two agent timestep
	// sampling goes forward to backward through the trajectories
    override double [] gibbs_sampler(sar [][][] sample, Model [] models, Agent [] policies, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest) {
    	
    	
    	int timestep_size = 2;
    	M /= 4 * timestep_size;
    	
    	// when samping a state, we need to also consider that only unobservable states can be selected from
    	// if this results in no feasible samples, then delete the problem part of the sample and continue
    	
    	double[State] occluded_states;
    	
    	foreach(s; models[0].S()) {
    		occluded_states[s] = 1.0;
    	}
    	
    	foreach(s; observableStates) {
    		occluded_states.remove(s);
    	}
    	
    	if (occluded_states.length != 0) {
	    	Distr!State.normalize(occluded_states);
	    	occluded_states.rehash;
	    			
    	}
    	
    	LinearReward [] ff;
    	
    	foreach (model; models)
    	  ff ~= cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[mu_E.length];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		size_t[] missing_timesteps_map;
		
		
		double [] min_features = new double[mu_E.length];
		min_features[] = 0;
		
		size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;
		
		e_step_samples.length = max_sample_length; 
		
		foreach(i; 0 .. max_sample_length) {
			foreach(agent_num, agent_traj; sample) {
				
				if (i < agent_traj[Y_num].length) {
					if (agent_traj[Y_num][i].s is null) {
						missing_timesteps ~= i;
						missing_timesteps_map ~= agent_num;
					} else {
						min_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(agent_traj[Y_num][i].s, agent_traj[Y_num][i].a)[];
					}
				}
				
			}
		}
		
		sar[][] working_sample;
		
		foreach(agent_num, agent_traj; sample) {
			working_sample ~= agent_traj[Y_num].dup;
		}
		
		
		if (missing_timesteps.length == 0) {
			// Z is empty, don't need sampling

			JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);

			foreach(timestep_1, jsa; curJSA) {
				e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
			}
						
			return min_features;
			
		} 

		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(i, idx; missing_timesteps) {
			size_t agent_num = missing_timesteps_map[i];
					
			if (idx > 0)
				working_sample[agent_num][idx].s = Distr!State.sample(models[agent_num].T(working_sample[agent_num][idx-1].s, working_sample[agent_num][idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = this.initial.dup;
				foreach(s, ref pr; initial_occluded)
					pr *= occluded_states.get(s, 0);
				
				// guard against empty initial distr
				double total = 0;
		
				foreach (key, val ; initial_occluded) {
					total += val;
				} 	
				
				if (total == 0) {
					// no valid starting states, just use the occluded ones
					
					if (occluded_states.length == 0) { 
						// full observability, but still missing some observations.  Fill with the initial state distribution
						occluded_states = initials[agent_num].dup;
					}
										
					initial_occluded = occluded_states.dup;
					
				}
				
				Distr!State.normalize(initial_occluded);	
					 
				working_sample[agent_num][idx].s = Distr!State.sample(initial_occluded);
			}	
				
			working_sample[agent_num][idx].a = policies[agent_num].sample(working_sample[agent_num][idx].s);
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][idx].s, working_sample[agent_num][idx].a)[];
		}
		
		// then loop for N times, generating a new sample, calculating feature expectations, saving them for later
		 
		double[] total_features = min_features.dup;
		total_features[] = 0;
		
		JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);
		
/*		if (N < 10000)
			N = 10000; 
		
		if (N > 400000)
			N = 400000;*/

		
		double [JointStateAction][] getJSADistr(long timestep_start, long timestep_end ) {
			
			
			if (timestep_end < timestep_start) {
				auto temp = timestep_start;
				timestep_start = timestep_end;
				timestep_end = temp;
				
			}
			
			if (timestep_start < 0)
				timestep_start = 0;
				
			if (timestep_end > max_sample_length)
				timestep_end = max_sample_length;

			double [JointStateAction][] returnval;
			
			
			
	    	double [JointStateAction] forward(double [JointStateAction] p_t, sar [] obs) {
	    		double [JointStateAction] returnval;
	     		State [] S0List;
	     		State [] S1List;
	     		
	     		if (obs[0].s !is null) {
	     			S0List = [obs[0].s];
	     		} else 
	     			S0List = occluded_states.keys;
	     		if (obs[1].s !is null) {
	     			S1List = [obs[1].s];
	     		} else 
	     			S1List = occluded_states.keys;
	     		
	    		foreach (s; S0List) {
		     		auto A0List = models[0].A(s);
		     		if (obs[0].a !is null) {
		     			A0List = [obs[0].a];
		     		}
	    			foreach(a; A0List) {
			    		foreach (s2; S1List) {
				     		auto A1List = models[1].A(s2);
				     		if (obs[1].a !is null) {
				     			A1List = [obs[1].a];
				     		}
			    			foreach(a2; A1List) {		    				
	
			    				double temp_prob = 1.0;
			    				
			    				bool interacting = is_interacting(s, s2);
			    				
								if (interacting)
									temp_prob *= equilibrium[0].get(a, 0.00000001) * equilibrium[1].get(a2, 0.00000001);
								else
									temp_prob *= last_stochastic_policies[0].actions(s).get(a, 0.00000001) * last_stochastic_policies[1].actions(s2).get(a2, 0.00000001);
								
			    				foreach(jsa, prob; p_t) {
			    					
			    					temp_prob += Tin(models[0], jsa.s, jsa.a, s, interacting) * Tin(models[1], jsa.s2, jsa.a2, s2, interacting) * prob;
			    				}
			    				
			    				returnval[new JointStateAction(s, a, s2, a2)] = temp_prob;
			    			}
			    		}
			    	}
	    		}
	    		return returnval;			
	    		
	    	}
	    	
	    	double [JointStateAction] backward(double [JointStateAction] prev_b, sar [] obs) {
	     		double [JointStateAction] returnval;
	    		foreach (s; models[0].S()) {
	    			foreach(a; models[0].A(s)) {
			    		foreach (s2; models[1].S()) {
			    			foreach(a2; models[1].A(s2)) {
			    				
	    						double total = 0;
	    						
	    						
					     		State [] S0List;
					     		State [] S1List;
					     		
					     		if (obs[0].s !is null) {
					     			S0List = [obs[0].s];
					     		} else 
					     			S0List = occluded_states.keys;
					     		if (obs[1].s !is null) {
					     			S1List = [obs[1].s];
					     		} else 
					     			S1List = occluded_states.keys;
					     		
					    		foreach (jsas; S0List) {
						     		auto A0List = models[0].A(jsas);
						     		if (obs[0].a !is null) {
						     			A0List = [obs[0].a];
						     		}
					    			foreach(jsaa; A0List) {
							    		foreach (jsas2; S1List) {
								     		auto A1List = models[1].A(jsas2);
								     		if (obs[1].a !is null) {
								     			A1List = [obs[1].a];
								     		}
							    			foreach(jsaa2; A1List) {
							    				JointStateAction jsa = new JointStateAction(jsas, jsaa, jsas2, jsaa2);
							    				
							    				auto prob = prev_b.get(jsa, 0.00000001);
							    				
							    										    				
				    							double actionprob = 0;
				    							bool interacting = is_interacting(jsa.s, jsa.s2);
						    				
												if (interacting)
													actionprob = equilibrium[0].get(jsa.a, 0.00000001) * equilibrium[1].get(jsa.a2, 0.00000001);
												else
													actionprob = last_stochastic_policies[0].actions(jsa.s).get(jsa.a, 0.00000001) * last_stochastic_policies[1].actions(jsa.s2).get(jsa.a2, 0.00000001);
												
				    							total += prob *  Tin(models[0], s, a, jsa.s, interacting) * Tin(models[1], s2, a2, jsa.s2, interacting) * actionprob;
	
							    				
							    			}
							    		}
							    	}
					    		}
	    						
	    						returnval[new JointStateAction(s, a, s2, a2)] = total;
	    					}
			    		}
			    	}		
	   			}
	    		return returnval;
	    	}			
			
			
			// prob, state/action, agent, timestep for forward value
	    	double [JointStateAction][] fv;
	    	
	    	// initialize prior
	    	
			double [JointStateAction] temp;

	    	if (timestep_start == 0) {
	    		foreach (s; models[0].S()) {
	    			foreach(action; models[0].A(s)) {
			    		foreach (s2; models[1].S()) {
			    			foreach(action2; models[1].A(s2)) {
	    						temp[new JointStateAction(s, action, s2, action2)] = last_stochastic_policies[0].actions(s).get(action, 0) * last_stochastic_policies[1].actions(s2).get(action2, 0);
	    					}
			    		}
	    			}
	    		}
    		} else {
    			temp[new JointStateAction(working_sample[0][timestep_start - 1].s, working_sample[0][timestep_start - 1].a, working_sample[1][timestep_start - 1].s, working_sample[1][timestep_start - 1].a)] = 1.0;
    		}
	    	Distr!JointStateAction.normalize(temp);
    		fv ~= temp;
    		debug{
    			writeln("Forward");
    		}
    		foreach(long i; timestep_start .. timestep_end) {
    			// create ev at time t
    			sar[] ev;
    			foreach(agent_num, agent_traj; sample) {
    				if (agent_traj[Y_num].length > i)
    					ev ~= agent_traj[Y_num][i];
    				else
    					ev ~= sar(null, null, 0);	
    			}
	    		debug {
	    			writeln(ev);
	    		}
    			
    			fv ~= forward(fv[$-1], ev);
    		}
    		// prob, state/action, timestep for final vector
    		
    		debug{
    			writeln("Backward");
    		}
    		double [JointStateAction] b;
    		
    		if (timestep_end == max_sample_length) {
    			foreach (s; models[0].S()) 
	    			foreach(action; models[0].A(s)) 
			    		foreach (s2; models[1].S()) 
			    			foreach(action2; models[1].A(s2)) 
    							b[new JointStateAction(s, action, s2, action2)] = 1.0;
    		} else {
    			b[new JointStateAction(working_sample[0][timestep_end].s, working_sample[0][timestep_end].a, working_sample[1][timestep_end].s, working_sample[1][timestep_end].a)] = 1.0;
    			sar[] ev;
	    		foreach(agent_num, agent_traj; sample) {
    				if (agent_traj[Y_num].length > timestep_end)
    					ev ~= agent_traj[Y_num][timestep_end];
    				else
    					ev ~= sar(null, null, 0);	
    			}
    			b = backward(b, ev);
    		}
    			
    		for(long i = timestep_end-1; i >= timestep_start; i --) {
				double[JointStateAction] temp_jsa;
				foreach(sasa, prob; b)
					temp_jsa[sasa] = fv[$-1].get(sasa, 0.00000001) * prob;
				fv.length = fv.length - 1;
				
				returnval ~= temp_jsa;	
				
	   			sar[] ev;
	    		foreach(agent_num, agent_traj; sample) {
    				if (agent_traj[Y_num].length > i)
    					ev ~= agent_traj[Y_num][i];
    				else
    					ev ~= sar(null, null, 0);	
    			}
	    		debug {
	    			writeln(ev);
	    			
	    		}
	    		
	    		b = backward(b, ev);
    			
    		}
			
			
			reverse(returnval);
			return returnval;
			
		} 
				
		size_t i = 0;
		size_t j1 = 0;

		int step = timestep_size;
		long timestep = -1;
				
//		foreach(i; 0..(N + M)) {
		while (true) {
			
			timestep += step;
			
			if (timestep >= max_sample_length) {
				step = -step;
				timestep = max_sample_length - 1;
			}
			
			if (timestep < 0) {
				step = -step;
				timestep = 0;
			}
			
			long next_timestep = timestep + step;
							
			if (next_timestep >= max_sample_length)
				next_timestep = max_sample_length - 1;
				
			if (next_timestep < 0)
				next_timestep = 0;

			
			auto joint_state_action_distr = getJSADistr(timestep, next_timestep);
			auto start = timestep;
			if (next_timestep < timestep)
				start = next_timestep;
			
			foreach(j, ref jsad; joint_state_action_distr) {
				foreach(agent_num; 0 .. models.length)
					cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] -= ff[agent_num].features(working_sample[agent_num][j + start].s, working_sample[agent_num][j + start].a)[];
			
			
				try {
					Distr!JointStateAction.normalize(jsad);
	
					auto jsa = Distr!JointStateAction.sample(jsad);
					
					working_sample[0][j + start].s = jsa.s;
					working_sample[0][j + start].a = jsa.a;
					working_sample[1][j + start].s = jsa.s2;
					working_sample[1][j + start].a = jsa.a2;
					
									
					curJSA[j + start].s = jsa.s;
					curJSA[j + start].a = jsa.a;
					curJSA[j + start].s2 = jsa.s2;
					curJSA[j + start].a2 = jsa.a2;
	
				} catch (Exception e) {
					
				}
				
				foreach(agent_num; 0 .. models.length)
					cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][j + start].s, working_sample[agent_num][j + start].a)[];
		
			}
			if (j1 > M) {
				total_features[] += cur_features[];
				foreach(timestep_1, jsa; curJSA) {
					e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
				}
				
				i ++;
				auto temp = total_features.dup;
				temp[] /= i;
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					break;
				}
				j1 = 0;
			}	
			
			j1 ++;	

			
		}
		
		
		// now average the feature expectations
		total_features[] /= i;
		

		return total_features;
    	
    }


}

class LatentMaxEntIrlZiebartApproxMultipleAgentsMultiTimestepSingleAgentBlockedGibbs : LatentMaxEntIrlZiebartApproxMultipleAgentsTimestepBlockedGibbs {

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1, bool delegate(State, State) is_interacting = null) {
		super(max_iter, solver, observableStates, n_samples, error, solverError, is_interacting);
	}

	// blocked gibbs sampler, with a block being one two agent timestep
	// sampling goes forward to backward through the trajectories
    override double [] gibbs_sampler(sar [][][] sample, Model [] models, Agent [] policies, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest) {
    	
    	
    	int timestep_size = 3;
    	M /= 2 * timestep_size;
    	
    	// when samping a state, we need to also consider that only unobservable states can be selected from
    	// if this results in no feasible samples, then delete the problem part of the sample and continue
    	
    	double[State] occluded_states;
    	
    	foreach(s; models[0].S()) {
    		occluded_states[s] = 1.0;
    	}
    	
    	foreach(s; observableStates) {
    		occluded_states.remove(s);
    	}
    	
    	if (occluded_states.length != 0) {
	    	Distr!State.normalize(occluded_states);
	    	occluded_states.rehash;
	    			
    	}
    	
    	LinearReward [] ff;
    	
    	foreach (model; models)
    	  ff ~= cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[mu_E.length];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		size_t[] missing_timesteps_map;
		
		
		double [] min_features = new double[mu_E.length];
		min_features[] = 0;
		
		size_t max_sample_length = 0;
		foreach(sl; sample_lengths)
			if (sl > max_sample_length)
				max_sample_length = sl;
		
		e_step_samples.length = max_sample_length; 
		
		foreach(i; 0 .. max_sample_length) {
			foreach(agent_num, agent_traj; sample) {
				
				if (i < agent_traj[Y_num].length) {
					if (agent_traj[Y_num][i].s is null) {
						missing_timesteps ~= i;
						missing_timesteps_map ~= agent_num;
					} else {
						min_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(agent_traj[Y_num][i].s, agent_traj[Y_num][i].a)[];
					}
				}
				
			}
		}
		
		
		sar[][] working_sample;
		
		foreach(agent_num, agent_traj; sample) {
			working_sample ~= agent_traj[Y_num].dup;
		}
		
		
		if (missing_timesteps.length == 0) {
			// Z is empty, don't need sampling

			JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);

			foreach(timestep_1, jsa; curJSA) {
				e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
			}
			
			return min_features;
			
		} 		
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(i, idx; missing_timesteps) {
			size_t agent_num = missing_timesteps_map[i];
					
			if (idx > 0)
				working_sample[agent_num][idx].s = Distr!State.sample(models[agent_num].T(working_sample[agent_num][idx-1].s, working_sample[agent_num][idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = this.initial.dup;
				foreach(s, ref pr; initial_occluded)
					pr *= occluded_states.get(s, 0);
				
				// guard against empty initial distr
				double total = 0;
		
				foreach (key, val ; initial_occluded) {
					total += val;
				} 	
				
				if (total == 0) {
					// no valid starting states, just use the occluded ones
					
					if (occluded_states.length == 0) { 
						// full observability, but still missing some observations.  Fill with the initial state distribution
						occluded_states = initials[agent_num].dup;
					}
										
					initial_occluded = occluded_states.dup;
					
				}
				
				Distr!State.normalize(initial_occluded);	
					 
				working_sample[agent_num][idx].s = Distr!State.sample(initial_occluded);
			}	
				
			working_sample[agent_num][idx].a = policies[agent_num].sample(working_sample[agent_num][idx].s);
			
			cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][idx].s, working_sample[agent_num][idx].a)[];
		}
		
		// then loop for N times, generating a new sample, calculating feature expectations, saving them for later
		 
		double[] total_features = min_features.dup;
		total_features[] = 0;
		
		JointStateAction [] curJSA = workingSampleToJSAArray(working_sample);
		
/*		if (N < 10000)
			N = 10000; 
		
		if (N > 400000)
			N = 400000;*/

		
		double [StateAction][] getSADistr(long timestep_start, long timestep_end, size_t agent_num ) {
			
			
			if (timestep_end < timestep_start) {
				auto temp = timestep_start;
				timestep_start = timestep_end;
				timestep_end = temp;
				
			}
			
			if (timestep_start < 0)
				timestep_start = 0;
				
			if (timestep_end > max_sample_length)
				timestep_end = max_sample_length;
				
			
			int other_agent_num =cast(int) agent_num + 1;
			if (other_agent_num >= models.length)
				other_agent_num = 0;
			
			double [StateAction][] returnval;
			
			
	    	double [StateAction] forward(double [StateAction] p_t, sar obs, sar other_agent, State other_agent_next_state) {
	    		double [StateAction] returnval;
	     		State [] S0List;
	     		State [] S1List;
	     		
	     		if (obs.s !is null) {
	     			S0List = [obs.s];
	     		} else 
	     			S0List = occluded_states.keys;
	     		
	    		foreach (s; S0List) {
		     		auto A0List = models[agent_num].A(s);
		     		if (obs.a !is null) {
		     			A0List = [obs.a];
		     		}
	    			foreach(a; A0List) {

	    				double temp_prob = 1.0;
	    				
	    				bool interacting = is_interacting(s, other_agent.s);
	    				
						if (interacting)
							temp_prob *= equilibrium[agent_num].get(a, 0.00000001) * equilibrium[other_agent_num].get(other_agent.a, 0.00000001);
						else
							temp_prob *= last_stochastic_policies[agent_num].actions(s).get(a, 0.00000001) * last_stochastic_policies[other_agent_num].actions(other_agent.s).get(other_agent.a, 0.00000001);
						
	    				foreach(sa, prob; p_t) {
	    					
	    					temp_prob += Tin(models[agent_num], sa.s, sa.a, s, interacting) * prob * Tin(models[other_agent_num], other_agent.s, other_agent.a, other_agent_next_state, interacting);
	    				}
	    				
	    				returnval[new StateAction(s, a)] = temp_prob;

			    	}
	    		}
	    		return returnval;			
	    		
	    	}
	    	
	    	double [StateAction] backward(double [StateAction] prev_b, sar obs, sar other_agent, State other_agent_prev_state, Action other_agent_prev_action, sar obs_minus_one) {
	     		double [StateAction] returnval;
	     		State [] S0List1;
	     		
	     		if (obs_minus_one.s !is null) {
	     			S0List1 = [obs_minus_one.s];
	     		} else 
	     			S0List1 = occluded_states.keys;
	
	    		foreach (s; S0List1) {
		     		auto A0List1 = models[agent_num].A(s);
		     		if (obs_minus_one.a !is null) {
		     			A0List1 = [obs_minus_one.a];
		     		}
	    			foreach(a; A0List1) {	     		
			    				
						double total = 0;
						
			     		State [] S0List;
			     		
			     		if (obs.s !is null) {
			     			S0List = [obs.s];
			     		} else 
			     			S0List = occluded_states.keys;
			     		
			    		foreach (sb; S0List) {
				     		auto A0List = models[agent_num].A(sb);
				     		if (obs.a !is null) {
				     			A0List = [obs.a];
				     		}
			    			foreach(sba; A0List) {

			    				StateAction sa = new StateAction(sb, sba);
			    				
			    				auto prob = prev_b.get(sa, 0.00000001);
			    				
			    										    				
    							double actionprob = 0;
    							bool interacting = is_interacting(sa.s, other_agent.s);
		    				
								if (interacting)
									actionprob = equilibrium[agent_num].get(sa.a, 0.00000001) * equilibrium[other_agent_num].get(other_agent.a, 0.00000001);
								else
									actionprob = last_stochastic_policies[agent_num].actions(sa.s).get(sa.a, 0.00000001) * last_stochastic_policies[other_agent_num].actions(other_agent.s).get(other_agent.a, 0.00000001);
								
    							total += prob * Tin(models[agent_num], s, a, sa.s, interacting) * actionprob * Tin(models[other_agent_num], other_agent_prev_state, other_agent_prev_action, other_agent.s, interacting);
			    				
					    	}
			    		}
						
						returnval[new StateAction(s, a)] = total;

			    	}		
	   			}
	    		return returnval;
	    	}			
			
			
			// prob, state/action, agent, timestep for forward value
	    	double [StateAction][] fv;
	    	
	    	// initialize prior
	    	
			double [StateAction] temp;

	    	if (timestep_start == 0) {
	    		foreach (s; models[0].S()) {
	    			foreach(action; models[0].A(s)) {
						temp[new StateAction(s, action)] = last_stochastic_policies[agent_num].actions(s).get(action, 0);
	    			}
	    		}
    		} else {
    			temp[new StateAction(working_sample[agent_num][timestep_start - 1].s, working_sample[agent_num][timestep_start - 1].a)] = 1.0;
    		}
	    	Distr!StateAction.normalize(temp);
    		fv ~= temp;
    		foreach(long i; timestep_start .. timestep_end) {
    			// create ev at time t
    			sar ev;

				if (sample[agent_num][Y_num].length > i)
					ev = sample[agent_num][Y_num][i];
				else
					ev = sar(null, null, 0);	
    			
    			fv ~= forward(fv[$-1], ev, working_sample[other_agent_num][i], ((i < working_sample[agent_num].length - 1) ? working_sample[other_agent_num][i+1].s : null));
    		}
    		// prob, state/action, timestep for final vector
    		debug{
    			writeln("Forward");
    		}
    		double [StateAction] b;
    		
    		if (timestep_end == max_sample_length) {
    			foreach (s; models[agent_num].S()) 
	    			foreach(action; models[agent_num].A(s)) 
    					b[new StateAction(s, action)] = 1.0;
    		} else {
    			b[new StateAction(working_sample[agent_num][timestep_end].s, working_sample[agent_num][timestep_end].a)] = 1.0;
    			sar ev;
				if (sample[agent_num][Y_num].length > timestep_end)
					ev = sample[agent_num][Y_num][timestep_end];
				else
					ev = sar(null, null, 0);	
    			sar ev2;
				if (sample[agent_num][Y_num].length > timestep_end-1 && timestep_end-1 >= 0)
					ev2 = sample[agent_num][Y_num][timestep_end-1];
				else
					ev2 = sar(null, null, 0);	
    			b = backward(b, ev, working_sample[other_agent_num][timestep_end], null, null, ev2);
    		}
    			
    		for(long i = timestep_end-1; i >= timestep_start; i --) {
				double[StateAction] temp_sa;
				foreach(sa, prob; b)
					temp_sa[sa] = fv[$-1].get(sa, 0.00000001) * prob;
				fv.length = fv.length - 1;
				
				returnval ~= temp_sa;
				
	   			sar ev;
				if (sample[agent_num][Y_num].length > i)
					ev = sample[agent_num][Y_num][i];
				else
					ev = sar(null, null, 0);	
	   			sar ev2;
				if (sample[agent_num][Y_num].length > i-1 && i-1 >= 0)
					ev2 = sample[agent_num][Y_num][i-1];
				else
					ev2 = sar(null, null, 0);	
				
	    		b = backward(b, ev, working_sample[other_agent_num][i],((i > 0) ? working_sample[other_agent_num][i-1].s : null), ((timestep_end > 0) ? working_sample[other_agent_num][i-1].a : null), ev2);
    			
    		}
			debug{
    			writeln("Backward");
    		}
			
			reverse(returnval);
			return returnval;
			
		} 
				
		size_t i = 0;
		size_t j1 = 0;

		int step = timestep_size;
		long timestep = -1;
				
//		foreach(i; 0..(N + M)) {
		while (true) {
			
			timestep += step;
			
			if (timestep >= max_sample_length) {
				step = -step;
				timestep = max_sample_length - 1;
			}
			
			if (timestep < 0) {
				step = -step;
				timestep = 0;
			}
			
			long next_timestep = timestep + step;
			
			if (next_timestep >= max_sample_length)
				next_timestep = max_sample_length - 1;
				
			if (next_timestep < 0)
				next_timestep = 0;
			
			foreach(agent_num; 0 .. models.length) {
			
			
				auto state_action_distr = getSADistr(timestep, next_timestep, agent_num);
				auto start = timestep;
				if (next_timestep < timestep)
					start = next_timestep;
				
				foreach(j, ref sad; state_action_distr) {
					cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] -= ff[agent_num].features(working_sample[agent_num][j + start].s, working_sample[agent_num][j + start].a)[];
					
					try {
						Distr!StateAction.normalize(sad);
		
						auto sa = Distr!StateAction.sample(sad);
						
						working_sample[agent_num][j + start].s = sa.s;
						working_sample[agent_num][j + start].a = sa.a;
						
						if (agent_num == 0) { 
							curJSA[j + start].s = sa.s;
							curJSA[j + start].a = sa.a;
						} else {
							curJSA[j + start].s2 = sa.s;
							curJSA[j + start].a2 = sa.a;					
						}	
				
		
					} catch (Exception e) {
						
					}
					
					cur_features[weight_map[agent_num] .. weight_map[agent_num] + ff[agent_num].dim()] += ff[agent_num].features(working_sample[agent_num][j + start].s, working_sample[agent_num][j + start].a)[];
			
				}
			
			}
			if (j1 > M) {
				total_features[] += cur_features[];
				foreach(timestep_1, jsa; curJSA) {
					e_step_samples[timestep_1][jsa] = e_step_samples[timestep_1].get(jsa, 0.0) + 1;
				}
				
				i ++;
				auto temp = total_features.dup;
				temp[] /= i;
				debug {
					
					writeln(i, " : ", cur_features, " -> ", temp);
				}
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					break;
				}	
				j1 = 0;			
			}	
			
			j1 ++;

		}
		
		
		// now average the feature expectations
		total_features[] /= i;
		

		return total_features;
    	
    }


}

class LatentMaxEntIrlZiebartApproxMultipleAgentsForwardBackward : LatentMaxEntIrlZiebartApproxMultipleAgents {


	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1, bool delegate(State, State) is_interacting = null) {
		super(max_iter, solver, observableStates, n_samples, error, solverError, is_interacting);
	}
	
	// [agent][traj_number][timestep]
    double [] calc_E_step_backup(sar[][][] true_samples) {
    	
    	if (true_samples.length != 2)
    		throw new Exception("ERROR: This method is only implemented for exactly 2 agents.");  
    	
		// figure out the maximum sample length (number of timesteps)
		size_t t = 0;
		foreach(sl; sample_lengths)
			if (sl > t)
				t = sl;

    	double[State] occluded_states;
    	
    	foreach(s; models[0].S()) {
    		occluded_states[s] = 1.0;
    	}
    	
    	foreach(s; observableStates) {
    		occluded_states.remove(s);
    	}
    	
    	if (occluded_states.length != 0) {
	    	Distr!State.normalize(occluded_states);
	    	occluded_states.rehash;
	    			
    	}
    	
    	LinearReward [] ff;
    	
    	foreach (m; models)
    	  ff ~= cast(LinearReward)m.getReward();
    	
    	
    	double [JointStateAction] forward(double [JointStateAction] p_t, sar [] obs) {
    		double [JointStateAction] returnval;
     		State [] S0List;
     		State [] S1List;
     		
     		if (obs[0].s !is null) {
     			S0List = [obs[0].s];
     		} else 
     			S0List = occluded_states.keys;
     		if (obs[1].s !is null) {
     			S1List = [obs[1].s];
     		} else 
     			S1List = occluded_states.keys;
     		
    		foreach (s; S0List) {
	     		auto A0List = models[0].A(s);
	     		if (obs[0].a !is null) {
	     			A0List = [obs[0].a];
	     		}
    			foreach(a; A0List) {
		    		foreach (s2; S1List) {
			     		auto A1List = models[1].A(s2);
			     		if (obs[1].a !is null) {
			     			A1List = [obs[1].a];
			     		}
		    			foreach(a2; A1List) {		    				

		    				double temp_prob = 1.0;
		    				
		    				bool interacting = is_interacting(s, s2);
		    				
							if (interacting)
								temp_prob *= equilibrium[0].get(a, 0.00000001) * equilibrium[1].get(a2, 0.00000001);
							else
								temp_prob *= last_stochastic_policies[0].actions(s).get(a, 0.00000001) * last_stochastic_policies[1].actions(s2).get(a2, 0.00000001);
							
		    				foreach(jsa, prob; p_t) {
		    					
		    					temp_prob += models[0].T(jsa.s, jsa.a).get(s, 0.00000001) * models[1].T(jsa.s2, jsa.a2).get(s2, 0.00000001) * prob;
		    				}
		    				
		    				returnval[new JointStateAction(s, a, s2, a2)] = temp_prob;
		    			}
		    		}
		    	}
    		}
    		return returnval;			
    		
    	}
    	
    	double [JointStateAction] backward(double [JointStateAction] prev_b, sar [] obs, sar [] obs_minus_one) {
     		double [JointStateAction] returnval;
     		State [] S0List1;
     		State [] S1List1;
     		
     		if (obs_minus_one[0].s !is null) {
     			S0List1 = [obs_minus_one[0].s];
     		} else 
     			S0List1 = occluded_states.keys;
     		if (obs_minus_one[1].s !is null) {
     			S1List1 = [obs_minus_one[1].s];
     		} else 
     			S1List1 = occluded_states.keys;

    		foreach (s; S0List1) {
	     		auto A0List1 = models[0].A(s);
	     		if (obs_minus_one[0].a !is null) {
	     			A0List1 = [obs_minus_one[0].a];
	     		}
    			foreach(a; A0List1) {
		    		foreach (s2; S1List1) {
			     		auto A1List1 = models[1].A(s2);
			     		if (obs_minus_one[1].a !is null) {
			     			A1List1 = [obs_minus_one[1].a];
			     		}
		    			foreach(a2; A1List1) {
		    				
    						double total = 0;
    						
    						
				     		State [] S0List;
				     		State [] S1List;
				     		
				     		if (obs[0].s !is null) {
				     			S0List = [obs[0].s];
				     		} else 
				     			S0List = occluded_states.keys;
				     		if (obs[1].s !is null) {
				     			S1List = [obs[1].s];
				     		} else 
				     			S1List = occluded_states.keys;
				     		
				    		foreach (jsas; S0List) {
					     		auto A0List = models[0].A(jsas);
					     		if (obs[0].a !is null) {
					     			A0List = [obs[0].a];
					     		}
				    			foreach(jsaa; A0List) {
						    		foreach (jsas2; S1List) {
							     		auto A1List = models[1].A(jsas2);
							     		if (obs[1].a !is null) {
							     			A1List = [obs[1].a];
							     		}
						    			foreach(jsaa2; A1List) {
						    				JointStateAction jsa = new JointStateAction(jsas, jsaa, jsas2, jsaa2);
						    				
						    				auto prob = prev_b.get(jsa, 0.00000001);
						    				
						    										    				
			    							double actionprob = 0;
			    							bool interacting = is_interacting(jsa.s, jsa.s2);
					    				
											if (interacting)
												actionprob = equilibrium[0].get(jsa.a, 0.00000001) * equilibrium[1].get(jsa.a2, 0.00000001);
											else
												actionprob = last_stochastic_policies[0].actions(jsa.s).get(jsa.a, 0.00000001) * last_stochastic_policies[1].actions(jsa.s2).get(jsa.a2, 0.00000001);
											
			    							total += prob *  models[0].T(s, a).get(jsa.s, 0.00000001) * models[1].T(s2, a2).get(jsa.s2, 0.00000001) * actionprob;

						    				
						    			}
						    		}
						    	}
				    		}
    						
    						returnval[new JointStateAction(s, a, s2, a2)] = total;
    					}
		    		}
		    	}		
   			}
    		return returnval;
    	}
    	  
    	// need to do this for each sample
    	double [] returnval = new double[mu_E.length];
    	returnval[] = 0;
    	
    	foreach(Y_num; 0 .. true_samples[0].length) {
    		    		
	    	// prob, state/action, agent, timestep for forward value
	    	double [JointStateAction][] fv;
	    	
	    	// initialize prior
	    	
    		double [JointStateAction] temp;
    		foreach (s; models[0].S()) {
    			foreach(action; models[0].A(s)) {
		    		foreach (s2; models[1].S()) {
		    			foreach(action2; models[1].A(s2)) {
    						temp[new JointStateAction(s, action, s2, action2)] = last_stochastic_policies[0].actions(s).get(action, 0) * last_stochastic_policies[1].actions(s2).get(action2, 0);
    					}
		    		}
    			}
    		}
	    	Distr!JointStateAction.normalize(temp);
    		fv ~= temp;
    		debug{
    			writeln("Forward");
    		}
    		foreach(i; 0 .. t) {
    			// create ev at time t
    			sar[] ev;
    			foreach(agent_num, agent_traj; true_samples) {
    				if (agent_traj[Y_num].length > i)
    					ev ~= agent_traj[Y_num][i];
    				else
    					ev ~= sar(null, null, 0);	
    			}
	    		debug {
	    			writeln(ev);
	    		}
    			
    			fv ~= forward(fv[$-1], ev);
    		}
    		// prob, state/action, timestep for final vector
    		double [JointStateAction][] sv;
    		
    		debug{
    			writeln("Backward");
    		}
    		double [JointStateAction] b;
			foreach (s; models[0].S()) 
    			foreach(action; models[0].A(s)) 
		    		foreach (s2; models[1].S()) 
		    			foreach(action2; models[1].A(s2)) 
							b[new JointStateAction(s, action, s2, action2)] = 1.0;
    		for(long i = t-1; i >= 0; i --) {
				double[JointStateAction] temp_jsa;
				foreach(sasa, prob; b)
					temp_jsa[sasa] = fv[i+1].get(sasa, 0.00000001) * prob;
				
  				Distr!JointStateAction.normalize(temp_jsa);	
				sv ~= temp_jsa;	
				
	   			sar[] ev;
	    		foreach(agent_num, agent_traj; true_samples) {
    				if (agent_traj[Y_num].length > i)
    					ev ~= agent_traj[Y_num][i];
    				else
    					ev ~= sar(null, null, 0);	
    			}
				sar[] ev2;
	    		foreach(agent_num, agent_traj; true_samples) {
    				if (agent_traj[Y_num].length > i - 1 && i-1 >= 0)
    					ev2 ~= agent_traj[Y_num][i - 1];
    				else
    					ev2 ~= sar(null, null, 0);	
    			}	    		
	    		debug {
	    			writeln(ev, " ", ev2);
	    			
	    		}
	    		
	    		b = backward(b, ev, ev2);
    			
    		}
    		
			foreach(timestep, sasa_arr; sv) {
				foreach(sasa, prob; sasa_arr) {
					returnval[weight_map[0] .. weight_map[0] + ff[0].dim()] += prob * ff[0].features(sasa.s, sasa.a)[];
					returnval[weight_map[1] .. weight_map[1] + ff[1].dim()] += prob * ff[1].features(sasa.s2, sasa.a2)[];
				}	
			}
	    		
	    	
		}
    	
    	returnval[] /= true_samples[0].length;
    	
    	return returnval;
    	
    }	
}

class LatentMaxEntIrlZiebartApprox : LatentMaxEntIrlZiebartExact {
	
	
	struct AlphaBeta {
		double alpha;
		double beta;
	}
	
	AlphaBeta [] rafteryProbs;
	
	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1, size_t sample_length = 0) {
		super(max_iter, solver, observableStates, n_samples, error, solverError);
		this.sample_length = cast(int)sample_length;
	}
    
   	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
		
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
        this.true_samples = true_samples;
        
        rafteryProbs.length = true_samples.length;
        
        LinearReward r = cast(LinearReward)model.getReward();

        mu_E.length = r.dim();
        mu_E[] = 0;
        
        
        double ignored;        
              
        foreach(ref t; init_weights)
            t = abs(t);
        init_weights[] /= l1norm(init_weights);
      
        lastQValue = -double.max;
        bool hasConverged;
        opt_weights = init_weights.dup;
        
        mu_E.length = init_weights.length;
        
        auto saved_observableStates = this.observableStates;
        this.observableStates = model.S();
        
        ExpectedEdgeFrequencyFeatures(init_weights);
        
        this.observableStates = saved_observableStates;
        
        auto iterations = 0;
        // loop until convergence
        do {
	        mu_E = calc_E_step(true_samples);

	        debug {
	        	writeln("mu_E: ", mu_E);
	        }
	        
	        
            auto temp_init_weights = init_weights.dup;
	        
	        debug {
	        	writeln("Initial Weights ", temp_init_weights);
			}
	        
	        
	        // reset observableStates so that we consider all features
	        saved_observableStates = this.observableStates;
        	this.observableStates = model.S();

	        opt_weights = exponentiatedGradient(opt_weights.dup, 1, error, sample_length);
//	        opt_weights = exponentiatedGradient(temp_init_weights, 1, error);

	        
	        r.setParams(opt_weights);


//			need to approximate the Q value now, similar to how the dual is approximated
	//		This is done by integrating the objective with respect to the weights, it should give the original dual 


        	// calculate Q value
        	double newQValue = calcQ(opt_weights);
        	
        	this.observableStates = saved_observableStates;

	        debug {
	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", opt_weights);
	        	
	        }
	        hasConverged = (abs(newQValue - lastQValue) <= error) || l2norm(opt_weights) / opt_weights.length > 70;
	        
	        lastQValue = newQValue;
	        
	        iterations ++;
	    } while (! hasConverged && iterations < max_iter);
        
//        writeln("EM Iterations, ", iterations);
        (cast(LinearReward)model.getReward()).setParams(opt_weights);
        
        opt_value = -lastQValue;
		
        return solver.createPolicy(model, solver.solve(model, solverError));
				
	}

	double calcQ(double [] weights) {
		auto features = ExpectedEdgeFrequencyFeatures(weights);
		
		features[] *= weights[];
				
		double returnval = -1 * reduce!("a + b")(0.0, features);
		
		debug {
			writeln("Q: log(Z) ", returnval, " for Features ", features);
			
		}
		
		double [] expectation = mu_E.dup;
		expectation[] *= weights[];
		
		returnval += reduce!("a + b")(0.0, expectation);
		
				
		return returnval;
		
		
	}

    double [] calc_E_step(sar[][] true_samples) {
    	import std.mathspecial;

    	// using the last policy we have and the list of Y's, generate a new set of expert feature expectations, using gibbs sampling
    	// to generate trajectory samples, 
    	
    	double [] returnval = new double[mu_E.length];
    	returnval[] = 0;
    	
    	
    	double q = 0.025; // quartile the true value is expected in, this is extremely conservative to ensure we get enough samples (ie, 0.5 would be the easiest for the sampler to estimate and so would have the least # of samples required)
    	double r = 0.0025; // acceptable error, the true rate should be + or - this 
    	double s = 0.99; // probability with which we have estimated the true rate
    	
    	double inv_erf = 1.959984301;
    	size_t repeats = 0;
    	double [] last_avg = returnval.dup;
    	last_avg[] = 0;
    	
    	while(true) {
    		
			double [] temp = new double[mu_E.length];
    		temp[] = 0;
	   		foreach(i, sample; true_samples) {
	    		
	    		// assign the burn in (m) and sample count (n) values from Raftery and Lewis 1992
		    	RunningAverageConvergenceThreshold convergenceTest = new RunningAverageConvergenceThreshold(error / 200, 20);
	    		
	    		if (isNaN(rafteryProbs[i].alpha)) {
	    			// need to do an initial sample in order to estimate alpha and beta
	    			size_t N_min = cast(size_t)((pow(inv_erf, 2) * q*(1 - q)) / (r*r));
	    			
	    			debug(raftery) {
	    				
	    				writeln(i, " N_min: ", N_min);
	    			} 
	    			
	    			gibbs_sampler(sample, model, last_stochastic_policy, i, 0, N_min, null);
	    		}
	    		
	    		
	    		size_t m = cast(size_t) (log( r * (rafteryProbs[i].alpha + rafteryProbs[i].beta) / (fmax(rafteryProbs[i].alpha, rafteryProbs[i].beta))) / log(fabs(1 - rafteryProbs[i].alpha - rafteryProbs[i].beta)));
	    		size_t n = cast(size_t) ( ( (rafteryProbs[i].alpha * rafteryProbs[i].beta * (2 - rafteryProbs[i].alpha - rafteryProbs[i].beta)) / (pow(rafteryProbs[i].alpha + rafteryProbs[i].beta, 3) * r * r) ) * pow(inv_erf, 2) );
	    		
	    		debug(raftery) {
	    			writeln(i, " M: ", m, " N: ", n);
	    		}	
	    		
	    		temp[] += gibbs_sampler(sample, model, last_stochastic_policy, i, cast(size_t)(2*sample.length * 0.1), n, convergenceTest)[];
	    	}
	    	
	    	debug(raftery) {
	    		writeln(rafteryProbs);
	    		
	    	}
	    	repeats ++;
	    	temp[] /= true_samples.length;
	    	
	    	returnval[] += temp[];
	    	
	    	double [] new_avg = returnval.dup;
	    	new_avg[] /= repeats;
	    	
	    	
	    	double max_diff = -double.max;
	    	
	    	foreach(i, k; new_avg) {
	    		auto tempdiff = abs(k - last_avg[i]);
	    		if (tempdiff > max_diff)
	    			max_diff = tempdiff;
	    		
	    	} 
//	    	writeln(max_diff, " ", new_avg, " ", temp, " ", error);
	    	if (max_diff < error && repeats > 10) {
	    		debug {
	    			writeln("Converged after ", repeats, " repeats");
	    		}	
	    		break;
	    		
	    	}
//	    	if (repeats > 30) 
//	    		break;
	    	
	    	last_avg = new_avg;
	    	
    	}
    	returnval[] /= repeats;
    	return returnval;
    }
    
    double [] gibbs_sampler(sar [] sample, Model model, Agent policy, size_t Y_num, size_t M, size_t N, RunningAverageConvergenceThreshold convergenceTest) {
    	
    	// when samping a state, we need to also consider that only unobservable states can be selected from
    	// if this results in no feasible samples, then delete the problem part of the sample and continue
    	
    	double[State] occluded_states;
    	
    	foreach(s; model.S()) {
    		occluded_states[s] = 1.0;
    	}
    	
    	foreach(s; observableStates) {
    		occluded_states.remove(s);
    	}
    	
    	if (occluded_states.length != 0) {
	    	Distr!State.normalize(occluded_states);
	    	occluded_states.rehash;
	    			
    	}
    	
    	// just assume there are no errors right now, will probably have to fix the patrolling experiment 
    	LinearReward ff = cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[mu_E.length];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		
		double [] min_features = new double[mu_E.length];
		min_features[] = 0;
		
		foreach(i, SAR; sample) {
			if (SAR.s is null) {
				missing_timesteps ~= i;
			} else {
				min_features[] += ff.features(SAR.s, SAR.a)[];
			}
		}
		
		if (missing_timesteps.length == 0) {
			// Z is empty, don't need sampling
			rafteryProbs[Y_num].alpha = 1;
			rafteryProbs[Y_num].beta = 1;
			
			return min_features;
			
		} 
		M = cast(size_t)(0.1 * missing_timesteps.length * 2);

		double [] max_features = min_features.dup;
		max_features[] += missing_timesteps.length;
		
		double binaryControlCutoffPercent = 0.5;
		double binaryControlCutoff = l2norm(min_features) + binaryControlCutoffPercent * (l2norm(max_features) - l2norm(min_features));  
		
		
		auto working_sample = sample.dup;
		
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(idx; missing_timesteps) {
			if (idx > 0)
				working_sample[idx].s = Distr!State.sample(model.T(working_sample[idx-1].s, working_sample[idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = this.initial.dup;
				foreach(s, ref pr; initial_occluded)
					pr *= occluded_states.get(s, 0);
				
				// guard against empty initial distr
				double total = 0;
		
				foreach (key, val ; initial_occluded) {
					total += val;
				} 	
				
				if (total == 0) {
					// no valid starting states, just use the occluded ones
					initial_occluded = occluded_states.dup;
					
				}
				
				Distr!State.normalize(initial_occluded);	
					 
				working_sample[idx].s = Distr!State.sample(initial_occluded);
			}	
				
			working_sample[idx].a = policy.sample(working_sample[idx].s);
			
			cur_features[] += ff.features(working_sample[idx].s, working_sample[idx].a)[];
		}
		
		// then loop for N times, generating a new sample, calculating feature expectations, saving them for later
		 
		double[] total_features = min_features.dup;
		total_features[] = 0;
		
		size_t alpha_count = 1;
		size_t alpha_total_count = 1;
		size_t beta_count = 1;
		size_t beta_total_count = 1;
		bool lastState = false;
		
/*		if (N < 10000)
			N = 10000; 
		
		if (N > 400000)
			N = 400000;*/
		
		size_t i = 0;
		size_t j = 0;
		
		
//		foreach(i; 0..(N + M)) {
		while (true) {
			
			auto r = uniform(0.0, .999);
			
			size_t timestep = missing_timesteps[cast(size_t)(r * missing_timesteps.length)];
			
			cur_features[] -= ff.features(working_sample[timestep].s, working_sample[timestep].a)[];
			
			if (uniform(0.0, .999) < 0.5) {
				// sample the state from T(s, a, s') T(s'', a', s''') Pr(a|s)
				
				double[State] state_distr = occluded_states.dup;
				
				foreach(s, ref v; state_distr) {
					if (timestep > 0) {
						v *= model.T(working_sample[timestep-1].s, working_sample[timestep-1].a).get(s, 0.00000001); 						
					}
					
					if (timestep < working_sample.length - 1) {
						v *= model.T(s, working_sample[timestep].a).get(working_sample[timestep+1].s, 0.00000001); 
					}
					
					v *= policy.actions(s).get(working_sample[timestep].a, 0.00000001);
				}
				
				try {
					Distr!State.normalize(state_distr);
				} catch (Exception e) {
					state_distr = occluded_states.dup;
				}
				
				working_sample[timestep].s = Distr!State.sample(state_distr);
				
				
			} else {
				// sample the action from T(s, a, s') Pr(a|s)
				
				double[Action] action_distr = policy.actions(working_sample[timestep].s).dup;
				
				foreach(a, ref v; action_distr) {
					
					if (timestep < working_sample.length - 1) {
						v *= model.T(working_sample[timestep].s, a).get(working_sample[timestep+1].s, 0.00000001); 
					}
				}
				
				Distr!Action.normalize(action_distr);
				
				working_sample[timestep].a = Distr!Action.sample(action_distr);
				
				
			}
			
			cur_features[] += ff.features(working_sample[timestep].s, working_sample[timestep].a)[];
			
			// classify the new trajectory based on its feature count, update alpha and beta counts
			bool newState = l2norm(cur_features) < binaryControlCutoff;
			
			if (lastState) {
				beta_total_count ++; // last was 1
				
				if (!newState) 
					beta_count ++;  // transitioned from 1 to 0
			} else {
				alpha_total_count ++; // last was 0
				
				if (newState)
					alpha_count ++; // transitioned from 0 to 1
			}
			
			
			if (j > M) {
				total_features[] += cur_features[];
				
				if (convergenceTest is null && i >= N + M) {
					break;
				}
				i ++;
				auto temp = total_features.dup;
				temp[] /= i;
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					debug(raftery) {
						writeln("Converged after ", i, " iterations. Raftery params: M: ", M, " N: ", N);
						if (i == 0) {
							writeln( cur_features);
							writeln(convergenceTest.storage);
							writeln(convergenceTest.last);
						}
					}
					break;
				}				
				j = 0;
			}	
			
			lastState = newState;
			j ++;	
		}
		
		
		// now average the feature expectations
		total_features[] /= i;
		
		// update the alpha and beta values for this sample
		
		debug(raftery) {
			writeln("Alphas: ", alpha_count, " ", alpha_total_count, "  Betas: ", beta_count, " ", beta_total_count);
			
		}
		// this is a weird convergence problem, if we converge on one set of trajectories the estimated samples needed goes really high
		rafteryProbs[Y_num].alpha = cast(double)(alpha_count) / cast(double)(alpha_total_count);
		rafteryProbs[Y_num].beta = cast(double)(beta_count) / cast(double)(beta_total_count);

		return total_features;
    	
    }
}

class LatentActionsMaxEntIrlZiebartExact : LatentMaxEntIrlZiebartExact {

	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1) {
		super(max_iter, solver, observableStates, n_samples, error, solverError);
	}


	override void calc_y_and_z(sar[][] true_samples) {
		
		// generate one complete trajectory, depth first
		// record Y for this trajectory
		// get features 
		// add to complete list
        LinearReward ff = cast(LinearReward)model.getReward();

        this.sample_length = 0;
        
		foreach(i, traj; true_samples) {
		
			if (traj.length > this.sample_length) 
				this.sample_length = traj.length;
			
			
			double[State][Action][] storage_stack;
			double[] pr_working_stack;
			double[][] fe_working_stack;
			sar[] traj_working_stack;
			
			storage_stack.length = traj.length;
			
			// is the first position empty?  If so, intiialize with initial
			if (traj[0].a is null) {
				// add all states in initial, but only if they're not visible

				foreach(a; model.A(traj[0].s)) {
					storage_stack[0][a][traj[0].s] = 1.0/model.A(traj[0].s).length;
				}
				
			} else {
				storage_stack[0][traj[0].a][traj[0].s] = traj[0].p;
			}
			
			sar[] hist;
			long cursor = 0;

			outer_loop: while (true) {
				
				/*
				
				when you push onto the working stacks, check if the next position's storage stack is empty, if so generate entries for it

				each step, check if the pr_traj is zero, if so pop the current working stack and continue
				
				if current pointer is past the end, we have a complete traj, record to output, backup, and pop working stacks
				
				if the current pointer storage stack is empty then backup, popping working stacks as we go until we find a 
				non-empty storage stack and pop or we 	go past the beginning, in which case we're done.
					
				
				idea is to build up the working stacks until we get to the end
				only pop working stacks on backup
				pop storage stack onto working stack
				*/
				
				
				while (cursor < traj.length) {
					
					// backtrack until we find some stack entries
					while (storage_stack[cursor].length == 0) {
//						writeln(cursor);
						cursor --;
						if (cursor < 0) {
							// we're done!
							break outer_loop;
						}
						pr_working_stack.length = pr_working_stack.length - 1;
						fe_working_stack.length = fe_working_stack.length - 1;
						traj_working_stack.length = traj_working_stack.length - 1;		

						
					}
					
					// pop entry off the storage stack or observed traj
					
					// place into working stack
					
					auto first_action = storage_stack[cursor].keys()[0];
					auto first_state = storage_stack[cursor][first_action].keys()[0];
					
					storage_stack[cursor][first_action].remove(first_state);
					if (storage_stack[cursor][first_action].length == 0) {
						storage_stack[cursor].remove(first_action);
					}
					
					auto SAR = sar(first_state, first_action, 1.0);
					traj_working_stack ~= SAR;
					
					
					// update fe and pr
					
					double [] features = ff.features(traj_working_stack[$-1].s, traj_working_stack[$-1].a);
					
					if (cursor == 0) {
						fe_working_stack ~= features;
					} else {
						fe_working_stack ~= fe_working_stack[$-1].dup;
						fe_working_stack[$-1][] += features[];
					}						
					
					double val = 0;
					if (cursor >= 1) {
		        		auto transitions =  model.T(traj_working_stack[$-2].s, traj_working_stack[$-2].a);
		        		
		        		foreach (state, pr; transitions) {
		        			if (traj_working_stack[$-1].s == state) {
		        				val = pr_working_stack[$-1] * pr;
		        				break;
		        			}
		        		}
	        		} else {
	        			val = 1.0; // Should this be initial(s)?
	        		}
	        		
	        		pr_working_stack ~= val;
		        	
					// is pr 0? if so, pop working stack and continue
					if (val == 0) {
						// no point in continuing this trajectory
						
						pr_working_stack.length = pr_working_stack.length - 1;
						fe_working_stack.length = fe_working_stack.length - 1;
						traj_working_stack.length = traj_working_stack.length - 1;						 
						
						continue;
						
					}
					
					
					// is the next timestep hidden? if so, generate possible states using the current one and place in storage
					
					if (cursor < traj.length - 1) {
						if (traj[cursor + 1].a is null && storage_stack[cursor + 1].length == 0) {

							foreach (Action action; model.A(traj[cursor + 1].s)) {
								storage_stack[cursor + 1][action][traj[cursor + 1].s] = 1.0/model.A(traj[cursor + 1].s).length;
							}
	
							
						} else if (traj[cursor + 1].a ! is null && storage_stack[cursor + 1].length == 0) {
							storage_stack[cursor + 1][traj[cursor + 1].a][traj[cursor + 1].s] = traj[cursor + 1].p;
							
						}
						
					}
					
				//	writeln(traj_working_stack);
					
					cursor ++;
					
				}
				
				
				// add trajectory to feature and pr vector 
	        	
	        	y_from_trajectory ~= i;
	        	feature_expectations_per_trajectory ~= fe_working_stack[$-1];
	        	pr_traj ~= pr_working_stack[$-1];
	        	
				// pop working stacks
				pr_working_stack.length = pr_working_stack.length - 1;
				fe_working_stack.length = fe_working_stack.length - 1;
				traj_working_stack.length = traj_working_stack.length - 1;
				
				cursor --;
				 
			}
		}
		
		
	}



}

class LatentMaxEntIrlZiebartDynamicOcclusionApprox : LatentMaxEntIrlZiebartApprox {

	protected State [][] observableStatesArray;

	
	public this(int max_iter, MDPSolver solver, State [][] observableStatesArr, int n_samples=500, double error=0.1, double solverError =0.1, size_t sample_length = 0) {
		
		super(max_iter, solver, null, n_samples, error, solverError, sample_length);
		this.observableStatesArray = observableStatesArr;
	}
   	
    override double [] gibbs_sampler(sar [] sample, Model model, Agent policy, size_t Y_num, size_t M, size_t N, RunningAverageConvergenceThreshold convergenceTest) {
    	
    	observableStates = observableStatesArray[Y_num];
    	return super.gibbs_sampler(sample, model, policy, Y_num, M, N, convergenceTest);
    }

	
}


class LatentMaxEntIrlZiebartDynamicOcclusionExact : LatentMaxEntIrlZiebartExact {

	protected State [][] observableStatesArray;


	public this(int max_iter, MDPSolver solver, State [][] observableStatesArr, int n_samples=500, double error=0.1, double solverError =0.1) {
		super(max_iter, solver, null, n_samples, error, solverError);
		this.observableStatesArray = observableStatesArr;
	}
	
	
	override void calc_y_and_z(sar[][] true_samples) {
		

		
		// actually, I don't think we should do this, we want repeats to be given higher counts
		/*
		sar [][] Y;
		
		// first check that each sample is not a duplicate
		foreach(traj; true_samples) {
			bool is_new = true;
			foreach(y; Y) {
				
				if (y.length != traj.length)
					break;
				
				bool all_found = true;	
				foreach(i, SAR; y) {
					if (SAR.s != traj[i].s || SAR.a != traj[i].a) {
						all_found = false;
						break;
					}
				}
				
				if (all_found) {
					is_new = false;
					break;
				}
				
			}
			
			if (is_new)
				Y ~= traj.dup;
		}
		*/
		
		// generate one complete trajectory, depth first
		// record Y for this trajectory
		// get features 
		// add to complete list
        LinearReward ff = cast(LinearReward)model.getReward();

        this.sample_length = 0;

		foreach(i, traj; true_samples) {
			
			if (traj.length > this.sample_length)
				this.sample_length = traj.length;
			
			double[State][Action][] storage_stack;
			double[] pr_working_stack;
			double[][] fe_working_stack;
			sar[] traj_working_stack;
			
			storage_stack.length = traj.length;
			
			bool is_visible(State s) {
				foreach(s2; observableStatesArray[i]) {
					if (s2.samePlaceAs(s))
						return true;
				}
				return false;
				
			}
			
			
			// is the first position empty?  If so, intiialize with initial
			if (traj[0].s is null) {
				// add all states in initial, but only if they're not visible
				
				foreach(i_s, pr; initial) {
					
					if (! is_visible(i_s)) {
						if (model.is_terminal(i_s)) {
							storage_stack[0][new NullAction()][i_s] = pr;
						} else {
							foreach(a; model.A(i_s)) {
								storage_stack[0][a][i_s] = pr;
							}	
						}
					}
					
					
				}
				
			} else {
				storage_stack[0][traj[0].a][traj[0].s] = traj[0].p;
			}
			
			sar[] hist;
			long cursor = 0;

			outer_loop: while (true) {
				
				/*
				
				when you push onto the working stacks, check if the next position's storage stack is empty, if so generate entries for it

				each step, check if the pr_traj is zero, if so pop the current working stack and continue
				
				if current pointer is past the end, we have a complete traj, record to output, backup, and pop working stacks
				
				if the current pointer storage stack is empty then backup, popping working stacks as we go until we find a 
				non-empty storage stack and pop or we 	go past the beginning, in which case we're done.
					
				
				idea is to build up the working stacks until we get to the end
				only pop working stacks on backup
				pop storage stack onto working stack
				*/
				
				
				while (cursor < traj.length) {
					
					// backtrack until we find some stack entries
					while (storage_stack[cursor].length == 0) {
//						writeln(cursor);
						cursor --;
						if (cursor < 0) {
							// we're done!
							break outer_loop;
						}
						pr_working_stack.length = pr_working_stack.length - 1;
						fe_working_stack.length = fe_working_stack.length - 1;
						traj_working_stack.length = traj_working_stack.length - 1;		

						
					}
					
					// pop entry off the storage stack or observed traj
					
					// place into working stack
					
					auto first_action = storage_stack[cursor].keys()[0];
					auto first_state = storage_stack[cursor][first_action].keys()[0];
					
					storage_stack[cursor][first_action].remove(first_state);
					if (storage_stack[cursor][first_action].length == 0) {
						storage_stack[cursor].remove(first_action);
					}
					
					auto SAR = sar(first_state, first_action, 1.0);
					traj_working_stack ~= SAR;
					
					
					// update fe and pr
					
					double [] features = ff.features(traj_working_stack[$-1].s, traj_working_stack[$-1].a);
					
					if (cursor == 0) {
						fe_working_stack ~= features;
					} else {
						fe_working_stack ~= fe_working_stack[$-1].dup;
						fe_working_stack[$-1][] += features[];
					}						
					
					double val = 0;
					if (cursor >= 1) {
		        		auto transitions =  model.T(traj_working_stack[$-2].s, traj_working_stack[$-2].a);
		        		
		        		foreach (state, pr; transitions) {
		        			if (traj_working_stack[$-1].s == state) {
		        				val = pr_working_stack[$-1] * pr;
		        				break;
		        			}
		        		}
	        		} else {
	        			val = 1.0; // Should this be initial(s)?
	        		}
	        		
/*	        		if (val == 0) { // prevent noisy trajectories from breaking the feature expectations
	        			val = .0001;
	        		}
	*/        		
	        		pr_working_stack ~= val;
		        	
					// is pr 0? if so, pop working stack and continue
					if (val == 0) {
						// no point in continuing this trajectory
						
						pr_working_stack.length = pr_working_stack.length - 1;
						fe_working_stack.length = fe_working_stack.length - 1;
						traj_working_stack.length = traj_working_stack.length - 1;						 
						
						continue;
						
					}
					
					
					// is the next timestep hidden? if so, generate possible states using the current one and place in storage
					
					if (cursor < traj.length - 1) {
						while (cursor < traj.length - 1 && storage_stack[cursor + 1].length == 0) {

							if (traj[cursor + 1].s ! is null && storage_stack[cursor + 1].length == 0) {
								storage_stack[cursor + 1][traj[cursor + 1].a][traj[cursor + 1].s] = traj[cursor + 1].p;
								
							} else {
								foreach(State newS, double newP; model.T(traj_working_stack[cursor].s, traj_working_stack[cursor].a)) {
									
									if (! is_visible(newS)) {
										if (model.is_terminal(newS)) {
											storage_stack[cursor + 1][new NullAction()][newS] = newP;	
										} else {	
											foreach (Action action; model.A(newS)) {
												storage_stack[cursor + 1][action][newS] = newP;
											}
										}
									}
								
								}
								if ( storage_stack[cursor + 1].length == 0) {
									// Observation error, no possible way the agent wasn't observed (according to our model)
									
									// add every state with equal probability, hopefully not screwing up the feature expectations too much
/*									foreach(tempState; model.S()) {
										storage_stack[cursor + 1][new NullAction()][tempState] = 1.0/model.S().length;
									}*/
									
									auto tempTraj = traj[0..cursor+1];
																		
									if (cursor < traj.length - 2)
										tempTraj ~= traj[cursor + 2 .. $];
									
									true_samples[i] = tempTraj;
									traj = tempTraj;
																		
									this.sample_length = 0;
									foreach(tt; true_samples) {
										if (tt.length > this.sample_length)
											this.sample_length = tt.length;
										
									}
									
								}

							}
						
						} 
						
					}
					/*
					writeln(traj_working_stack);
					writeln(fe_working_stack);
					writeln(pr_working_stack);*/
					cursor ++;
					
				}
				
				// add trajectory to feature and pr vector 
	        	
	        	y_from_trajectory ~= i;
	        	feature_expectations_per_trajectory ~= fe_working_stack[$-1];
	        	pr_traj ~= pr_working_stack[$-1];
	        	
				// pop working stacks
				pr_working_stack.length = pr_working_stack.length - 1;
				fe_working_stack.length = fe_working_stack.length - 1;
				traj_working_stack.length = traj_working_stack.length - 1;
				
				cursor --;
				 
			}
		}
		
/*		writeln(y_from_trajectory);
		writeln(feature_expectations_per_trajectory);*/
		
	}

	
}

class LatentMaxEntIrlZiebartPolicyApprox : LatentMaxEntIrlZiebartApprox {
	
	public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, double error=0.1, double solverError =0.1, size_t sample_length = 0, double qval_thresh = 0.01) {
		super(max_iter, solver, observableStates, n_samples, error, solverError);
		this.sample_length = cast(int)sample_length;
		this.qval_thresh = qval_thresh;
	}

   	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
		
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
        this.true_samples = true_samples;
        
        rafteryProbs.length = true_samples.length;
        
        LinearReward r = cast(LinearReward)model.getReward();

        mu_E.length = r.dim();
        mu_E[] = 0;
        
        
        double ignored;        
        
        
        foreach(ref t; init_weights)
            t = abs(t);
        init_weights[] /= l1norm(init_weights);
	    
              
        lastQValue = -double.max;
        bool hasConverged;
        opt_weights = init_weights.dup;
        
        mu_E.length = init_weights.length;
        
        ExpectedEdgeFrequencyFeatures(init_weights);
        
        auto iterations = 0;
        // loop until convergence
        do {
	        mu_E = calc_E_step(true_samples);

	        debug {
	        	writeln("mu_E: ", mu_E);
	        }
	        
	        
            auto temp_init_weights = init_weights.dup;
	        
	        debug {
	        	writeln("Initial Weights ", temp_init_weights);
			}
	        

	        opt_weights = exponentiatedGradient(opt_weights.dup, 0.33, error, sample_length);
//	        opt_weights = exponentiatedGradient(temp_init_weights, 0.5, error);

	        
	        r.setParams(opt_weights);


//			need to approximate the Q value now, similar to how the dual is approximated
	//		This is done by integrating the objective with respect to the weights, it should give the original dual 


        	// calculate Q value
        	double newQValue = calcQ(opt_weights);
	        debug {

	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", opt_weights);
	        	
	        
		        hasConverged = (abs(newQValue - lastQValue) <= error);
		        
	        }
	        lastQValue = newQValue;
        	
	        iterations ++;
	    } while (! hasConverged && iterations < max_iter);
        
//        writeln("EM Iterations, ", iterations);
        (cast(LinearReward)model.getReward()).setParams(opt_weights);
        
        opt_value = -lastQValue;
		
        return solver.createPolicy(model, solver.solve(model, solverError));
				
	}
   	
	override double [] ExpectedEdgeFrequencyFeatures(double [] weights) {

		// return the feature expectations
		
    	LinearReward r = cast(LinearReward)model.getReward();
    	
    	auto mag_weights = weights.dup;
    	mag_weights[] *= l2norm(mu_E);
    	
    	r.setParams(mag_weights);
/*    	r.setParams(weights);*/
    	
//    	double[StateAction] Q_value = QValueSolve(model, qval_thresh);		
    	double[StateAction] Q_value = QValueSoftMaxSolve(model, qval_thresh, this.sample_length);        
        Agent policy = CreateStochasticPolicyFromQValue(model, Q_value);
//		Agent policy = CreatePolicyFromQValue(model, Q_value);
        
        double [] returnval = new double[r.dim()];
        returnval[] = 0;

        if (this.n_samples > 0) { 
        	
        	double [] total = returnval.dup;
        	double [] last_avg = returnval.dup;
        	size_t repeats = 0;
        	while(true) {
        	
	    		sar [][] samples = generate_samples(model, policy, initial, this.n_samples, sample_length);

	        	total[] += feature_expectations(model, samples)[];
	        
	        	repeats ++;
	        	
	        	double [] new_avg = total.dup;
	        	new_avg[] /= repeats;
	        	
		    	double max_diff = -double.max;
		    	
		    	foreach(i, k; new_avg) {
		    		auto tempdiff = abs(k - last_avg[i]);
		    		if (tempdiff > max_diff)
		    			max_diff = tempdiff;
		    		
		    	} 
		    	
		    	if (max_diff < 0.01) {
		    		debug {
///		    			writeln("mu Converged after ", repeats, " repeats, ", n_samples * repeats, " simulations");
		    		}	
		    		break;
		    		
		    	}	  
		    	
		    	last_avg = new_avg;      	
	        	
	        }
        	returnval[] = total[] / repeats;
	        
        } else {
//        	auto Ds = calcStateFreq(policy, initial, model, sample_length);
        	
        	
        	size_t N = this.sample_length;
        	
			double [State][] D;
			
			D ~= initial.dup;
			
			foreach (t; 1..N) {
				double [State] temp;
				foreach (k; model.S()) {
					temp[k] = 0.0;
					foreach(i; model.S()) {
						if (model.is_terminal(i)) {
							if (i == k)
								temp[k] += D[t-1].get(k, 0.0);
						} else {
							foreach(j; model.A(i)) {
								temp[k] += D[t-1].get(i, 0.0) * policy.actions(i).get(j, 0.0000001) * model.T(i, j).get(k, 0.0);
							}
						}
					}
				}
				D ~= temp;
			
			}
			returnval[] = 0;
			
			double [State] Ds;
			foreach(t; 0..N) {
				foreach (i; model.S()) {
					if (t == 0)
						Ds[i] = 0.0;
					Ds[i] += D[t].get(i, 0.0);
				}
			}
			
			foreach (i; model.S()) {	
				if (model.is_terminal(i)) {
					returnval[] += Ds.get(i, 0.0) * r.features(i, new NullAction())[];
				} else {
					foreach (j; model.A(i)) {
						returnval[] += Ds.get(i, 0.0) * policy.actions(i).get(j, 0.0000001) * r.features(i, j)[]; 
					}
				}
			}        	
        	
        	
        }
        
        last_stochastic_policy = policy; 
        
        return returnval;
	}
	
    override double [] feature_expectations(Model model, sar [][] samples) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		double [] f = ff.features(SAR.s, SAR.a);
        		returnval[] += f[];
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}	
    
/*
	override double [] exponentiatedGradient(double [] w, double c, double err) {
		c = 0.20;
		double [] newW = w.dup;
		double [] oldW;
		double diff;
		double lastdiff = 0;
		double [] delta_W = new double[newW.length];
		delta_W[] = 0;
		
		do {
			
			oldW = newW.dup;
			
			auto y_prime = ExpectedEdgeFrequencyFeatures(oldW);
			
			debug {
//				writeln(y_prime);
			}
			double [] test = y_prime.dup;
			test[] -= mu_E[];
			
			
			diff = -double.max;
			foreach(t; test) {
				if (abs(t) > diff)
					diff = abs(t);
			}
			
			newW[] = oldW[] - c * test[] + 0.25*delta_W[];
			
			
			debug {
				writeln(y_prime, " -> ", diff, " ", newW, " ", c);
			}
//			c /= 1.15;
			if (diff > lastdiff) {
				c /= 2;
			} else {
				c *= 1.01;
			}
			if (abs(lastdiff - diff) < .01 || diff < err)
				break;	
			lastdiff = diff;	
			delta_W = newW.dup;
			delta_W[] -= oldW[];
			
		} while(true);
		
		return newW;
		
	}
*/

}

class MaxEntIrlExact : MaxEntIrl {
	
	protected double qval_thresh;
	public Model model;
	protected double[State] initial;
	protected sar[][] true_samples;
	protected int sample_length;
	protected double [] mu_E;
	protected size_t non_terminal_states;
	
	protected double[][] feature_expectations_for_policies;
	
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError);

		this.qval_thresh = qval_thresh;
	}


    public void setFeatureExpectations(double [][] feature_expectations) {
    	feature_expectations_for_policies = feature_expectations;
    }	
    
	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
		
        // Compute feature expectations of agent = mu_E from samples
        
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.max_iterations = max_iter;
//        param.epsilon = error;
        param.min_step = .00001; 
        
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
        this.true_samples = true_samples;
        this.sample_length = cast(int)true_samples.length;
        
        count_non_terminal_states(model);
        
        mu_E = feature_expectations2(model, true_samples, 0);
        
        
        feature_expectations_for_policies = all_feature_expectations(model);
        
        debug {
        	writeln("All feature expectations calculated");
        	
        }
        
        opt_weights = init_weights.dup;
                
        double * x = lbfgs_malloc(cast(int)opt_weights.length);
        scope(exit) {
        	lbfgs_free(x);
        } 
        
   		debug {
   			writeln("True Samples ", mu_E, " L: ", sample_length);
   		}
        
        LinearReward r = cast(LinearReward)model.getReward();


		nelderMeadReference = this;
		
		double reqmin = 1.0E-06;
		
		
		double step[] = new double[init_weights.length];
		step[] = 1;
		
        auto temp_init_weights = init_weights.dup;
        
        for (int i = 0; i < opt_weights.length; i ++) 
        	x[i] = temp_init_weights[i];
        
        auto temp = this;
     	int ret = lbfgs(cast(int)opt_weights.length, x, &opt_value, &evaluate_maxent, &progress, &temp, &param);


        for (int i = 0; i < opt_weights.length; i ++)
        	opt_weights[i] = x[i];
		
		
		/*
		int konvge = 3;
		int kcount = max_iter;
		int icount;
		int numres;
		int ifault;
		
		
		opt_value = evaluate_nelder_mead ( init_weights.ptr, cast(int)init_weights.length );
		
		opt_weights.length = init_weights.length;
		nelmin ( &evaluate_nelder_mead, cast(int)init_weights.length, init_weights.ptr, opt_weights.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );
*/
		
        						
		if (l2norm(opt_weights) / opt_weights.length > 70) {
			opt_weights[] /= l2norm(opt_weights);
			opt_weights[] *= 69;
			
		}
		
        (cast(LinearReward)model.getReward()).setParams(opt_weights);
        
        return solver.createPolicy(model, solver.solve(model, solverError));
		
 
	}

public Agent solve_exact(Model model, double[State] initial, double[] policy_distr, double[][] feature_expectations, double [] init_weights, out double opt_value, out double [] opt_weights, int algorithm = 1) {
		
        // Compute feature expectations of agent = mu_E from samples
        
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.max_iterations = max_iter;
//        param.epsilon = error;
        param.min_step = .00001; 
        
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
    
        count_non_terminal_states(model);
        
        mu_E = full_feature_expectations_exact(model, policy_distr);
        
        /*
        feature_expectations_for_policies = all_feature_expectations(model);
        return null;
        debug {
        	writeln("All feature expectations calculated");
        	
        }*/
        
        opt_weights = init_weights.dup;
                
        double * x = lbfgs_malloc(cast(int)opt_weights.length);
        scope(exit) {
        	lbfgs_free(x);
        } 
        
   		debug {
   			writeln("True Samples ", mu_E, " L: ", sample_length);
   		}
        
        LinearReward r = cast(LinearReward)model.getReward();


		nelderMeadReference = this;
		
		double reqmin = 1.0E-06;
		
		
		double step[] = new double[init_weights.length];
		step[] = 1;
		
        auto temp_init_weights = init_weights.dup;
        
        if (algorithm == 0) {
	        for (int i = 0; i < opt_weights.length; i ++) 
	        	x[i] = temp_init_weights[i];
	        
	        auto temp = this;
	     	int ret = lbfgs(cast(int)opt_weights.length, x, &opt_value, &evaluate_maxent, &progress, &temp, &param);
	
	
	        for (int i = 0; i < opt_weights.length; i ++)
	        	opt_weights[i] = x[i];
		
		} else {
		
			int konvge = 3;
			int kcount = max_iter;
			int icount;
			int numres;
			int ifault;
			
			
			opt_value = evaluate_nelder_mead ( init_weights.ptr, cast(int)init_weights.length );
			
			opt_weights.length = init_weights.length;
		nelmin ( &evaluate_nelder_mead, cast(int)init_weights.length, init_weights.ptr, opt_weights.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );
		}
		
        						
		if (l2norm(opt_weights) / opt_weights.length > 70) {
			opt_weights[] /= l2norm(opt_weights);
			opt_weights[] *= 69;
			
		}
		
        (cast(LinearReward)model.getReward()).setParams(opt_weights);
        
        return solver.createPolicy(model, solver.solve(model, solverError));
		
 
	}


    double [] full_feature_expectations_exact(Model model, double[] policy_distr) {

    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
    	foreach(i, pr; policy_distr) {
    		
    		returnval[] += (pr * feature_expectations_for_policies[i][]);
    	
    	}

        return returnval;
	}	
    
	
	void count_non_terminal_states(Model m) {
		size_t a = 0;
		
		foreach(s; m.S()) {
			if (! m.is_terminal(s)) {
				a ++;
			}
		}
		
		non_terminal_states = a;
	}
	
	override double evaluate(double [] w, out double [] g, double step) {
    	

    	double objective = 0;
    	double normalizer;
    	double[] policy_distribution = getPolicyDistribution(w, normalizer);

        g.length = w.length;
        g[] = 0;

    	objective += log(normalizer);
    	
    	
    	g = w.dup;
    	
    	g[] *= mu_E[];
    	objective -= reduce!("a + b")(0.0, g);
    	
    	

    	double weighted_features[];
    	weighted_features.length = g.length;
    	weighted_features[] = 0;
            	
        foreach(i; 0 .. pow(model.A().length, non_terminal_states)) {
	    	weighted_features[] += policy_distribution[i] * feature_expectations_for_policies[i][];
	    	
	    }
    	
    	g[] = weighted_features[]  - mu_E[];
    	
        return objective;		
        
	}

   
    double [] getPolicyDistribution(double [] weights, out double normalizer) {
        normalizer = 0;
    	double [] policy_distr = new double[pow(model.A().length, non_terminal_states)];
        
        foreach(i; 0 .. pow(model.A().length, non_terminal_states)) {
        	double [] temp = weights.dup;
        	temp[] *= feature_expectations_for_policies[i][];
	    	policy_distr[i] = exp(reduce!("a + b")(0.0, temp));
	    	
	    	normalizer += policy_distr[i];
    	}

    	if (normalizer == 0)
    		normalizer = double.min_normal;
    	debug {
  //  		writeln("Normalizer: ", normalizer);
    		
    	}
    	
    	policy_distr[] /= normalizer;
    	
    	return policy_distr;
    }
    
	double [][] all_feature_expectations(Model model) {
		double [][] returnval;
		returnval.length = pow(model.A().length, non_terminal_states);
		
		foreach(i; 0 .. pow(model.A().length, non_terminal_states)) {
			returnval[i] = feature_expectations_exact(model, calcStateActionFreqExact(get_policy_num(model, i), initial, model, qval_thresh));
//			writeln(returnval[i]);
		}
		
		return returnval;
		
	}
	
	Agent get_policy_num(Model model, size_t num) {
		Action [] actions = model.A(); 
		Action[State] map;
		
		size_t remainder = num;
		
		foreach (s; model.S()) {
			
			if (model.is_terminal(s)) {
				map[s] = new NullAction();
			} else {				
				map[s] = actions[remainder % actions.length];
				remainder /= actions.length;
			}
			
		}
		
		return new MapAgent(map);
	}

}

class MaxEntIrlApproxReducedPolicySpace : MaxEntIrlExact {
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError);

		this.qval_thresh = qval_thresh;
	}	
	
	
	
	

	override double evaluate(double [] w, out double [] g, double step) {
		
    	double objective = 0;

    	double normalizer;
    	double[Agent] policy_distribution = getApproxPolicyDistribution(w, normalizer);


    	objective += log(normalizer);
    	
    	
    	g = w.dup;
    	
    	g[] *= mu_E[];
    	objective -= reduce!("a + b")(0.0, g);
    	

    	double weighted_features[];
    	weighted_features.length = g.length;
    	weighted_features[] = 0;
    	
        foreach(ag, pr_pi; policy_distribution) {
	    	weighted_features[] += pr_pi * feature_expectations_for_policy(ag)[];
	    }
    	
    	g[] = weighted_features[]  - mu_E[];

    	// square to handle saddle points
        return objective;
			
	}
	
	protected double[][Agent] feature_expectations_cache;
	
	protected double[] feature_expectations_for_policy(Agent a) {
		auto t = a in feature_expectations_cache;
		if (t) {
			return *t;
		}
		
		double [] fe = feature_expectations_exact(model, calcStateActionFreqExact(a, initial, model, qval_thresh));
		
		feature_expectations_cache[a] = fe;
		return fe;
		
	}
	
	
	protected double[Agent] getApproxPolicyDistribution(double [] weights, out double normalizer) {
		
		
		size_t policies_added = 0;
		double[Agent] returnval;
		
		// find optimal policy
		LinearReward r = cast(LinearReward)model.getReward();
    	r.setParams(weights);
    	 
    	double[State] V = solver.solve(model, solverError); 
    	Agent opt_policy = solver.createPolicy(model, V);
    	
    	Action[State] opt_pi = (cast(MapAgent)opt_policy).getPolicy();
    	
    	double [] temp_weights = weights.dup;
        temp_weights[] *= feature_expectations_for_policy(opt_policy)[];
	    double numerator = exp(reduce!("a + b")(0.0, temp_weights));
    	returnval[opt_policy] = numerator;
    	policies_added ++;
    	
    	auto actions = model.A();
    	
		// find all policies that are one action away from the optimum
		foreach(s; model.S()) {
			if (model.is_terminal(s))
				continue;
			
			Action opt_a = opt_policy.actions(s).keys[0];	
			foreach(a; model.A()) {
				if (a != opt_a) {
					Action[State] temp = opt_pi.dup;
					
					temp[s] = a;
					
					Agent temp_agent = new MapAgent(temp);
					
					temp_weights = weights.dup;
					temp_weights[] *= feature_expectations_for_policy(temp_agent)[];
					numerator = exp(reduce!("a + b")(0.0, temp_weights));

        			returnval[temp_agent] = numerator;
					policies_added ++;
				}				
			}
			
			
		}
		
		// continue adding random policies until we reach our goal
/*		size_t state_size = model.S().length;
		
		
		while (policies_added < size_of_policy_samples) {
			Action[State] temp = opt_pi.dup;
			
			double reassignProb = uniform(2.0/state_size, 1.0);
			
			foreach (s; model.S()) {
				if (model.is_terminal(s))
					continue;
				// rechoose this action?
				if (uniform(0.0, 1.0) < reassignProb) {
					
					temp[s] = actions[uniform(0, actions.length)];
					
				}
				
			}
			
			Agent temp_agent = new MapAgent(temp);
			
			temp_weights = weights.dup;
			temp_weights[] *= feature_expectations_for_policy(temp_agent)[];
			numerator = exp(reduce!("a + b")(0.0, temp_weights));
			returnval[temp_agent] = numerator;
			policies_added ++;
		}*/
		
		
    	/*
    	// completely random policies
		while (policies_added < size_of_policy_samples) {
			Action[State] temp;
			foreach (s; model.S()) {
				if (model.is_terminal(s)) {
					temp[s] = new NullAction();
					continue;
				}
					
				temp[s] = actions[uniform(0, actions.length)];
			}

			
			Agent temp_agent = new MapAgent(temp);
			
			temp_weights = weights.dup;
			temp_weights[] *= feature_expectations_for_policy(temp_agent)[];
			numerator = exp(reduce!("a + b")(0.0, temp_weights));
			returnval[temp_agent] = numerator;
			policies_added ++;
		}
    	*/
    	
		// divide all values by the normalizer 
		
		normalizer = 0;
		foreach(policy, val; returnval) {
			normalizer += val;
		}
		
		foreach(policy, ref val; returnval) {
			val /= normalizer;
		}
//		writeln(returnval);
		return returnval;
	}
		
}

class MaxEntIrlPartialVisibility : MaxEntIrl {
	protected State [] observableStates;
	
	public this(int max_iter, MDPSolver solver, int n_samples, double error, double solverError, double qval_thresh, State [] observableStatesList, size_t sample_length) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	
		this.observableStates = observableStatesList;
		this.sample_length = cast(int)sample_length;
	}
	
	
	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
        this.true_samples = true_samples;
//        this.sample_length = cast(int)true_samples.length;

        this.mu_E = feature_expectations2(model, true_samples, 0);
//        this.mu_E[] /= this.sample_length;
		debug {
			writeln("mu_E ", this.mu_E);
        }
        nelderMeadReference = this;
        
        double reqmin = 1.0E-06;
	
	
        double step[] = new double[init_weights.length];
        step[] = 1;
	
        int konvge = 3;
        int kcount = max_iter;
        int icount;
        int numres;
        int ifault;
        
	
	opt_value = evaluate_nelder_mead ( init_weights.ptr, cast(int)init_weights.length );

        opt_weights.length = init_weights.length;
        nelmin ( &evaluate_nelder_mead, cast(int)init_weights.length, init_weights.ptr, opt_weights.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );

        LinearReward r = cast(LinearReward)model.getReward();
        r.setParams(opt_weights);
        
        return solver.createPolicy(model, solver.solve(model, solverError));
	}	


	override double evaluate(double [] w, out double [] g, double step) {
    	
    	
    	// solve policy with policy iteration
    	// find state frequency
    	// use both to calc objective function
    	
    	LinearReward r = cast(LinearReward)model.getReward();
    	
    	r.setParams(w);
    	
    	// what methods can we use to incorporate the weight deririvates?
    	// 1.  Add as a squared constraint
    	// 2.  Use penalty method (still squared constraint)
    	// 3.  Use augmented lagrangian
    	
    	
    	double[StateAction] Q_value = QValueSolve(model, qval_thresh, true);
        double sum = 0;
     
 /*       foreach (sa, v; Q_value) {
        	writeln("Q:", sa.s, " ", sa.a, " ", v); 
        }*/
        
        foreach (sa, count ; sa_freq[0] ) {
        	sum += Q_value[sa] * count;
        }
        
        double p_of_pi = exp(sum);
        
        sum *= -1;
        
        Agent policy = CreatePolicyFromQValue(model, Q_value);
        
        double [] sum2 = new double[r.dim()];
        sum2[] = 0;

        if (this.n_samples > 0) {
	    	sar [][] samples = generate_samples(model, policy, initial, this.n_samples, sample_length);

	        sum2 = feature_expectations(model, samples);
	        debug {
	        	writeln(sum2, " ", w);
	        }
//	        sum2[] /= sample_length;
	        
        } /*else {
	        double[State] mu = calcStateFreq(policy, initial, model, sample_length/5);
	        
	        foreach (s; model.S())
	        	mu[s] /= (sample_length/5);

//	        double[State] mu = calcStateFreqAlt(policy, initial, model, this.solverError);
	        
	        foreach (s; observableStates) {
	        	double[Action] a_pi = policy.actions(s);
	        	foreach (a, v; a_pi)  
	        		sum2[] += mu[s] * r.features(s, a)[] * v;
		        
	    	}
	    }*/
//    	writeln(sum2, " -> ", mu_E);
    	sum2[] -= mu_E[];

    	sum2[] += p_of_pi * sum2[];
    	sum2[] *= sum2[];
    	
    	double sum3 = 0;
    	foreach (k ; sum2)
    		sum3 += k;
    	
    	sum += sqrt(sum3);
    	
        
    	// no point in calculating gradient
    	g.length = 1;
    	g[] = 0;
        	

        return sum;		
		
	}	
	
    override double [] feature_expectations(Model model, sar [][] samples) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		foreach (s ; this.observableStates) {
        			if (SAR.s  == s) {
		        		double [] f = ff.features(SAR.s, SAR.a);
		        		returnval[] += f[];
		        		break;
		        	}
        		}
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}
	
}

class MaxEntIrlPartialVisibilityNewApprox : MaxEntIrl {
	protected State [] observableStates;
	
	public this(int max_iter, MDPSolver solver, int n_samples, double error, double solverError, double qval_thresh, State [] observableStatesList, size_t sample_length) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	
		this.observableStates = observableStatesList;
		this.sample_length = cast(int)sample_length;
	}
	
	
	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;
        this.true_samples = true_samples;
//        this.sample_length = cast(int)true_samples.length;

        this.mu_E = feature_expectations2(model, true_samples, 0);
//        this.mu_E[] /= this.sample_length;
		debug {
			writeln("mu_E ", this.mu_E);
        }
        nelderMeadReference = this;
        
        double reqmin = 1.0E-06;
	
	
        double step[] = new double[init_weights.length];
        step[] = 1;
	
        int konvge = 3;
        int kcount = max_iter;
        int icount;
        int numres;
        int ifault;
        
	
	  	opt_value = evaluate_nelder_mead ( init_weights.ptr, cast(int)init_weights.length );

        opt_weights.length = init_weights.length;
        nelmin ( &evaluate_nelder_mead, cast(int)init_weights.length, init_weights.ptr, opt_weights.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );

        LinearReward r = cast(LinearReward)model.getReward();
        r.setParams(opt_weights);
        
        return solver.createPolicy(model, solver.solve(model, solverError));
	}	


	override double evaluate(double [] w, out double [] g, double step) {
    	
    	
    	// solve policy with policy iteration
    	// find state frequency
    	// use both to calc objective function
    	
    	LinearReward r = cast(LinearReward)model.getReward();
    	
    	r.setParams(w);
    	
    	// what methods can we use to incorporate the weight deririvates?
    	// 1.  Add as a squared constraint
    	// 2.  Use penalty method (still squared constraint)
    	// 3.  Use augmented lagrangian
    	
    	
    	double[StateAction] Q_value = QValueSolve(model, qval_thresh, true);
        double sum = 0;
     
 /*       foreach (sa, v; Q_value) {
        	writeln("Q:", sa.s, " ", sa.a, " ", v); 
        }*/
        
        foreach (sa, count ; sa_freq[0] ) {
        	sum += Q_value[sa] * count;
        }
        
        double p_of_pi = exp(sum);
        
        sum *= -1;
        
        
        
        Agent policy = CreatePolicyFromQValue(model, Q_value);
        
        double [] sum2 = new double[r.dim()];
        sum2[] = 0;

        if (this.n_samples > 0) {
	    	sar [][] samples = generate_samples(model, policy, initial, this.n_samples, sample_length);

	        sum2 = feature_expectations(model, samples);
	        debug {
	        	writeln(sum2, " ", w);
	        }
//	        sum2[] /= sample_length;
	        
        } /*else {
	        double[State] mu = calcStateFreq(policy, initial, model, sample_length/5);
	        
	        foreach (s; model.S())
	        	mu[s] /= (sample_length/5);

//	        double[State] mu = calcStateFreqAlt(policy, initial, model, this.solverError);
	        
	        foreach (s; observableStates) {
	        	double[Action] a_pi = policy.actions(s);
	        	foreach (a, v; a_pi)  
	        		sum2[] += mu[s] * r.features(s, a)[] * v;
		        
	    	}
	    }*/
//    	writeln(sum2, " -> ", mu_E);

		double[] temp1 = w.dup;
		temp1[] *= sum2[];

		sum += reduce!("a + b")(0.0, temp1);
		
		sum *= sum;	

    	sum2[] *= p_of_pi;
    	sum2[] -= mu_E[];

    	sum2[] *= sum2[];
    	
    	sum += reduce!("a + b")(0.0, sum2);
    	
    	sum = sqrt(sum);
    	
        
    	// no point in calculating gradient
    	g.length = 1;
    	g[] = 0;
        	

        return sum;		
		
	}	
	
    override double [] feature_expectations(Model model, sar [][] samples) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		foreach (s ; this.observableStates) {
        			if (SAR.s  == s) {
		        		double [] f = ff.features(SAR.s, SAR.a);
		        		returnval[] += f[];
		        		break;
		        	}
        		}
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}
	
}


class MaxEntIrlPartialVisibilityMultipleAgents : MaxEntIrl {
	protected State [] observableStates;
	protected Model [] models;
	protected double[State][] initials;
	protected sar[][][] true_samples;
	protected int [] sample_lengths;
	protected double [][] mu_Es;
	protected double[Action][] equilibrium;
	protected double [][] init_weights;
	protected int interactionLength;
	
	public this(int max_iter, MDPSolver solver, int n_samples, double error, double solverError, double qval_thresh, State [] observableStatesList) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	
		this.observableStates = observableStatesList;
	}
	
	
	public Agent [] solve(Model [] models, double[State][] initials, sar[][][] true_samples, size_t [] sample_lengths, double [][] init_weights, double[Action][] NE, int interactionLength, out double opt_value, out double [][] opt_weights) {
        this.init_weights = init_weights;
        this.models = models;
        this.initials = initials;
        this.equilibrium = NE;
        this.interactionLength = interactionLength;
        foreach (ref i; this.initials) 
        	i.rehash;
        	
        this.true_samples = true_samples;
        foreach(sample_length; sample_lengths)
        	this.sample_lengths ~= cast(int)sample_length;
        
        mu_Es.length = models.length;
        foreach (int i, Model model; models) {
        	mu_Es[i] = feature_expectations2(model, true_samples[i], i);
//        	mu_Es[i][] /= this.sample_lengths[i];
        }
        
        nelderMeadReference = this;
        
        double reqmin = 1.0E-06;
	
        int x_len = 0;
        opt_weights.length = models.length;
        foreach (int i, Model model; models) {
  	        opt_weights[i].length = init_weights[i].length;
	        x_len += init_weights[i].length;
        }
        	
        double step[] = new double[x_len];
        step[] = .6;
	
        int konvge = 3;
        int kcount = max_iter;
        int icount;
        int numres;
        int ifault;
        

        double [] combined_init_weights = new double[x_len];
        int i = 0;
        foreach(m; init_weights) {
        	foreach(init; m) {
        		combined_init_weights[i] = init;
        		i ++;
        	}
        }
        double [] combined_output = new double[x_len];
	
	  	opt_value = evaluate_nelder_mead ( combined_init_weights.ptr, x_len );

        nelmin ( &evaluate_nelder_mead, x_len, combined_init_weights.ptr, combined_output.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );


//        writeln("Final: ", opt_value, " ", combined_output);

        opt_weights.length = init_weights.length;
        foreach(k, ref o; opt_weights) {
        	o.length = init_weights[k].length;
        }
        i = 0;
     	Agent [] returnval = new Agent[models.length];
        foreach(j, ref o; opt_weights) {
        	foreach(ref o2; o) {
        		o2 = combined_output[i++];
        	}
     	
	        LinearReward r = cast(LinearReward)models[j].getReward();
	        r.setParams(o);
        
            returnval[j] = solver.createPolicy(models[j], solver.solve(models[j], solverError));
             
        }
        
        return returnval;
	}	


	override double evaluate(double [] w, out double [] g, double step) {
    	
    	// solve policy with policy iteration
    	// find state frequency
    	// use both to calc objective function
    	
    	double [][] weights;
    	weights.length = init_weights.length;
        foreach(k, ref o; weights) {
        	o.length = init_weights[k].length;
        }
        int i = 0;
        foreach(ref o; weights) {
        	foreach(ref o2; o) {
        		o2 = w[i++];
        	}
     	}
    	
    	double [] sums;
    	double [] p_of_pis;
    	Agent [] policies;
    	
    	foreach (j, model; models) {
	    	
	    	LinearReward r = cast(LinearReward)model.getReward();
	    	
	    	r.setParams(weights[j]);
	    	
	    	// what methods can we use to incorporate the weight deririvates?
	    	// 1.  Add as a squared constraint
	    	// 2.  Use penalty method (still squared constraint)
	    	// 3.  Use augmented lagrangian
	    	
	    	
	    	double[StateAction] Q_value = QValueSolve(model, qval_thresh);
	        double sum = 0;
	     
	 /*       foreach (sa, v; Q_value) {
	        	writeln("Q:", sa.s, " ", sa.a, " ", v); 
	        }*/
	        
	        foreach (sa, count ; sa_freq[0] ) {
	        	sum += Q_value[sa] * count;
	        }
	        
	        p_of_pis ~= exp(sum);
	        
	        sum *= -1;
	        
	        sums ~= sum;
	        
	        policies ~= CreatePolicyFromQValue(model, Q_value);
	    }
	    

    	sar [][][] samples = generate_samples_interaction(models, policies, initials, this.n_samples, sample_lengths, equilibrium, this.interactionLength);

    	foreach (j, model; models) {
	    	LinearReward r = cast(LinearReward)model.getReward();
	
	        double [] sum2 = new double[r.dim()];
	        sum2[] = 0;
	        
	        sum2 = feature_expectations(model, samples[j]);
//	        sum2[] /= sample_lengths[j];
		        

//	    	writeln(sum2, " -> ", mu_Es[j]);
	    	sum2[] -= mu_Es[j][];
	
	    	sum2[] += p_of_pis[j] * sum2[];
	    	sum2[] *= sum2[];
	    	
	    	double sum3 = 0;
	    	foreach (k ; sum2)
	    		sum3 += k;
	    	
	    	
	    	sums ~= sqrt(sum3);
	    	
	    
    	}
        
    	// no point in calculating gradient
    	g.length = 1;
    	g[] = 0;
        
        double overallsum = 0;
        foreach (s; sums)
        	overallsum += s;

        return overallsum;		
		
	}	
	
    override double [] feature_expectations(Model model, sar [][] samples) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		foreach (s ; this.observableStates) {
        			if (SAR.s  == s) {
		        		double [] f = ff.features(SAR.s, SAR.a);
		        		returnval[] += f[];
		        		break;
		        	}
        		}
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}
	
	
}	



class MaxEntIrlPartialVisibilityMultipleAgentsUnknownNE : MaxEntIrl {
	protected State [] observableStates;
	protected Model [] models;
	protected double[State][] initials;
	protected sar[][][] true_samples;
	protected int [] sample_lengths;
	protected double [][] mu_Es;
	protected double[Action][][] equilibria;
	protected double [][] init_weights;
	protected int interactionLength;
	
	protected int chosen_equilibrium;
	protected double minFoundSoFar;
	
	protected double[][] neWeightsArchive;
	
	public double[][] getNeWeightsArchive() {
		return neWeightsArchive;
	}
	
	debug {
		private uint iter;
		
	}
	private uint iter2;

	public this(int max_iter, MDPSolver solver, int n_samples, double error, double solverError, double qval_thresh, State [] observableStatesList) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	
		this.observableStates = observableStatesList;
	}
	
	
	public Agent [] solve(Model [] models, double[State][] initials, sar[][][] true_samples, size_t [] sample_lengths, double [][] init_weights, double[Action][][] NEs, int interactionLength, out double opt_value, out double [][] opt_weights) {
        this.init_weights = init_weights;
        this.models = models;
        this.initials = initials;
        this.equilibria = NEs;
        this.interactionLength = interactionLength;
        foreach (ref i; this.initials) 
        	i.rehash;
        	
        this.true_samples = true_samples;
        foreach(sample_length; sample_lengths)
        	this.sample_lengths ~= cast(int)sample_length;
        this.minFoundSoFar = double.max;
        
        mu_Es.length = models.length;
        foreach (int i, Model model; models) {
        	mu_Es[i] = feature_expectations2(model, true_samples[i], i);
//        	mu_Es[i][] /= this.sample_lengths[i];
        }
        debug(WeightsmIRLstar) {
        	writeln("Sample lengths: ", sample_lengths);
        	writeln("Expert Feature Expectations: ", mu_Es);
        	//iter = 0;
        }
        //iter2 = 1;
        nelderMeadReference = this;
        
        double reqmin = 1.0E-06;
	
        int x_len = 0;
        opt_weights.length = models.length;
        foreach (int i, Model model; models) {
  	        opt_weights[i].length = init_weights[i].length;
	        x_len += init_weights[i].length;
        }
        	
        double step[] = new double[x_len];
        step[] = .6;
	
        int konvge = 3;
        int kcount = max_iter;
        int icount;
        int numres;
        int ifault;
        

        double [] combined_init_weights = new double[x_len];
        int i = 0;
        foreach(m; init_weights) {
        	foreach(init; m) {
        		combined_init_weights[i] = init;
        		i ++;
        	}
        }
        double [] combined_output = new double[x_len];
	
	  	opt_value = evaluate_nelder_mead ( combined_init_weights.ptr, x_len );

        nelmin ( &evaluate_nelder_mead, x_len, combined_init_weights.ptr, combined_output.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );


//        writeln("Final: ", opt_value, " ", combined_output);

        opt_weights.length = init_weights.length;
        foreach(k, ref o; opt_weights) {
        	o.length = init_weights[k].length;
        }
        i = 0;
     	Agent [] returnval = new Agent[models.length];
        foreach(j, ref o; opt_weights) {
        	foreach(ref o2; o) {
        		o2 = combined_output[i++];
        	}
        	debug {
        		writeln ("Final Weights:");
        		writeln (o);
        		
        	}
	        LinearReward r = cast(LinearReward)models[j].getReward();
	        r.setParams(o);
	        debug {
	        	
	        	writeln(solver.solve(models[j], solverError));
	        }
            returnval[j] = solver.createPolicy(models[j], solver.solve(models[j], solverError));
        }
        

        return returnval;
	}	

	public int getEquilibrium() {
		return chosen_equilibrium;
	}

	override double evaluate(double [] w, out double [] g, double step) {
    	
    	// solve policy with policy iteration
    	// find state frequency
    	// use both to calc objective function
    	
    	double [][] weights;
    	weights.length = init_weights.length;
        foreach(k, ref o; weights) {
        	o.length = init_weights[k].length;
        }
        int i = 0;
        foreach(ref o; weights) {
        	foreach(ref o2; o) {
        		o2 = w[i++];
        	}
     	}
    	
    	double [] sums;
    	double [] p_of_pis;
    	Agent [] policies;
    	
    	foreach (j, model; models) {
	    	
	    	LinearReward r = cast(LinearReward)model.getReward();
	    	
	    	r.setParams(weights[j]);
	    	
	    	// what methods can we use to incorporate the weight deririvates?
	    	// 1.  Add as a squared constraint
	    	// 2.  Use penalty method (still squared constraint)
	    	// 3.  Use augmented lagrangian
	    	
	    	
	    	double[StateAction] Q_value = QValueSolve(model, qval_thresh);
	        double sum = 0;
	     
	 /*       foreach (sa, v; Q_value) {
	        	writeln("Q:", sa.s, " ", sa.a, " ", v); 
	        }*/
	        
	        foreach (sa, count ; sa_freq[j] ) {
	        	sum += Q_value[sa] * count;
	        }
	        
	        p_of_pis ~= exp(sum);
	        
	        sum *= -1;
	        
	        sums ~= sum;
	        
	        policies ~= CreatePolicyFromQValue(model, Q_value);
	    }
	    
//	    writeln(mu_Es);
	     
	    double [][][] eqSums;
	    eqSums.length = equilibria.length;
	    foreach (k, equilibrium; equilibria) {
	    
	    	sar [][][] samples = generate_samples_interaction(models, policies, initials, this.n_samples, sample_lengths, equilibrium, this.interactionLength);
	    	
	
	    	foreach (j, model; models) {
		    	LinearReward r = cast(LinearReward)model.getReward();
		
		        double [] sum2 = new double[r.dim()];
		        sum2[] = 0;
		        
		        sum2 = feature_expectations(model, samples[j]);
	//	        sum2[] /= sample_lengths[j];

/*			    debug {
			    	write(sum2, " ");
			    }*/
			    
		    	eqSums[k] ~= sum2;
		    	
		    
	    	}
//	    	writeln();
    	}
	    debug {
	    	iter += 1;
	    	writeln("Iteration: ", iter, " Cur Feature Expectations: ", eqSums); 
	    	
	    }
	    iter2 ++;
		double y = 100.0 / iter2;
    	// calc array of l2norms
    	double [] neWeights;
	    foreach (s; eqSums) {
	    	double [] temp = new double[s[0].length] ;
	    	temp[] = 0;
	    	foreach (j, model; models) {
	    		temp[] += s[j][] - mu_Es[j][];
	    		
	    	}
//	    	temp[] /= y;
    		neWeights ~= exp( - l2norm(temp));
//	    	writeln(temp, " => ", l2norm(temp) ," => ", neWeights[$-1]);
	    }
	    	    	
    	// calc normalized weight of each NE
    	auto weightSum = reduce!("a + b")(0.0, neWeights);
    	neWeights[] = neWeights[] / weightSum;
    	
    	neWeightsArchive ~= neWeights;
    	
    	debug {
    		writeln("neWeights", neWeights);
    	}	
    	
//    	write("Final Feature Expectations: ");
    	// build single feature expectations array as linear combination of eqSums
   		foreach (j, model; models) {
	    	double [] sum2 = new double[mu_Es[j].length]; 
	    	sum2[] = 0;
	    	
   			foreach(k, entry; eqSums) {
	    		sum2[] += neWeights[k] * entry[j][];
	    	}
//	    	write(sum2, " ");
	    	
	    	sum2[] -= mu_Es[j][];
		   	sum2[] += p_of_pis[j] * sum2[];
	    	sum2[] *= sum2[];
	    	sums ~= sqrt(reduce!("a + b")(0.0, sum2));
	    }
//	    writeln();
        
    	// no point in calculating gradient
    	g.length = 1;
    	g[] = 0;
        
        double overallsum = 0;
        foreach (s; sums)
        	overallsum += s;
        	
        if (overallsum < minFoundSoFar) {
        	chosen_equilibrium = cast(int)(minPos!("a > b")(neWeights).ptr - neWeights.ptr);
        	minFoundSoFar = overallsum;
        }

        return overallsum;		
		
	}	
	
    override double [] feature_expectations(Model model, sar [][] samples) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		foreach (s ; this.observableStates) {
        			if (SAR.s  == s) {
		        		double [] f = ff.features(SAR.s, SAR.a);
		        		returnval[] += f[];
		        		break;
		        	}
        		}
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}
    
	
}	

    
sar [][] generate_samples(Model model, Agent agent, double[State] initial, int num_samples, size_t sample_length) {
/*        Generate self.n_samples different histories of length t_max by
        following agent.  Each history of the form,
            [ (s_0,a_0), (s_1,a_1), ...]
         t_max such that gamma^t_max = 0.01 */
    
//        int t_max = min(num_samples, cast(int) ceil( log(0.01)/log(model.getGamma()) ));

    sar [][] result;
    result.length = num_samples;
    for (int i = 0; i < result.length; i ++) {        	
        result[i] = simulate(model, agent, initial, sample_length);
    }
    return result;
}
 
sar [][][] generate_samples_interaction(Model [] models, Agent [] agents, double[State][] initials, int num_samples, int [] sample_lengths, double[Action][] equilibria, int interactionLength) {
/*        Generate self.n_samples different histories of length t_max by
        following agent.  Each history of the form,
            [ (s_0,a_0), (s_1,a_1), ...]
         t_max such that gamma^t_max = 0.01 */
        
    sar [][][] result;
    result.length = models.length; 
    int t_max = 0;
    foreach (int i, int sample_length; sample_lengths) {
//    	t_max = max(t_max, min(sample_length, cast(int) ceil( log(0.01)/log(models[i].getGamma()) )));
    	t_max = max(t_max, sample_length);
    	result[i].length = num_samples;
    }

    for (int i = 0; i < num_samples; i ++) {
    	
    	sar [][] temp = multi_simulate(models, agents, initials, t_max, equilibria, interactionLength);
    	
    	foreach (int j, sar [] traj; temp) {
			while ( traj.length < t_max) {
				auto temp2 = traj[$ - 1];
				traj ~= temp2;				
			}
    		result[j][i] = traj; 
    	}

    }

    return result;
}



sar [][] insertTurnArounds(sar [][] patroller, Model pmodel, int[State] primaryDistanceMeasures, Action turnAction) {
	
	sar [][] hist;
	int j = 0;
	int lastSeenAt = 0;
	// hmm, this implementation has issues, with 4 orientations  

	LinearReward lr = cast(LinearReward)pmodel.getReward();
	
//	Should I replace distance calculation with timecalculation?  Time is what I have, not distance!
//	and there seems to be too many timesteps for the distance travelled by the patroller, need to increase timestep time?
	
	foreach (sar [] t; patroller) {
		foreach (sar timestep; t) {
			State s = timestep.s;
			Action a = timestep.a;
			double p = timestep.p;
			
			if (j - lastSeenAt > 1 && lastSeenAt > 0) {
//				writeln(hist, j," ", lastSeenAt);
				
				int[State] distanceMeasures;
				assignDistance(pmodel, hist[lastSeenAt][0].s, distanceMeasures);
				
				State[][] inverseDistance;
				foreach (i; distanceMeasures.values) {
					if (i >= inverseDistance.length)
						inverseDistance.length = i + 1;
					
				}
				
				foreach (temp; distanceMeasures.keys) {
					inverseDistance[distanceMeasures[temp]] ~= temp;
				}
	
				auto newDistance = ((j - lastSeenAt) / 2);

				if (inverseDistance.length <= newDistance) {
					newDistance = cast(int)inverseDistance.length - 1;
				}
//				writeln(newDistance);
				
				// add empty entries to hist, this way we can append the turn arrounds at the right timestep
				
				foreach (i; 0 .. ((j - lastSeenAt) - 2)/ 2) {
					sar [] tempArray;
					hist ~= tempArray; 										
				} 
				sar [] tempArray; 

				int[State] distanceMeasures2;
				assignDistance(pmodel, s, distanceMeasures2);

				foreach (possibleState; inverseDistance[newDistance]) {
					// check if possibleState has a non-zero feature for performing this action (we only add it if it does, since we don't care about other states)
					
					if (lr.features(possibleState, turnAction)[1 + primaryDistanceMeasures[possibleState]] != 0) {
						
						// now check if it's the correct distance away from the newly observed state s
						if (newDistance - distanceMeasures2[possibleState] <= 0 && newDistance - distanceMeasures2[possibleState] >= -1) {
							tempArray ~= sar(possibleState, turnAction, 1.0);
						}
					
					}
				}
				
				foreach (ref sap; tempArray) {
					sap.p /= tempArray.length;
					
				} 


				hist ~= tempArray;
				
				
				
				while (hist.length < j) {
					sar [] tempArray2;
					hist ~= tempArray2;		
				}
				
				
			} 
				
			sar [] tempArray;
			tempArray ~= sar(s, a, p);
			hist ~= tempArray;


			lastSeenAt = j;

		}
		if (t.length == 0 && lastSeenAt == 0) {
			sar [] tempArray;
			hist ~= tempArray; 
		}
			
		j ++;
		
	}
	return hist;
}




sar [][] naiveInterpolateTraj(sar [][] patroller, Model pmodel, State [] observedStatesList) {


/*	writeln(observedStatesList);
	writeln("Interpolate");
	foreach ( sar [] temp; patroller) {
		foreach (sar SAR; temp) {
			write(SAR.s, " - ", SAR.a, " : ", SAR.p, ", ");
		}
		writeln();
	}
	writeln(); */  
	
	
	sar [][] hist;
	int j = 0;
	int lastSeenAt = 0;
	
	foreach (sar [] t; patroller) {
		foreach (sar timestep; t) {
			State s = timestep.s;
			Action a = timestep.a;
			double p = timestep.p;
			writeln(j, " ", lastSeenAt);
			if (j - lastSeenAt > 1 && lastSeenAt > 0) {
				
				// add all possible state/action pairs
				for( int k = 1; k < j - lastSeenAt; k ++) {
					sar [] allStates;
					
					foreach (sar sap; hist[hist.length - 1]) {
						State s2 = sap.s;
						Action a2 = sap.a;
						double prob = sap.p;
						
						foreach(State newS, double newP; pmodel.T(s2, a2)) {
							
							bool addState = true;
							foreach (checkState; observedStatesList) {
								if (checkState.samePlaceAs(newS)) {
									addState = false;
									break;
								}
							}
							if (addState) {	
								foreach (Action action; pmodel.A(newS)) {
								
									foreach (int target, sar l; allStates) {
										if (l.s == newS && l.a == action) {
											allStates[target] = sar(newS, action, (prob * newP) + l.p);
											addState = false;
											break;
										}
									}
									if (addState) {
										allStates ~= sar(newS, action, prob * newP);
									}
										
									
								}
							}
						
						}					
					}
	
						
					hist ~= allStates;
				}
				sar [] tempArray;
				tempArray ~= sar(s, a, 1.0);
				hist ~= tempArray;
				
					
				// work backwards removing state/action pairs that can't ever reach the states in the next timestep
				for (auto k = hist.length - 2; k > (hist.length - 2) - (j - lastSeenAt - 1); k --) {
					sar [] curhist = hist[k];
					
					sar [] nexthist;
					
					double sumP = 0;
	
					foreach (sap; curhist) {
						bool foundChild = false;
						State s2 = sap.s;
						Action a2 = sap.a;
						double prob = sap.p;
						
						if (prob > 0) {
							outerLoop: foreach (newS, newP; pmodel.T(s2, a2)) {
								foreach (sap2; hist[k+1]) {
									if (sap2.s == newS) {
										foundChild = true;
										nexthist ~= sap;
										sumP += prob;
										break outerLoop;
									}
								}
								
							}
						}
						
					}
					
	/*				sar [] appendhist;
					foreach( SAP; nexthist) {
						if (SAP.p / sumP > .001)
							appendhist ~= sap(SAP.s, SAP.a, SAP.p / sumP);
					} */
	//				writeln(sumP);
					foreach(int pos, SAP; nexthist) {
						nexthist[pos] = sar(SAP.s, SAP.a, SAP.p /= sumP);
					}
									
					
					hist[k] = nexthist;
				}
				
				lastSeenAt = j;
				
			} else {
				lastSeenAt = j;
				
				sar [] tempArray;
				tempArray ~= sar(s, a, 1.0);
				hist ~= tempArray;
			}
		
			if (t.length == 0 && lastSeenAt == 0) {
				sar [] tempArray;
				hist ~= tempArray; 
			}
				
			j ++;
		}
	}
	
	writeln("hist ", hist, lastSeenAt);

	
	while(hist.length < patroller.length) {
		sar [] t = hist[$ - 1];

		sar [] allStates;
		foreach (sar timestep; t) {
			State s = timestep.s;
			Action a = timestep.a;
			double p = timestep.p;
			

			// add all possible state/action pairs
			
			foreach(State newS, double newP; pmodel.T(s, a)) {
				
				bool addState = true;
				foreach (checkState; observedStatesList) {
					if (checkState.samePlaceAs(newS)) {
						addState = false;
						break;
					}
				}
				if (addState) {	
					foreach (Action action; pmodel.A(newS)) {
					
						foreach (int target, sar l; allStates) {
							if (l.s == newS && l.a == action) {
								allStates[target] = sar(newS, action, (p * newP) + l.p);
								addState = false;
								break;
							}
						}
						if (addState) {
							allStates ~= sar(newS, action, p * newP);
						}
							
						
					}
				}
			
			}					

		}		
							
		sar [] nexthist;
		
		double sumP = 0;

		foreach (sap; allStates) {
			bool foundChild = false;
			State s2 = sap.s;
			Action a2 = sap.a;
			double prob = sap.p;
			
			if (prob > 0) {
				nexthist ~= sap;
				sumP += prob;
			}
			
		}
		
/*				sar [] appendhist;
				foreach( SAP; nexthist) {
					if (SAP.p / sumP > .001)
						appendhist ~= sap(SAP.s, SAP.a, SAP.p / sumP);
				} */
//				writeln(sumP);
		foreach(int pos, SAP; nexthist) {
			nexthist[pos] = sar(SAP.s, SAP.a, SAP.p /= sumP);
		}
						
		
		hist ~= nexthist;
		
	}
	
	writeln("hist ", hist, lastSeenAt);
	
	
/*
	writeln("To:");
	
	foreach ( sar [] temp; hist) {
		foreach (sar SAR; temp) {
			write(SAR.s, " - ", SAR.a, " : ", SAR.p, ", ");
		}
		writeln();
	} */  
	
	
	return hist;

}
