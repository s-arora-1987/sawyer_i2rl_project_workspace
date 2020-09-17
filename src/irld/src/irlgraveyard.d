import irl;
import mdp;
import std.typecons;
import std.stdio;
import std.random;
import std.math;
import std.algorithm;
import std.numeric;


import std.bitmanip;

class MaxEntIrlExactEMDoshi : MaxEntIrlExact {

	BitArray [][] pi_feature_map;
	BitArray [] inv_Y_feature_map;
	double [][] tilde_mu_Y;
	size_t [][] pis_in_Y;
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
		
	}
	
	
	public void set_mu_tilde(BitArray [][] pi_feature_map_in, BitArray [] inv_Y_feature_map_in, double[][] tilde_mu_Y_in, size_t[][] pis_in_Y_in) {
		pi_feature_map = pi_feature_map_in;
		inv_Y_feature_map = inv_Y_feature_map_in;
		tilde_mu_Y = tilde_mu_Y_in;
		pis_in_Y = pis_in_Y_in;
	}
	
	public void get_mu_tilde(out BitArray [][] pi_feature_map_out, out BitArray [] inv_Y_feature_map_out, out double[][] tilde_mu_Y_out, out size_t[][] pis_in_Y_out) {
		
		pi_feature_map_out = pi_feature_map;
		inv_Y_feature_map_out = inv_Y_feature_map;
		tilde_mu_Y_out = tilde_mu_Y;
		pis_in_Y_out = pis_in_Y;
		
	}

	public Agent solve2(Model model, double[State] initial, sar[][] true_samples, size_t sample_length, double [] init_weights, double [][] state_visitations, out double opt_value, out double [] opt_weights, int algorithm = 1) {
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
        this.sample_length = cast(int)sample_length;
        
        count_non_terminal_states(model);

        
        // estimate initial policy from provided samples
//        StochasticAgent stochastic_policy = new BayesStochasticAgent(true_samples, model);
                      
        // start with a uniform stochastic policy
/*        double [Action][State] uniform_policy;
        foreach (s; model.S()) {
        	if (model.is_terminal(s)) {
        		uniform_policy[s][new NullAction] = 1.0;
        	} else {
	        	foreach(a; model.A()) {
	        		uniform_policy[s][a] = 1.0/model.A().length;
	        	}
	        }
        }
        StochasticAgent stochastic_policy = new StochasticAgent(uniform_policy);

        double [] policy_distr = policy_distr_from_stochastic_policy(model, stochastic_policy);
  */
        double ignored;
        double [] policy_distr = getPolicyDistribution(init_weights, ignored);
        
              
        double lastConvergeValue = -double.max;
        bool hasConverged;
        opt_weights = init_weights.dup;
        
        mu_E.length = init_weights.length;
        
        if (pi_feature_map.length == 0)
        	calc_Y_feature_expectations(model, true_samples);
                
        double * x = lbfgs_malloc(cast(int)opt_weights.length);
        scope(exit) {
        	lbfgs_free(x);
        } 
        
        auto iterations = 0;
        // loop until convergence
        do {
	        mu_E = calc_E_step(policy_distr);

	        debug {
	        	writeln("mu_E: ", mu_E);
	        }
	        
	        
	        LinearReward r = cast(LinearReward)model.getReward();
	        
	        nelderMeadReference = this;
	        
	        double reqmin = 1.0E-06;
		
		
	        double step[] = new double[init_weights.length];
	        step[] = 1;
		
	        int konvge = 3;
	        int kcount = max_iter;
	        int icount;
	        int numres;
	        int ifault;
	        
	        
	        auto temp_init_weights = opt_weights.dup;
	        
	        if (algorithm == 0) {
		        for (int i = 0; i < opt_weights.length; i ++) 
		        	x[i] = temp_init_weights[i];
		        
	
		        
		     	int ret = lbfgs(cast(int)opt_weights.length, x, &opt_value, &evaluate_maxent, &progress, &this, &param);
		
		
		        for (int i = 0; i < opt_weights.length; i ++)
		        	opt_weights[i] = x[i];
		        
	        } else {
	        
			  	opt_value = evaluate_nelder_mead ( temp_init_weights.ptr, cast(int)init_weights.length );
		
		        opt_weights.length = init_weights.length;
		        nelmin ( &evaluate_nelder_mead, cast(int)temp_init_weights.length, temp_init_weights.ptr, opt_weights.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );
		
		        
	        }
	        r.setParams(opt_weights);

	        double normalizer;
	    	policy_distr = getPolicyDistribution(opt_weights, normalizer);

        	// calculate Q value
        	double newQValue = calcQ(opt_weights, normalizer);
	        debug {
	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", opt_weights);
	        	
	        }
	        hasConverged = (abs(newQValue - lastConvergeValue) <= error) || l2norm(opt_weights) / opt_weights.length > 70;
	        
	        lastConvergeValue = newQValue;
	        
	        
	        
	        iterations ++;
	    } while (! hasConverged && iterations < 1000);
        						
		if (l2norm(opt_weights) / opt_weights.length > 70) {
			opt_weights[] /= l2norm(opt_weights);
			opt_weights[] *= 69;

	        (cast(LinearReward)model.getReward()).setParams(opt_weights);
			
		}
        
        return solver.createPolicy(model, solver.solve(model, solverError));
				
	}

	public double calcQ(double [] new_weights, double normalizer) {
		
		double returnval = -log(normalizer);
		
		double [] expectation = mu_E.dup;
		expectation[] *= new_weights[];
		
		returnval += reduce!("a + b")(0.0, expectation);
		
				
		return returnval;
		
	}

	// converts a distribution over deterministic policies to a stochastic one
	public Agent getStochasticPolicy(double [] policy_distribution) {
		double [Action][State] mapping;
		
		auto S = this.model.S();
		foreach(i, pr; policy_distribution) {
			Agent ag = get_policy_num(this.model, i);
			
			foreach (s; S) {
				
				auto action = ag.actions(s).keys[0]; 
				auto t = s in mapping;
				if (!t) {
					mapping[s][action] = pr; 
				} else if (action in (*t)) {
					(*t)[action] += pr;
				} else {
					(*t)[action] = pr;
				}
			}
			
		}
		
		return new StochasticAgent(mapping);
	}
	
   double [] feature_expectations_for_trajectory(Model model, sar [] traj, out BitArray feature_map) {

    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
    	bool [] boolArr = new bool[model.S().length];
    	boolArr[] = true;
    	feature_map.init(boolArr);

    	foreach(SAR; traj) {
        	double [] f = ff.features(SAR.s, SAR.a);
       		returnval[] = f[] + returnval[];
       		
       		foreach (s_idx, s; model.S()) {
       			if (s == SAR.s) {
       				feature_map[s_idx] = false;
       				break;
       			}
       			
       		}
    	}
    	        
        return returnval;
    }

    double [] calc_E_step(double [] policy_distr) {
    	
    	double returnval [];
    	returnval.length = mu_E.length;
    	returnval[] = 0;

    	foreach(Y_idx, Y_fe; tilde_mu_Y) {
    		double pr_z_norm = 0;
    		double temp [] = new double[returnval.length];
    		temp[] = 0;
    		foreach (policy_num; pis_in_Y[Y_idx]) {
    			pr_z_norm += policy_distr[policy_num];
    			
    			auto temp2 = Y_fe.dup;
    			auto pi_features = pi_feature_map[policy_num].dup;
    			foreach(feature_idx, ref BitArray feature_arr; pi_features) {  // remove states that are present in this Y from the feature matrix
    				feature_arr &= inv_Y_feature_map[Y_idx];
    				foreach(b; feature_arr) {	 // add to the feature expectations
    					if (b)
    						temp2[feature_idx] += 1;
    				}
    			}
    			
    			temp[] += (policy_distr[policy_num] / tilde_mu_Y.length) * temp2[]; 
    			
    		}
    		
    		temp[] /= pr_z_norm;
    		
    		returnval[] += temp[];
    		
    	}

    	return returnval;
    	
    }
    
    double [] policy_distr_from_stochastic_policy(Model model, StochasticAgent stoc) {
    	
    	auto stoc_policy = stoc.getPolicy();
    	double [] returnval = new double[pow(model.A().length, non_terminal_states)];
    	returnval[] = 1;
    	
    	foreach (i; 0 .. returnval.length) {
    		Agent policy = get_policy_num(model, i);
    		
    		foreach (s; model.S()) {	
    			foreach(a, pr_a; policy.actions(s)) {
    				returnval[i] *= pr_a * stoc_policy[s][a]; 
    			}
    		}	
    	}    	
    	
    	return returnval;
    	
    }
    
    void calc_pi_feature_map(Model model) {

    	LinearReward ff = cast(LinearReward)model.getReward();

    	pi_feature_map.length = pow(model.A().length, non_terminal_states);
    	
    	
    	foreach(pi_idx, ignored; pi_feature_map) {
    		Agent policy = get_policy_num(model, pi_idx);
    		
    		bool emptyArr [] = new bool[model.S().length];
    		emptyArr[] = false;
    		
    		foreach(feature_idx; 0..ff.dim()) {
    			pi_feature_map[pi_idx] ~= BitArray();
    			pi_feature_map[pi_idx][feature_idx].init(emptyArr);
    		}
    	
    		foreach(s_idx, s; model.S()) {
    			foreach(a, pr_a; policy.actions(s)) {
    				double [] f = ff.features(s, a);
    				
    				foreach(feature_idx, feature_val; f) {
    					pi_feature_map[pi_idx][feature_idx][s_idx] = (feature_val > 0);
    				}
    			}
    		
    		}
    		
    	}
    	
    }
  
    void calc_Y_feature_expectations(Model model, sar[][] true_samples) {
    	
    	tilde_mu_Y.length = true_samples.length + 1;
    	pis_in_Y.length = true_samples.length + 1;
    	inv_Y_feature_map.length = true_samples.length + 1;

    	calc_pi_feature_map(model);
    	
		if (true_samples.length == 0) {
	    	// add in the null case (matches all policies)
	    	foreach (policy_num;  0..pow(model.A().length, non_terminal_states)) {
				pis_in_Y[$-1] ~= policy_num;
			}    		
			
			bool [] boolArr = new bool[model.S().length];
			boolArr[] = true;
			inv_Y_feature_map[$-1] = BitArray();
			inv_Y_feature_map[$-1].init(boolArr);
		
	    	LinearReward ff = cast(LinearReward)model.getReward();
	
			tilde_mu_Y[$-1].length = ff.dim();
			tilde_mu_Y[$-1][] = 0; 
			
		} else {
	    	foreach(Y_idx, Y; true_samples) {
	    		foreach (policy_num;  0..pow(model.A().length, non_terminal_states)) {
	    			Agent policy = get_policy_num(model, policy_num);
	    			
	    			// is this policy consistent with the given Y?
	    			bool Y_is_subset = true;
	    			foreach (SAR; Y) {
			    		
			    		// is y a subset of this policy?
		    			bool found = false;
		    			foreach (a, pr_a; policy.actions(SAR.s)) {
		    				if (a == SAR.a && pr_a > 0){
		    					found = true;
		    					break;
		    				}
		    			}
			    			
		    			if (! found) {
		    				Y_is_subset = false;
		    				break;
		    			}
			    		
		    		} 
	    			
			    	if (!Y_is_subset) {
			    		continue;
			    	}
			    		
	    			
	    			pis_in_Y[Y_idx] ~= policy_num;
	    			
	    			inv_Y_feature_map[Y_idx] = BitArray();
	    			tilde_mu_Y[Y_idx] = feature_expectations_for_trajectory(model, Y, inv_Y_feature_map[Y_idx]);
	    			
	    			    			
	    		}
	    		
	    	}
    	}
    		
    	
    }	
        
}

class MaxEntIrlExactEMDoshiFullZMap : MaxEntIrlExactEMDoshi {
		
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
		
	}
	
    override double [] calc_E_step(double [] policy_distr) {
    	
    	double returnval [];
    	returnval.length = mu_E.length;
    	returnval[] = 0;

    	BitArray all_Y_bitArrays = BitArray();
    	bool [] all_true = new bool[model.S().length];
    	all_true[] = true;
    	all_Y_bitArrays.init(all_true);
    	
    	foreach(ba; inv_Y_feature_map) {
    		all_Y_bitArrays &= ba;
    	} 
    	 
    	double Z_state_visitation = 1.0 / model.A().length; 

    	foreach(Y_idx, Y_fe; tilde_mu_Y) {
    		double pr_z_norm = 0;
    		double temp [] = new double[returnval.length];
    		temp[] = 0;
    		foreach (policy_num; pis_in_Y[Y_idx]) {
    			pr_z_norm += policy_distr[policy_num];
    			
    			auto temp2 = Y_fe.dup;
    			auto pi_features = pi_feature_map[policy_num].dup;
    			
    			foreach(feature_idx, ref BitArray feature_arr; pi_features) {// remove states that are present in any Y from the feature matrix
    				feature_arr &= all_Y_bitArrays;
					foreach(b; feature_arr) {	 // add to the feature expectations
						if (b)
							temp2[feature_idx] += Z_state_visitation;;
					}
				}
    			
    			temp[] += (policy_distr[policy_num] / tilde_mu_Y.length) * temp2[]; 
    			
    		}
    		
    		temp[] /= pr_z_norm;
    		
    		returnval[] += temp[];
    		
    	}

    	return returnval;
    	
    }    
	
}


class MaxEntIrlExactEM : MaxEntIrlExact {

	double [] pr_y;	
	double [] true_state_visitations_of_y;
	double [][] true_state_visitations;
	size_t[] y_from_policy;
	sar[][] Y;
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
		
	}

	public void setPrY(double [] pr_y_in, size_t[] Y_in, double [] true_mu_y) {
		pr_y = pr_y_in;
		y_from_policy = Y_in;
		true_state_visitations_of_y = true_mu_y;
	}
	
	public void getPrY(out double [] pr_y_out, out size_t[] Y_out, out double [] true_mu_y) {
		pr_y_out = pr_y;
		Y_out = y_from_policy;
		true_mu_y = true_state_visitations_of_y;
		
	}
	public Agent solve2(Model model, double[State] initial, sar[][] true_samples, size_t sample_length, double [] init_weights, double [][] state_visitations, out double opt_value, out double [] opt_weights, int algorithm = 1) {
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
        this.sample_length = cast(int)sample_length;
        
        count_non_terminal_states(model);
        true_state_visitations = state_visitations;
        
//        if (pr_y == null || y_from_policy == null)
        	calc_y_and_z(true_samples);
        
        debug {
//        	writeln("Y: ", Y);
//        	writeln("Pr(y): ", pr_y);
        	
        }
        
        // estimate initial policy from provided samples
//        StochasticAgent stochastic_policy = new BayesStochasticAgent(true_samples, model);
                      
        // start with a uniform stochastic policy
/*        double [Action][State] uniform_policy;
        foreach (s; model.S()) {
        	if (model.is_terminal(s)) {
        		uniform_policy[s][new NullAction] = 1.0;
        	} else {
	        	foreach(a; model.A()) {
	        		uniform_policy[s][a] = 1.0/model.A().length;
	        	}
	        }
        }
        StochasticAgent stochastic_policy = new StochasticAgent(uniform_policy);

        double [] policy_distr = policy_distr_from_stochastic_policy(model, stochastic_policy);
  */
        double ignored;
        double [] policy_distr = getPolicyDistribution(init_weights, ignored);
        
              
        double lastConvergeValue = -double.max;
        bool hasConverged;
        opt_weights = init_weights.dup;
        
        mu_E.length = init_weights.length;
        
        
                
        double * x = lbfgs_malloc(cast(int)opt_weights.length);
        scope(exit) {
        	lbfgs_free(x);
        } 
        
        auto iterations = 0;
        // loop until convergence
        do {
	        mu_E = calc_E_step(policy_distr);

	        debug {
	        	writeln("mu_E: ", mu_E);
	        }
	        
	        
	        LinearReward r = cast(LinearReward)model.getReward();
	        
	        nelderMeadReference = this;
	        
	        double reqmin = 1.0E-06;
		
		
	        double step[] = new double[init_weights.length];
	        step[] = 1;
		
	        int konvge = 3;
	        int kcount = max_iter;
	        int icount;
	        int numres;
	        int ifault;
	        
	        
	        auto temp_init_weights = opt_weights.dup;
	        
	        if (algorithm == 0) {
		        for (int i = 0; i < opt_weights.length; i ++) 
		        	x[i] = temp_init_weights[i];
		        
	
		        
		     	int ret = lbfgs(cast(int)opt_weights.length, x, &opt_value, &evaluate_maxent, &progress, &this, &param);
		
		
		        for (int i = 0; i < opt_weights.length; i ++)
		        	opt_weights[i] = x[i];
		        
	        } else {
	        
			  	opt_value = evaluate_nelder_mead ( temp_init_weights.ptr, cast(int)init_weights.length );
		
		        opt_weights.length = init_weights.length;
		        nelmin ( &evaluate_nelder_mead, cast(int)temp_init_weights.length, temp_init_weights.ptr, opt_weights.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );
		
		        
	        }
	        r.setParams(opt_weights);

	        double normalizer;
	    	policy_distr = getPolicyDistribution(opt_weights, normalizer);

        	// calculate Q value
        	double newQValue = calcQ(opt_weights, normalizer);
	        debug {
	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", opt_weights);
	        	
	        }
	        hasConverged = (abs(newQValue - lastConvergeValue) <= error) || l2norm(opt_weights) / opt_weights.length > 70;
	        
	        lastConvergeValue = newQValue;
	        
	        
	        
	        iterations ++;
	    } while (! hasConverged && iterations < 1000);
        						
		if (l2norm(opt_weights) / opt_weights.length > 70) {
			opt_weights[] /= l2norm(opt_weights);
			opt_weights[] *= 69;

	        (cast(LinearReward)model.getReward()).setParams(opt_weights);
			
		}
        
        return solver.createPolicy(model, solver.solve(model, solverError));
				
	}

	public double calcQ(double [] new_weights, double normalizer) {
		
		double returnval = -log(normalizer);
		
		double [] expectation = mu_E.dup;
		expectation[] *= new_weights[];
		
		returnval += reduce!("a + b")(0.0, expectation);
		
				
		return returnval;
		
	}

	// converts a distribution over deterministic policies to a stochastic one
	public Agent getStochasticPolicy(double [] policy_distribution) {
		double [Action][State] mapping;
		
		auto S = this.model.S();
		foreach(i, pr; policy_distribution) {
			Agent ag = get_policy_num(this.model, i);
			
			foreach (s; S) {
				
				auto action = ag.actions(s).keys[0]; 
				auto t = s in mapping;
				if (!t) {
					mapping[s][action] = pr; 
				} else if (action in (*t)) {
					(*t)[action] += pr;
				} else {
					(*t)[action] = pr;
				}
			}
			
		}
		
		return new StochasticAgent(mapping);
	}
	
	
	public sar[][] generate_stochastic_samples(sar[][] starting_samples, Agent stochastic_policy) {
		sar[][] returnval;
		
		foreach(timestep; starting_samples) {
			sar [] entries;
			foreach (sample; timestep) {

				foreach (action, prob; stochastic_policy.actions(sample.s)) {
					entries ~= sar(sample.s, action, sample.p * prob); 					
				}
			}
			returnval ~= entries;
		}
		
		return returnval;
	}
	
   override double [] feature_expectations2(Model model, sar [][] trajs, int num) {
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
    	
        foreach (int i, sar [] traj; trajs) {
        	foreach (sar SAR; traj) {
        		double [] f = ff.features(SAR.s, SAR.a);
        		f[] = f[] * SAR.p;
//        		writeln(SAR.s, " -> ", f);
        		returnval[] = f[] + returnval[];
        		StateAction key = new StateAction(SAR.s, SAR.a);
        		sa_freq[num][key] = sa_freq[num].get(key, 0.0) + SAR.p;
        	} 
        }
/*        foreach (key, value; sa_freq) {
        	sa_freq[key] /= samples.length;
        	
        } */
//        returnval [] = returnval[];
        return returnval;
    }

    
    double pr_z_y(size_t policy_number, double [] policy_distr, double [] denominators) {
    	
    	double numerator = policy_distr[policy_number];
    	
    	double denominator = denominators[y_from_policy[policy_number]];
    	
    	return numerator / denominator;
    }
    
    double [] pr_z_denominator(double [] policy_distr) {
    	double [] denominator = new double[pr_y.length];
    	
    	double norm = 0;

/*
		Action [] actions = model.A(); 
    	denominator[] = 1;

		double [Action][State] mu_pi_E;
		
		foreach (i, pr; policy_distr) {
			size_t remainder = i;
			size_t next = 0;
			size_t pos = 0;
			Action action;
			
			foreach (i_s, s; model.S()) {
				auto t = s in mu_pi_E;
				if (model.is_terminal(s)) {
					pos = next;
					next += 1;
					action = new NullAction();
				} else {
					pos = next + (remainder % actions.length);
					action = actions[remainder % actions.length];
					next += actions.length; 
					remainder /= actions.length;
				}
				if (!t) {
					mu_pi_E[s][action] = pr;
				} else if (action in (*t)) {
					(*t)[action] += pr * true_state_visitations[i][pos];
				} else {
					(*t)[action] = pr * true_state_visitations[i][pos];
				}
			}	
		}
    			
		foreach(i_y, y; Y){

			foreach (SAR; y) {
   				denominator[i_y] += mu_pi_E[SAR.s][SAR.a];
   			}	
    	}
    
    */
    
    
    

    	denominator[] = 0;		
		foreach (i, pr; policy_distr) {
			denominator[y_from_policy[i]] += pr;
		}
	
    	
    	
    	
    	
		foreach (d; denominator)
			norm += d;
			
		denominator[] /= norm;
		
//		writeln("denom ", denominator);
    		
 //   		Agent policy = get_policy_num(model, i);
    		/*   	THIS IS NOT RIGHT, WE NEED TO CONSIDER THE Y THAT IS THE LARGEST SUBSET OF A POLICY
	    	foreach (y_idx, y; Y) {
	    		
	    		// is y a subset of this policy?
	    		bool is_subset = true;
	    		foreach (SAR; y) {
	    			bool found = false;
	    			foreach (a, pr_a; policy.actions(SAR.s)) {
	    				if (a == SAR.a && pr_a > 0){
	    					found = true;
	    					break;
	    				}
	    			}
	    			
	    			if (! found) {
	    				is_subset = false;
	    				break;
	    			}
	    			
	    		}
	    		
	    		if (is_subset) {
	    			denominator[y_idx] += pr_pi;
	    		}
	    		
	    	} */
    	debug {
//    		writeln("pr(z|y) denominator: ", denominator);
    		
    	}
    	
    	return denominator;
    	
    }
double KLD(double [] true_distr, double[] est_distr) {
	
	double returnval = 0;
	foreach (i, pr; true_distr) {
		returnval += pr * log ( pr / est_distr[i]);
	}
	return returnval;
	
}    
    double [] calc_E_step(double [] policy_distr) {
    	
    	double temp [];
    	temp.length = mu_E.length;
    	temp[] = 0;

    	double [] pr_z_denom = pr_z_denominator(policy_distr);
    	debug {
    		writeln("KLD: ", KLD(pr_y, pr_z_denom));
    	}
    	
    	foreach(i, pr_pi; policy_distr) {
   			temp[] += pr_y[y_from_policy[i]] * pr_z_y(i, policy_distr, pr_z_denom) * feature_expectations_for_policies[i][];
    	}

/*		foreach(i_pi, i_y; y_from_policy) {
			temp[] += pr_y[i_y] * pr_z_y(i_pi, policy_distr, pr_z_denom) * feature_expectations_for_policies[i_pi][];
			
		}
  */  	
    	return temp;
    	
    }
    
    double [] policy_distr_from_stochastic_policy(Model model, StochasticAgent stoc) {
    	
    	auto stoc_policy = stoc.getPolicy();
    	double [] returnval = new double[pow(model.A().length, non_terminal_states)];
    	returnval[] = 1;
    	
    	foreach (i; 0 .. returnval.length) {
    		Agent policy = get_policy_num(model, i);
    		
    		foreach (s; model.S()) {	
    			foreach(a, pr_a; policy.actions(s)) {
    				returnval[i] *= pr_a * stoc_policy[s][a]; 
    			}
    		}	
    	}    	
    	
    	return returnval;
    	
    }
    
    void calc_y_and_z(sar[][] true_samples) {
    	// create a mapping from policy_number => y number
    	// same for z
    	
    	//first create \dot{Y} and tilde{mu}


    	sar [] dot_Y;
    	double[Action][State] tilde_mu;
    	long total = 0;
    	
    	foreach(a; true_samples) {
    		foreach(SAR; a) {
    			
    			auto t = SAR.s in tilde_mu;
    			if (!t) {
					tilde_mu[SAR.s][SAR.a] = 1;
					dot_Y ~= SAR; 
				} else if (SAR.a in (*t)) {
					(*t)[SAR.a] += 1;
				} else {
					(*t)[SAR.a] = 1;
					dot_Y ~= SAR;
				}
    			total ++;
    		}
    	}
    	
    	// normalize tilde_mu
    	foreach(s, a_v; tilde_mu) {
    		foreach(a, ref val; a_v) {
    			val /= cast(double)total;
    		}
    	}
    	
    	
    	debug {
    		writeln("tilde_mu: ", tilde_mu);
    	}
    	
    	y_from_policy = new size_t[pow(model.A().length, non_terminal_states)];
    	pr_y.length = y_from_policy.length;
    	true_state_visitations_of_y = new double[pow(model.A().length, non_terminal_states)];
    	true_state_visitations_of_y[] = 0;
    	
    	//go through each policy, and get a list of sar's that are in \dot{Y}
    	foreach(i; 0 .. pow(model.A().length, non_terminal_states)) {
    		
    		sar[] temp_y;
    		Agent policy = get_policy_num(model, i);
    		
    		foreach (s; model.S()) {
    			
    			auto t = s in tilde_mu;
    			if (t) {
	    			foreach (a, pr_a; policy.actions(s)) {
	    				if (pr_a > 0) {
	    					if (a in (*t)) {
	    						sar SAR = sar(s, a, 0);
	    						temp_y ~= SAR;
	    					}
	    					
	    				}
	    			}
    			}
    		}
    		
    		// have we seen temp_y before?
    		bool found_y = false;
    		size_t y_idx = 0;
    		
    		foreach (y_index, y; Y) {
    			
				if (temp_y.length != y.length) 
					continue;
					
    			found_y = true;
    			
    			foreach (SAR; y) { // test to make sure y and temp y contain the exact same elements
    				
    				bool subfound = false;
    				foreach (SAR2; temp_y) {
    					if (SAR2.s == SAR.s && SAR2.a == SAR.a) {
    						subfound = true;
    						break;
    					}
    				}
    				
    				if (!subfound) { // this isn't the y we're looking for
    					found_y = false;
    					break;
    				}
    				
    			}
    			if (found_y) {
    				y_idx = y_index;
    				break;
    			}
    		}
    		
    		if (!found_y) {
    			y_idx = Y.length;
    			Y ~= temp_y;
    			
    		}
    		
    		y_from_policy[i] = y_idx;
		
    		
//    		Y ~= temp_y;
 //   		y_from_policy[i] = i;
    	}
    	
    	// calculate the total tilde_mu of each y
    	
    	pr_y = new double[Y.length];
//    	pr_y[] = 0;
    	double norm = 0;
    	
    	foreach (y_idx, y; Y) {
    		double temp = 0;
    		foreach (SAR; y) {
    			temp += tilde_mu[SAR.s][SAR.a];  
    		}
    		pr_y[y_idx] = temp;
    		norm += pr_y[y_idx];
    	}
    	
    	// build the pr_y by normalizing the count
    	
    	pr_y[] /= norm;
    	
    	
    	debug {
    		writeln(pr_y);
    		
			double test = 0;
			foreach (pr; pr_y) {
				test += pr * log(pr);
				
			}
		//    	writeln(pr_y);
			writeln("Entropy of pr_y: ", test);
    	}
    	// we actually don't care about z, at all 
    	
    	
    }
}


class MaxEntIrlApproxEM : MaxEntIrlExactEM {

	size_t size_of_policy_samples;
	
	public this(int max_iter, MDPSolver solver, size_t size_of_policy_samples, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
		this.size_of_policy_samples = size_of_policy_samples;
	}	
	
	public Agent solve2(Model model, double[State] initial, sar[][] true_samples, size_t sample_length, double [] init_weights, Agent correct_policy, out double opt_value, out double [] opt_weights, int algorithm = 1) {
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
        this.sample_length = cast(int)sample_length;
        
             
        count_non_terminal_states(model);
        
        if (pr_y == null || y_from_policy == null)
        	calc_y_and_z(true_samples);
        
  //      Agent stochastic_policy = new BayesStochasticAgent(true_samples, model);
        /*
        double [Action][State] uniform_policy;
        foreach (s; model.S()) {
        	if (model.is_terminal(s)) {
        		uniform_policy[s][new NullAction] = 1.0;
        	} else {
	        	foreach(a; model.A()) {
	        		uniform_policy[s][a] = 1.0/model.A().length;
	        	}
	        }
        }
        stochastic_policy = new StochasticAgent(uniform_policy);
        */
        
        double ignored;
        double [] zeros = new double[init_weights.length];
        zeros[] = 0;
        double [Agent] policy_distr = getApproxPolicyDistribution(zeros, ignored);
                
        double lastConvergeValue = -double.max;
        bool hasConverged;
        opt_weights = init_weights.dup;
        
        mu_E.length = init_weights.length;
        
        double * x = lbfgs_malloc(cast(int)opt_weights.length);
        scope(exit) {
        	lbfgs_free(x);
        } 
        
        auto iterations = 0; 
        // loop until convergence
        do {
	        
	        // generate samples from the current stochastic policy 
	        
	        mu_E = calc_approx_E_step(policy_distr);
	        
	        
	        feature_expectations_cache = feature_expectations_cache.rehash;
	   
	   		debug {
	   			writeln("True Samples ", mu_E, " L: ", sample_length);
	   		}
	        
	        LinearReward r = cast(LinearReward)model.getReward();
	
	
	        opt_weights.length = r.dim();
	        
	        nelderMeadReference = this;
	        
	        double reqmin = 1.0E-06;
		
		
	        double step[] = new double[init_weights.length];
	        step[] = 1;
		
	        int konvge = 3;
	        int kcount = max_iter;
	        int icount;
	        int numres;
	        int ifault;
	        
	        auto temp_init_weights = opt_weights.dup;
	        
	        if (algorithm == 0) {
		        for (int i = 0; i < opt_weights.length; i ++) 
		        	x[i] = temp_init_weights[i];
		        
		        
		     	int ret = lbfgs(cast(int)opt_weights.length, x, &opt_value, &evaluate_maxent, &progress, &this, &param);
		
		
		        for (int i = 0; i < opt_weights.length; i ++)
		        	opt_weights[i] = x[i];
	        } else {
			
			  	opt_value = evaluate_nelder_mead ( temp_init_weights.ptr, cast(int)temp_init_weights.length );
		
		        opt_weights.length = init_weights.length;
		        nelmin ( &evaluate_nelder_mead, cast(int)temp_init_weights.length, temp_init_weights.ptr, opt_weights.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );
		  
	  		
	  		}
	        // find next stochastic policy
	        double normalizer;
	    	policy_distr = getApproxPolicyDistribution(opt_weights, normalizer);
	        

	        r.setParams(opt_weights);

   	        // calculate convergence value
   	        
	        double newQValue = calcQ(opt_weights, normalizer);
//	        debug {
	        	writeln("Q(", iterations, ") = ", newQValue, " For weights: ", opt_weights);	        	
//	        }
	        
	        hasConverged = (abs(newQValue - lastConvergeValue) <= error);
	        
	        lastConvergeValue = newQValue;
	        iterations ++;
	    } while (! hasConverged);        
   
	    debug {
	    	writeln("Q Iterations: ", iterations);
	    }	
        
        return solver.createPolicy(model, solver.solve(model, solverError));
				
	}
	
	
	override double evaluate(double [] w, out double [] g, double step) {
	/*
    	double objective = 0;

        g.length = w.length;
        g[] = 0;

        double normalizer;
        double[Agent] policy_distribution = getApproxPolicyDistribution(w, normalizer);
        
        // need to square each partial derivative, starting with Pr(pi)

        foreach(ag, pr_pi; policy_distribution) {

	    	objective -= log (pr_pi) + log(normalizer);
	    	
	    	// calculate gradient
	    	
	    	auto temp = feature_expectations_for_policy(ag).dup;
	    	g[] += temp[] * pr_pi;
	    	temp[] *= w[];
	    	
	    	objective += reduce!("a + b")(0.0, temp); 

	    }
        objective *= objective;


        g[] -= mu_E[];
        
        g[] = g[] * g[];
        
        objective += reduce!("a + b")(0.0, g);
        
        return objective;		
*/
	
		
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
    	
    	g[] = 2;
    	        
    	g[] *= (weighted_features[]  - mu_E[])*(log(normalizer) - w[]*mu_E[]);

    	// square to handle saddle points
        return objective*objective;
			
	}
	
	protected double[][Agent] feature_expectations_cache;
	
	protected double[] feature_expectations_for_policy(Agent a) {
		auto t = a in feature_expectations_cache;
		if (t) {
			return *t;
		}
		
		double [] fe = feature_expectations_exact(model, calcStateActionFreq(a, initial, model, sample_length));
		
		feature_expectations_cache[a] = fe;
		return fe;
		
	}
	
	// feature expectations with the actions specified by a stochastic policy
	double [] feature_expectations3(Model model, sar [][] true_trajs, Agent stochastic_policy) {
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
    	
        foreach (int i, sar [] traj; true_trajs) {
        	foreach (sar SAR; traj) {
        		auto actions = stochastic_policy.actions(SAR.s);
        		foreach (a, pr; actions) {
        			double [] f = ff.features(SAR.s, a);
        			f[] = f[] * SAR.p * pr;
        			returnval[] = f[] + returnval[];
        		}
        	} 
        }
        return returnval;
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
		size_t state_size = model.S().length;
		
		
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
		}
		
		
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
	
	public Agent getStochasticPolicy(double [Agent] policy_distribution) {
		double [Action][State] mapping;
		
		auto S = this.model.S();
		foreach(ag, pr; policy_distribution) {
			
			foreach (s; S) {
				
				auto action = ag.actions(s).keys[0]; 
				auto t = s in mapping;
				if (!t) {
					mapping[s][action] = pr; 
				} else if (action in (*t)) {
					(*t)[action] += pr;
				} else {
					(*t)[action] = pr;
				}
			}
			
		}
		return new StochasticAgent(mapping);
	}
	

    
    double [] calc_approx_E_step(double [Agent] policy_distr) {
    	
    	double temp [];
    	temp.length = mu_E.length;
    	temp[] = 0;

   // 	double [] pr_z_denom = approx_pr_z_denominator(policy_distr);
    	
    	foreach(pi, pr_pi; policy_distr) {
  // 			temp[] += approx_pr_y[y_from_policy(pi)] * approx_pr_z_y(pi, policy_distr, pr_z_denom) * feature_expectations_for_policy[pi][];
    	}
    	    	
    	return temp;
    	
    }	
} 


class MaxEntIrlApproxEMPartialVisiblity : MaxEntIrlApproxEM {
	
	protected State [] observableStates;
	
	
	public this(int max_iter, MDPSolver solver, size_t size_of_policy_samples, int n_samples, double error, double solverError, double qval_thresh, State [] observableStatesList) {
		super(max_iter, solver, size_of_policy_samples, n_samples, error, solverError, qval_thresh);

		this.observableStates = observableStatesList;
		
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
    			if (SAR.s  !is null) {
	        		double [] f = ff.features(SAR.s, SAR.a);
	        		returnval[] += f[];
	        	}
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}	
    
    	// feature expectations with the actions specified by a stochastic policy
	override double [] feature_expectations3(Model model, sar [][] true_trajs, Agent stochastic_policy) {
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
    	
        foreach (int i, sar [] traj; true_trajs) {
        	foreach (sar SAR; traj) {
    			if (SAR.s  !is null) {
	        		auto actions = stochastic_policy.actions(SAR.s);
	        		foreach (a, pr; actions) {
	        			double [] f = ff.features(SAR.s, a);
	        			f[] = f[] * SAR.p * pr;
	        			returnval[] = f[] + returnval[];
	        		}
	        	}
        	} 
        }
        return returnval;
    }
	
	override double [] feature_expectations_exact(Model model, double[StateAction] D) {

    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
    	foreach(sa, v; D) {
			foreach (s; observableStates) {
				if (sa.s == s) {
		    		double [] f = ff.features(sa.s, sa.a);
		    		f[] *= v;
		    		
		    		returnval[] += f[];
		    		break;
	    		}
	    	}
    	}

        return returnval;
	}	


	public double calcPrTraj(Agent stochastic_policy) {
		
		double [] temps;
		temps.length = true_samples.length;
		temps[] = 1; 
		
		foreach (t, sar [] sample_traj; true_samples) {
//			writeln(temps);
				
			foreach(i; 0 .. sample_traj.length - 1) {
				if (sample_traj[i].s !is null && sample_traj[i+1].s !is null) {  
					auto actions = stochastic_policy.actions(sample_traj[i].s);
					double temp = 0;
					
					foreach(a, pr; actions) {
						temp += pr * model.T(sample_traj[i].s, a).get(sample_traj[i+1].s, 0);
					}
					
					temps[t] *=  temp;
				}
			}
			
			temps[t] *=  sample_traj[0].r;
		}
		
		return reduce!("a + b")(1.0, temps);
		
	}
}




class MaxEntIrlApproxEMPartialVisibilityMultipleAgentsUnknownNE : MaxEntIrl {
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
	
	protected double [][][] opt_weights_each_iter;
	
	debug {
		private uint iter;
		
	}
	bool delegate(State, State) is_interacting;
	
	public this(int max_iter, MDPSolver solver, int n_samples, double error, double solverError, double qval_thresh, State [] observableStatesList, bool delegate(State, State) is_interacting) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	
		this.observableStates = observableStatesList;		
		this.is_interacting = is_interacting;
	}
	
	public Agent [] solve2(Model [] models, double[State][] initials, sar[][][] true_samples, size_t [] sample_lengths, double [][] init_weights, double[Action][][] NEs, int interactionLength, Agent [] policy_priors, double[] NE_prior, out double opt_value, out double [][] opt_weights, out double[] opt_NE_probs) {
        this.init_weights = init_weights;
        this.models = models;
        this.initials = initials;
        this.equilibria = NEs;
        this.interactionLength = interactionLength;
        foreach (ref i; this.initials) 
        	i.rehash;
        	
        this.true_samples = true_samples;
        this.sample_lengths.length = this.true_samples.length;
        foreach (i, sample_length; sample_lengths) {
        	this.sample_lengths[i] = cast(int)sample_length;
        }
        this.minFoundSoFar = double.max;
        
        Agent [] stochastic_policies = policy_priors;
        double[] NE_distr = NE_prior;
        
        bool converged = false;
        double lastConvergeValue = -double.max;
        size_t count = 0;
        
        this.opt_weights_each_iter = null;
        
        opt_weights = init_weights;
        
        // loop until convergence
        do {
	        
	        // calculate convergence value
	        double newConvergeValue = calcPrTraj(stochastic_policies, NE_distr);
	        debug {
//	        	writeln("log Pr(tau|PI) ", newConvergeValue, " For policies: ", (cast(StochasticAgent)stochastic_policies[0]).getPolicy(), " and: ", (cast(StochasticAgent)stochastic_policies[1]).getPolicy());
	        	writeln("log Pr(tau|PI) ", newConvergeValue);
	        	
	        }
	        
	        /*
	        newConvergeValue = calcPrTraj(correct_policy);
	        debug {
	        	writeln("true log Pr(tau|PI) ", newConvergeValue);
	        	
	        }
	        */
	        // have we converged?
	        if (abs(newConvergeValue - lastConvergeValue) < 1)
	        	break;
	        
	        if (count > 2)
	        	break;
	        
	        count ++;	
	        
	        lastConvergeValue = newConvergeValue;


	        // generate samples from the current stochastic policy 
	        
	        mu_Es = feature_expectations4(this.models, true_samples, stochastic_policies, NE_distr);
	        
/*	        feature_expectations_for_policies = all_feature_expectations(model);
	        return null;
	        debug {
	        	writeln("All feature expectations calculated");
	        	
	        }*/
	   
	   		debug {
	   			writeln("True Samples ", mu_Es, " L: ", sample_lengths);
	   			writeln();
	   		}
	        
	        
	        
	        nelderMeadReference = this;
	        
	        double reqmin = 1.0E-06;
		
	        int x_len = 0;
//	        opt_weights.length = models.length;
	        foreach (int i, Model model; models) {
//	  	        opt_weights[i].length = init_weights[i].length;
		        x_len += init_weights[i].length;
	        }
	        	
	        double step[] = new double[x_len];
	        step[] = .6;
		
	        int konvge = 3;
	        int kcount = max_iter;
	        int icount;
	        int numres;
	        int ifault;
	        
	        debug {
	        	iter = 0;
	        }
	        
	        double [] combined_init_weights = new double[x_len];
	        auto i = 0;
	        foreach(m; opt_weights) {
	        	foreach(init; m) {
	        		combined_init_weights[i] = init;
	        		i ++;
	        	}
	        }
	        double [] combined_output = new double[x_len];
		
		  	opt_value = evaluate_nelder_mead ( combined_init_weights.ptr, x_len );
	
	        nelmin ( &evaluate_nelder_mead, x_len, combined_init_weights.ptr, combined_output.ptr, &opt_value, reqmin, step.ptr, konvge, kcount, &icount, &numres, &ifault );
	
	
	//        writeln("Final: ", opt_value, " ", combined_output);

			opt_weights = null;
	        opt_weights.length = init_weights.length;
	        foreach(k, ref o; opt_weights) {
	        	o.length = init_weights[k].length;
	        }
	        i = 0;
	        foreach(j, ref o; opt_weights) {
	        	foreach(ref o2; o) {
	        		o2 = combined_output[i++];
	        	}
	        	debug {
	        		writeln ("Weights: ", o);
	        	}
	        }
	        // save copy of weights for telemetry
	        opt_weights_each_iter ~= opt_weights.dup;
	        
	        // find next stochastic policy
	        double [][Agent][Agent] NE_weights;
	    	double [Agent][Agent] policy_distrs = getApproxPolicyDistributions(opt_weights, NE_weights);
	        
	        stochastic_policies = getStochasticPolicies(policy_distrs);
	        NE_distr = getStochasticNE(NE_weights, policy_distrs);

	        debug {
	        	writeln("NE distr: ", NE_distr);
	        	
	        }
	        opt_NE_probs = NE_distr;
	        
	    } while (! converged);
        
     	Agent [] returnval = new Agent[models.length];
        foreach(j, ref o; opt_weights) {
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
	    	
	    	
	    	double[StateAction] Q_value = QValueSolve(model, qval_thresh, true);
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
			    	write(samples[j], " -> ", sum2, " ");
			    }*/
			    
		    	eqSums[k] ~= sum2;
		    	
		    
	    	}
/*	    	debug {
	    		writeln();
	    	}*/
    	}
	    debug {
	    	iter += 1;
//	    	writeln("Iteration: ", iter, " Cur Feature Expectations: ", eqSums); 
	    	
	    }

    	// calc array of l2norms
    	double [] neWeights;
	    foreach (s; eqSums) {
	    	double [] temp = new double[s[0].length] ;
	    	temp[] = 0;
	    	foreach (j, model; models) {
	    		temp[] += s[j][] - mu_Es[j][];
	    		
	    	}
    		neWeights ~= exp( - l2norm(temp));
//	    	writeln(temp, " => ", l2norm(temp) ," => ", neWeights[$-1]);
	    }
	    	    	
    	// calc normalized weight of each NE
    	auto weightSum = reduce!("a + b")(0.0, neWeights);
    	neWeights[] = neWeights[] / weightSum;
    	
    	debug {
 //   		writeln("neWeights", neWeights);
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
    	debug {
    		g.length = 0;
	   		foreach (j, model; models) {
		    	double [] sum2 = new double[mu_Es[j].length]; 
		    	sum2[] = 0;
		    	
	   			foreach(k, entry; eqSums) {
		    		sum2[] += neWeights[k] * entry[j][];
		    	}
	   			
	   			sum2[] -= mu_Es[j][];
	   			
	   			foreach (entry; sum2)
	   				g ~= entry;
   			}
    		
    	}
        
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
  //              writeln(returnval);

        returnval [] /= samples.length;
        return returnval;
	}
    
	// two-agent feature expectations with the actions specified by a stochastic policies and NE distribution
	double [][] feature_expectations4(Model [] models, sar [][][] agent_true_trajs, Agent [] stochastic_policies, double[] NE_distr) {
    	
    	LinearReward [] ffs;
    	foreach (m; models) {
    		ffs ~= cast(LinearReward)m.getReward();
    	}
    	double [][] returnval;
    	returnval.length = stochastic_policies.length;
    	foreach(i, ref r; returnval) {
    		r.length = ffs[i].dim();
    		r[] = 0;
    	}
    	
    	sa_freq.length = 0;
    	
    	
    	foreach(a_id, true_trajs; agent_true_trajs) {
    		if (sa_freq.length < a_id + 1)
    			sa_freq.length = a_id + 1;
    			
	        foreach (t, sar [] traj; true_trajs) {
	        	foreach (i, sar SAR; traj) {
	        		// should we use the policy or NE?
        			if (SAR.s !is null) {
				
		        		if (agent_true_trajs[(a_id + 1) % agent_true_trajs.length][t].length > i &&
		        			agent_true_trajs[(a_id + 1) % agent_true_trajs.length][t][i].s !is null &&
		        			is_interacting(SAR.s, agent_true_trajs[(a_id + 1) % agent_true_trajs.length][t][i].s)
		        			&& ! models[a_id].is_terminal(SAR.s)) {
		        			
		        			foreach (ne_id, pr; NE_distr) {
		        				foreach(a, a_pr; equilibria[ne_id][a_id]) {
		        					double [] f = ffs[a_id].features(SAR.s, a);
		        					f[] = f[] * SAR.p * pr * a_pr;
			        				returnval[a_id][] = f[] + returnval[a_id][];
					        		StateAction key = new StateAction(SAR.s, a);
					        		sa_freq[a_id][key] = sa_freq[a_id].get(key, 0.0) + SAR.p * pr * a_pr;				        				
		        				}	
		        			}
			        				
		        		} else {
		        		
			        		auto actions = stochastic_policies[a_id].actions(SAR.s);
			        		if ( models[a_id].is_terminal(SAR.s)) {
			        			actions = null;
			        			actions[new NullAction()] = 1.0;
			        		}
			        		foreach (a, pr; actions) {
			        			double [] f = ffs[a_id].features(SAR.s, a);
			        			f[] = f[] * SAR.p * pr;
			        			returnval[a_id][] = f[] + returnval[a_id][];
				        		StateAction key = new StateAction(SAR.s, a);
				        		sa_freq[a_id][key] = sa_freq[a_id].get(key, 0.0) + SAR.p * pr;				        				

			        		}
		        		}

		        	}
        		
	        	}
	        }
        }
        return returnval;
    }		

	public double calcPrTraj(Agent [] stochastic_policies, double [] NE_distr) {
		
		double [] temps;
		temps.length = true_samples[0].length;
		temps[] = 1; 
		
		foreach (agent_idx, agent_traj; true_samples) {
			foreach (t, sar [] sample_traj; agent_traj) {
				foreach(i; 0 .. sample_traj.length - 1) {
					
					if (sample_traj[i].s !is null && sample_traj[i + 1].s !is null) { 

		        		if (true_samples[(agent_idx + 1) % true_samples.length][t].length > i &&
		        			true_samples[(agent_idx + 1) % true_samples.length][t][i].s !is null &&
		        			is_interacting(sample_traj[i].s, true_samples[(agent_idx + 1) % true_samples.length][t][i].s)) {
		        			
							double temp = 0;
		        			foreach (ne_id, pr; NE_distr) {
		        				foreach(a, a_pr; equilibria[ne_id][agent_idx]) {
		        					temp += pr * a_pr * models[agent_idx].T(sample_traj[i].s, a).get(sample_traj[i+1].s, 0);
		        				}	
		        			}
							temps[t] +=  temp;
			        				
		        		} else {
			        		
							auto actions = stochastic_policies[agent_idx].actions(sample_traj[i].s);
							double temp = 0;
							
							foreach(a, pr; actions) {
								temp += pr * models[agent_idx].T(sample_traj[i].s, a).get(sample_traj[i+1].s, 0);
							}
							temps[t] +=  temp;
							
						}	
					}
				}
				
//				temps[t] *=  sample_traj[0].r;
			}
		}
		
		return reduce!("a + b")(1.0, temps);
		
	}
	
	// converts a distribution over deterministic policies to a stochastic one
	public Agent [] getStochasticPolicies(double [Agent][Agent] policy_distributions) {
		Agent [] returnval;
		double [Action][State] mapping1;		
		double [Action][State] mapping2;

		foreach (ag1, policy_distribution; policy_distributions) {
			
			foreach(ag2, pr; policy_distribution) {
				auto S = this.models[0].S();
				foreach (s; S) {
					
					auto action = ag1.actions(s).keys[0]; 
					auto t = s in mapping1;
					if (!t) {
						mapping1[s][action] = pr; 
					} else if (action in (*t)) {
						(*t)[action] += pr;
					} else {
						(*t)[action] = pr;
					}
				}
				
				S = this.models[1].S();
				foreach (s; S) {
					
					auto action = ag2.actions(s).keys[0]; 
					auto t = s in mapping2;
					if (!t) {
						mapping2[s][action] = pr; 
					} else if (action in (*t)) {
						(*t)[action] += pr;
					} else {
						(*t)[action] = pr;
					}
				}
				
			}
		}
		returnval ~= new StochasticAgent(mapping1);
		returnval ~= new StochasticAgent(mapping2);
		return returnval;
	}	

	public double [] getStochasticNE(double [][Agent][Agent] NE_weights, double [Agent][Agent] policy_distributions) {
		double norm = 0;
		double [] returnval;
		
		bool initialized = false;
		
		foreach (agent1, var; NE_weights) {
			foreach (agent2, arr; var) {
				if (!initialized) {
					returnval.length = arr.length;
					returnval[] = 0;
					initialized = true;
					
				}
				auto weight = policy_distributions[agent1][agent2];
				returnval[] += weight * arr[];
				norm += weight;
			}
			
		}
		
		returnval[] /= norm;
		
		return returnval;
		
	}
	
	protected double numeratorAndNEWeights(Agent [] policies, double [][] weights, out double [] NE_weights) {
		
	    double [][][] eqSums;
	    eqSums.length = equilibria.length;
	    foreach (k, equilibrium; equilibria) {
	    
	    	sar [][][] samples = generate_samples_interaction(models, policies, initials, this.n_samples / 10, sample_lengths, equilibrium, this.interactionLength);
	    	
	
	    	foreach (j, model; models) {
		    	LinearReward r = cast(LinearReward)model.getReward();
		
		        double [] sum2 = new double[r.dim()];
		        sum2[] = 0;
		        
		        sum2 = feature_expectations(model, samples[j]);

/*			    debug {
			    	write(sum2, " ");
			    }*/
			    
		    	eqSums[k] ~= sum2;
		    	
		    
	    	}
//	    	writeln();
    	}


    	// calc array of l2norms
    	double [] neWeights;
	    foreach (s; eqSums) {
	    	double [] temp = new double[s[0].length] ;
	    	temp[] = 0;
	    	foreach (j, model; models) {
	    		temp[] += s[j][] - mu_Es[j][];
	    		
	    	}
    		neWeights ~= exp( - l2norm(temp));
//	    	writeln(temp, " => ", l2norm(temp) ," => ", neWeights[$-1]);
	    }
	    	    	
    	// calc normalized weight of each NE
    	auto weightSum = reduce!("a + b")(0.0, neWeights);
    	neWeights[] = neWeights[] / weightSum;
    	
/*    	debug {
    		writeln("neWeights", neWeights);
    	}*/	
    	
    	double sum = 0;
   		foreach (j, model; models) {
	    	double [] sum2 = new double[mu_Es[j].length]; 
	    	sum2[] = 0;
	    	
   			foreach(k, entry; eqSums) {
	    		sum2[] += neWeights[k] * entry[j][];
	    	}
   			sum2[] *= weights[j][];
	    	sum += reduce!("a + b")(0.0, sum2);
	    }    	
	
    	NE_weights = neWeights;
	    return sum;
	}
	
	
	protected double[Agent][Agent] getApproxPolicyDistributions(double [][] weights, out double [][Agent][Agent] NE_weights) {
		
		
		double[Agent][Agent] returnval;
		double [][Agent][Agent] NE_weights_returnval;
		
		// find optimal policies
		LinearReward [] r;
		Agent [] opt_policies; 
		foreach (i, model; models) {
			r ~= cast(LinearReward)model.getReward();
			r[i].setParams(weights[i]);
    	 
	    	double[State] V = solver.solve(model, solverError); 
	    	opt_policies ~= solver.createPolicy(model, V);
    	} 

  /*  	get initial numerator and add to the returnvals;

    	
    	now we can go in a loop for each permutation, one agent after another, generating numerators
    	
    	then sum and normalize*/

    	double [] out_weights;

    	returnval[opt_policies[0]][opt_policies[1]] = numeratorAndNEWeights(opt_policies, weights, out_weights);
    	NE_weights_returnval[opt_policies[0]][opt_policies[1]] = out_weights;
    	
 
    	foreach (i, model; models) {
    	
    	
	    	auto actions = model.A();
	    	
			// find all policies that are one action away from the optimum
			foreach(s; model.S()) {
				if (model.is_terminal(s))
					continue;
				
				Action opt_a = opt_policies[i].actions(s).keys[0];	
				foreach(a; model.A()) {
					if (a != opt_a) {
						Action[State] temp = (cast(MapAgent)opt_policies[i]).getPolicy().dup;
						
						temp[s] = a;
						
						Agent temp_agent = new MapAgent(temp);
						
						Agent [] temp_agents = opt_policies.dup;
						temp_agents[i] = temp_agent;
						
						returnval[temp_agents[0]][temp_agents[1]] = numeratorAndNEWeights(temp_agents, weights, out_weights);
						NE_weights_returnval[temp_agents[0]][temp_agents[1]] = out_weights;
						
					}				
				}
			}
		}
		
		
		// continue adding random policies until we reach our goal
		/*
		size_t state_size = model.S().length;
		
		
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
		}
		*/
		
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
		
		double normalizer = 0;
		foreach(policy, ref val1; returnval) { 
			foreach(policy2, val; val1) {
				normalizer += val;
			}
		}
		
		foreach(policy, ref val1; returnval) {
			foreach(policy2, ref val; val1) {
				val /= normalizer;
			}
		}
		NE_weights = NE_weights_returnval;
		
		return returnval;
	}
	

	double [][][] getWeightsForEachIter() {
		return opt_weights_each_iter;
	}
	
}




class MaxEntIrlFullMdp : MaxEntIrl {
	
	private double qval_thresh;
	private Model model;
	private double[State] initial;
	private sar[][] true_samples;
	private int sample_length;
	private double [] mu_E;
	
	private Agent lastAgent;
	private sar [][] lastSamples;
	private bool[State] observedStates;
	private double[] savedWeights;

	private double min_value;

	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError=0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	}
	
	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
		
        // Compute feature expectations of agent = mu_E from samples
        
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.max_iterations = max_iter;
 //       param.epsilon = error;
        param.min_step = .0000001; 
        
        this.model = model;
        this.initial = initial;
        this.initial = this.initial.rehash;        
        this.true_samples = true_samples;
        this.sample_length = cast(int)true_samples[0].length;
        this.min_value = double.max;
        
        mu_E = feature_expectations2(model, true_samples, 0);
        
        
/*        foreach (sar [] sar1; true_samples) {
        	foreach (sar SAR; sar1) {
        		foreach (State s; model.S()) {
        			if (s.samePlaceAs(SAR.s)) { 
        				observedStates[s] = true;
        			}
        		}
        	}
        } */
        
//        writeln("True Samples ", mu_E);
        
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

        lastAgent = new RandomAgent(model.A(null));
        
     	int ret = lbfgs(cast(int)opt_weights.length, x, &finalValue, &evaluate_maxent, &progress, &this, &param);
  
        for (int i = 0; i < opt_weights.length; i ++)
        	opt_weights[i] = x[i];
  
  
/*        opt_weights[] = init_weights[];
        opt_value = exponentiatedGradient(opt_weights,1);*/
                   
        writeln(ret, ": ", finalValue, " -> ", opt_weights);
        r.setParams(opt_weights);
        
        opt_value = finalValue;
        
        return solver.createPolicy(model, solver.solve(model, solverError));
		
 
	}

	
	
	
	override double evaluate(double [] w, out double [] g, double step) {
    	
    	LinearReward r = cast(LinearReward)model.getReward();
    	
    	r.setParams(w);
    	
//        double[StateAction] Q_value = QValueSoftMaxSolve(model, qval_thresh);
        double[StateAction] Q_value = QValueSolve(model, qval_thresh);
        double sum = 0;

        foreach (sa, count ; sa_freq[0] ) {

        	sum += Q_value[sa] * count;
        } 
/* 
        double [] e;
    	
        foreach (sar [] sar1; true_samples) {
        	foreach (sar SAR; sar1) {
        		e ~= exp(dotProduct(w, r.features(SAR.s, SAR.a)));
        	}
        }
        
        double sum_e = 0;
        foreach (entry; e)
        	sum_e += entry;
        	
        e[] /= sum_e;
        
        foreach (entry; e)
        	sum += log(entry);
 */       	
        	

        
        Action[State] V;
        foreach (State s; model.S()) {
        	double[Action] actions;
        	foreach (Action a; model.A(s)) {
        		actions[a] = Q_value[new StateAction(s,a)];
        	}
        	V[s] = Distr!Action.argmax(actions);
        	if (V[s] is null) {
        		writeln("Got a null action");
        		foreach (SA, v; Q_value) {
        			writeln(SA.s, " ", SA.a, " -> ", v);
        		}
        		writeln(s); 
        		assert (false);
        		
        	}
        }
        Agent agent = new MapAgent(V);
        
        // Check if the policy is identical to the last one, if so, don't bother resampling
        bool resample = true;
/*    	foreach (State s; model.S()) {
			if (agent.actions(s) != lastAgent.actions(s)) {
				resample = true;
				break;
			}	
	    }*/
    	
    	lastAgent = agent;
    	
/*        foreach (State s; model.S()) {
        	writeln(s, " ", agent.actions(s));
        	
        } */
    	
 /*   	double[StateAction] D = calcStateActionFreq(agent, initial, model, sample_length);
//    	double[StateAction] D = calcStateActionFreq3(agent, initial, model, sample_length);
        
        double [] _mu = feature_expectations(model, D);*/

        sar [][] samples;
        if (resample) {
        	samples = generate_samples(model, agent, initial, this.n_samples, sample_length);
        } else {
        	samples = lastSamples;
        }
        lastSamples = samples;

        double [] _mu = feature_expectations(model, samples);
                
        g.length = w.length;

        g[] = -( mu_E[] - _mu[] );

        return -sum;		
		
	}

    override double [] feature_expectations(Model model, sar [][] samples) {
    	
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





}



class MaxEntIrlBothPatrollers : MaxEntIrl {
		
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError=0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError, qval_thresh);
	}
	
	private Agent otherpolicy;
	
	public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights, Agent otherpolicy) {
		this.otherpolicy = otherpolicy;
		
		return super.solve(model, initial, true_samples, init_weights, opt_value, opt_weights);
		
	}
	
    
    
    public Tuple!(sar[], sar[]) add_delay(ref sar [] hist1, ref sar [] hist2) {
    	sar[] newhist1;
    	sar[] newhist2;
    	
    	for (int i = 0; i < hist1.length; i ++) {
    		newhist1 ~= hist1[i];
    		newhist2 ~= hist2[i];
    		
    		if (hist1[i].s.samePlaceAs(hist2[i].s) || (i > 0 && (hist1[i - 1].s.samePlaceAs(hist2[i].s) || hist1[i].s.samePlaceAs(hist2[i - 1].s))) ) {
	    		newhist1 ~= hist1[i];
	    		newhist2 ~= hist2[i];
	    		newhist1 ~= hist1[i];
	    		newhist2 ~= hist2[i];
    			
    		}
    	}
    	
    	return tuple(newhist1, newhist2);
    	
    }
	
}



class MaxEntIrlDelayAgents : MaxEntIrl {
	private double qval_thresh;
	private Model [] models;
	private double[State][] initials;
	private sar[][][] true_samples;
	private int [] sample_lengths;
	private double [][] mu_Es;
	
	private Agent[] lastAgents;
	private bool[State][] observedStates;
	private double[Action][] equilibrium;
	private int bestEquilibria;
	private int interactionLength;
	
	private double[] savedWeights;
	private double minValue;
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError =0.1, double qval_thresh = 0.01) {
		super(max_iter, solver, n_samples, error, solverError);

		this.qval_thresh = qval_thresh;
		
	}

	
	public Agent [] solve(Model [] models, double[State][] initials, sar[][][] true_samples, double [][] init_weights, double[Action][] ne, int interactionLength, out double opt_value, out double [][] opt_weights) {
		
        // Compute feature expectations of agent = mu_E from samples
        
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);
        param.max_iterations = max_iter;
//        param.epsilon = error;
//        param.min_step = .0000001;
        
        this.models = models;
        this.initials = initials;
        foreach (ref initial; this.initials) 
        	initial = initial.rehash;
        
        this.true_samples = true_samples;
        
        this.interactionLength = interactionLength;
        this.minValue = double.max;
        
        this.equilibrium = ne;
        
        // compute the average trajectory length for each agent
        this.sample_lengths.length = this.true_samples.length;
        foreach (int i, sar [][] SARarr; true_samples) {
        	this.sample_lengths[i] = cast(int)SARarr.length;
        }

        // feature expectations for each expert
        mu_Es.length = models.length;
        foreach (int i, Model model; models) {
        	mu_Es[i] = feature_expectations2(model, true_samples[i], i);
        }

//        writeln(mu_Es);
        
        // observed states from the trajectories
        observedStates.length = true_samples.length;
        foreach (int i, sar [][] sararr; true_samples) {
	        foreach (sar [] sar1; sararr) {
	        	foreach (sar SAR; sar1) {
	        		foreach (State s; models[i].S()) {
	        			if (s.samePlaceAs(SAR.s)) { 
	        				observedStates[i][s] = true;
	        			}
	        		}
	        	}
	        }
        }
        
//        writeln("True Samples ", mu_E);
        
        int x_len = 0;
        opt_weights.length = models.length;
        foreach (int i, Model model; models) {
        	
	        LinearReward r = cast(LinearReward)model.getReward();
	        opt_weights[i].length = r.dim();
	        x_len += r.dim();
        }
        
        double * x = lbfgs_malloc(x_len);
        scope(exit) {
        	lbfgs_free(x);
        } 
        
        int pos = 0;
        lastAgents.length = models.length;
        foreach (int i, Model model; models) {
	        for (int j = 0; j < opt_weights[i].length; j ++) { 
	        	x[pos] = init_weights[i][j];
				pos ++;
			}

			lastAgents[] = new RandomAgent(model.A(null));
        }	
        
        double finalValue;

     	int ret = lbfgs(x_len, x, &finalValue, &evaluate_maxent, &progress, &this, &param);
 
 
     	Agent [] returnval = new Agent[models.length];
     	pos = 0;
     	foreach (int i, Model model; models) {
	        for (int j = 0; j < opt_weights[i].length; j ++) {
	        	opt_weights[i][j] = savedWeights[pos];
	        	pos ++;
	        }

	        LinearReward r = cast(LinearReward)model.getReward();
	        r.setParams(opt_weights[i]);
        
            returnval[i] = solver.createPolicy(model, solver.solve(model, solverError));
             
       //     writeln(ret, ": ", finalValue, " -> ", opt_weights[i]);
        }
     	     
        opt_value = finalValue;

        
        return returnval;
		
	}
	
	
	
	override double evaluate(double [] w_all, out double [] g, double step) {
		
        double sum = 0;
        int pos = 0;
        foreach (int i, Model model; models) {
	    	
	    	LinearReward r = cast(LinearReward)model.getReward();
	    	
	    	double [] weights = new double[r.dim()];
	    	for (int j = 0; j < weights.length; j ++) {
	    		weights[j] = w_all[pos];
	    		pos ++;
	    	}
	    	r.setParams(weights);
	    	
//	        double[StateAction] Q_value = QValueSoftMaxSolve(model, qval_thresh);
	        double[StateAction] Q_value = QValueSolve(model, qval_thresh);

	        foreach (sa, count ; sa_freq[i] ) {
	        	sum += Q_value[sa] * count;
	        } 
	
	        
	        Action[State] V;
	        foreach (State s; model.S()) {
	        	double[Action] actions;
	        	foreach (Action a; model.A(s)) {
	        		actions[a] = Q_value[new StateAction(s,a)];
	        	}
	        	V[s] = Distr!Action.argmax(actions);
	        	if (V[s] is null) {
	        		writeln("Got a null action");
	        		foreach (SA, v; Q_value) {
	        			writeln(SA.s, " ", SA.a, " -> ", v);
	        		}
	        		writeln(s); 
	        		assert (false);
	        		
	        	}
	        }
	        Agent agent = new MapAgent(V);
	        	    	
	    	lastAgents[i] = agent;
    	}
/*        foreach (State s; model.S()) {
        	writeln(s, " ", agent.actions(s));
        	
        } */
    	
		
    	sar [][][] samples = generate_samples_interaction(models, lastAgents, initials, this.n_samples, sample_lengths, equilibrium, this.interactionLength);
        
        double [] temp_g;
        
        pos = 0;
        temp_g.length = 0;
        foreach (int j, Model model; models) {
	        double [] _mu = feature_expectations(model, samples[j], observedStates[j]);
	        temp_g.length = temp_g.length + _mu.length;
	
	        temp_g[pos .. (pos + _mu.length)] = -( mu_Es[j][] - _mu[] );
	        pos += _mu.length;
        }
        
       	g = temp_g;

        double l2 = l2norm(temp_g); 

        if (l2 < minValue) {
        	minValue = l2;
        	savedWeights = w_all;

        }
        return -sum;
		
	}

    double [] feature_expectations(Model model, sar [][] samples, bool[State] observedStates) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	// modified to only include observed states
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
        		if (observedStates.get(SAR.s, false)) {
	        		double [] f = ff.features(SAR.s, SAR.a);
	        		
	        		returnval[] += f[];
        		}
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}	
	
	
}



class NgProjIrl : IRL{
	
	private Model model;
	private double[State] initial;
	private sar[][] true_samples;
	private int sample_length;
	private double [] mu_E;
	private bool[State] observedStates;
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError=0.1) {
		super(max_iter, solver, n_samples, error, solverError);
	}
	
	override public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights) {
       // Compute feature expectations of agent = mu_E from samples
        
        this.model = model;
        this.initial = initial;
        this.true_samples = true_samples;
        this.sample_length = cast(int)true_samples.length;
        
        mu_E = feature_expectations2(model, true_samples, 0);
//        writeln("True Samples ", mu_E);
        
        /*
        foreach (sar [] sar1; true_samples) {
        	foreach (sar SAR; sar1) {
        		foreach (State s; model.S()) {
        			if (s.samePlaceAs(SAR.s)) { 
        				observedStates[s] = true;
        			}
        		}
        	}
        }*/
        LinearReward r = cast(LinearReward)model.getReward();
        opt_weights.length = r.dim();
//        for (int i = 0; i < r.dim(); i ++)
//        	opt_weights[i] = uniform(0.0, 1.0);
        
        
        Agent agent;
        sar [][] samples;
        double [] mu = new double[r.dim()];

        double lastT = 0;
        double [] mu_bar = new double[r.dim()];
        double [] mmmb = new double[r.dim()];
        double [] w = new double[r.dim()];
        w[] = init_weights[];

    	r.setParams(w);

    	agent = solver.createPolicy(model, solver.solve(model, solverError));    
                    
        samples = generate_samples(model, agent, initial, this.n_samples, sample_length);
        mu = feature_expectations(model, samples);

//        writeln(w, " ", mu);
        
       	mu_bar[] = mu[];
       	w[] = mu_E[] - mu[];
       	
       	double [] bestmu = new double[mu.length];
       	bestmu[] = mu[];
       	double [] bestw = new double[w.length];
       	bestw[] = w[]; 
       	double bestDiff = l2norm(w);
                              
        for (int i = 0; i < max_iter; i ++) {
        	
            
        	
        	r.setParams(w);

        	agent = solver.createPolicy(model, solver.solve(model, solverError));    
                        
//            Compute feature expectations of pi^(i) = mu^(i)
            samples = generate_samples(model, agent, initial, this.n_samples, sample_length);
            mu = feature_expectations(model, samples);

//            writeln(w, " ", mu, " " , mu_bar);
        	
        	double [] tempTest = new double[w.length];
        	tempTest[] = mu_E[] - mu[];
        	double l2test = l2norm(tempTest);
        	if (l2test < bestDiff) {
        		bestDiff = l2test;
        		bestw[] = w[];
        		bestmu[] = mu[];
        	}
                    	
    		double [] temp;
    		temp.length = r.dim();

    		mmmb[] = mu[];
    		mmmb[] = mmmb[] - mu_bar[];

    		temp[] = mu_E[] - mu_bar[];
    		double dotp = dotProduct(mmmb, temp) / dotProduct(mmmb, mmmb);
	    	mu_bar[] += dotp * mmmb [];
    		
        	
        	w[] = mu_E[] - mu_bar[]; 
        	
        	double t = l2norm(w);

 //       	writeln("IRLApproxSolver Iteration #", i, " w: ", w, " ", t);
        	
            if (t < error)
                break;
            if (abs(t - lastT) < .000001)
                break;

            lastT = t;
        } 
//        writeln("Best: ", bestw, " ", bestmu);
              
        opt_weights[] = bestw[];
        r.setParams(opt_weights);

        double[StateAction] Q_value = QValueSolve(model, solverError);
        opt_value = 0;
     
        
        foreach (sa, count ; sa_freq[0] ) {
        	opt_value += Q_value[sa] * count;
        } 
        
        return solver.createPolicy(model, solver.solve(model, solverError));
      
	}
    		
}



class NgProjIrlBothPatrollers : NgProjIrl {
		
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError=0.1) {
		super(max_iter, solver, n_samples, error, solverError);
	}
	
	private Agent otherpolicy;
	
	public Agent solve(Model model, double[State] initial, sar[][] true_samples, double [] init_weights, out double opt_value, out double [] opt_weights, Agent otherpolicy) {
		this.otherpolicy = otherpolicy;
		
		return super.solve(model, initial, true_samples, init_weights, opt_value, opt_weights);
		
	}
    
    
    public Tuple!(sar[], sar[]) add_delay(ref sar [] hist1, ref sar [] hist2) {
    	sar[] newhist1;
    	sar[] newhist2;
    	
    	for (int i = 0; i < hist1.length; i ++) {
    		newhist1 ~= hist1[i];
    		newhist2 ~= hist2[i];
    		
    		if (hist1[i].s.samePlaceAs(hist2[i].s) || (i > 0 && (hist1[i - 1].s.samePlaceAs(hist2[i].s) || hist1[i].s.samePlaceAs(hist2[i - 1].s))) ) {
	    		newhist1 ~= hist1[i];
	    		newhist2 ~= hist2[i];
	    		newhist1 ~= hist1[i];
	    		newhist2 ~= hist2[i];
    			
    		}
    	}
    	
    	return tuple(newhist1, newhist2);
    	
    }
	
}



class NgProjIrlDelayAgents : NgProjIrl{
	
	Model [] models;
	double[State][] initials;
	sar[][][] true_samples;
	int [] sample_lengths;
	double [][] mu_Es;
	private bool[State][] observedStates;
	private double[Action][][] equilibria;
	private int interactionLength;
	
	public this(int max_iter, MDPSolver solver, int n_samples=500, double error=0.1, double solverError=0.1) {
		super(max_iter, solver, n_samples, error, solverError);
	}
	
	public Agent [] solve(Model [] models, double[State][] initials, sar[][][] true_samples, double [][] init_weights, double[Action][][] equilibria, out double opt_value, out double [][] opt_weights, out int bestEquilibrium, int interactionLength) {
       // Compute feature expectations of agent = mu_E from samples
        
        this.models = models;
        this.initials = initials;
        this.true_samples = true_samples;
        this.equilibria = equilibria;        
        this.interactionLength = interactionLength;
        
        // compute the average trajectory length for each agent
        this.sample_lengths.length = this.true_samples.length;
        foreach (int i, sar [][] SARarr; true_samples) {
        	this.sample_lengths[i] = cast(int)SARarr.length;
        }
        // feature expectations for each expert
        mu_Es.length = models.length;
        foreach (int i, Model model; models) {
        	mu_Es[i] = feature_expectations2(model, true_samples[i], i);
        }
        
        // observed states from the trajectories
        observedStates.length = true_samples.length;
        /*foreach (int i, sar [][] sararr; true_samples) {
	        foreach (sar [] sar1; sararr) {
	        	foreach (sar SAR; sar1) {
	        		foreach (State s; models[i].S()) {
	        			if (s.samePlaceAs(SAR.s)) { 
	        				observedStates[i][s] = true;
	        			}
	        		}
	        	}
	        }
        }*/
        
        opt_weights.length = models.length;
        LinearReward [] r;
        double [][] w;
        w.length = models.length;
        
        foreach (int i, Model model; models) {
	        r ~= cast(LinearReward)model.getReward();
	        opt_weights[i].length = r[i].dim();
        	w[i].length = r[i].dim();
	        w[i][] = init_weights[i][];
	        r[i].setParams(w[i]);
        }
        Agent [] agents;
        foreach (m; models)
        	agents ~= solver.createPolicy(m, solver.solve(m, solverError));
        	
       	double [][] bestw;
       	bestw.length = w.length;
       	double bestDiff = 0;
        double [][] temp_mu;
        temp_mu.length = w.length;

        sar [][][] samples = generate_samples_interaction(models, agents, initials, this.n_samples, sample_lengths, equilibria[0], this.interactionLength);
        double [][] mu_bar;
        foreach (int i, Model model; models) {
        	mu_bar ~= feature_expectations(model, samples[i], observedStates[i]);
        	w[i][] = mu_Es[i][] - mu_bar[i][];
        	bestw[i] = new double[w[i].length];
        	bestw[i][] = w[i][];
        	bestDiff += l2norm(bestw[i]);
        	temp_mu[i].length = w[i].length; 
        }

        double [][] mu;
        mu.length = w.length;
        
        
        double lastT = 0;
        
        for (int i = 0; i < max_iter; i ++) {
                        
                        
	    	double min_expectation_difference = double.max;
	    	double [][] next_mu_bar;
	    	
        	foreach (int j, Model model; models) {	    		        	
	        	r[j].setParams(w[j]);
	            
	            agents[j] = solver.createPolicy(model, solver.solve(model, solverError));
	        }
                
	    	foreach (int j, double[Action][] equilibrium; equilibria) {

		    	samples = generate_samples_interaction(models, agents, initials, this.n_samples, sample_lengths, equilibrium, this.interactionLength);
		    	
		    	
		        double [][] temp_mu_bar;
		        temp_mu_bar.length = models.length;
		        double l2 = 0;
		        
		        foreach (int k, Model model; models) {
			        mu[k] = feature_expectations(model, samples[k], observedStates[k]);
		
		    		mu[k][] -= mu_bar[k][];
		    		double [] temp = new double[mu[k].length];
		    		temp[] = mu_Es[k][] - mu_bar[k][];
		
		    		double dotp = dotProduct(mu[k], temp) / dotProduct(mu[k], mu[k]);

		    		temp_mu_bar[k].length = r[k].dim();
		    		temp_mu_bar[k][] = mu_bar[k][] + dotp * mu[k][];

		    		temp[] = mu_Es[k][] - temp_mu_bar[k][];
		    		l2 += l2norm(temp); 

		        }
		        
		        
		        if (l2 < min_expectation_difference) {
		        	foreach (int k, Model model; models) {
		        		temp_mu[k][] = mu[k][];
		        	}
		        	min_expectation_difference = l2;
		        	next_mu_bar = temp_mu_bar;
		        	bestEquilibrium = j;
		        }
	        }
                        
            mu_bar = next_mu_bar;            


            // test if this is the best mu we've seen
            double l2test = 0;
            foreach (int k, Model model; models) {
            	
	        	double [] tempTest = new double[w[k].length];
	        	tempTest[] = mu_Es[k][] - temp_mu[k][];
	        	l2test += l2norm(tempTest);
       	
            }
        	if (l2test < bestDiff) {
        		bestDiff = l2test;
        		
	            foreach (int k, Model model; models) {
	            	bestw[k][] = w[k][];	       	
	            }
        	}                 

        	double t = 0;
        	foreach (int j, Model model; models) {
	        	w[j][] = mu_Es[j][] - mu_bar[j][]; 
	        	
	        	t += l2norm(w[j]);

	        }                

//        	writeln("IRLApproxSolver Iteration #", i);
        	
            if (t < error)
                break;
            if (abs(t - lastT) < .000001)
                break;

            lastT = t;

        } 
                
        opt_weights = bestw;
        
 
     	Agent [] returnval = new Agent[models.length];
        opt_value = 0;
     	foreach (int i, Model model; models) {

	        LinearReward reward = cast(LinearReward)model.getReward();
	        reward.setParams(opt_weights[i]);
        
            returnval[i] = solver.createPolicy(model, solver.solve(model, solverError));
             
	        double[StateAction] Q_value = QValueSolve(model, solverError);
	        
	        foreach (sa, count ; sa_freq[i] ) {
	        	opt_value += Q_value[sa] * count;
	        } 
//        writeln(finalValue, " -> ", opt_weights);
        }
     	
     	
        
        return returnval;
      
	}
	
    double [] feature_expectations(Model model, sar [][] samples, bool[State] observedStates) {
/*        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t) */
    	
    	// modified to only include observed states
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	double [] returnval;
    	returnval.length = ff.dim();
    	returnval[] = 0;
    	
        foreach (sar [] sample; samples) {
        	foreach (sar SAR; sample) {
//        		if (observedStates.get(SAR.s, false)) {
	        		double [] f = ff.features(SAR.s, SAR.a);
	        		returnval[] += f[];
//        		}
        	}
        }
        returnval [] /= samples.length;
        return returnval;
	}	

}
