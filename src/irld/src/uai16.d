import mdp;
import std.stdio;
import std.random;
import std.math;
    
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
    
    double [] calc_E_step_single_agent_gibbs(Model model, Agent policy, sar[][] true_samples, State [] observableStates, double error, int max_repeats, double[State] initial) {

    	// using the last policy we have and the list of Y's, generate a new set of expert feature expectations, using gibbs sampling
    	// to generate trajectory samples, 
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[ff.dim()];
    	returnval[] = 0;
    	
    	size_t repeats = 0;
    	double [] last_avg = returnval.dup;
    	last_avg[] = 0;
    	
    	while(true) {
    		
			double [] temp = new double[ff.dim()];
    		temp[] = 0;
	   		foreach(i, sample; true_samples) {
	    		
	    		// assign the burn in (m) and sample count (n) values from Raftery and Lewis 1992
		    	RunningAverageConvergenceThreshold convergenceTest = new RunningAverageConvergenceThreshold(error / 10, 20);
	    			    		
	    		temp[] += gibbs_sampler(sample, model, policy, i, 20, convergenceTest, observableStates, initial)[];
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
	    	if (max_diff < error || repeats > max_repeats) {
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
    
    double [] gibbs_sampler(sar [] sample, Model model, Agent policy, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest, State [] observableStates, double[State] initial) {
    	
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
    	
    	double [] returnval = new double[ff.dim()];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		
		double [] min_features = new double[ff.dim()];
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
			
			return min_features;
			
		} 

		double [] max_features = min_features.dup;
		max_features[] += missing_timesteps.length;

		
		auto working_sample = sample.dup;
		
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(idx; missing_timesteps) {
			if (idx > 0)
				working_sample[idx].s = Distr!State.sample(model.T(working_sample[idx-1].s, working_sample[idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = initial.dup;
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
		
		size_t i = 0;
		
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
			
			auto timestep = missing_timesteps[timestep_idx];
			
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
			
			if (i > M) {
				total_features[] += cur_features[];
				
				auto temp = total_features.dup;
				temp[] /= (i + 1) - M;
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					debug(raftery) {
						writeln("Converged after ", i, " iterations.");
						if (i == 0) {
							writeln( cur_features);
							writeln(convergenceTest.storage);
							writeln(convergenceTest.last);
						}
					}
					break;
				}				
			}	
			
			i ++;	
		}
		
		
		// now average the feature expectations
		total_features[] /= (i + 1) - M;
		
		return total_features;
    	
    }
    
    
    
    
    
	
	size_t calc_E_step_exact_single_agent(Model model, Agent policy, sar[][] true_samples, State [] observableStates, double error, int max_repeats, double[State] initial) {
		
		
		double [][] feature_expectations_per_trajectory;
		size_t[] y_from_trajectory;	
		double [] pr_traj;
	
		calc_y_and_z(model, true_samples, observableStates, initial, y_from_trajectory, feature_expectations_per_trajectory, pr_traj);
		
		auto traj_distr = getTrajectoryDistribution(pr_traj);
	
	   	LinearReward ff = cast(LinearReward)model.getReward();


    	double temp [];
    	temp.length = ff.dim();
    	temp[] = 0;

    	double [] pr_z_denom = pr_z_denominator(traj_distr, true_samples, y_from_trajectory);
    	
    	foreach(i, pr_zeta; traj_distr) {
   			temp[] += ( pr_z_y(i, traj_distr, pr_z_denom, y_from_trajectory) * feature_expectations_per_trajectory[i][] ) / true_samples.length;
    	}
    	
    	return pr_traj.length;
	}
		
	double[] getTrajectoryDistribution(double [] pr_traj) {
		
		
		// return anything, we don't actually care about the probs
		
		return pr_traj;
	}
	    	    
    double pr_z_y(size_t trajectory_number, double [] traj_distr, double [] denominators, size_t[] y_from_trajectory) {
    	
    	double numerator = traj_distr[trajectory_number];
    	
    	double denominator = denominators[y_from_trajectory[trajectory_number]];
    	
    	return numerator / denominator;
    }

	
	void calc_y_and_z(Model model, sar[][] true_samples, State[] observableStates, double[State] initial,  out size_t[] y_from_trajectory, out double[][] feature_expectations_per_trajectory, out double [] pr_traj) {
		

		
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

        size_t sample_length = 0;

		foreach(i, traj; true_samples) {
			
			if (traj.length > sample_length)
				sample_length = traj.length;
			
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
																		
									sample_length = 0;
									foreach(tt; true_samples) {
										if (tt.length > sample_length)
											sample_length = tt.length;
										
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


    double [] pr_z_denominator(double [] traj_distr, sar [][] true_samples, size_t [] y_from_trajectory) {
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
    
    
    
    
    
   double [] calc_E_step_single_agent_forward_backward(Model model, Agent policy, sar[][] true_samples, State [] observableStates, double error, int max_repeats, double[State] initial) {
    	
		// figure out the maximum sample length (number of timesteps)
		size_t t = true_samples[0].length;

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
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[ff.dim()];
		
		// first create an index of missing timesteps
		
    	int Y_num = 0;
    	
 			
	    	double [StateAction] forward(double [StateAction] p_t, sar obs) {
	    		double [StateAction] returnval;
	     		State [] S0List;
	     		State [] S1List;
	     		
	     		if (obs.s !is null) {
	     			S0List = [obs.s];
	     		} else 
	     			S0List = occluded_states.keys;
	     		
	    		foreach (s; S0List) {
		     		auto A0List = model.A(s);
		     		if (obs.a !is null) {
		     			A0List = [obs.a];
		     		}
	    			foreach(a; A0List) {

	    				double temp_prob = policy.actions(s).get(a, 0.00000001);
						
	    				foreach(sa, prob; p_t) {
	    					
	    					temp_prob += model.T(sa.s, sa.a).get(s, 0.00000001) * prob;
	    				}
	    				
	    				returnval[new StateAction(s, a)] = temp_prob;

			    	}
	    		}
	    		return returnval;			
	    		
	    	}
	    	
	    	double [StateAction] backward(double [StateAction] prev_b, sar obs) {
	     		double [StateAction] returnval;
	    		foreach (s; model.S()) {
	    			foreach(a; model.A(s)) {
			    				
						double total = 0;
						
			     		State [] S0List;
			     		
			     		if (obs.s !is null) {
			     			S0List = [obs.s];
			     		} else 
			     			S0List = occluded_states.keys;
			     		
			    		foreach (sb; S0List) {
				     		auto A0List = model.A(sb);
				     		if (obs.a !is null) {
				     			A0List = [obs.a];
				     		}
			    			foreach(sba; A0List) {

			    				StateAction sa = new StateAction(sb, sba);
			    				
			    				auto prob = prev_b.get(sa, 0.00000001);
			    				
    							double actionprob = policy.actions(sa.s).get(sa.a, 0.00000001);
								
    							total += prob * model.T(s, a).get(sa.s, 0.00000001) * actionprob;
			    				
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

    		foreach (s; model.S()) {
    			foreach(action; model.A(s)) {
					temp[new StateAction(s, action)] = policy.actions(s).get(action, 0);
    			}
    		}
	    	Distr!StateAction.normalize(temp);
    		fv ~= temp;
    		foreach(long i; 0..t) {
    			// create ev at time t
    			sar ev;

				if (true_samples[Y_num].length > i)
					ev = true_samples[Y_num][i];
				else
					ev = sar(null, null, 0);	
    			
    			fv ~= forward(fv[$-1], ev);
    		}
    		// prob, state/action, timestep for final vector
    		
    		double [StateAction] b;
    		double [StateAction][] sv;

			foreach (s; model.S()) 
    			foreach(action; model.A(s)) 
					b[new StateAction(s, action)] = 1.0;
    			
    		for(long i = t-1; i >= 0; i --) {
				double[StateAction] temp_sa;
				foreach(sa, prob; b)
					temp_sa[sa] = fv[$-1].get(sa, 0.00000001) * prob;
				fv.length = fv.length - 1;
				
				sv ~= temp_sa;
				
	   			sar ev;
				if (true_samples[Y_num].length > i)
					ev = true_samples[Y_num][i];
				else
					ev = sar(null, null, 0);	
				
	    		b = backward(b, ev);
    			
    		}
		foreach(timestep, sa_arr; sv) {
				foreach(sa, prob; sa_arr) {
					returnval[] += prob * ff.features(sa.s, sa.a)[];
				}	
		}
    	
    	
    	return returnval;
    	
    }	    
   
   
   
   
    
    double [] calc_E_step_single_agent_blocked_gibbs(Model model, Agent policy, sar[][] true_samples, State [] observableStates, double error, int max_repeats, double[State] initial) {

    	// using the last policy we have and the list of Y's, generate a new set of expert feature expectations, using gibbs sampling
    	// to generate trajectory samples, 
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[ff.dim()];
    	returnval[] = 0;
    	
    	size_t repeats = 0;
    	double [] last_avg = returnval.dup;
    	last_avg[] = 0;
    	
    	while(true) {
    		
			double [] temp = new double[ff.dim()];
    		temp[] = 0;
	   		foreach(i, sample; true_samples) {
	    		
	    		// assign the burn in (m) and sample count (n) values from Raftery and Lewis 1992
		    	RunningAverageConvergenceThreshold convergenceTest = new RunningAverageConvergenceThreshold(error / 10, 20);
	    			    		
	    		temp[] += gibbs_sampler_blocked(sample, model, policy, i, 20, convergenceTest, observableStates, initial)[];
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
	    	if (max_diff < error || repeats > max_repeats) {
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
    
    double [] gibbs_sampler_blocked(sar [] sample, Model model, Agent policy, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest, State [] observableStates, double[State] initial) {
   	
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
    	
    	double [] returnval = new double[ff.dim()];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		
		double [] min_features = new double[ff.dim()];
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
			
			return min_features;
			
		} 

		double [] max_features = min_features.dup;
		max_features[] += missing_timesteps.length;

		
		auto working_sample = sample.dup;
		
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(idx; missing_timesteps) {
			if (idx > 0)
				working_sample[idx].s = Distr!State.sample(model.T(working_sample[idx-1].s, working_sample[idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = initial.dup;
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
		
		size_t i = 0;
		

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
			
			auto timestep = missing_timesteps[timestep_idx];
						
			
			cur_features[] -= ff.features(working_sample[timestep].s, working_sample[timestep].a)[];
			
			// sample the state from T(s, a, s') T(s'', a', s''') Pr(a|s)
						
			double[StateAction] state_action_distr;
			
			foreach(s, s_v; occluded_states) {
				
				foreach (a; model.A(s)) {
					StateAction sa = new StateAction(s, a);
					
					double v = 1.0;
					
					
					if (timestep > 0) {
						v *= model.T(working_sample[timestep-1].s, working_sample[timestep-1].a).get(s, 0.00000001);
					}
					
					if (timestep < working_sample.length - 1) {
						v *= model.T(s, a).get(working_sample[timestep+1].s, 0.00000001);
					}
					
					v *= policy.actions(s).get(a, 0.00000001);
					
					state_action_distr[sa] = v;	
				}	
			}
			
			try {
				Distr!StateAction.normalize(state_action_distr);

				auto sa = Distr!StateAction.sample(state_action_distr);
				
				working_sample[timestep].s = sa.s;
				working_sample[timestep].a = sa.a;

			} catch (Exception e) {
				
			}
			
			cur_features[] += ff.features(working_sample[timestep].s, working_sample[timestep].a)[];
			
			if (i > M) {
				total_features[] += cur_features[];
				
				auto temp = total_features.dup;
				temp[] /= (i + 1) - M;
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					debug(raftery) {
						writeln("Converged after ", i, " iterations.");
						if (i == 0) {
							writeln( cur_features);
							writeln(convergenceTest.storage);
							writeln(convergenceTest.last);
						}
					}
					break;
				}				
			}	
			
			i ++;	
		}
		
		
		// now average the feature expectations
		total_features[] /= (i + 1) - M;
		
		return total_features;
    				
			
			    	
    }
    
    
    double [] calc_E_step_single_agent_multi_timestep_blocked_gibbs(Model model, Agent policy, sar[][] true_samples, State [] observableStates, double error, int max_repeats, double[State] initial) {

    	// using the last policy we have and the list of Y's, generate a new set of expert feature expectations, using gibbs sampling
    	// to generate trajectory samples, 
    	
    	LinearReward ff = cast(LinearReward)model.getReward();
    	
    	double [] returnval = new double[ff.dim()];
    	returnval[] = 0;
    	
    	size_t repeats = 0;
    	double [] last_avg = returnval.dup;
    	last_avg[] = 0;
    	
    	while(true) {
    		
			double [] temp = new double[ff.dim()];
    		temp[] = 0;
	   		foreach(i, sample; true_samples) {
	    		
	    		// assign the burn in (m) and sample count (n) values from Raftery and Lewis 1992
		    	RunningAverageConvergenceThreshold convergenceTest = new RunningAverageConvergenceThreshold(error / 10, 20);
	    			    		
	    		temp[] += gibbs_sampler_blocked_multi_timestep(sample, model, policy, i, 20, convergenceTest, observableStates, initial)[];
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
	    	if (max_diff < error || repeats > max_repeats) {
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
    
    double [] gibbs_sampler_blocked_multi_timestep(sar [] sample, Model model, Agent policy, size_t Y_num, size_t M, RunningAverageConvergenceThreshold convergenceTest, State [] observableStates, double[State] initial) {
   	
    	int timestep_size = 3;
    	
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
    	
    	double [] returnval = new double[ff.dim()];
		
		// first create an index of missing timesteps
		
		size_t[] missing_timesteps;
		
		double [] min_features = new double[ff.dim()];
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
			
			return min_features;
			
		} 

		double [] max_features = min_features.dup;
		max_features[] += missing_timesteps.length;

		
		auto working_sample = sample.dup;
		
		
		// then create a copy of sample and fill it in randomly
		
		double[] cur_features = min_features.dup;
		
		foreach(idx; missing_timesteps) {
			if (idx > 0)
				working_sample[idx].s = Distr!State.sample(model.T(working_sample[idx-1].s, working_sample[idx-1].a));
			else {
				// create initial distribution from initial * occluded
				
				auto initial_occluded = initial.dup;
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
		
		double [StateAction][] getSADistr(long timestep_start, long timestep_end ) {
			
			
			if (timestep_end < timestep_start) {
				auto temp = timestep_start;
				timestep_start = timestep_end;
				timestep_end = temp;
				
			}
			
			if (timestep_start < 0)
				timestep_start = 0;
				
			if (timestep_end > sample.length)
				timestep_end = sample.length;
				
			
			double [StateAction][] returnval;
			
			
	    	double [StateAction] forward(double [StateAction] p_t, sar obs) {
	    		double [StateAction] returnval;
	     		State [] S0List;
	     		
	     		if (obs.s !is null) {
	     			S0List = [obs.s];
	     		} else 
	     			S0List = occluded_states.keys;
	     		
	    		foreach (s; S0List) {
		     		auto A0List = model.A(s);
		     		if (obs.a !is null) {
		     			A0List = [obs.a];
		     		}
	    			foreach(a; A0List) {

	    				double temp_prob = policy.actions(s).get(a, 0.00000001);
						
	    				foreach(sa, prob; p_t) {
	    					
	    					temp_prob += model.T(sa.s, sa.a).get(s, 0.00000001) * prob;
	    				}
	    				
	    				returnval[new StateAction(s, a)] = temp_prob;

			    	}
	    		}
	    		return returnval;			
	    		
	    	}
	    	
	    	double [StateAction] backward(double [StateAction] prev_b, sar obs) {
	     		double [StateAction] returnval;
	    		foreach (s; model.S()) {
	    			foreach(a; model.A(s)) {
			    				
						double total = 0;
						
			     		State [] S0List;
			     		
			     		if (obs.s !is null) {
			     			S0List = [obs.s];
			     		} else 
			     			S0List = occluded_states.keys;
			     		
			    		foreach (sb; S0List) {
				     		auto A0List = model.A(sb);
				     		if (obs.a !is null) {
				     			A0List = [obs.a];
				     		}
			    			foreach(sba; A0List) {

			    				StateAction sa = new StateAction(sb, sba);
			    				
			    				auto prob = prev_b.get(sa, 0.00000001);
			    				
    							double actionprob = policy.actions(sa.s).get(sa.a, 0.00000001);
								
    							total += prob * model.T(s, a).get(sa.s, 0.00000001) * actionprob;
			    				
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
	    		foreach (s; model.S()) {
	    			foreach(action; model.A(s)) {
						temp[new StateAction(s, action)] = policy.actions(s).get(action, 0);
	    			}
	    		}
    		} else {
    			temp[new StateAction(working_sample[timestep_start - 1].s, working_sample[timestep_start - 1].a)] = 1.0;
    		}
	    	Distr!StateAction.normalize(temp);
    		fv ~= temp;
    		foreach(long i; timestep_start .. timestep_end) {
    			// create ev at time t
    			sar ev;

				if (sample.length > i)
					ev = sample[i];
				else
					ev = sar(null, null, 0);	
    			
    			fv ~= forward(fv[$-1], ev);
    		}
    		// prob, state/action, timestep for final vector
    		
    		double [StateAction] b;
    		
    		if (timestep_end == sample.length) {
    			foreach (s; model.S()) 
	    			foreach(action; model.A(s)) 
    					b[new StateAction(s, action)] = 1.0;
    		} else {
    			b[new StateAction(working_sample[timestep_end].s, working_sample[timestep_end].a)] = 1.0;
    			sar ev;
				if (sample.length > timestep_end)
					ev = sample[timestep_end];
				else
					ev = sar(null, null, 0);	
    			b = backward(b, ev);
    		}
    			
    		for(long i = timestep_end-1; i >= timestep_start; i --) {
				double[StateAction] temp_sa;
				foreach(sa, prob; b)
					temp_sa[sa] = fv[$-1].get(sa, 0.00000001) * prob;
				fv.length = fv.length - 1;
				
				returnval ~= temp_sa;
				
	   			sar ev;
				if (sample.length > i)
					ev = sample[i];
				else
					ev = sar(null, null, 0);	
				
	    		b = backward(b, ev);
    			
    		}
			
			
			return returnval.reverse;
			
		} 		
		
		size_t i = 0;
		

		int step = timestep_size;
		long timestep = -1;
				
//		foreach(i; 0..(N + M)) {
		while (true) {
						
			timestep += step;
			
			if (timestep >= sample.length) {
				step = -step;
				timestep = sample.length - 1;
			}
			
			if (timestep < 0) {
				step = -step;
				timestep = 0;
			}
			
			long next_timestep = timestep + step;
			
			if (next_timestep >= sample.length)
				next_timestep = sample.length - 1;
				
			if (next_timestep < 0)
				next_timestep = 0;
			
			auto state_action_distr = getSADistr(timestep, next_timestep);
			auto start = timestep;
			if (next_timestep < timestep)
				start = next_timestep;
			
			foreach(j, ref sad; state_action_distr) {
				cur_features[] -= ff.features(working_sample[j + start].s, working_sample[j + start].a)[];
				
				try {
					Distr!StateAction.normalize(sad);
	
					auto sa = Distr!StateAction.sample(sad);
					
					working_sample[j + start].s = sa.s;
					working_sample[j + start].a = sa.a;
					
	
				} catch (Exception e) {
					
				}
				
				cur_features[] += ff.features(working_sample[j + start].s, working_sample[j + start].a)[];
		
			}
			
			if (i > M) {
				total_features[] += cur_features[];
				
				auto temp = total_features.dup;
				temp[] /= (i + 1) - M;
				if (convergenceTest !is null && convergenceTest.hasAllConverged(temp)) {
					debug(raftery) {
						writeln("Converged after ", i, " iterations.");
						if (i == 0) {
							writeln( cur_features);
							writeln(convergenceTest.storage);
							writeln(convergenceTest.last);
						}
					}
					break;
				}				
			}	
			
			i ++;	
		}
		
		
		// now average the feature expectations
		total_features[] /= (i + 1) - M;
		
		return total_features;
    				
			
			    	
    }    