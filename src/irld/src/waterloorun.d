import waterloomdp;
import mdp;
import std.stdio;
import irl;
import std.format;
import std.string;
import std.file;
import std.math;
import std.algorithm;
import std.random;


int main(char[][] args) {
	
	const double minTime = 0.3;
	const double maxTime = 1.3;
	const int timeSlots = WaterlooActionTimeSlots;

	// initialize model
	WaterlooModel model = new WaterlooModel();

	LinearReward reward = new WaterlooReward();

	double [] temp = new double[reward.dim()];
	reward.setParams(temp);
	
	model.setReward(reward);
		
	double [1] transition_weights = [0.9];


	double[State][Action][State] trans = model.createTransitionFunction(transition_weights, null);

	
	
	model.setGamma(0.95);
	
	ValueIteration vi = new ValueIteration(int.max, true);
	
	
	double[State] initial;
	foreach (State s ; model.S()) {
		WaterlooState ws = cast(WaterlooState)s;
		if (ws.position == 0 && ws.holdingBall == false) 
			initial[s] = 1.0;
		
	}
	Distr!State.normalize(initial);

	
	// read in trajectories

	char [] filename;
	if (args.length > 1) {
		formattedRead(args[1], "%s", &filename);
	}
	
	sar [][] trajs;
	
	auto chars = readText(filename);
	
	sar [] tempTraj;
	foreach(i, line; splitLines(chars)) {
		if (i == 0)  // skip the header line
			continue;
			
		if (line.length <= 5) {
			// new trajectory marker
			trajs ~= tempTraj.dup;
			tempTraj.length = 0;
			continue;
		}	
			
		int a;
		int b;
		char c;
		double d;
		string e;
		formattedRead(line, "%s, %s, %s, %s, %s", &a, &b, &c, &d, &e);
		
		int f = -1;
		if (e.length > 0) {
			formattedRead(e, "%s", &f);
		}	
		
		bool holdingBall = (c == 'y');
		
		size_t slot = map_to_range(d, minTime, maxTime, timeSlots);
		
		State s = new WaterlooState(a, b, cast(int)slot, holdingBall);
		Action action = null;
		
		foreach(act; model.A(s)) {
			if (act.toHash() == f) {
				action = act;
				break;
			}	
		}
		tempTraj ~= sar(s, action, 1.0);
	}
	if (tempTraj.length > 0)
		trajs ~= tempTraj.dup;
	
	// clean trajectories (remove any timesteps after a terminal state)
	foreach (ref traj; trajs) {
		
		foreach(i, SAR; traj) {
			if (model.is_terminal(SAR.s)) {
				traj.length = i + 1;
				SAR.a = new NullAction();
				break;
			}	
		}
		
	}

	timeTransitionsFromTrajectories(trans, trajs, model);

	model.setT(trans);

	// run IRL
	
	
	double max_entropy = -1;
	double [] found_weights_max_entropy; 
	double [] last_weights_max_entropy;
	Agent max_entropy_best_policy;
	double val;
	
	
	size_t [] saved_y_from_trajectory;
	double [][] saved_feature_expectations;
	double [] saved_pr_traj;
	size_t saved_sample_length = 0;

	foreach (repeat; 0..10) {
		Agent policy;
		double [] foundWeights;

					
		auto lastWeights = new double[reward.dim()];
		for (int i = 0; i < lastWeights.length; i ++)
			lastWeights[i] = uniform(0, 1.0);	
					
		LatentActionsMaxEntIrlZiebartExact irl = new LatentActionsMaxEntIrlZiebartExact(200, vi, model.S(), 0, .0005, .1);

		irl.setYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);

		policy = irl.solve(model, initial, trajs, lastWeights, val, foundWeights);
		
		irl.getYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);

		auto entropy = getEntropy(getTrajDistribution(model, foundWeights, saved_feature_expectations, saved_pr_traj));

		
		if (entropy > max_entropy) {
			max_entropy = entropy;
			found_weights_max_entropy = foundWeights.dup;
			last_weights_max_entropy = lastWeights.dup;
			max_entropy_best_policy = policy;
		}
		reward.setParams(found_weights_max_entropy);
		model.setReward(reward);
		
	}
	writeln("LME Done");
	
	State [] observableStates = model.S();
	
	MaxEntIrlZiebartExact legacy_irl = new MaxEntIrlZiebartExact(200, vi, observableStates, 0, .0005, .1);
	
	double [] foundWeights_legacy;
	
//	Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
	
	reward.setParams(foundWeights_legacy);
	model.setReward(reward);

	writeln("Legacy Done");

	// have to remove all timesteps with a null state
	// update observable states with these removals

	sar [][] trajs_edited;
	trajs_edited.length = trajs.length;
	foreach(i, traj; trajs) {
		foreach (entry; traj) {
			if (entry.a ! is null) {
				trajs_edited[i] ~= sar(entry.s, entry.a, 1.0 / trajs.length);				
			} else {
				trajs_edited[i] ~= sar(null, new NullAction(), 1.0 / trajs.length);
				
				foreach(i2, os; observableStates) {
					if (os == entry.s) {
						// remove this from observableStates
						
						auto tempOS = observableStates[0..i2];
						
						if (i2 < observableStates.length -1)
							tempOS ~= observableStates[i2+1..$];
						
						observableStates = tempOS;
						
						break;
						
					}	
				}
			}	
		}
	}
	writeln(observableStates);
	
	MaxEntIrlPartialVisibility mirl_irl = new MaxEntIrlPartialVisibility(200, vi, 200, .0005, .1, 0.09, observableStates, 5);
	
	double [] foundWeights_mirl;
	
//	Agent policy_mirl = mirl_irl.solve(model, initial, trajs_edited, last_weights_max_entropy, val, foundWeights_mirl);
	
	reward.setParams(foundWeights_mirl);
	model.setReward(reward);

	writeln("mIRL* Done");

	// find most likely action, run ziebart's algorithm
	
	// search for null actions, see which action gives the most probability for the next state, assign that one.

	sar [][] trajs_mla;
	trajs_mla.length = trajs.length;
	foreach(i, traj; trajs) {
		foreach (l, entry; traj) {
			if (entry.a ! is null) {
				trajs_mla[i] ~= sar(entry.s, entry.a, 1.0);				
			} else {
				if (l < traj.length - 1) {
					Action mla;
					double prob = 0;
					
					foreach (a; model.A(entry.s)) {
						foreach ( s, pr; model.T(entry.s, a) ) {
							if (s == traj[l+1].s && pr > prob) {
								mla = a;
								prob = pr;
							}	
						}
					}	
					
					trajs_mla[i] ~= sar(entry.s, mla, 1.0);
				} else {
					trajs_mla[i] ~= sar(entry.s, new NullAction(), 1.0);
				}
			}	
		}
	}
	writeln(trajs_mla);
	

	double [] foundWeights_legacy_mla;
	
	MaxEntIrlZiebartExact legacy_mla_irl = new MaxEntIrlZiebartExact(200, vi, model.S(), 0, .0005, .1);
		
//	Agent policy_legacy_mla = legacy_mla_irl.solve(model, initial, trajs_mla, last_weights_max_entropy, val, foundWeights_legacy_mla);
	
	reward.setParams(foundWeights_legacy_mla);
	model.setReward(reward);


	// assign true action, run ziebart's algorithm

	sar [][] trajs_true;
	trajs_true.length = trajs.length;
	foreach(i, traj; trajs) {
		foreach (l, entry; traj) {
			if (entry.a ! is null) {
				trajs_true[i] ~= sar(entry.s, entry.a, 1.0);				
			} else {
				WaterlooState ws = cast(WaterlooState)entry.s;
				Action action;
			
				if (! ws.holdingBall) {
					switch (ws.position) {
						case 0:
							// hand on the table, haven't grabbed the ball yet, action is grab the ball
							if (ws.ballType == 1 || ws.ballType == 3)
								action = new WaterlooGrabHardAction();
							else 
								action = new WaterlooGrabSoftAction();
							break;
						default:
							break;
					}	
				
				} else {
					
					switch (ws.position) {
						case 2:
						case 3:
							// hand in one of the boxes, action is to drop ball
							if (ws.ballType == 1 || ws.ballType == 3)
								action = new WaterlooReleaseHardAction();
							else 
								action = new WaterlooReleaseSoftAction();
							break;
						default:
							break;
						
					}
				}
				if (action is null)
					action = new NullAction();
				trajs_true[i] ~= sar(entry.s, action, 1.0);				
			}	
		}
	}
	writeln(trajs_true);
	

	double [] foundWeights_legacy_true;
	
	MaxEntIrlZiebartExact legacy_true_irl = new MaxEntIrlZiebartExact(200, vi, model.S(), 0, .0005, .1);
		
	Agent policy_legacy_true = legacy_true_irl.solve(model, initial, trajs_true, last_weights_max_entropy, val, foundWeights_legacy_true);
	
	reward.setParams(foundWeights_legacy_true);
	model.setReward(reward);
		
	// output reward function and policy

	writeln("LME reward weights: ", found_weights_max_entropy);
	writeln("Legacy reward weights: ", foundWeights_legacy);
	writeln("mIRL* reward weights: ", foundWeights_mirl);
	writeln("Most Likely Action reward weights: ", foundWeights_legacy_mla);
	writeln("True reward weights: ", foundWeights_legacy_true);

/*    auto legacy_found_values = analyzeIRLResults([model, model], [policy, policy_legacy], [opt_reward, opt_reward], [initial, initial], 5);
	legacy_found_values[] -= opt_policy_value[];
		
	foreach (fpe; legacy_found_values) {
		Legacy_errors ~= -fpe;
	}					
*/
	

	return 0;
}


double getEntropy(double [] distr) {
	
	double returnval = 0;
	
	foreach(pr; distr) {
		returnval += pr * log(pr);
	}
	
	return -returnval;
}

double [] getTrajDistribution(Model model, double [] weights, double [][] feature_expectations_for_trajectories, double [] pr_traj) {
    double normalizer = 0;
	double [] traj_distr = new double[feature_expectations_for_trajectories.length];
    foreach(i, feature_exp; feature_expectations_for_trajectories) {
			
		double [] weighted_fe = weights.dup;
		weighted_fe[] *= feature_exp[];
		double temp = exp(reduce!("a + b")(0.0, weighted_fe));
		
		normalizer += temp;
		
    	temp *= pr_traj[i];
		
		traj_distr[i] = temp;
    	
	}

	if (normalizer == 0)
		normalizer = double.min;
	debug {
		writeln("Normalizer: ", normalizer);
		
	}
	
	traj_distr[] /= normalizer;
	
	return traj_distr;
}


double [] analyzeIRLResults(Model [] models, Agent [] policies, LinearReward [] true_rewards, double[State][] initial_states, int length) {
	
	// the value of a policy is the sum of state visitation frequency times the reward for the state
	double [] returnval;
	
	foreach (i, model; models) {
		auto sa_freq = calcStateActionFreq(policies[i], initial_states[i], model, length);
		
		double val = 0;
		foreach (sa, freq; sa_freq) {
			val += freq * true_rewards[i].reward(sa.s, sa.a);
		}
		
		returnval ~= val;
	}
	
	return returnval;
	
}

size_t map_to_range(double input, double low, double high, size_t slots) {
	
	if (input < low)
		return 0;
		
	if (input > high)
		return slots - 1;
		
	return cast(size_t)(((input - low) / ((high - low) / (slots - 2))) + 1);
	
}


void timeTransitionsFromTrajectories(double[State][Action][State] trans, sar [][] trajs, Model model) {
	
	// assuming we know the true action at each timestep, build a distribution over action lengths
	// then update trans
	
	
	// initialize
	const int timeSlots = WaterlooActionTimeSlots;
	
	Action [] allActions;
	
	allActions ~= new WaterlooMoveCenterAction();
	allActions ~= new WaterlooMoveBin1Action();
	allActions ~= new WaterlooMoveBin2Action();
	allActions ~= new WaterlooGrabHardAction();
	allActions ~= new WaterlooGrabSoftAction();
	allActions ~= new WaterlooReleaseHardAction();
	allActions ~= new WaterlooReleaseSoftAction();
	
	
	double[][Action] actionTimes;
	
	foreach(a; allActions) {
		actionTimes[a].length = timeSlots;
		actionTimes[a][] = 0;
	}
	
	
	
	
	foreach(i, traj; trajs) {
		
		foreach(t, SAR; traj) {
			
			debug {
				writeln(SAR);
			}	
			
			if (SAR.a == new NullAction() || t == traj.length - 1) {
				continue; // don't care about the terminal state
			}
			
			int slot = (cast(WaterlooState)traj[t+1].s).previousActionTime;
			
			if (SAR.s ! is null && SAR.a ! is null) {
				actionTimes[SAR.a][slot] ++;
				continue;
			}	
			
			WaterlooState ws = cast(WaterlooState)SAR.s;
			
			
			if (! ws.holdingBall) {
				switch (ws.position) {
					case 0:
						// hand on the table, haven't grabbed the ball yet, action is grab the ball
						if (ws.ballType == 1 || ws.ballType == 3)
							actionTimes[new WaterlooGrabHardAction()][slot] ++;
						else 
							actionTimes[new WaterlooGrabSoftAction()][slot] ++;
						break;
					default:
						break;
				}	
			
			} else {
				
				switch (ws.position) {
					case 2:
					case 3:
						// hand in one of the boxes, action is to drop ball
						if (ws.ballType == 1 || ws.ballType == 3)
							actionTimes[new WaterlooReleaseHardAction()][slot] ++;
						else 
							actionTimes[new WaterlooReleaseSoftAction()][slot] ++;
						break;
					default:
						break;
					
				}
			}
			
		}
	}
	
	foreach(a; allActions) {
		double sum = 0;
		foreach(c; actionTimes[a]) {
			sum += c;
		}
		
		actionTimes[a][] /= sum;
	}	
	
	debug {
		writeln(actionTimes);
	}
	
	
	
	// now update trans using the time data, at this point trans will have all probability mass in a single outcome state timeslot (0)
	// afterwards this mass will be distributed according to actionTimes
	
	foreach (s, asp; trans) {
		if (model.is_terminal(s))
			continue;
			
		foreach(a, s_prime_pr; asp) {
			double[State] new_s_prime_pr;
			
			foreach(s_prime, pr; s_prime_pr) {
				// distribute pr to new states that are identical to s_prime except for the previousActionTime value
				auto pr_copy = pr;
				WaterlooState ws_s_prime = cast(WaterlooState) s_prime;
				
				foreach (timeslot, timeslot_pr; actionTimes[a]) {
					new_s_prime_pr[new WaterlooState(ws_s_prime.position, ws_s_prime.ballType, cast(int)timeslot, ws_s_prime.holdingBall)] = pr_copy * timeslot_pr;
					
				}	
				
				
			}
			asp[a] = new_s_prime_pr;
		}
		
	}
	
	debug {
		writeln(trans);
	}	
	
}