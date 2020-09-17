import toymdpmirl;
import mdp;
import std.stdio;
import irl;
import irlgraveyard;
import std.random;
import std.algorithm;
import std.math;
import std.range;
import std.traits;
import std.numeric;
import std.file;
import std.string;
import std.format;
import std.datetime;
import std.conv;
import uai16;

import std.bitmanip;

int main(char[][] args) {

	bool useUniqueFeatures = false;
	
	int experiment_num = 0;
	
	int algorithm = 0;
	
	if (args.length > 1) {
		formattedRead(args[1], "%d", &experiment_num);
		
		if (args.length >= 3) {
			if (args[2] == "w") {
				useUniqueFeatures = true;
			}
			
			if (args.length >= 4) {
				formattedRead(args[3], "%d", &algorithm);
				
			}
		} 
		
	}
	
	ToyModel model = new ToyModel([new ToyState([0, 3]), new ToyState([1, 3])], 3, 4, [ [1, 1] ]);

	LinearReward reward = new ToyRewardSimple(model);
	
	double [3] reward_weights = [.1, 10, -10];
	
	reward.setParams(reward_weights);
	
	model.setReward(reward);
	
	LinearReward opt_reward = new ToyRewardSimple(model);
	opt_reward.setParams(reward_weights);

	double [1] transition_weights = [0.9];
	
	model.setT(model.createTransitionFunction(transition_weights, &otherActionsErrorModel));
	
	model.setGamma(0.95);
	
	ValueIteration vi = new ValueIteration(int.max, true);
//	ValueIteration vi = new ValueIteration();
	
	double[State] V = vi.solve(model, .01);

	Agent opt_policy = vi.createPolicy(model, V);
	
	
	double[State] initial;
	foreach (State s ; model.S()) {
//		if (!model.is_terminal(s))
			initial[s] = 1.0;
//		else
//			initial[s] = 0.0;	
		
	}
//	initial[new ToyState([2, 0])] = 1.0;
	Distr!State.normalize(initial);


	if (experiment_num == -1) {
		// generate the feature expectations
		
		if (useUniqueFeatures) {
			reward = new UniqueFeatureReward(model);
		    double [] temp_params;
		    temp_params.length = reward.dim();
		    temp_params[] = 0;
		    reward.setParams(temp_params);
		    model.setReward(reward);
		}
		
		all_feature_expectations(model, initial, algorithm);
		
		return 0;
		
	}
	
	writeln(experiment_num, ", ", useUniqueFeatures);	
	writeln("Value:");
	foreach (State s ; model.S()) {
		writeln(s, " ", V[s]);
		
	}
	
	writeln();
	writeln("Policy:");
	foreach (s ; model.S()) {
		if (model.is_terminal(s)) {
			writeln(s, " END");
		} else {
			writeln(s, " ", opt_policy.actions(s));
		}	
	}
	
	
	// generate sample trajectories
	
	// Exact IRL
	
//	Need to precalculate the state visitation frequencies or load them from a file
	
	double [][] feature_expectations;
	if (!useUniqueFeatures) {
		auto chars = readText("feature_expectations");
		
		
		foreach(line; splitLines(chars)) {
			double a;
			double b;
			double c;
			formattedRead(line, "[%s, %s, %s]", &a, &b, &c);
			feature_expectations ~= [a, b, c];
		}
				
		debug {
			writeln("Read in: ", feature_expectations.length, " entries");
		}
	} else {
	
	
		auto chars = readText("feature_expectations2");	
		
		foreach(line; splitLines(chars)) {
			double [] fe;
			fe.length = 38;
	
			formattedRead(line, "[%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s]", &fe[0], &fe[1], &fe[2], &fe[3], &fe[4], &fe[5], &fe[6], &fe[7], &fe[8], &fe[9], &fe[10], &fe[11], &fe[12], &fe[13], &fe[14], &fe[15], &fe[16], &fe[17], &fe[18], &fe[19], &fe[20], &fe[21], &fe[22], &fe[23], &fe[24], &fe[25], &fe[26], &fe[27], &fe[28], &fe[29], &fe[30], &fe[31], &fe[32], &fe[33], &fe[34], &fe[35], &fe[36], &fe[37]);
			feature_expectations ~= fe;
		}
				
		debug {
			writeln("Read in: ", feature_expectations.length, " entries");
		}
		
		
		// Exact IRL uninformative features
		
		reward = new UniqueFeatureReward(model);
	    double [] temp_params;
	    temp_params.length = reward.dim();
	    temp_params[] = 0;
	    reward.setParams(temp_params);
	    model.setReward(reward);
    }
	
	/*
	foreach(iter;0..1) {
		
		foreach(sample_length; [5]) {
	
			auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  

			foreach(num_samples; [100]) {
				sar [][] trajs;
				
				trajs.length = sample_length;
				foreach (q; 0..num_samples) { 
					sar [] traj = simulate(model, opt_policy, initial, sample_length);
					
					foreach(i, sar; traj) {
						sar.p = 1.0/num_samples;
						trajs[i] ~= sar;
						
					}
					foreach (i; traj.length .. sample_length) {
						auto temp = traj[$ - 1];
						temp.p = 1.0/num_samples;
						trajs[i] ~= temp;
						
					}
				}
			
				// solve IRL exactly
	
				Agent policy = new RandomAgent(model.A(null));
				double [] foundWeights;
							
				double val;
							
				auto lastWeights = new double[reward.dim()];
				for (int i = 0; i < lastWeights.length; i ++)
					lastWeights[i] = uniform(0, .1);	
							
				MaxEntIrlExact irl = new MaxEntIrlExact(200,vi, 0, .1, .1, .1);
				irl.setFeatureExpectations(feature_expectations);
				
				policy = irl.solve(model, initial, trajs, lastWeights, val, foundWeights);
//				return 0;
				reward.setParams(foundWeights);
	
				model.setReward(reward);
				
				V = vi.solve(model, .01);
				writeln("Found Weights: ", foundWeights);
				writeln("Value:");
				foreach (State s ; model.S()) {
					writeln(s, " ", V[s]);
					
				}
//				policy = vi.createPolicy(model, V);
	
				foreach (s; model.S()) {
					if (! model.is_terminal(s))
						writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", policy.actions(s));
			    	else 
						writeln(s, " - TERMINATE");    	
			    }	
			    auto found_values = analyzeIRLResults([model], [policy], [opt_reward], [initial], sample_length);
			    
				found_values[] -= opt_policy_value[];
					
				write(iter, ", ", num_samples, ", ");
				foreach (fpe; found_values)
					write(-fpe, ", ");
				writeln();	
			}


		}
	}
*/
	
	// Exact EM, limited data, actions visible
	if (experiment_num == 0 ){
			double [][] feature_expectations_true;
		
		double [][] state_visitation_frequency;

		if (!useUniqueFeatures) {
			feature_expectations_true = feature_expectations;
			
		
			auto chars = readText("feature_expectations2");	
			
			foreach(line; splitLines(chars)) {
				double [] fe;
				fe.length = 38;
		
				formattedRead(line, "[%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s]", &fe[0], &fe[1], &fe[2], &fe[3], &fe[4], &fe[5], &fe[6], &fe[7], &fe[8], &fe[9], &fe[10], &fe[11], &fe[12], &fe[13], &fe[14], &fe[15], &fe[16], &fe[17], &fe[18], &fe[19], &fe[20], &fe[21], &fe[22], &fe[23], &fe[24], &fe[25], &fe[26], &fe[27], &fe[28], &fe[29], &fe[30], &fe[31], &fe[32], &fe[33], &fe[34], &fe[35], &fe[36], &fe[37]);
				state_visitation_frequency ~= fe;
			}
			chars = null;			
			
		} else {
			auto chars = readText("feature_expectations");
			
			
			foreach(line; splitLines(chars)) {
				double a;
				double b;
				double c;
				formattedRead(line, "[%s, %s, %s]", &a, &b, &c);
				feature_expectations_true ~= [a, b, c];
			}
			chars = null;
			
			state_visitation_frequency = feature_expectations;
		}
		
    foreach(iter;0..10) {
		
		foreach(sample_length; [5]) {
	
			auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  

			foreach(num_samples; [0, 1, 2, 3, 4, 5, 10, 50, 100, 250, 500]) {
				foreach (repeat; 0..30) {
					sar [][] trajs;
					trajs.length = sample_length;
	
	//				trajs.length = sample_length;
					foreach (q; 0..num_samples) { 
						sar [] traj = simulate(model, opt_policy, initial, sample_length);
	
						foreach(i, sar; traj) {
							sar.p = 1.0/num_samples;
							trajs[i] ~= sar;
							
						}
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							trajs[i] ~= temp;
							
						}		
					}
					// solve IRL exactly
					debug {
						writeln("Trajs");
						writeln(trajs);
					}
					
					double max_entropy = -1;
					double [] found_weights_max_entropy; 
					double [] last_weights_max_entropy;
					Agent max_entropy_best_policy;
					
					size_t[] savedY;
					double [] saved_pr_y;
					double [] saved_state_v_y;
				
		
					Agent policy = new RandomAgent(model.A(null));
					double [] foundWeights;

					double val;
								
					auto lastWeights = new double[reward.dim()];
					for (int i = 0; i < lastWeights.length; i ++)
						lastWeights[i] = uniform(-.1, .1);	
								
					MaxEntIrlExactEM irl = new MaxEntIrlExactEM(200,vi, 0, .00005, .1, .1);
					irl.setFeatureExpectations(feature_expectations);
					
					if (saved_pr_y != null) {
						irl.setPrY(saved_pr_y, savedY, saved_state_v_y);
					}
					
					policy = irl.solve2(model, initial, trajs, sample_length, lastWeights, state_visitation_frequency, val, foundWeights, algorithm);

					if (saved_pr_y == null) {
						irl.getPrY(saved_pr_y, savedY, saved_state_v_y);
					}
					
					auto entropy = getEntropy(getPolicyDistribution(model, foundWeights, feature_expectations));
					
					if (entropy > max_entropy) {
						max_entropy = entropy;
						found_weights_max_entropy = foundWeights.dup;
						last_weights_max_entropy = lastWeights.dup;
						max_entropy_best_policy = policy;
					}
					reward.setParams(found_weights_max_entropy);
					model.setReward(reward);
					
					debug {
						
						V = vi.solve(model, .01);
					
						writeln("Value:");
						foreach (State s ; model.S()) {
							writeln(s, " ", V[s]);
							
						}
		
						foreach (s; model.S()) {
							if (! model.is_terminal(s))
								writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
					    	else 
								writeln(s, " - TERMINATE");    	
					    }
					
					}
					
				    auto found_values = analyzeIRLResults([model], [max_entropy_best_policy], [opt_reward], [initial], sample_length);
				    
					found_values[] -= opt_policy_value[];
						
					write(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ");
					foreach (fpe; found_values)
						write(-fpe, ", ");
					writeln();
					
					
					
					
					MaxEntIrlExact legacy_irl = new MaxEntIrlExact(200,vi, 0, .00005, .1, .1);
					legacy_irl.setFeatureExpectations(feature_expectations);
					
					double [] foundWeights_legacy;
					
					Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
										
				    auto legacy_found_values = analyzeIRLResults([model], [policy_legacy], [opt_reward], [initial], sample_length);
					legacy_found_values[] -= opt_policy_value[];
						
					write(iter, "-", "Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, ", ");
					foreach (fpe; legacy_found_values)
						write(-fpe, ", ");
					writeln();										
					
					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
				}			
			}						
		}
	}	
    
	}
			

	size_t sample(double[] distr) { // modified version from mdp.d, for non-associative arrays
			
			auto r = uniform(0.0, .999999);
			
			auto total = 0.0;
			
			// binary search through the cdf
			size_t i_min = 0;
			size_t i_max = distr.length - 1;
			
			while (i_min < i_max) {
				size_t i_mid = (i_max - i_min) / 2 + i_min;
				
				if (distr[i_mid] < r) {
					i_min = i_mid + 1;
				} else {
					i_max = i_mid;
				}
				
			}
			return i_min;				
	}
	
	if (experiment_num == 1) {
		// compare LME to the previous algorithm when the expert is using a distribution of policies instead of just one
		// use the maxent distribution with the true weights as the expert's

		double [][] feature_expectations_true;
		
		double [][] state_visitation_frequency;

		if (!useUniqueFeatures) {
			feature_expectations_true = feature_expectations;
			
		
			auto chars = readText("feature_expectations2");	
			
			foreach(line; splitLines(chars)) {
				double [] fe;
				fe.length = 38;
		
				formattedRead(line, "[%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s]", &fe[0], &fe[1], &fe[2], &fe[3], &fe[4], &fe[5], &fe[6], &fe[7], &fe[8], &fe[9], &fe[10], &fe[11], &fe[12], &fe[13], &fe[14], &fe[15], &fe[16], &fe[17], &fe[18], &fe[19], &fe[20], &fe[21], &fe[22], &fe[23], &fe[24], &fe[25], &fe[26], &fe[27], &fe[28], &fe[29], &fe[30], &fe[31], &fe[32], &fe[33], &fe[34], &fe[35], &fe[36], &fe[37]);
				state_visitation_frequency ~= fe;
			}
			chars = null;			
			
		} else {
			auto chars = readText("feature_expectations");
			
			
			foreach(line; splitLines(chars)) {
				double a;
				double b;
				double c;
				formattedRead(line, "[%s, %s, %s]", &a, &b, &c);
				feature_expectations_true ~= [a, b, c];
			}
			chars = null;
			
			state_visitation_frequency = feature_expectations;
		}
	
		double [] true_policy_distr = getPolicyDistribution(model, reward_weights, feature_expectations_true);

		double [] policy_cdf = new double[true_policy_distr.length];
		
		double totalcdf = 0;
		foreach (i, pr; true_policy_distr) {
			totalcdf += pr;
			policy_cdf[i] = totalcdf;
			
		}

			
	    foreach(iter;0..10) {
			
			foreach(sample_length; [1]) {
		
				auto opt_policy_distr_value = analyzeIRLDistrResults(model, true_policy_distr, opt_reward, initial, sample_length, state_visitation_frequency);  
				foreach(num_samples; [5, 10, 15, 20, 50, 250, 1000, 5000]) {
					foreach (repeat; 0..1) {
						sar [][] trajs;
//						trajs.length = sample_length;
		
						foreach (q; 0..num_samples) { 
							sar [] traj = simulate(model, get_policy_num(model, sample(policy_cdf)), initial, sample_length);
							foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
								auto temp = traj[$ - 1];
								temp.p = 1.0;///num_samples;
								traj ~= temp;
								
							}		
							foreach(i, ref sar; traj) {
								sar.p = 1.0;///num_samples;
							}
							trajs ~= traj;
						}
						double max_entropy = -1;
						double [] found_weights_max_entropy; 
						double [] last_weights_max_entropy;
						Agent max_entropy_best_policy;
						
						size_t[] savedY;
						double [] saved_pr_y;
						double [] saved_state_v_y;
					
			
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
									
						double val;
//foreach (repeat2; 0 .. 10) {
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(-5.0, 5.0);	
						
						
						MaxEntIrlExactEM irl = new MaxEntIrlExactEM(200,vi, 0, 0.00005, .1, .1);
						irl.setFeatureExpectations(feature_expectations);
						if (saved_pr_y != null) {
							irl.setPrY(saved_pr_y, savedY, saved_state_v_y);
						}
						
						policy = irl.solve2(model, initial, trajs, sample_length, lastWeights, state_visitation_frequency, val, foundWeights, algorithm);
	
						if (saved_pr_y == null) {
							irl.getPrY(saved_pr_y, savedY, saved_state_v_y);
						}
						auto new_distr = getPolicyDistribution(model, foundWeights, feature_expectations);
						auto entropy = getEntropy(new_distr);
						
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
						auto kld = KLD(true_policy_distr,  new_distr);
						writeln(iter, "-", repeat, " ", lastWeights, ", ", foundWeights, ", ", num_samples, ", ",entropy, ", ", kld);
//}
						reward.setParams(found_weights_max_entropy);
						model.setReward(reward);
						
						debug {
							
							V = vi.solve(model, .01);
						
							writeln("Value:");
							foreach (State s ; model.S()) {
								writeln(s, " ", V[s]);
								
							}
			
							foreach (s; model.S()) {
								if (! model.is_terminal(s))
									writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
						    	else 
									writeln(s, " - TERMINATE");    	
						    }
						
						}
						
					    auto found_values = analyzeIRLDistrResults(model, getPolicyDistribution(model, found_weights_max_entropy, feature_expectations), opt_reward, initial, sample_length, state_visitation_frequency);
					    
						found_values -= opt_policy_distr_value;
							
						writeln(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ", -found_values);
						
						
						
						
						MaxEntIrlExact legacy_irl = new MaxEntIrlExact(200,vi, 0, .00005, .1, .1);
						legacy_irl.setFeatureExpectations(feature_expectations);
						
						double [] foundWeights_legacy;
						
						Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
						
						reward.setParams(foundWeights_legacy);
						model.setReward(reward);
						
											
					    auto legacy_found_values = analyzeIRLDistrResults(model, getPolicyDistribution(model, foundWeights_legacy, feature_expectations), opt_reward, initial, sample_length, state_visitation_frequency);
						legacy_found_values -= opt_policy_distr_value;
							
						writeln(iter, "-", "Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, ", ", -legacy_found_values);
						
						compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy, true_policy_distr);
					}			
					
				}
				
			}		
		}
	
	}

	// run lme with the true policy distribution as input
	if (experiment_num == 2) {
		// compare LME to the previous algorithm when the expert is using a distribution of policies instead of just one
		// use the maxent distribution with the true weights as the expert's

		double [][] feature_expectations_true;
		
		double [][] state_visitation_frequency;

		if (!useUniqueFeatures) {
			feature_expectations_true = feature_expectations;
			
		
			auto chars = readText("feature_expectations2");	
			
			foreach(line; splitLines(chars)) {
				double [] fe;
				fe.length = 38;
		
				formattedRead(line, "[%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s]", &fe[0], &fe[1], &fe[2], &fe[3], &fe[4], &fe[5], &fe[6], &fe[7], &fe[8], &fe[9], &fe[10], &fe[11], &fe[12], &fe[13], &fe[14], &fe[15], &fe[16], &fe[17], &fe[18], &fe[19], &fe[20], &fe[21], &fe[22], &fe[23], &fe[24], &fe[25], &fe[26], &fe[27], &fe[28], &fe[29], &fe[30], &fe[31], &fe[32], &fe[33], &fe[34], &fe[35], &fe[36], &fe[37]);
				state_visitation_frequency ~= fe;
			}
			
			chars = null;			
			
		} else {
			auto chars = readText("feature_expectations");
			
			
			foreach(line; splitLines(chars)) {
				double a;
				double b;
				double c;
				formattedRead(line, "[%s, %s, %s]", &a, &b, &c);
				feature_expectations_true ~= [a, b, c];
			}
			
			chars = null;
			
			state_visitation_frequency = feature_expectations;
		}
	
		double [] true_policy_distr = getPolicyDistribution(model, reward_weights, feature_expectations_true);

		double [] policy_cdf = new double[true_policy_distr.length];
		
		double totalcdf = 0;
		foreach (i, pr; true_policy_distr) {
			totalcdf += pr;
			policy_cdf[i] = totalcdf;
			
		}
		
			
	    foreach(iter;0..1) {
			
			foreach(sample_length; [5]) {
		
				auto opt_policy_distr_value = analyzeIRLDistrResults(model, true_policy_distr, opt_reward, initial, sample_length, state_visitation_frequency);  
//				foreach(num_samples; [10, 50, 100, 500, 1000]) {
				foreach(num_samples; [ 1	]) {
					sar [][] trajs;
					trajs.length = sample_length;
	
					foreach (q; 0..num_samples) { 
						sar [] traj = simulate(model, get_policy_num(model, sample(policy_cdf)), initial, sample_length);
						foreach(i, sar; traj) {
							sar.p = 1.0/num_samples;
							trajs[i] ~= sar;
							
						}
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							trajs[i] ~= temp;
							
						}		
					}
				
					double max_entropy = -1;
					double [] found_weights_max_entropy; 
					double [] last_weights_max_entropy;
					Agent max_entropy_best_policy;
					
					size_t[] savedY;
					double [] saved_pr_y;
					savedY.length = true_policy_distr.length;
					saved_pr_y.length = true_policy_distr.length;
					double [] total_state_visit = new double[true_policy_distr.length];
					
					double norm = 0;
					foreach (i, pr; true_policy_distr) {
						total_state_visit[i] = reduce!("a + b")(0.0, state_visitation_frequency[i]);
						saved_pr_y[i] = 1 + pr * total_state_visit[i];
						norm += saved_pr_y[i];
						savedY[i] = i;
					}
					
					saved_pr_y[] /= norm;
					
					
					
					foreach (repeat; 0..1) {
			
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
									
						double val;
									
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(-.1, .1);	
						
						
						MaxEntIrlExactEM irl = new MaxEntIrlExactEM(200,vi, 0, .00005, .1, .1);
						irl.setFeatureExpectations(feature_expectations);
						irl.setPrY(saved_pr_y, savedY, total_state_visit);
						
						policy = irl.solve2(model, initial, trajs, sample_length, lastWeights, state_visitation_frequency, val, foundWeights, algorithm);
	
						auto entropy = getEntropy(getPolicyDistribution(model, foundWeights, feature_expectations));
						
						
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
					}			
					reward.setParams(found_weights_max_entropy);
					model.setReward(reward);
					
					debug {
						
						V = vi.solve(model, .01);
					
						writeln("Value:");
						foreach (State s ; model.S()) {
							writeln(s, " ", V[s]);
							
						}
		
						foreach (s; model.S()) {
							if (! model.is_terminal(s))
								writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
					    	else 
								writeln(s, " - TERMINATE");    	
					    }
					
					}
					
				    auto found_values = analyzeIRLDistrResults(model, getPolicyDistribution(model, found_weights_max_entropy, feature_expectations), opt_reward, initial, sample_length, state_visitation_frequency);
				    
					found_values -= opt_policy_distr_value;
						
					writeln(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ", -found_values);
					
					
					
					
					MaxEntIrlExact legacy_irl = new MaxEntIrlExact(200,vi, 0, .00005, .1, .1);
					legacy_irl.setFeatureExpectations(feature_expectations);
					
					double [] foundWeights_legacy;
					double val;
					
					Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
										
				    auto legacy_found_values = analyzeIRLDistrResults(model, getPolicyDistribution(model, foundWeights_legacy, feature_expectations), opt_reward, initial, sample_length, state_visitation_frequency);
					legacy_found_values -= opt_policy_distr_value;
						
					writeln(iter, "-", "Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, ", ", -legacy_found_values);
					
					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy, true_policy_distr);
					
				}
				
			}		
		}
	
	}	
	
	// run legacy with the true policy distribution as input
	if (experiment_num == 3) {
		// compare LME to the previous algorithm when the expert is using a distribution of policies instead of just one
		// use the maxent distribution with the true weights as the expert's

		double [][] feature_expectations_true;
		
		double [][] state_visitation_frequency;

		if (!useUniqueFeatures) {
			feature_expectations_true = feature_expectations;
			
		
			auto chars = readText("feature_expectations2");	
			
			foreach(line; splitLines(chars)) {
				double [] fe;
				fe.length = 38;
		
				formattedRead(line, "[%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s]", &fe[0], &fe[1], &fe[2], &fe[3], &fe[4], &fe[5], &fe[6], &fe[7], &fe[8], &fe[9], &fe[10], &fe[11], &fe[12], &fe[13], &fe[14], &fe[15], &fe[16], &fe[17], &fe[18], &fe[19], &fe[20], &fe[21], &fe[22], &fe[23], &fe[24], &fe[25], &fe[26], &fe[27], &fe[28], &fe[29], &fe[30], &fe[31], &fe[32], &fe[33], &fe[34], &fe[35], &fe[36], &fe[37]);
				state_visitation_frequency ~= fe;
			}
			
			chars = null;			
			
		} else {
			auto chars = readText("feature_expectations");
			
			
			foreach(line; splitLines(chars)) {
				double a;
				double b;
				double c;
				formattedRead(line, "[%s, %s, %s]", &a, &b, &c);
				feature_expectations_true ~= [a, b, c];
			}
			
			chars = null;
			
			state_visitation_frequency = feature_expectations;
		}
	
		double [] true_policy_distr = getPolicyDistribution(model, reward_weights, feature_expectations_true);

			
	    foreach(iter;0..1) {
			
			foreach(sample_length; [5]) {
		
				auto opt_policy_distr_value = analyzeIRLDistrResults(model, true_policy_distr, opt_reward, initial, sample_length, state_visitation_frequency);  
//				foreach(num_samples; [10, 50, 100, 500, 1000]) {
				foreach(num_samples; [ 1 ]) {

					auto lastWeights = new double[reward.dim()];
					for (int i = 0; i < lastWeights.length; i ++)
						lastWeights[i] = uniform(-.1, .1);	
						
					MaxEntIrlExact legacy_irl = new MaxEntIrlExact(200,vi, 0, .00005, .1, .1);
					legacy_irl.setFeatureExpectations(feature_expectations);
					
					double [] foundWeights_legacy;
					double val;
					
					Agent policy_legacy = legacy_irl.solve_exact(model, initial, true_policy_distr, feature_expectations, lastWeights, val, foundWeights_legacy, algorithm);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
										
				    auto legacy_found_values = analyzeIRLDistrResults(model, getPolicyDistribution(model, foundWeights_legacy, feature_expectations), opt_reward, initial, sample_length, state_visitation_frequency);
					legacy_found_values -= opt_policy_distr_value;
						
					writeln(iter, "-", "Legacy, ", lastWeights, ", ", foundWeights_legacy, ", ", num_samples, ", ", -legacy_found_values);
					
					compare_policy_distrs(model, feature_expectations, foundWeights_legacy, foundWeights_legacy, true_policy_distr);
					
				}
				
			}		
		}
	
	}		
	
	// Exact EM, limited data, actions visible, doshi's algorithm
	if (experiment_num == 4 ){
		double [][] feature_expectations_true;
		
		double [][] state_visitation_frequency;

		if (!useUniqueFeatures) {
			feature_expectations_true = feature_expectations;
			
		
			auto chars = readText("feature_expectations2");	
			
			foreach(line; splitLines(chars)) {
				double [] fe;
				fe.length = 38;
		
				formattedRead(line, "[%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s]", &fe[0], &fe[1], &fe[2], &fe[3], &fe[4], &fe[5], &fe[6], &fe[7], &fe[8], &fe[9], &fe[10], &fe[11], &fe[12], &fe[13], &fe[14], &fe[15], &fe[16], &fe[17], &fe[18], &fe[19], &fe[20], &fe[21], &fe[22], &fe[23], &fe[24], &fe[25], &fe[26], &fe[27], &fe[28], &fe[29], &fe[30], &fe[31], &fe[32], &fe[33], &fe[34], &fe[35], &fe[36], &fe[37]);
				state_visitation_frequency ~= fe;
			}
			chars = null;			
			
		} else {
			auto chars = readText("feature_expectations");
			
			
			foreach(line; splitLines(chars)) {
				double a;
				double b;
				double c;
				formattedRead(line, "[%s, %s, %s]", &a, &b, &c);
				feature_expectations_true ~= [a, b, c];
			}
			chars = null;
			
			state_visitation_frequency = feature_expectations;
		}
		
	    foreach(iter;0..30) {
			
			foreach(sample_length; [5]) {
		
				auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  
	
				foreach(num_samples; [0, 1, 2, 3, 4, 5, 10, 50/*, 100, 250, 500*/]) {
						
					sar [][] trajs;
//						trajs.length = sample_length;
	
					foreach (q; 0..num_samples) { 
						sar [] traj = simulate(model, opt_policy, initial, sample_length);
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							traj ~= temp;
							
						}		
						foreach(i, ref sar; traj) {
							sar.p = 1.0/num_samples;
						}
						trajs ~= traj;
					}
					
					// solve IRL exactly
					debug {
						writeln("Trajs");
						writeln(trajs);
					}
					
					double max_entropy = -1;
					double [] found_weights_max_entropy; 
					double [] last_weights_max_entropy;
					Agent max_entropy_best_policy;
					double val;
					
					BitArray [][] saved_pi_feature_map;
					BitArray [] saved_inv_Y_feature_map;
					double [][] saved_tilde_mu_Y;
					size_t [][] saved_pis_in_Y;
	
					foreach (repeat; 0..10) {
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
	
									
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(-1.0, 1.0);	
									
						MaxEntIrlExactEMDoshi irl = new MaxEntIrlExactEMDoshi(200,vi, 0, .00005, .1, .1);
						irl.setFeatureExpectations(feature_expectations);

						irl.set_mu_tilde(saved_pi_feature_map, saved_inv_Y_feature_map, saved_tilde_mu_Y, saved_pis_in_Y);
						
						policy = irl.solve2(model, initial, trajs, sample_length, lastWeights, state_visitation_frequency, val, foundWeights, algorithm);
						
						auto entropy = getEntropy(getPolicyDistribution(model, foundWeights, feature_expectations));

						irl.get_mu_tilde(saved_pi_feature_map, saved_inv_Y_feature_map, saved_tilde_mu_Y, saved_pis_in_Y);
						
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
						reward.setParams(found_weights_max_entropy);
						model.setReward(reward);
						
						debug {
							
							V = vi.solve(model, .01);
						
							writeln("Value:");
							foreach (State s ; model.S()) {
								writeln(s, " ", V[s]);
								
							}
			
							foreach (s; model.S()) {
								if (! model.is_terminal(s))
									writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
						    	else 
									writeln(s, " - TERMINATE");    	
						    }
						
						}
					}
						
				    auto found_values = analyzeIRLResults([model], [max_entropy_best_policy], [opt_reward], [initial], sample_length);
				    
					found_values[] -= opt_policy_value[];
						
					write(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ");
					foreach (fpe; found_values)
						write(-fpe, ", ");
					writeln();
						
					
					
					MaxEntIrlExact legacy_irl = new MaxEntIrlExact(200,vi, 0, .00005, .1, .1);
					legacy_irl.setFeatureExpectations(feature_expectations);
					
					double [] foundWeights_legacy;
					
					Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
										
				    auto legacy_found_values = analyzeIRLResults([model], [policy_legacy], [opt_reward], [initial], sample_length);
					legacy_found_values[] -= opt_policy_value[];
						
					write(iter, "-", "Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, ", ");
					foreach (fpe; legacy_found_values)
						write(-fpe, ", ");
					writeln();										
					
					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
								
				}						
			}
		}	
	    
	}	
	
	
	
		// Exact EM, limited data, actions visible, doshi's algorithm
	if (experiment_num == 5 ){
		double [][] feature_expectations_true;
		
		double [][] state_visitation_frequency;

		if (!useUniqueFeatures) {
			feature_expectations_true = feature_expectations;
			
		
			auto chars = readText("feature_expectations2");	
			
			foreach(line; splitLines(chars)) {
				double [] fe;
				fe.length = 38;
		
				formattedRead(line, "[%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s]", &fe[0], &fe[1], &fe[2], &fe[3], &fe[4], &fe[5], &fe[6], &fe[7], &fe[8], &fe[9], &fe[10], &fe[11], &fe[12], &fe[13], &fe[14], &fe[15], &fe[16], &fe[17], &fe[18], &fe[19], &fe[20], &fe[21], &fe[22], &fe[23], &fe[24], &fe[25], &fe[26], &fe[27], &fe[28], &fe[29], &fe[30], &fe[31], &fe[32], &fe[33], &fe[34], &fe[35], &fe[36], &fe[37]);
				state_visitation_frequency ~= fe;
			}
			chars = null;			
			
		} else {
			auto chars = readText("feature_expectations");
			
			
			foreach(line; splitLines(chars)) {
				double a;
				double b;
				double c;
				formattedRead(line, "[%s, %s, %s]", &a, &b, &c);
				feature_expectations_true ~= [a, b, c];
			}
			chars = null;
			
			state_visitation_frequency = feature_expectations;
		}
		
	    foreach(iter;0..30) {
			
			foreach(sample_length; [5]) {
		
				auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  
	
				foreach(num_samples; [0, 1, 2, 3, 4, 5, 10, 50/*, 100, 250, 500*/]) {
						
					sar [][] trajs;
//						trajs.length = sample_length;
	
					foreach (q; 0..num_samples) { 
						sar [] traj = simulate(model, opt_policy, initial, sample_length);
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							traj ~= temp;
							
						}		
						foreach(i, ref sar; traj) {
							sar.p = 1.0/num_samples;
						}
						trajs ~= traj;
					}
					
					// solve IRL exactly
					debug {
						writeln("Trajs");
						writeln(trajs);
					}
					
					double max_entropy = -1;
					double [] found_weights_max_entropy; 
					double [] last_weights_max_entropy;
					Agent max_entropy_best_policy;
					double val;
					
					BitArray [][] saved_pi_feature_map;
					BitArray [] saved_inv_Y_feature_map;
					double [][] saved_tilde_mu_Y;
					size_t [][] saved_pis_in_Y;
	
					foreach (repeat; 0..10) {
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
	
									
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(-1.0, 1.0);	
									
						MaxEntIrlExactEMDoshiFullZMap irl = new MaxEntIrlExactEMDoshiFullZMap(200,vi, 0, .00005, .1, .1);
						irl.setFeatureExpectations(feature_expectations);

						irl.set_mu_tilde(saved_pi_feature_map, saved_inv_Y_feature_map, saved_tilde_mu_Y, saved_pis_in_Y);
						
						policy = irl.solve2(model, initial, trajs, sample_length, lastWeights, state_visitation_frequency, val, foundWeights, algorithm);
						
						auto entropy = getEntropy(getPolicyDistribution(model, foundWeights, feature_expectations));

						irl.get_mu_tilde(saved_pi_feature_map, saved_inv_Y_feature_map, saved_tilde_mu_Y, saved_pis_in_Y);
						
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
						reward.setParams(found_weights_max_entropy);
						model.setReward(reward);
						
						debug {
							
							V = vi.solve(model, .01);
						
							writeln("Value:");
							foreach (State s ; model.S()) {
								writeln(s, " ", V[s]);
								
							}
			
							foreach (s; model.S()) {
								if (! model.is_terminal(s))
									writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
						    	else 
									writeln(s, " - TERMINATE");    	
						    }
						
						}
					}
						
				    auto found_values = analyzeIRLResults([model], [max_entropy_best_policy], [opt_reward], [initial], sample_length);
				    
					found_values[] -= opt_policy_value[];
						
					write(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ");
					foreach (fpe; found_values)
						write(-fpe, ", ");
					writeln();
						
					
					
					MaxEntIrlExact legacy_irl = new MaxEntIrlExact(200,vi, 0, .00005, .1, .1);
					legacy_irl.setFeatureExpectations(feature_expectations);
					
					double [] foundWeights_legacy;
					
					Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
										
				    auto legacy_found_values = analyzeIRLResults([model], [policy_legacy], [opt_reward], [initial], sample_length);
					legacy_found_values[] -= opt_policy_value[];
						
					write(iter, "-", "Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, ", ");
					foreach (fpe; legacy_found_values)
						write(-fpe, ", ");
					writeln();										
					
					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
								
				}						
			}
		}	
	    
	}	
	
	
	// Exact EM, limited data, actions visible, Ziebart's algorithm
	if (experiment_num == 6 ){

		foreach(sample_length; [5]) {
			foreach(num_samples; [2, 3, 4, 5, 10, 50/*, 100, 250, 500*/]) {
				double [] LME_errors;
				double [] Legacy_errors;
				double [] mIRL_errors;
			
				foreach(iter;0..40) {
	    		
					auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  
	
						
					sar [][] trajs;
//						trajs.length = sample_length;
	
					foreach (q; 0..num_samples) { 
						sar [] traj = simulate(model, opt_policy, initial, sample_length);
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							traj ~= temp;
							
						}		
						foreach(i, ref sar; traj) {
							sar.p = 1.0/num_samples;
						}
						trajs ~= traj;
					}
					
					// solve IRL exactly

					
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
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
	
									
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(0, 1.0);	
									
						LatentMaxEntIrlZiebartExact irl = new LatentMaxEntIrlZiebartExact(200, vi, model.S(), 0, .0005, .1);

						irl.setYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);
						
						policy = irl.solve(model, initial, trajs, lastWeights, val, foundWeights);
						
						auto entropy = getEntropy(getTrajDistribution(model, foundWeights, trajs));

						irl.getYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);
						
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
						reward.setParams(found_weights_max_entropy);
						model.setReward(reward);
						
						debug {
							
							V = vi.solve(model, .01);
						
							writeln("Value:");
							foreach (State s ; model.S()) {
								writeln(s, " ", V[s]);
								
							}
			
							foreach (s; model.S()) {
								if (! model.is_terminal(s))
									writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
						    	else 
									writeln(s, " - TERMINATE");    	
						    }
						
						}
					}
						
				    auto found_values = analyzeIRLResults([model], [max_entropy_best_policy], [opt_reward], [initial], sample_length);
				    
					found_values[] -= opt_policy_value[];
						
//					write(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ");
					foreach (fpe; found_values)
						LME_errors ~= -fpe;
//						write(-fpe, ", ");
//					writeln();
						

					MaxEntIrlZiebartExact legacy_irl = new MaxEntIrlZiebartExact(200,vi, model.S(), 0, .0005, .1);
					
					double [] foundWeights_legacy;
					
					Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
										
				    auto legacy_found_values = analyzeIRLResults([model], [policy_legacy], [opt_reward], [initial], sample_length);
					legacy_found_values[] -= opt_policy_value[];
						
//					write(iter, "-Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, "-", sample_length, ", ");
					foreach (fpe; legacy_found_values)
						Legacy_errors ~= -fpe;
//						write(-fpe, ", ");
					
//					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
					
					

					MaxEntIrlPartialVisibility mirl_irl = new MaxEntIrlPartialVisibility(200, vi, 200, .0005, .1, 0.09, model.S(), sample_length);
					
					double [] foundWeights_mirl;
					
					Agent policy_mIRL = mirl_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_mirl);
					
					reward.setParams(foundWeights_mirl);
					model.setReward(reward);
					
										
				    auto mirl_found_values = analyzeIRLResults([model], [policy_mIRL], [opt_reward], [initial], sample_length);
					mirl_found_values[] -= opt_policy_value[];
						
//					writeln(iter, "-mIRL, ", last_weights_max_entropy, ", ", foundWeights_mirl, ", ", num_samples, "-", sample_length, ", ");
					foreach (fpe; mirl_found_values)
						mIRL_errors ~= -fpe;
//						write(-fpe, ", ");
					
//					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
					
					
					
				}
				writeln(num_samples, ", ", sample_length, ", LME, ", LME_errors);
				writeln(num_samples, ", ", sample_length, ", Legacy, ", Legacy_errors);
				writeln(num_samples, ", ", sample_length, ", mIRL, ", mIRL_errors);
									
			}
		}	
	    
	}	

	// Exact EM, limited data, goal states hidden, Ziebart's algorithm
	if (experiment_num == 7 ){
		
		State [] observableStates;
		
		foreach(s; model.S()) {
			ToyState ts = cast(ToyState)s;
			if (ts.getLocation()[1] < 3)
				observableStates ~= s;
			
		}

		foreach(sample_length; [5]) {
			foreach(num_samples; [2, 3, 4, 5, 10, 50/*, 100, 250, 500*/]) {
				double [] LME_errors;
				double [] Legacy_errors;
				double [] mIRL_errors;

				foreach(iter;0..40) {
	    		
					auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  
	
						
					sar [][] trajs;
//						trajs.length = sample_length;
	
					foreach (q; 0..num_samples) { 
						sar [] traj = simulate(model, opt_policy, initial, sample_length);
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							traj ~= temp;
							
						}		
						foreach(i, ref sar; traj) {
							bool found = false;
							foreach (os; observableStates) {
								if (os == sar.s) {
									found = true;
									break;
								}	
							}
							if (! found) {
								sar.s = null;
								sar.a = null;
							}	
							sar.p = 1.0/num_samples;
						}
						trajs ~= traj;
					}
					
					// solve IRL exactly
					debug {
//						writeln("Trajs");
//						writeln(trajs);
					}
					
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
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
	
									
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(0, 1.0);	
									
						LatentMaxEntIrlZiebartExact irl = new LatentMaxEntIrlZiebartExact(10, vi, observableStates, 0, .0005, .1);

						irl.setYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);

						policy = irl.solve(model, initial, trajs, lastWeights, val, foundWeights);
						
						auto entropy = val;

						irl.getYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);
						
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
						reward.setParams(found_weights_max_entropy);
						model.setReward(reward);
						
						debug {
							
							V = vi.solve(model, .01);
						
							writeln("Value:");
							foreach (State s ; model.S()) {
								writeln(s, " ", V[s]);
								
							}
			
							foreach (s; model.S()) {
								if (! model.is_terminal(s))
									writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
						    	else 
									writeln(s, " - TERMINATE");    	
						    }
						
						}
					}
						
				    auto found_values = analyzeIRLResults([model], [max_entropy_best_policy], [opt_reward], [initial], sample_length);
				    
					found_values[] -= opt_policy_value[];
						
//					write(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ");
					foreach (fpe; found_values)
						LME_errors ~= -fpe;
//						write(-fpe, ", ");
//					writeln();
					
					
	/*				last_weights_max_entropy = new double[reward.dim()];
					for (int i = 0; i < last_weights_max_entropy.length; i ++)
						last_weights_max_entropy[i] = uniform(0, 1.0);						
*/
					MaxEntIrlZiebartExact legacy_irl = new MaxEntIrlZiebartExact(200,vi, observableStates, 0, .0005, .1);
					
					double [] foundWeights_legacy;
					
					Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
					
					
			/*		auto V2 = vi.solve(model, .01);

										writeln("Value:");
					foreach (State s ; model.S()) {
						writeln(s, " ", V2[s]);
						
					}
					
					policy_legacy = vi.createPolicy(model, V2);
					writeln();
					writeln("Policy:");
					foreach (s ; model.S()) {
						if (model.is_terminal(s)) {
							writeln(s, " END");
						} else {
							writeln(s, " ", policy_legacy.actions(s));
						}	
					}
										*/
			
				    auto legacy_found_values = analyzeIRLResults([model], [policy_legacy], [opt_reward], [initial], sample_length);
					legacy_found_values[] -= opt_policy_value[];
						
//					write(iter, "-Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, "-", sample_length, ", ");
					foreach (fpe; legacy_found_values) {
						Legacy_errors ~= -fpe;
//						write(-fpe, ", ");
					}					
//					writeln();writeln();
//					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
							
							
							

					MaxEntIrlPartialVisibility mirl_irl = new MaxEntIrlPartialVisibility(200, vi, 200, .0005, .1, 0.09, observableStates, sample_length);
					
					double [] foundWeights_mirl;
					
					Agent policy_mIRL = mirl_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_mirl);
					
					reward.setParams(foundWeights_mirl);
					model.setReward(reward);
					
										
				    auto mirl_found_values = analyzeIRLResults([model], [policy_mIRL], [opt_reward], [initial], sample_length);
					mirl_found_values[] -= opt_policy_value[];
						
//					writeln(iter, "-mIRL, ", last_weights_max_entropy, ", ", foundWeights_mirl, ", ", num_samples, "-", sample_length, ", ");
					foreach (fpe; mirl_found_values)
						mIRL_errors ~= -fpe;
//						write(-fpe, ", ");
					
//					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
												
				}				
				writeln(num_samples, ", ", sample_length, ", LME, ", LME_errors);
				writeln(num_samples, ", ", sample_length, ", Legacy, ", Legacy_errors);
				writeln(num_samples, ", ", sample_length, ", mIRL, ", mIRL_errors);
			}
		}	
	    
	}	
	
	// Exact EM, limited data, random visibility, Ziebart's algorithm
	if (experiment_num == 8 ){
		
		foreach(sample_length; [5]) {
			foreach(num_samples; [2, 3, 4, 5, 10, 15, 20, 25/*, 50, 100/*, 250, 500*/]) {
				double [] LME_errors;

				foreach(iter;0..100) {
	    		
					auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  
	
					State [][] observableStatesArray;
						
					sar [][] trajs;
//						trajs.length = sample_length;
	
					foreach (q; 0..num_samples) { 
						State [] observableStates;
		
						// choose a starting state
						ToyState observerStartingState = cast(ToyState)(Distr!State.sample(initial));
						
						foreach(s; model.S()) {
							ToyState ts = cast(ToyState)s;
							if (abs(cast(int)ts.getLocation[0] - cast(int)observerStartingState.getLocation()[0]) <= 1 &&
							    abs(cast(int)ts.getLocation[1] - cast(int)observerStartingState.getLocation()[1]) <= 1)
								observableStates ~= s;
							
						}
						observableStatesArray ~= observableStates;

						sar [] traj = simulate(model, opt_policy, initial, sample_length);
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							traj ~= temp;
							
						}		
						foreach(i, ref sar; traj) {
							bool found = false;
							foreach (os; observableStates) {
								if (os == sar.s) {
									found = true;
									break;
								}	
							}
							if (! found) {
								sar.s = null;
								sar.a = null;
							}	
							sar.p = 1.0/num_samples;
						}
						trajs ~= traj;
					}
					
					// solve IRL exactly
					debug {
//						writeln("Trajs");
//						writeln(trajs);
					}
					
					double max_entropy = -double.max;
					double [] found_weights_max_entropy; 
					double [] last_weights_max_entropy;
					Agent max_entropy_best_policy;
					double val;
					
					
/*					size_t [] saved_y_from_trajectory;
					double [][] saved_feature_expectations;
					double [] saved_pr_traj;
					size_t saved_sample_length = 0;
	*/
					foreach (repeat; 0..10) {
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
	
									
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(0, 1.0);	
									
//						LatentMaxEntIrlZiebartDynamicOcclusionApprox irl = new LatentMaxEntIrlZiebartDynamicOcclusionApprox(10, vi, observableStatesArray, 0, .01, .1, sample_length);
						LatentMaxEntIrlZiebartDynamicOcclusionExact irl = new LatentMaxEntIrlZiebartDynamicOcclusionExact(10, vi, observableStatesArray, 0, .0005, .1);

//						irl.setYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);

						policy = irl.solve(model, initial, trajs, lastWeights, val, foundWeights);
						
//						auto entropy = getEntropy(getTrajDistribution(model, foundWeights, trajs));
						auto entropy = val;

//						irl.getYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);
						
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
						reward.setParams(found_weights_max_entropy);
						model.setReward(reward);
						
						debug {
							writeln("Entropy: ", entropy);
							
							V = vi.solve(model, .01);
						
							writeln("Value:");
							foreach (State s ; model.S()) {
								writeln(s, " ", V[s]);
								
							}
			
							foreach (s; model.S()) {
								if (! model.is_terminal(s))
									writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
						    	else 
									writeln(s, " - TERMINATE");    	
						    }
						
						}
					}
						
				    auto found_values = analyzeIRLResults([model], [max_entropy_best_policy], [opt_reward], [initial], sample_length);
				    
					found_values[] -= opt_policy_value[];
						
//					write(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ");
					foreach (fpe; found_values) {
						LME_errors ~= -fpe;
//						write(-fpe, ", ");
					}	
	//				writeln();
									
				}				
				writeln(num_samples, ", ", sample_length, ", LME, ", LME_errors);
			}
		}	
	    
	}		

	// Approx EM, limited data, goal states hidden, Ziebart's algorithm
	if (experiment_num == 9 ){
		
		State [] observableStates;
		
		foreach(s; model.S()) {
			ToyState ts = cast(ToyState)s;
			if (ts.getLocation()[1] < 3)
				observableStates ~= s;
			
		}

		foreach(sample_length; [8]) {
			foreach(num_samples; [2, 3, 4, 5, 10, 15, 20, 25, 50/*, 100, 250, 500*/]) {
				double [] LME_errors;
				double [] Legacy_errors;
				double [] mIRL_errors;

				foreach(iter;0..30) {
	    		
					auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  
	
						
					sar [][] trajs;
//						trajs.length = sample_length;
	
					foreach (q; 0..num_samples) { 
						sar [] traj = simulate(model, opt_policy, initial, sample_length);
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							traj ~= temp;
							
						}		
						foreach(i, ref sar; traj) {
							bool found = false;
							foreach (os; observableStates) {
								if (os == sar.s) {
									found = true;
									break;
								}	
							}
							if (! found) {
								sar.s = null;
								sar.a = null;
							}	
							sar.p = 1.0/num_samples;
						}
						trajs ~= traj;
					}
					
					// solve IRL exactly
					debug {
//						writeln("Trajs");
//						writeln(trajs);
					}
					
					double max_entropy = -double.max;
					double [] found_weights_max_entropy; 
					double [] last_weights_max_entropy;
					Agent max_entropy_best_policy;
					double val;
					
						
					foreach (repeat; 0..10) {
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
	
									
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(0, 1.0);	
									
						LatentMaxEntIrlZiebartApprox irl = new LatentMaxEntIrlZiebartApprox(10, vi, observableStates, 0, .01, .1, sample_length);
//						LatentMaxEntIrlZiebartPolicyApprox irl = new LatentMaxEntIrlZiebartPolicyApprox(200, vi, observableStates, 0, .0005, .01, sample_length, 0.00001);

						policy = irl.solve(model, initial, trajs, lastWeights, val, foundWeights);
						
						auto entropy = val;
												
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
						reward.setParams(found_weights_max_entropy);
						model.setReward(reward);
						
						debug {
							writeln("Entropy: ", entropy);
							
							V = vi.solve(model, .01);
						
							writeln("Value:");
							foreach (State s ; model.S()) {
								writeln(s, " ", V[s]);
								
							}
			
							foreach (s; model.S()) {
								if (! model.is_terminal(s))
									writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
						    	else 
									writeln(s, " - TERMINATE");    	
						    }
						
						}
					}
						
				    auto found_values = analyzeIRLResults([model], [max_entropy_best_policy], [opt_reward], [initial], sample_length);
				    
					found_values[] -= opt_policy_value[];
						
//					write(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ");
					foreach (fpe; found_values) {
						LME_errors ~= -fpe;
//						write(-fpe, ", ");
					}	
//					writeln();
					
					
	/*				last_weights_max_entropy = new double[reward.dim()];
					for (int i = 0; i < last_weights_max_entropy.length; i ++)
						last_weights_max_entropy[i] = uniform(0, 1.0);						
*/
					MaxEntIrlZiebartExact legacy_irl = new MaxEntIrlZiebartExact(200,vi, observableStates, 0, .0005, .1);
					
					double [] foundWeights_legacy;
					
					Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
					
					
			/*		auto V2 = vi.solve(model, .01);

										writeln("Value:");
					foreach (State s ; model.S()) {
						writeln(s, " ", V2[s]);
						
					}
					
					policy_legacy = vi.createPolicy(model, V2);
					writeln();
					writeln("Policy:");
					foreach (s ; model.S()) {
						if (model.is_terminal(s)) {
							writeln(s, " END");
						} else {
							writeln(s, " ", policy_legacy.actions(s));
						}	
					}
										*/
			
				    auto legacy_found_values = analyzeIRLResults([model], [policy_legacy], [opt_reward], [initial], sample_length);
					legacy_found_values[] -= opt_policy_value[];
						
//					write(iter, "-Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, "-", sample_length, ", ");
					foreach (fpe; legacy_found_values) {
						Legacy_errors ~= -fpe;
//						write(-fpe, ", ");
					}					
//					writeln();writeln();
//					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
							
							
							

					MaxEntIrlPartialVisibility mirl_irl = new MaxEntIrlPartialVisibility(200, vi, 200, .0005, .1, 0.09, observableStates, sample_length);
					
					double [] foundWeights_mirl;
					
					Agent policy_mIRL = mirl_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_mirl);
					
					reward.setParams(foundWeights_mirl);
					model.setReward(reward);
					
										
				    auto mirl_found_values = analyzeIRLResults([model], [policy_mIRL], [opt_reward], [initial], sample_length);
					mirl_found_values[] -= opt_policy_value[];
						
//					writeln(iter, "-mIRL, ", last_weights_max_entropy, ", ", foundWeights_mirl, ", ", num_samples, "-", sample_length, ", ");
					foreach (fpe; mirl_found_values)
						mIRL_errors ~= -fpe;
//						write(-fpe, ", ");
					
//					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
												
				}				
				writeln(num_samples, ", ", sample_length, ", LME, ", LME_errors);
				writeln(num_samples, ", ", sample_length, ", Legacy, ", Legacy_errors);
				writeln(num_samples, ", ", sample_length, ", mIRL, ", mIRL_errors);
			}
		}	
	    
	}		
	
	// Approx EM, limited data, random visibility (per run), Ziebart's algorithm
	if (experiment_num == 10 ){
		
		foreach(sample_length; [8]) {
			foreach(num_samples; [2, 3, 4, 5, 10, 15, 20, 25, 50, 100/*, 250, 500*/]) {
				double [] LME_errors;
				double [] Legacy_errors;
				double [] mIRL_errors;
				
				foreach(iter;0..30) {
					
					State [] observableStates;
		
					// choose a starting state
//					ToyState observerStartingState = new ToyState();
					ToyState observerStartingState = cast(ToyState)Distr!State.sample(initial);
					debug {
						writeln("Observer Position: ", observerStartingState);
					} 
					foreach(s; model.S()) {
						ToyState ts = cast(ToyState)s;
						if (abs(cast(int)ts.getLocation[0] - cast(int)observerStartingState.getLocation()[0]) <= 1 &&
						    abs(cast(int)ts.getLocation[1] - cast(int)observerStartingState.getLocation()[1]) <= 1)
							observableStates ~= s;
						
					}
					auto opt_policy_value = analyzeIRLResults([model], [opt_policy], [opt_reward], [initial], sample_length);  
	
						
					sar [][] trajs;
//						trajs.length = sample_length;
	
					foreach (q; 0..num_samples) { 

						sar [] traj = simulate(model, opt_policy, initial, sample_length);
						foreach (i; traj.length .. sample_length) {  // Should I do this? does it affect the new technique? Try without it next
							auto temp = traj[$ - 1];
							temp.p = 1.0/num_samples;
							traj ~= temp;
							
						}	
						debug {
							writeln(traj);
						}	
						foreach(i, ref sar; traj) {
							bool found = false;
							foreach (os; observableStates) {
								if (os == sar.s) {
									found = true;
									break;
								}	
							}
							if (! found) {
								sar.s = null;
								sar.a = null;
							}	
							sar.p = 1.0/num_samples;
						}
						trajs ~= traj;
					}
					
					// solve IRL exactly
					debug {
						writeln("Trajs");
						writeln(trajs);
					}
					
					double max_entropy = -double.max;
					double [] found_weights_max_entropy; 
					double [] last_weights_max_entropy;
					Agent max_entropy_best_policy;
					double val;
					
					
/*					size_t [] saved_y_from_trajectory;
					double [][] saved_feature_expectations;
					double [] saved_pr_traj;
					size_t saved_sample_length = 0;
	*/
					foreach (repeat; 0..10) {
						Agent policy = new RandomAgent(model.A(null));
						double [] foundWeights;
	
									
						auto lastWeights = new double[reward.dim()];
						for (int i = 0; i < lastWeights.length; i ++)
							lastWeights[i] = uniform(0, 1.0);	
									
						LatentMaxEntIrlZiebartApprox irl = new LatentMaxEntIrlZiebartApprox(10, vi, observableStates, 0, .01, .1, sample_length);
//						LatentMaxEntIrlZiebartPolicyApprox irl = new LatentMaxEntIrlZiebartPolicyApprox(10, vi, observableStates, 0, .01, .1, sample_length);

//						irl.setYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);

						policy = irl.solve(model, initial, trajs, lastWeights, val, foundWeights);
						
//						auto entropy = getEntropy(getTrajDistribution(model, foundWeights, trajs));
						auto entropy = val;

//						irl.getYAndZ(saved_y_from_trajectory, saved_pr_traj, saved_feature_expectations, saved_sample_length);
						
						if (entropy > max_entropy) {
							max_entropy = entropy;
							found_weights_max_entropy = foundWeights.dup;
							last_weights_max_entropy = lastWeights.dup;
							max_entropy_best_policy = policy;
						}
						reward.setParams(found_weights_max_entropy);
						model.setReward(reward);
						
						debug {
							writeln("Max Entropy: ", max_entropy);
							
							V = vi.solve(model, .01);
						
							writeln("Value:");
							foreach (State s ; model.S()) {
								writeln(s, " ", V[s]);
								
							}
			
							foreach (s; model.S()) {
								if (! model.is_terminal(s))
									writeln(s, " - Opt ", opt_policy.actions(s), "  Found Opt: ", max_entropy_best_policy.actions(s));
						    	else 
									writeln(s, " - TERMINATE");    	
						    }
						
						}
					}
					
				    auto found_values = analyzeIRLResults([model], [max_entropy_best_policy], [opt_reward], [initial], sample_length);
				    
					found_values[] -= opt_policy_value[];
						
//					write(iter, ", ", last_weights_max_entropy, ", ", found_weights_max_entropy, ", ", num_samples, ", ");
					foreach (fpe; found_values) {
						LME_errors ~= -fpe;
//						write(-fpe, ", ");
					}	
//					writeln();
	
					
	/*				last_weights_max_entropy = new double[reward.dim()];
					for (int i = 0; i < last_weights_max_entropy.length; i ++)
						last_weights_max_entropy[i] = uniform(0, 1.0);						
*/
					MaxEntIrlZiebartExact legacy_irl = new MaxEntIrlZiebartExact(200,vi, observableStates, 0, .0005, .1);
					
					double [] foundWeights_legacy;
					
					Agent policy_legacy = legacy_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_legacy);
					
					reward.setParams(foundWeights_legacy);
					model.setReward(reward);
					
					
					
			/*		auto V2 = vi.solve(model, .01);

										writeln("Value:");
					foreach (State s ; model.S()) {
						writeln(s, " ", V2[s]);
						
					}
					
					policy_legacy = vi.createPolicy(model, V2);
					writeln();
					writeln("Policy:");
					foreach (s ; model.S()) {
						if (model.is_terminal(s)) {
							writeln(s, " END");
						} else {
							writeln(s, " ", policy_legacy.actions(s));
						}	
					}
										*/
				    auto legacy_found_values = analyzeIRLResults([model], [policy_legacy], [opt_reward], [initial], sample_length);
					legacy_found_values[] -= opt_policy_value[];
						
//					write(iter, "-Legacy, ", last_weights_max_entropy, ", ", foundWeights_legacy, ", ", num_samples, "-", sample_length, ", ");
					foreach (fpe; legacy_found_values) {
						Legacy_errors ~= -fpe;
//						write(-fpe, ", ");
					}					
//					writeln();writeln();
//					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
							
							
							

					MaxEntIrlPartialVisibility mirl_irl = new MaxEntIrlPartialVisibility(200, vi, 200, .0005, .1, 0.09, observableStates, sample_length);
					
					double [] foundWeights_mirl;
					
					Agent policy_mIRL = mirl_irl.solve(model, initial, trajs, last_weights_max_entropy, val, foundWeights_mirl);
					
					reward.setParams(foundWeights_mirl);
					model.setReward(reward);
					
										
				    auto mirl_found_values = analyzeIRLResults([model], [policy_mIRL], [opt_reward], [initial], sample_length);
					mirl_found_values[] -= opt_policy_value[];
						
//					writeln(iter, "-mIRL, ", last_weights_max_entropy, ", ", foundWeights_mirl, ", ", num_samples, "-", sample_length, ", ");
					foreach (fpe; mirl_found_values)
						mIRL_errors ~= -fpe;
//						write(-fpe, ", ");
					
//					compare_policy_distrs(model, feature_expectations, found_weights_max_entropy, foundWeights_legacy);
												
				}				
				writeln(num_samples, ", ", sample_length, ", LME, ", LME_errors);
				writeln(num_samples, ", ", sample_length, ", Legacy, ", Legacy_errors);
				writeln(num_samples, ", ", sample_length, ", mIRL, ", mIRL_errors);
			}
		}	
	    
	}		


	// UAI 2016 Timing experiment
	if (experiment_num == 11 ){
		
		// get a sample trajectory
		
		// go in a loop, time the E step for each method 10 times
		// then remove a state from observability and the given trajecotry
		
		// repeat for uniform and deterministic policies
		 
		int sample_length = 8;
		int num_samples = 1;
		int benchmark_samples = 10;
		
		// Build trajectory;
		double[State] startingState;
		startingState[new ToyState([2, 0])] = 1.0;

		sar [] traj = simulate(model, opt_policy, startingState, sample_length);
		writeln(traj);
		
		State [] observableStates = model.S().dup;
		State [] occludedStates;
		
		int t = 0;
		
		// create uniform policy
		double[Action][State] uniform_policy;
		foreach(s; model.S()) {
			if (model.is_terminal(s)) {
				uniform_policy[s][new NullAction()] = 1.0;
				
			} else {
				foreach(a; model.A(s)) {
					uniform_policy[s][a] = 1.0 / model.A(s).length;
				}
			}
			
		}
		
		Agent cur_policy = new StochasticAgent(uniform_policy);
		foreach(i; 0..(observableStates.length)) {
		
			
			
			size_t Z_size = 0;
			
			foreach(s; occludedStates) {
				auto temp = model.A(s).length;
				if (! model.is_terminal(s)) 
					Z_size += temp;
			}
			
			Z_size = pow(Z_size, t);

			size_t actual_Z_size;
		
			void exact() {  
//				if (Z_size < 2500000000)
				actual_Z_size = calc_E_step_exact_single_agent(model, cur_policy, [traj], observableStates, 0.001, 50, initial);
			}	

			void gibbs() {
				calc_E_step_single_agent_gibbs(model, cur_policy, [traj], observableStates, 0.001, 50, initial);
			}

			void forwback() {
				calc_E_step_single_agent_forward_backward(model, cur_policy, [traj], observableStates, 0.001, 50, initial);
			}

			void blockedgibbs() {
				calc_E_step_single_agent_blocked_gibbs(model, cur_policy, [traj], observableStates, 0.001, 50, initial);
			}
			
			void multitimestepblockedgibbs() {
				calc_E_step_single_agent_multi_timestep_blocked_gibbs(model, cur_policy, [traj], observableStates, 0.001, 50, initial);
			}
			
			auto r = benchmark!(exact, gibbs, forwback, blockedgibbs, multitimestepblockedgibbs)(benchmark_samples);
			
			Duration [] totaltime;
			foreach(b; r) {
				totaltime ~= to!Duration(b);
			}
			
			long [] msecs;
			foreach(duration; totaltime) {
				msecs ~= duration.total!"msecs" / benchmark_samples;
			}
			writeln(occludedStates.length, " |Z| = ", Z_size, " ", " Actually considered Z: ", actual_Z_size, " ", msecs);
			
			
			// pop a state off of observableStates and remove it from the trajectory if present
			
			
			foreach(ref entry; traj) {
				if (entry.s == observableStates[$-1]) {
					entry.s = null;
					entry.a = null;
					t ++;
				}
			}
			occludedStates ~= observableStates[$-1];
			observableStates.length = observableStates.length - 1; 
			 
		}

	}		
				
	return 0;
	
}

double[Action][][] genEquilibria() {
	
	double[Action][][] returnval;
	returnval.length = 6;  // six equilibria (<MU,ML>, <ML, MU>, <MR, ML>, <ML, MR>, <MD, MU>, <MU, MD>)
	
	double[Action][] one;
	one.length = 2;
	
	one[0][new MoveUpAction()] = 1.0;
	one[1][new MoveLeftAction()] = 1.0;
	returnval[0] = one;
	
	
	double[Action][] two;
	two.length = 2;

	two[0][new MoveLeftAction()] = 1.0;
	two[1][new MoveUpAction()] = 1.0;
	returnval[1] ~= two;


	double[Action][] three;
	three.length = 2;

	three[0][new MoveRightAction()] = 1.0;
	three[1][new MoveLeftAction()] = 1.0;
	returnval[2] ~= three;


	double[Action][] four;
	four.length = 2;

	four[0][new MoveLeftAction()] = 1.0;
	four[1][new MoveRightAction()] = 1.0;
	returnval[3] ~= four;


	double[Action][] five;
	five.length = 2;

	five[0][new MoveDownAction()] = 1.0;
	five[1][new MoveUpAction()] = 1.0;
	returnval[4] ~= five;


	double[Action][] six;
	six.length = 2;

	six[0][new MoveUpAction()] = 1.0;
	six[1][new MoveDownAction()] = 1.0;
	returnval[5] ~= six;

	
	return returnval;
	
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

double analyzeIRLDistrResults(Model model, double [] policy_distr, LinearReward true_rewards, double[State] initial_states, int length, double[][] state_action_freq) {
	
	// the value of a policy is the sum of state visitation frequency times the reward for the state
	double returnval = 0;
	
	foreach(i, pr; policy_distr) {

		double val = 0;
		size_t pos = 0;
		foreach (s; model.S()) {
			if (model.is_terminal(s)) {
				val += state_action_freq[i][pos] * true_rewards.reward(s, new NullAction());
				pos ++;				
			} else {
				foreach(a; model.A()) {
					val += state_action_freq[i][pos] * true_rewards.reward(s, a);
					pos ++;
				}
			}
			
		}
		
		returnval += val * pr;
	}
	
	return returnval;

}

double KLD(double [] true_distr, double[] est_distr) {
	
	double returnval = 0;
	foreach (i, pr; true_distr) {
		returnval += pr * log ( pr / est_distr[i]);
	}
	return returnval;
	
}

double [] getTrajDistribution(Model model, double [] weights, sar [][] trajs) {
		
	double [] returnval;
	double norm = 0;
	LinearReward ff = cast(LinearReward)model.getReward();

	foreach(i, traj; trajs) {
		double pr_traj = 1.0;
		
		double [] weighted_fe = weights.dup;
		double []traj_fe = new double[weights.length];
		traj_fe[] = 0;
		foreach (j, SAR; traj) {
			
			if (j < traj.length - 2 && SAR.s ! is null) {
        		auto transitions =  model.T(SAR.s, SAR.a);
        		
        		foreach (state, pr; transitions) {
        			if (traj[j+1].s == state) {
        				pr_traj *= pr;
        				break;
        			}
        		}
			}
			
			if (SAR.s ! is null)
				traj_fe[] += ff.features(SAR.s, SAR.a)[];
		}
		
		weighted_fe[] *= traj_fe[];
		
		double temp = exp(reduce!("a + b")(0.0, weighted_fe));
		
		norm += temp;
		
		temp *= pr_traj;
		
		returnval ~= temp;
		
	}
	
	foreach (ref r; returnval) {
		r /= norm;
	}
	
	return returnval;
}	

double [] getPolicyDistribution(Model model, double [] weights, double [][] feature_expectations_for_policies) {
        double normalizer = 0;
    	double [] policy_distr = new double[feature_expectations_for_policies.length];
        foreach(i, feature_exp; feature_expectations_for_policies) {
        	double [] temp = weights.dup;
        	temp[] *= feature_exp[];
	    	policy_distr[i] = exp(reduce!("a + b")(0.0, temp));
	    	
	    	normalizer += policy_distr[i];
    	}

    	if (normalizer == 0)
    		normalizer = double.min;
    	debug {
    		writeln("Normalizer: ", normalizer);
    		
    	}
    	
    	policy_distr[] /= normalizer;
    	
    	return policy_distr;
    }

double getEntropy(double [] distr) {
	
	double returnval = 0;
	
	foreach(pr; distr) {
		returnval += pr * log(pr);
	}
	
	return -returnval;
}

void compare_policy_distrs(Model model, double [][] feature_expectations, double [] weights_new, double [] weights_old, double [] true_distr = null) {
	
	// calculate the entropy of each
	
	double [] new_distr = getPolicyDistribution(model, weights_new, feature_expectations);
	double new_entropy = getEntropy(new_distr);
	
	double [] old_distr = getPolicyDistribution(model, weights_old, feature_expectations);
	double old_entropy = getEntropy(old_distr);
	
	// printout results
	
	writeln("Entropy LME vs old, ", new_entropy, " , ", old_entropy); 
	
	if (true_distr != null) {
		writeln("KLD to true Distr, New, ", KLD(true_distr,  new_distr), ", Old, ", KLD(true_distr, old_distr));
	}
	
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

size_t count_non_terminal_states(Model m) {
	size_t a = 0;
	
	foreach(s; m.S()) {
		if (! m.is_terminal(s)) {
			a ++;
		}
	}
	
	return a;
}
void all_feature_expectations(Model model, double[State] initial, int sample_length) {
	double [][] returnval;
	returnval.length = pow(model.A().length, count_non_terminal_states(model));
	
	foreach(i; 0 .. returnval.length) {
		returnval[i] = feature_expectations_exact(model, calcStateActionFreq(get_policy_num(model, i), initial, model, sample_length));
		writeln(returnval[i]);
	}
	
}
