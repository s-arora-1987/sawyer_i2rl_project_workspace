import mdp;
import boydmdp;
import irl;
import std.stdio;
import std.format;
import std.string;
import std.math;
import std.random;
import std.algorithm;
import core.stdc.stdlib : exit;
import std.datetime;
import std.numeric;

int main() {
	
	sac [][] SAC;
    sac [][] SACfull;
	string mapToUse;

    int num_Trajsofar;
    string st;
	double last_val;
	string buf;
	string algorithm;
	int length_subtrajectory;
	double LBA; 
	double conv_threshold_stddev_diff_moving_wdw;
	int restart_attempts;
	int moving_window_length_muE;
	int use_ImpSampling;
	double conv_threshold_gibbs;

	buf = readln();
	debug {
		writeln("starting "); 
	    auto starttime = Clock.currTime();
	} 
	//writeln("starting "); 

	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	double[State][Action][State][] T;
	int curT = 0;
	T.length = 1;	
	while ((buf = readln()) != null) {
    	buf = strip(buf);
    	
    	if (buf == "ENDT") {
			break;
    	}
    	
    	State s;
    	Action a;
    	State s_prime;
    	double p;
    	
    	debug {
    		writeln("buf ",buf);
    	}

        p = parse_transitions(mapToUse, buf, s, a, s_prime);

    	T[curT][s][a][s_prime] = p;
    }

    debug{
    	writeln("read Tran ");
    }

	byte[][] map;
	LinearReward reward;
    Model model;

	map = boyd2PatrollerMap();
	model = new BoydModel(null, map, T[0], 1, &simplefeatures);

    //model = new BoydModel2(null, map, 0.05);

	buf = readln();
	formattedRead(buf, "%s", &algorithm);
	algorithm = strip(algorithm);

	double [] reward_weights;
	int dim;
    reward = new Boyd2RewardGroupedFeatures(model);
    reward_weights = new double[reward.dim()];
    reward_weights[] = 0;
	reward.setParams(reward_weights);

	model.setReward(reward);
	model.setGamma(0.99);
	//model.setGamma(0.999);

	debug {
		writeln("started reading trajectories");
	}

    while ((buf = readln()) != null) {
    	buf = strip(buf);
	    debug {
	    	//writeln("buf ",buf);
	    }
    
    	if (buf == "ENDTRAJ") {
			break;

    	} else {
    		sac [] newtraj;
    		
    		while (buf.countUntil(";") >= 0) {
    			string percept = buf[0..buf.countUntil(";")];
    			buf = buf[buf.countUntil(";") + 1 .. buf.length];
    			
    			string state;
    			string action;
    			double p;
    			
   				formattedRead(percept, "%s:%s:%s", &state, &action, &p);
   				
   				int x;
   				int y;
   				int z;
   				int cg;

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
   				
   				newtraj ~= sac(new BoydState([x, y, z]), a, p);

			    debug {
			    	writeln("finished reading ",[x, y, z], a, p);
			    }

    		}
    		
    		SAC ~= newtraj;
    		
    	}
    }
    debug {
    	writeln("finished reading trajectories");
    }

    double[] trueWeights; 
    trueWeights.length = reward.dim(); 
    debug {
    	writeln("reading trueWeights ");
    }
    buf = readln();
    formattedRead(buf, "[%s]", &st);
    for (int j = 0; j < reward.dim()-1; j++) {
        formattedRead(st,"%s, ",&trueWeights[j]);
    }
    formattedRead(st,"%s",&trueWeights[reward.dim()-1]);
    debug {
    	writeln(trueWeights);
    }

    buf = readln();
	formattedRead(buf, "%s", &length_subtrajectory);
    debug {
    	writeln("length_subtrajectory ",length_subtrajectory);
    }

    buf = readln();
	formattedRead(buf, "%s", &conv_threshold_stddev_diff_moving_wdw);
    debug {
    	writeln("conv_threshold_stddev_diff_moving_wdw ",conv_threshold_stddev_diff_moving_wdw);
    }
    buf = readln();
	formattedRead(buf, "%s", &restart_attempts);
    debug {
    	writeln("restart_attempts ",restart_attempts);
    }
    buf = readln();
	formattedRead(buf, "%s", &moving_window_length_muE);
    debug {
    	writeln("moving_window_length_muE ",moving_window_length_muE);
    }
    buf = readln();
	formattedRead(buf, "%s", &use_ImpSampling);
    debug {
    	writeln("moving_window_length_muE ",use_ImpSampling);
    }
    buf = readln();
	formattedRead(buf, "%s", &conv_threshold_gibbs);
    debug {
    	writeln("conv_threshold_gibbs ",conv_threshold_gibbs);
    }


    // change this acc to choice of reward model
    double [] lastWeightsI2RL;
    double [] featureExpecExpert;
    double [] featureExpecExpertfull;
    double [] foundWeightsGlbl;
    lastWeightsI2RL.length = reward.dim();
    lastWeightsI2RL[] = 0.0;
    featureExpecExpertfull.length = reward.dim();
    featureExpecExpertfull[] = 0.0;
    featureExpecExpert.length = reward.dim();
    featureExpecExpert[] = 0.0;
    foundWeightsGlbl.length = reward.dim();

    if (algorithm == "MAXENTZAPPROXNOISYOBS") {

        buf = readln();
        formattedRead(buf, "[%s]", &st);
        for (int j = 0; j < reward.dim()-1; j++) {
            formattedRead(st,"%s, ",&lastWeightsI2RL[j]);
        }
        formattedRead(st,"%s",&lastWeightsI2RL[reward.dim()-1]);
	    debug {
	    	writeln("lastWeightsI2RL ",lastWeightsI2RL);
	    }


        buf = readln();
        formattedRead(buf, "[%s]", &st);
        for (int j = 0; j < reward.dim()-1; j++) {
            formattedRead(st,"%s, ",&featureExpecExpert[j]);
        }
        formattedRead(st,"%s",&featureExpecExpert[reward.dim()-1]);

        buf = readln();
        formattedRead(buf, "%s", &num_Trajsofar);

    } 

	double [] lastWeights = lastWeightsI2RL.dup; //new double[reward_weights.length];
	//for (int i = 0; i < lastWeights.length; i ++) {
	//	lastWeights[i] = uniform(-0.99, 0.99);
	//	if (mapToUse == "sorting") lastWeights[i] = uniform(0.01, 0.99);
	//}

	debug {
		writeln(lastWeightsI2RL);
        writeln("initialized Weights -- ",lastWeights);
    }
    debug {
    	writeln("num_Trajsofar ");
        writeln(num_Trajsofar);
    }

	double[State] initial;
	foreach (s; model.S()) {
		initial[s] = 1.0;
	}
	Distr!State.normalize(initial); 
	
	sac [][] samples = SAC;
    
	Agent policy = new RandomAgent(model.A(null));

	sac [][] trajectoriesg;

	double VI_threshold = 0.2; 
	double grad_descent_threshold = 0.0000001; // Not being used in current descent method 
	double Ephi_thresh = 0.1;
	double gradient_descent_step_size = 0.00001;
	int vi_duration_thresh_secs = 45; //30;
	int descent_duration_thresh_secs = 3*60;
	int nSamplesTrajSpace = 100;

	debug {
		// testing to get stochastic policy
		writeln(" stochastic policy for true weights ",trueWeights);
		reward.setParams(trueWeights);
		double qval_thresh = 0.01;
		ulong max_iter_QSolve = 100;
		double[StateAction] Q_value = QValueSoftMaxSolve(model, qval_thresh, max_iter_QSolve);        
		Agent trueStochPolicy = CreateStochasticPolicyFromQValue(model, Q_value);
		writeln("\nSimulation:");
		sar [] trajsim;
		for(int i = 0; i < 5; i++) {
			trajsim = simulate(model, trueStochPolicy, initial, 20);
			foreach (sar pair ; trajsim) {
				//writeln(pair.s, " ", pair.a, " ", pair.r);
			}
			//writeln(" ");
		}
	} 
	
	double diff_wrt_muE_wo_sc;
	double diff_wrt_muE_scores1;
	
	if (algorithm == "MAXENTZAPPROXNOISYOBS") {

		double [] foundWeights1;
		double val1;
		
		sac [][] trajectories;

		// divide them into equal sized trajectories for each of two experts
			
		int t = 1;
		int ct = 0;
		trajectories.length = 1;
		foreach (entry; SAC) {
			if (entry.length > 0)
				trajectories[ct] ~= entry[0];
			else
				trajectories[ct] ~= sac(null, null, 1.0);

			// if reached desired length, make next trajectory 
			if (t % length_subtrajectory == 0) {
				ct += 1;
				trajectories.length += 1;
			}

		t = (t + 1) % length_subtrajectory;
		//writeln(trajectories[agent_num]);
		}
		trajectories.length -= 1;
		//writeln(trajectories[agent_num]);
		
		size_t max_sample_length = length_subtrajectory;
		//writeln("input to irl1:",trajectories[0]);

		MaxEntIrlZiebartApproxNoisyObs irl = new MaxEntIrlZiebartApproxNoisyObs(restart_attempts, 
			new TimedValueIteration(int.max,false,vi_duration_thresh_secs), model.S(), 
			nSamplesTrajSpace, grad_descent_threshold, VI_threshold);

		writeln("calling MaxEntIrlZiebartApproxNoisyObs.solve");
		policy = irl.solve(model, initial, trajectories, max_sample_length, 
			lastWeights, val1, foundWeights1, featureExpecExpert, num_Trajsofar, 
			Ephi_thresh, gradient_descent_step_size, descent_duration_thresh_secs,
			trueWeights, conv_threshold_stddev_diff_moving_wdw, 
			moving_window_length_muE, use_ImpSampling, conv_threshold_gibbs,
			diff_wrt_muE_wo_sc, diff_wrt_muE_scores1); 

		//reward_weights =[0.15, 0.0, -0.1, 0.2, -0.1, 0.0, 0.3, -0.15];
		//reward.setParams(reward_weights);
		TimedValueIteration vi = new TimedValueIteration(int.max,false,vi_duration_thresh_secs);
		policy = vi.createPolicy(model,vi.solve(model, 0.1));

        foundWeightsGlbl=foundWeights1;
        trajectoriesg = trajectories;
        last_val=val1; 

        reward.setParams(trueWeights);
		Agent truePolicy = vi.createPolicy(model,vi.solve(model, 0.1));
        reward.setParams(foundWeightsGlbl);		
		Agent learnedPolicy = vi.createPolicy(model,vi.solve(model, 0.1));

		LBA = irl.computeLBA(cast(MapAgent)learnedPolicy,cast(MapAgent)truePolicy);

    } 
	
	writeln("BEGPARSING");

	foreach (State s; model.S()) {
		foreach (Action a, double chance; policy.actions(s)) {

            BoydState ps = cast(BoydState)s;
            writeln( ps.toString(), " = ", a);

            //writeln(str_s," = ", a);

		}
	}
	
	writeln("ENDPOLICY");

    writeln(foundWeightsGlbl);
    writeln(featureExpecExpert);
    writeln(num_Trajsofar);

	debug {
		writeln("\nSimulation for 1:");
		sar [] trajs;
		for(int i = 0; i < 2; i++) {
			trajs = simulate(model, policy, initial, 50);
			//foreach (sar pair ; traj) {
			//	writeln(pair.s, " ", pair.a, " ", pair.r);
			//}
			//writeln(" ");
		}
        //Compute average EVD
        double avg_EVD = 0.0;
	    double trajval_trueweight, trajval_learnedweight;
	    double [][] fk_Xms_demonstration;

		MaxEntIrlZiebartApproxNoisyObs irl = new MaxEntIrlZiebartApproxNoisyObs(restart_attempts, 
			new TimedValueIteration(int.max,false,vi_duration_thresh_secs), model.S(), 
			nSamplesTrajSpace, grad_descent_threshold, VI_threshold);
        fk_Xms_demonstration.length = trajectoriesg.length;
        fk_Xms_demonstration = irl.calc_feature_expectations_per_sac_trajectory_impSampling(model, trajectoriesg, foundWeightsGlbl);
        foreach (j; 0 .. fk_Xms_demonstration.length) {
            trajval_learnedweight = dotProduct(foundWeightsGlbl,fk_Xms_demonstration[j]);
            trajval_trueweight = dotProduct(trueWeights,fk_Xms_demonstration[j]);
            avg_EVD += abs(trajval_trueweight - trajval_learnedweight)/(trajval_trueweight*cast(double)fk_Xms_demonstration.length);
        }
        writeln("\n EVD",avg_EVD);
        //writeln("\n learned weights",foundWeightsGlbl);

	} 
	
    writeln("\n LBA",LBA,"ENDLBA");

    writeln("\n DIFF1",diff_wrt_muE_wo_sc,"ENDDIFF1");
    writeln("\n DIFF2",diff_wrt_muE_scores1,"ENDDIFF2");

	writeln("ENDPARSING");

	debug {
	    auto endttime = Clock.currTime();
	    auto duration = endttime - starttime;
	    writeln("Runtime Duration ==> ", duration);
	}
	
	return 0;
}
