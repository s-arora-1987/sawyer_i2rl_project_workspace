import mdp;
import boydmdp;
import irlWithoutLBFGS;
import std.stdio;
import std.format;
import std.string;
import std.math;
import std.random;
import std.algorithm;
import sortingMDP;
import core.stdc.stdlib : exit;
import std.datetime;
import std.numeric;

int main() {
	
	sar [][] SAR;
    sar [][] SARfull;
    int num_Trajsofar;
    string st;
	double last_val;
	string mapToUse;
	string buf;
	string algorithm;
	int length_subtrajectory;
	double LBA; 

	buf = readln();

	debug {
		writeln("starting "); 
	    auto stattime = Clock.currTime();
	} 
	//writeln("starting "); 

	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	buf = readln();
	formattedRead(buf, "%s", &algorithm);
	algorithm = strip(algorithm);


	byte[][] map;
	LinearReward reward;
    Model model;

	if (mapToUse == "sorting") {
		//models ~= new sortingModel(0.05,null);
		//models ~= new sortingModel(0.05,null);
		//models ~= new sortingModel2(0.05,null);
		//models ~= new sortingModel2(0.05,null);
		//models ~= new sortingModelbyPSuresh(0.05,null);
		//models ~= new sortingModelbyPSuresh(0.05,null);
		//models ~= new sortingModelbyPSuresh2(0.05,null);
		//models ~= new sortingModelbyPSuresh2(0.05,null);
		//models ~= new sortingModelbyPSuresh3(0.05,null);
		//models ~= new sortingModelbyPSuresh3(0.05,null);
		//models ~= new sortingModelbyPSuresh4(0.05,null);
		//model = new sortingModelbyPSuresh4(0.05,null);
		model = new sortingModelbyPSuresh2WOPlaced(0.05,null);
		//model = new sortingModelbyPSuresh3multipleInit(0.05,null);
		
	} 

	double [] reward_weights;
	int dim;
	if (mapToUse == "sorting") {
		// Which reward type is it? 
		//dim = 8;
		//reward = new sortingReward2(models[i],dim); 
		//dim = 10;
		//reward = new sortingReward3(models[i],dim); 
		//reward = new sortingReward4(models[i],dim); 
		//reward = new sortingReward5(models[i],dim); 
		dim = 11;
		//reward = new sortingReward6(models[i],dim); 
		//reward = new sortingReward7WPlaced(models[i],dim); 
		reward = new sortingReward7(model,dim); 

		reward_weights = new double[reward.dim()];
		reward_weights[] = 0;
		reward.setParams(reward_weights);
	}

	model.setReward(reward);
	model.setGamma(0.99);
	model.setGamma(0.999);


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
    		sar [] newtraj;
    		
    		while (buf.countUntil(";") >= 0) {
    			string percept = buf[0..buf.countUntil(";")];
    			buf = buf[buf.countUntil(";") + 1 .. buf.length];
    			
    			string state;
    			string action;
    			double p;
    			
   				formattedRead(percept, "%s:%s:%s", &state, &action, &p);
   				
   				if (mapToUse == "sorting") {

	                int ol;
	                int pr;
	                int el;
	                int ls;

					state = state[1..state.length];
	                formattedRead(state, " %s, %s, %s, %s]", &ol, &pr, &el, &ls);

	   				Action a;
	   				if (action == "InspectAfterPicking") {
	   					a = new InspectAfterPicking();
	   				} else if (action == "InspectWithoutPicking" ) {
	   					a = new InspectWithoutPicking();
	                } else if (action == "Pick" ) {
	                    a = new Pick();
	                } else if (action == "PlaceOnConveyor" ) {
	                    a = new PlaceOnConveyor();
	                } else if (action == "PlaceInBin" ) {
	                    a = new PlaceInBin();
	                } else if (action == "ClaimNewOnion" ) {
	                    a = new ClaimNewOnion();
	   				} else if (action == "PlaceInBinClaimNextInList") {
	   					a = new PlaceInBinClaimNextInList();
	                } else {
	                    a = new ClaimNextInList();
	   				}
	   				
	   				newtraj ~= sar(new sortingState([ol, pr, el, ls]),a,p);

				    debug {
				    	//writeln("finished reading ",[ol, pr, el, ls],action);
				    }

   				} 
    		}
    		
    		SAR ~= newtraj;
    		
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

    if (algorithm == "MAXENTZAPPROXI2RL") {

        buf = readln();
        formattedRead(buf, "[%s]", &st);
        for (int j = 0; j < reward.dim()-1; j++) {
            formattedRead(st,"%s, ",&lastWeightsI2RL[j]);
        }
        formattedRead(st,"%s",&lastWeightsI2RL[reward.dim()-1]);


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
        writeln("initialized Weights -- ",lastWeights);
    }
    debug {
    	writeln("num_Trajsofar ");
        writeln(num_Trajsofar);
    }

	double[State] initial;
    // START FROm  0,2,0,2
    // sortingState iss = new sortingState([0,2,0,2]);
    sortingState iss = new sortingState([0,2,0,0]);
	if (mapToUse == "sorting") {

	    //initial[iss] = 1.0;
		foreach (s; model.S()) {
			sortingState ss = cast(sortingState)s;
			if (ss._onion_location == 0 && ss._prediction == 2 && ss._listIDs_status == 0) initial[ss] = 1.0;
			//if (ss._onion_location == 0 && ss._listIDs_status == 0) initial[ss] = 1.0;
		}
		writeln("numebr of initial states ",initial.length);
		Distr!State.normalize(initial); 

	} else {
		foreach (s; model.S()) {
			initial[s] = 1.0;
		}
		Distr!State.normalize(initial);
	}
	
	sar [][] samples = SAR;
    
	Agent policy = new RandomAgent(model.A(null));
	
	sar [][] trajectoriesg;
	
	if (algorithm == "MAXENTZEXACT") {

			double [] foundWeights1;
			double val1;

			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][] trajectories;

			trajectories.length = 1;

			foreach (entry; SAR) {
				if (entry.length > 0)
					trajectories[0] ~= entry[0];
				else
					trajectories[0] ~= sar(null, null, 1.0);
			}

			LatentMaxEntIrlZiebartExact irl1 = new LatentMaxEntIrlZiebartExact(50, new ValueIteration(), model.S(), 50, .0005, .1);
			//writeln("calling LatentMaxEntIrlZiebartExact.solve");
			policy = irl1.solve(model, initial, trajectories, lastWeights, val1, foundWeights1);
	        foundWeightsGlbl=foundWeights1;
	        trajectoriesg = trajectories;
	        last_val=val1; 

	} else if (algorithm == "MAXENTZAPPROX" || algorithm == "MAXENTZAPPROXI2RL") {

			double [] foundWeights1;
			double val1;

			
			sar [][] trajectories;
			if (mapToUse == "sorting") {
				// divide them into equal sized trajectories for each of two experts
					
				int t = 1;
				int ct = 0;
				trajectories.length = 1;
				foreach (entry; SAR) {
					if (entry.length > 0)
						trajectories[ct] ~= entry[0];
					else
						trajectories[ct] ~= sar(null, null, 1.0);

					// if reached desired length,  make next trajectory
					if (t % length_subtrajectory == 0) {
						ct += 1;
						trajectories.length += 1;
					}

				t = (t + 1) % length_subtrajectory;
				//writeln(trajectories[agent_num]);
				}
				trajectories.length -= 1;
				//writeln(trajectories[agent_num]);
			}
			size_t max_sample_length = length_subtrajectory;

			//writeln("input to irl1:",trajectories[0]);

			double VI_threshold = 0.05; // 0.05 works 100% of times for forward RL in patrolling problem
			double grad_descent_threshold;
			double Ephi_thresh = 0.5;
	    	double gradient_descent_step_size = 0.25; // patrolling task
			int vi_duration_thresh_secs = 45; //30;
			int descent_duration_thresh_secs = 3*60;
			
			Ephi_thresh = 0.1;
			grad_descent_threshold = 0.0001; // patrolling problem
			//grad_descent_threshold = 0.00005;
	    	gradient_descent_step_size = 0.01; // reducing stepsize made it worse for patrolling domain

			if (mapToUse == "sorting") {
				// specific to suresh' mdp 
				grad_descent_threshold = 0.000005; 
				// specific to suresh' mdp 
				grad_descent_threshold = 0.0000001; 

				VI_threshold = 0.2; 

				Ephi_thresh = 0.1;
				//Ephi_thresh = 0.01; // Ephi thresh tighter than  0.1 didn't imporve results for pick-inspect-place

		    	gradient_descent_step_size = 0.0001; 
		    	// specific to suresh' mdp
		    	gradient_descent_step_size = 0.00001; 
			}

			// roll-pick-place 
			int nSamplesTrajSpace = 100;
			int restart_attempts = 3;

			MaxEntIrlZiebartApprox irl1 = new MaxEntIrlZiebartApprox(restart_attempts, 
				new TimedValueIteration(int.max,false,vi_duration_thresh_secs), model.S(), 
				nSamplesTrajSpace, grad_descent_threshold, VI_threshold);
			writeln("calling MaxEntIrlZiebartApprox.solve");
			policy = irl1.solve(model, initial, trajectories, max_sample_length, 
				lastWeights, val1, foundWeights1, featureExpecExpert, num_Trajsofar, 
				Ephi_thresh, gradient_descent_step_size, descent_duration_thresh_secs,
				trueWeights); 

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
			writeln("LBA ",irl1.computeLBA(cast(MapAgent)learnedPolicy,cast(MapAgent)truePolicy));
			LBA = irl1.computeLBA(cast(MapAgent)learnedPolicy,cast(MapAgent)truePolicy);

    } 
	
	writeln("BEGPARSING");
	foreach (State s; model.S()) {
		foreach (Action a, double chance; policy.actions(s)) {

            if (mapToUse == "sorting") {
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
            } else {
                BoydState ps = cast(BoydState)s;
                writeln( ps.getLocation(), " = ", a);
        	}

		}
	}
	
	writeln("ENDPOLICY");
    if (algorithm == "MAXENTZAPPROXI2RL") {

       writeln(foundWeightsGlbl);
       writeln(featureExpecExpert);
       writeln(num_Trajsofar);

    }


	debug {
		if (mapToUse == "sorting") {
			writeln("\nSimulation for 1:");
			sar [] traj;
			for(int i = 0; i < 2; i++) {
				traj = simulate(model, policy, initial, 50);
				//foreach (sar pair ; traj) {
				//	writeln(pair.s, " ", pair.a, " ", pair.r);
				//}
				//writeln(" ");
			}
            //Compute average EVD
            double avg_EVD1 = 0.0;
		    double trajval_trueweight, trajval_learnedweight;
		    double [][] fk_Xms_demonstration;

            MaxEntIrlZiebartExact irl = new MaxEntIrlZiebartExact(50, new ValueIteration(), model.S(), 50, .0005, .1);
            fk_Xms_demonstration.length = trajectoriesg.length;
	        fk_Xms_demonstration = irl.calc_feature_expectations_per_trajectory(model, trajectoriesg);
            foreach (j; 0 .. fk_Xms_demonstration.length) {
                trajval_learnedweight = dotProduct(foundWeightsGlbl,fk_Xms_demonstration[j]);
                trajval_trueweight = dotProduct(trueWeights,fk_Xms_demonstration[j]);
                avg_EVD1 += abs(trajval_trueweight - trajval_learnedweight)/(trajval_trueweight*cast(double)fk_Xms_demonstration.length);
            }
            writeln("\n EVD1:",avg_EVD1);
            //writeln("\n learned weights",foundWeightsGlbl);
            writeln("\n LBA:",LBA);

		}
	} 
	
	writeln("ENDPARSING");

	debug {
	    auto endttime = Clock.currTime();
	    auto duration = endttime - stattime;
	    writeln("Runtime Duration ==> ", duration);
	}
	
	return 0;
}
