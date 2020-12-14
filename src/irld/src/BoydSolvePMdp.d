import mdp;
import boydmdp;
import std.stdio;
import std.math;
import std.string;
import std.format;

int main() {
	
	string mapToUse;
	double[State][Action][State] T;
	
	string buf;
	int useRegions;
	buf = readln();
	formattedRead(buf, "%s", &mapToUse);
	
	mapToUse = strip(mapToUse);
	buf = readln();
	formattedRead(buf, "%s", &useRegions);

	while ((buf = readln()) != null) {
    	buf = strip(buf);
    	
    	if (buf == "ENDT") {
    		break;
    	}
    	
    	State s;
    	Action a;
    	State s_prime;
    	double p;

    	if (mapToUse == "largeGridPatrol") {
            p = parse_transitions2(mapToUse, buf, s, a, s_prime);
    	} else {
            p = parse_transitions(mapToUse, buf, s, a, s_prime);
    	}

    	T[s][a][s_prime] = p;
    	
    }

	byte[][] map;
	LinearReward reward;
	Model model;
	ValueIteration vi = new ValueIteration();
	Agent a;

	if (mapToUse == "boyd2") {

	    // BoydModel model;
		map = boyd2PatrollerMap();
		model = new BoydModel(null, map, T, 1, &simplefeatures);
		reward = new Boyd2RewardGroupedFeatures(model);
		double [6] reward_weights = [1, 0, 0, 0, 0.75, 0];
        // [0, 0, 0, 0, 0, 1];//[0.6, 0, 0.4, 0, 0, 0];//[1, 0, 0, 0, 0.75, 0];
        // reward_weights = [0.465577137, 0.063009014, 0.063009014, 0.063009014, 0.282386808, 0.063009014]; // softmax of the above
		reward.setParams(reward_weights);
        model.setReward(reward);
        model.setGamma(0.99); 

        //writeln("starting V = vi.solve");
        double[State] V = vi.solve(model, .05);


        debug {
                writeln(V);
        }
        a = vi.createPolicy(model, V);

	} else {
		
		if (mapToUse == "largeGridPatrol") {

		    map = largeGridPatrollerMap();
			//State terminal = cast(State)();
			//model = new BoydModel(new BoydState([6, 8, 0]), map, T, 1, &simplefeatures);

			model = new BoydExtendedModel2(new BoydExtendedState2([-1,-1,-1],0), map, T, 1, &simplefeatures);
			//model = new BoydExtendedModel(new BoydExtendedState(null,null,0), map, T, 1, &simplefeatures);
            //foreach (bess; model.S()){
             //   if ((cast(BoydExtendedState2)bess).location == [6,0,0]){
             //       writeln("found",bess);
             //   }
            //}

			double [] reward_weights;
			//reward = new largeGridRewardGroupedFeatures(model);
            if (useRegions == 1) {
                reward = new largeGridRewardGroupedFeatures3(model);
            }
            else {
                //reward = new largeGridRewardGroupedFeatures(model);
                reward = new largeGridRewardGroupedFeatures2(model);
                //writeln("largeGridRewardGroupedFeatures");
            }
			reward_weights = new double[reward.dim()];
			// incentive for moving to different cell
			// incentive for coming to junction and  turning left, and moving forward
			// incentive for coming to junction without a left or forward and turning right
			// and moving forward
			// incentives for turning around in goals
			// same for goal regions

            //reward_weights[0 .. 6] = [0, 1, 0, 1, 1, 0];
            // incentive for changing goal
            //for (int number = 6; number < 10; ++number) {
            //    reward_weights[number] = 1;
            //}

            //reward_weights = [1,1,1,1];
            reward_weights = [1,1,1,1,1,1,1,1];
            // incentive for changing goal
            //for (int number = 0; number < 2; ++number) {
            //    reward_weights[number] = 1;
            //}

            //BoydState bs = new BoydState;
            //State s = model.S()[0];
            //bs = cast(BoydState)s;
            //writeln("",bs.location);
            //writeln("",(cast(BoydState)s).location[0]);
            //writeln("",model.states[0].location[0]);

            /*foreach(i, st; model.states) {
                //BoydState s = cast(BoydState)st;
                if (st.location[0] == goal[0] && st.location[1] == goal[1] && st.location[2] == goal[2])
                    reward_weights[i] = 1;
            }
            */
			//reward_weights = [1, 0, 0, 0, 0.75, 0, 1.5];//[1, -1, -1, -1, 0.5, -1];
			reward.setParams(reward_weights);
            model.setReward(reward);
            model.setGamma(0.995);

            //writeln("starting V = vi.solve");
            double[State] V = vi.solve(model, .01);
            writeln(V);
            debug {
                writeln(V);
            }
            //writeln("a = vi.createPolicy");
            a = vi.createPolicy(model, V);
            //writeln("after a = vi.createPolicy");

		} else {

            if (mapToUse == "boydright2") {

                map = boydright2PatrollerMap();
                model = new BoydModel(null, map, T, 1, &simplefeatures);
                reward = new BoydRight2Reward(model);
                double [7] reward_weights = [1, 0, 0, 0, 0, 0, 0.5];//[1, -1, -1, -1, 0.5, -1];
                reward.setParams(reward_weights);
                model.setReward(reward);
                model.setGamma(0.99);

                // writeln("starting V = vi.solve");
                double[State] V = vi.solve(model, .1);
                debug {
                    writeln(V);
                }
                // writeln("a = vi.createPolicy");
                a = vi.createPolicy(model, V);

			} else {
                //BoydModel model;
                map = boydrightPatrollerMap();
                model = new BoydModel(null, map, T, 1, &simplefeatures);
                reward = new BoydRightReward(model);
                double [5] reward_weights = [1, -1, .1, 0, 0];
                reward.setParams(reward_weights);

                model.setReward(reward);
                model.setGamma(0.99);

                //writeln("starting V = vi.solve");
                double[State] V = vi.solve(model, .1);
                debug {
                        writeln(V);
                }
                a = vi.createPolicy(model, V);
            }
		}
	}
	
    writeln("BEGPARSING");
	foreach (State s; model.S()) {
		foreach (Action act, double chance; a.actions(s)) {
		    if (mapToUse == "largeGridPatrol" || mapToUse == "reducedGridPatrol") {

                //BoydExtendedState ps = cast(BoydExtendedState)s;
                BoydExtendedState2 ps = cast(BoydExtendedState2)s;
                //writeln(ps.getLocation(),";",ps.getAction(), " = ", act);
                //writeln(ps.getLocation(),";",ps.getLastLocation(), " = ", act);
                writeln(ps.getLocation(),";",ps.getCurrentGoal(), " = ", act);
                //writeln(ps.getLocation(),";",ps.getLastLocation(),";",ps.getCurrentGoal(), " = ", act);

		    } else {

                BoydState ps = cast(BoydState)s;
                writeln(ps.getLocation(), " = ", act);

			}
		}
	}
	
    writeln("ENDPARSING");
	 
	return 0;	
	
}
