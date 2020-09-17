import mdp;
import boydmdp;
import attacker;
import std.stdio;
import std.math;
import std.string;
import std.format;
import std.algorithm : reduce;

int main() {
	bool addDelay = false;
	State [] startingStates;
	int [] startingTimes;
	int detectDistance;
	int predictTime;
	double p_fail;
	int reward;
	double penalty;
	int interactionLength;
	double[State][Action][State][] T;
	
	string mapToUse;
	
	string buf;
	buf = readln();
	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	buf = readln();
	formattedRead(buf, "%s", &addDelay);

	buf = readln();
	while (buf.indexOf(";") >= 0) {
		string state = buf[0..buf.indexOf(";")];
		buf = buf[buf.indexOf(";") + 1 .. buf.length];
				
		int x;
		int y;
		int z;

		state = state[1..state.length];
		if (mapToUse== "largeGridPatrol") {
		    int cg;
            formattedRead(state, "%s, %s, %s], %s", &x, &y, &z, &cg);
            startingStates ~= new BoydExtendedState2([x, y, z],cg);
		} else {
            formattedRead(state, "%s, %s, %s]", &x, &y, &z);
            startingStates ~= new BoydState([x, y, z]);
		}

	}

	buf = readln();
	while (buf.indexOf(";") >= 0) {
		string time = buf[0..buf.indexOf(";")];
		buf = buf[buf.indexOf(";") + 1 .. buf.length];
		
		int t;
		formattedRead(time, "%s", &t);
		
		startingTimes ~= t;
	}

	int maxStartingTime = reduce!((a, b) {return a > b ? a : b;})(0, startingTimes);
	buf = readln();
	formattedRead(buf, "%s", &detectDistance);
	buf = readln();
	formattedRead(buf, "%s", &predictTime);
	buf = readln();
	formattedRead(buf, "%s", &p_fail);
	buf = readln();
	formattedRead(buf, "%s", &reward);
	buf = readln();
	formattedRead(buf, "%s", &penalty);
	buf = readln();
	formattedRead(buf, "%s", &interactionLength);

	int curT = 0;
	T.length = 1;
	
	size_t entryCount;
	while ((buf = readln()) != null) {
    	buf = strip(buf);
    	entryCount += 1;
    	
    	if (buf == "ENDT") {
    		curT ++;
    		T.length = T.length + 1;
    		if (T.length > startingStates.length)
    			break;
    		continue;	
    	}
    	
    	State s;
    	Action a;
    	State s_prime;
    	double p;
    	
    	p = parse_transitions2(mapToUse, buf, s, a, s_prime);
    	T[curT][s][a][s_prime] = p;
    	
    }
    T.length = T.length - 1;
   	
	int curPolicy = 0;
	Action[State][] policies;
	policies.length = 1;

	State bs2;
    while ((buf = readln()) != null) {
    	buf = strip(buf);
    	if (buf.length <= 1)
    		continue;
    
    	if (buf == "ENDPOLICY") {
    		curPolicy ++;
    		policies.length = policies.length + 1;
    		if (policies.length > startingStates.length)
    			break;    		
    	} else {
    		
			string action;
			
			int x;
			int y;
			int z;
			auto buf2 = buf[1..buf.indexOf("=") - 1];
            if (mapToUse== "largeGridPatrol") {
                int cg;
                formattedRead(buf2, "%s, %s, %s], %s", &x, &y, &z, &cg);
                bs2 = new BoydExtendedState2([x, y, z],cg);
            } else {
                formattedRead(buf2, "%s, %s, %s]", &x, &y, &z);
                bs2 = new BoydState([x, y, z]);
            }
			//formattedRead(buf2, "%s, %s, %s]", &x, &y, &z);

			action = buf[buf.indexOf("=") + 2 .. buf.length];
			
			Action a;
			if (action == "MoveForwardAction") {
				a = new MoveForwardAction();
			} else if (action == "TurnLeftAction") {
				a = new TurnLeftAction();
			} else if (action == "TurnRightAction") {
				a = new TurnRightAction();
			} else if (action == "TurnAroundAction") {
				a = new TurnAroundAction();
			} else {
				a = new StopAction();
			}
    		policies[curPolicy][bs2] = a;
    		
    	}
    }
    policies.length = policies.length - 1;

    double[Action][] equilibria = new double[Action][policies.length]; 

    if (addDelay) {
    	// parse game equilibria
    	int curE = 0;
    	
	    while ((buf = readln()) != null) {
	    	buf = strip(buf);
	    	if (buf.length <= 1)
	    		continue;
	    
	    	if (buf == "ENDE") {
	    		curE ++;
	    		if (curE >= equilibria.length)
	    			break;
	    			
	    	} else {
	    		
				string action;
				
				double value;
				
				action = buf[0..buf.indexOf("=") - 1];

				auto buf2 = buf[buf.indexOf("=") + 2 .. buf.length];
				formattedRead(buf2, "%s", &value);
				
				Action a;
				if (action == "MoveForwardAction") {
					a = new MoveForwardAction();
				} else if (action == "TurnAroundAction") {
					a = new TurnAroundAction();
				} else {
					a = new StopAction();
				}

	    		equilibria[curE][a] = value;
	    		
	    	}
	    }
    } else {
    	equilibria = null;
    }
    
    Agent [] agents;
    foreach (p; policies)
    	agents ~= new MapAgent(p);



	byte[][] map;
	LinearReward lreward;

	size_t projectionSamples = 0;

    if (mapToUse == "largeGridPatrol") {
        BoydExtendedModel2 [] pmodels;
        map = largeGridPatrollerMap();

        foreach (ref t; T) {
		 	pmodels ~= new BoydExtendedModel2(new BoydExtendedState2([-1,-1,-1],0), map, t, 1, &simplefeatures);

			int[State] distances;

			foreach (s; pmodels[$-1].S()) {
				BoydExtendedState2 bes = cast(BoydExtendedState2)s;
				distances[s] = abs(bes.getLocation()[0] - cast(int)((map.length - 1)/ 2) ) + bes.getLocation()[1];

			}

			lreward = new largeGridRewardGroupedFeatures2(pmodels[$-1]);
			pmodels[$-1].setReward(lreward);

			pmodels[$-1].setGamma(0.99);
		}

		if (entryCount > 0) {
			projectionSamples = pmodels[0].S().length * pmodels[0].A().length * predictTime / 3;

		}

		map = largeGridAttackerMap();

		AttackerExtendedModel aModel = new AttackerExtendedModel(p_fail, map, [new AttackerState([6, 1],2)], predictTime,
		cast(Model[])pmodels, agents, cast(State[])startingStates, startingTimes, detectDistance, interactionLength, equilibria, projectionSamples);

		AttackerRewardPatrollerProjectionBoyd aReward = new AttackerRewardPatrollerProjectionBoyd(aModel, reward, penalty);

	    aReward.setParams([1]);

		aModel.setReward(aReward);

		aModel.setGamma(.99);


		ValueIteration vi = new ValueIteration();

		double[State] V = vi.solve(aModel, 1);

		Agent a = vi.createPolicy(aModel, V);

		double[][State][] agentProbs = aModel.getProjection();

		foreach (i; 0..policies.length){
			foreach(t, stateprob; agentProbs) {
				foreach(state, prob; stateprob) {
					BoydState ps = cast(BoydState)state;
					writeln( "[", ps.getLocation(), ", ", cast(int)t - maxStartingTime, "] = ", prob[i]);

				}
			}
			writeln("ENDP");
		}
		writeln("ENDPROBS");

		foreach (State s; aModel.S()) {

			if (aModel.is_terminal(s)) {
				AttackerState ps = cast(AttackerState)s;
				writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = ", V[s] ," = null");
	//			writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = null");
			} else {
				foreach (Action act, double chance; a.actions(s)) {
					AttackerState ps = cast(AttackerState)s;
					writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = ", V[s] ," = ", act);
	//				writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = ", act);
				}
			}
		}

    }

	if (mapToUse == "boyd2") {
		//writeln(" reached if (mapToUse == ) {");

		map = boyd2PatrollerMap();
		
		BoydModel [] pmodels;
		foreach (ref t; T) {
		 	pmodels ~= new BoydModel(null, map, t, 1, &simplefeatures);

			int[State] distances;
			
			foreach (s; pmodels[$-1].S()) {
				BoydState bs = cast(BoydState)s;
				distances[s] = abs(bs.getLocation()[0] - cast(int)((map.length - 1)/ 2) ) + bs.getLocation()[1];
				
			}
			
			lreward = new Boyd2Reward(pmodels[$-1], distances);
			pmodels[$-1].setReward(lreward);
		
			pmodels[$-1].setGamma(0.99);
		}

		if (entryCount > 0) {
			projectionSamples = pmodels[0].S().length * pmodels[0].A().length * predictTime / 3;
			
		}

		map = boyd2AttackerMap();				     

		AttackerModel aModel = new AttackerModel(p_fail, map, [new AttackerState([9, 1],2)], predictTime, cast(Model[])pmodels, agents, cast(State[])startingStates, startingTimes, detectDistance, interactionLength, equilibria, projectionSamples);
	
		AttackerRewardPatrollerProjectionBoyd aReward = new AttackerRewardPatrollerProjectionBoyd(aModel, reward, penalty);
		
	    aReward.setParams([1]);
	
		aModel.setReward(aReward);
		
		aModel.setGamma(.99);
		
		
		ValueIteration vi = new ValueIteration();
		
		double[State] V = vi.solve(aModel, 1);
		
		Agent a = vi.createPolicy(aModel, V);
		
		double[][State][] agentProbs = aModel.getProjection();
		
		foreach (i; 0..policies.length){
			foreach(t, stateprob; agentProbs) {
				foreach(state, prob; stateprob) {
					BoydState ps = cast(BoydState)state;
					writeln( "[", ps.getLocation(), ", ", cast(int)t - maxStartingTime, "] = ", prob[i]);
					
				}
			}
			writeln("ENDP");
		}			
		writeln("ENDPROBS");
		
		foreach (State s; aModel.S()) {
			
			if (aModel.is_terminal(s)) {
				AttackerState ps = cast(AttackerState)s;
				writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = ", V[s] ," = null");
	//			writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = null");
			} else {
				foreach (Action act, double chance; a.actions(s)) {
					AttackerState ps = cast(AttackerState)s;
					writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = ", V[s] ," = ", act);
	//				writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = ", act);
				}
			}
		}
		

	}

	if (mapToUse == "boydright" || mapToUse == "boydright2") {
		BoydModel [] pmodels;

		if (mapToUse == "boydright") {
			map = boydrightPatrollerMap();

			foreach (ref t; T) {
				pmodels ~= new BoydModel(null, map, t, 1, &simplefeatures);

				lreward = new BoydRightReward(pmodels[$-1]);
				pmodels[$-1].setReward(lreward);

				pmodels[$-1].setGamma(0.99);
			}

			map = boydrightAttackerMap();
		} else {
			map = boydright2PatrollerMap();
			foreach (ref t; T) {
				pmodels ~= new BoydModel(null, map, t, 1, &simplefeatures);

				lreward = new BoydRight2Reward(pmodels[$-1]);
				pmodels[$-1].setReward(lreward);

				pmodels[$-1].setGamma(0.99);
			}

			map = boydright2AttackerMap();

		}


		if (entryCount > 0) {
			projectionSamples = pmodels[0].S().length * pmodels[0].A().length;
			
		}
		debug {
			writeln("Projections to sample: ", projectionSamples);
			
		}
		AttackerModel aModel = new AttackerModel(p_fail, map, [new AttackerState([1, 0],0), new AttackerState([13, 13],0)], predictTime, cast(Model[])pmodels, agents, cast(State[])startingStates, startingTimes, detectDistance, interactionLength, equilibria, projectionSamples);
	
		
		AttackerRewardPatrollerProjectionBoyd aReward = new AttackerRewardPatrollerProjectionBoyd(aModel, reward, penalty);
		
	    aReward.setParams([1]);
	
		aModel.setReward(aReward);
		
		aModel.setGamma(0.95);
		
		
		ValueIteration vi = new ValueIteration();
	//	PolicyIteration vi = new PolicyIteration();
		
		double[State] V = vi.solve(aModel, .5);	
		
		
		Agent a = vi.createPolicy(aModel, V);
		
		double[][State][] agentProbs = aModel.getProjection();
		
		foreach (i; 0..policies.length){
			foreach(t, stateprob; agentProbs) {
				foreach(state, prob; stateprob) {
					BoydState ps = cast(BoydState)state;
					writeln( "[", ps.getLocation(), ", ", cast(int)t - maxStartingTime, "] = ", prob[i]);
					
				}
			}
			writeln("ENDP");
		}
		writeln("ENDPROBS");
		
		foreach (State s; aModel.S()) {
			
			if (aModel.is_terminal(s)) {
				AttackerState ps = cast(AttackerState)s;
				writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = ", V[s] ," = null");
			} else {
				foreach (Action act, double chance; a.actions(s)) {
					AttackerState ps = cast(AttackerState)s;
					writeln( "[", ps.getLocation(), ", ", ps.getOrientation(), ", ", ps.getTime(), "] = ", V[s] ," = ", act);
				}
			}
		}
	}
	
	 
	return 0;	
	
}
