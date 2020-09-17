import mdp;
import boydmdp;
import irl;
import irlgraveyard;
import std.stdio;
import std.format;
import std.string;
import std.math;
import std.random;
import std.algorithm : minPos;


int main() {
	
	// Read in stdin to get settings and trajectory
	
	bool addDelay = false;
	sar [][][] SAR;
	int interactionLength;
	int ne;
	double statesVisible;
	double[State][Action][State][] T;

	string mapToUse;
	string buf;
	buf = readln();
	
	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	buf = readln();	
	formattedRead(buf, "%s", &addDelay);
	buf = readln();
	string algorithm;
	
	formattedRead(buf, "%s", &algorithm);
	algorithm = strip(algorithm);

	buf = readln();
	formattedRead(buf, "%s", &ne);

	buf = readln();
	formattedRead(buf, "%s", &interactionLength);

	buf = readln();
	formattedRead(buf, "%s", &statesVisible);
	
	int curT = 0;
	T.length = 1;	
	while ((buf = readln()) != null) {
    	buf = strip(buf);
    	
    	if (buf == "ENDT") {
    		curT ++;
    		T.length = T.length + 1;
    		if (T.length > 2)
    			break;
    		continue;	
    	}
    	
    	State s;
    	Action a;
    	State s_prime;
    	double p;
    	
    	p = parse_transitions(buf, s, a, s_prime);
    	T[curT][s][a][s_prime] = p;
    	
    }
	T.length = T.length - 1;
	
	int curPatroller = 0;
	SAR.length = 1;
	
    while ((buf = readln()) != null) {
    	buf = strip(buf);
    
    	if (buf == "ENDTRAJ") {
    		// update each observed trajectory's probs
    		foreach(traj; SAR[curPatroller]) {
    			foreach (entry; traj) {
    				entry.p = 1.0 / SAR[curPatroller].length;
    			}
    		}
    		
    		curPatroller ++;
    		SAR.length = SAR.length + 1;
    		
    		if (SAR.length > 2)
    			break;
    		
    	} else {
    		sar [] newtraj;
    		
    		while (buf.indexOf(";") >= 0) {
    			string percept = buf[0..buf.indexOf(";")];
    			buf = buf[buf.indexOf(";") + 1 .. buf.length];
    			
    			if (percept.length == 0) {
    				newtraj ~= sar(null, new NullAction(), 0.0);
    				continue;
    			}
    			
    			string state;
    			
   				formattedRead(percept, "%s", &state);
   				
   				int x;
   				int y;
   				int z;
   				
				state = state[1..state.length];
   				formattedRead(state, "%s, %s, %s]", &x, &y, &z);
   				
   				
   				newtraj ~= sar(new BoydState([x, y, z]), new NullAction(), 1.0);

    		}
    		
    		SAR[curPatroller] ~= newtraj;
    		
    	}
    	
    }
	SAR.length = SAR.length - 1;
	
	Agent [] policy_priors;
		
	double[Action][State] temp_policy;
	
    while ((buf = readln()) != null) {
    	buf = strip(buf);
    
    	if (buf == "ENDPOLICY") {
    		policy_priors ~= new StochasticAgent(temp_policy);
    		temp_policy = null;
    		
    		if (policy_priors.length == 2)
    			break;
    	} else {
    		
			string percept = buf;
			
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
			
			
			temp_policy[new BoydState([x, y, z])][a] = p;
    	}
    	
    }
    
    double[Action][][] NEs = genEquilibria();
    
    double [] NE_prior;
    NE_prior.length = NEs.length;
    
    while ((buf = readln()) != null) {
    	buf = strip(buf);

		string percept = buf;
		
		string action1;
		string action2;
		double p;
		
		formattedRead(percept, "%s:%s:%s", &action1, &action2, &p);
				
		Action a1;
		if (action1 == "MoveForwardAction") {
			a1 = new MoveForwardAction();
		} else if (action1 == "StopAction") {
			a1 = new StopAction();
		} else if (action1 == "TurnLeftAction") {
			a1 = new TurnLeftAction();
		} else if (action1 == "TurnAroundAction") {
			a1 = new TurnAroundAction();
		} else {
			a1 = new TurnRightAction();
		}
		
		Action a2;
		if (action2 == "MoveForwardAction") {
			a2 = new MoveForwardAction();
		} else if (action2 == "StopAction") {
			a2 = new StopAction();
		} else if (action2 == "TurnLeftAction") {
			a2 = new TurnLeftAction();
		} else if (action2 == "TurnAroundAction") {
			a2 = new TurnAroundAction();
		} else {
			a2 = new TurnRightAction();
		}
		
		// gah, this is wrong!
		foreach (i, equilibrium; NEs) {
			if (a1 in equilibrium[0] && a2 in equilibrium[1]) {
				NE_prior[i] = p;
				break;
			}
			
		}
    	
    } 
    
    
    
//	double p_fail = 0.33;
//	double p_fail = 0;


	byte[][] map;
	LinearReward reward;
	 
	if (mapToUse == "boyd2") {
		map = boyd2PatrollerMap();
		
	} else {
		map = boydrightPatrollerMap();
		
	}
	
	BoydModel [] models;
	models ~= new BoydModel(null, map, T[0], 1, &simplefeatures);
	models ~= new BoydModel(null, map, T[1], 1, &simplefeatures);

	
	State [] observableStatesList;

	double [] reward_weights;
	foreach(i;0..models.length) {
		if (mapToUse == "boyd2") {
					
	/*		if (statesVisible >= 1 && !addDelay) {
				reward = new Boyd2Reward(model, distances);
				reward_weights = new double[reward.dim()];
			} else { */
				reward = new Boyd2RewardGroupedFeatures(models[i]);
				reward_weights = new double[reward.dim()];
				reward_weights[] = 0;
	//		}
			reward.setParams(reward_weights);
			
			// build the observed states list
			
			foreach (s; models[i].S()) {
				if (boyd2isvisible(cast(BoydState)s, statesVisible))
					observableStatesList ~= s;
			}
			
		} else {
			reward = new BoydRightReward(models[i]);		     
			reward_weights = new double[reward.dim()];
			reward_weights[] = 0;
		
			reward.setParams(reward_weights);
	
			foreach (s; models[i].S()) {
				if (boydrightisvisible(cast(BoydState)s, statesVisible)) 
					observableStatesList ~= s;
				
			}
		}			
		models[i].setReward(reward);
		
		models[i].setGamma(0.95);
	}

	double[State][] initials;
	initials.length = SAR.length;
	
	
	// Use an initial state distribution that matches the observed trajectories
	foreach (int num, sar [][] temp; SAR) {
		bool initialized = false;
		
		foreach(sar [] SAR2; temp) {
			foreach (SAR3; SAR2) {
				if (SAR3.s !is null) {
					initials[num][SAR3.s] = 1.0;
					initialized = true;
					break;
				}
			}
		}
		if (! initialized) {
			foreach (s; models[num].S()) {
				initials[num][s] = 1.0;
			}
		}
		Distr!State.normalize(initials[num]);
		
	} 
	
	/*
	foreach (int num, sar [][] temp; SAR) {
		foreach (s; models[0].S()) {
			initials[num][s] = 1.0;
		}
		Distr!State.normalize(initials[num]);
	}*/
	
	
	
//	sar [][] samples1 = naiveInterpolateTraj(SAR[0], model, observedStatesList[0]);
//	sar [][] samples2 = naiveInterpolateTraj(SAR[1], model, observedStatesList[1]);

	sar [][] samples1 = SAR[0];
	sar [][] samples2 = SAR[1];
	
//	sar [][] samples1 = insertTurnArounds(SAR[0], model, distances, new TurnAroundAction());
//	sar [][] samples2 = insertTurnArounds(SAR[1], model, distances, new TurnAroundAction());
	
	Agent policy1 = new RandomAgent(models[0].A(null));
	Agent policy2 = new RandomAgent(models[1].A(null));
	
	int counter = 0;
	
	double [] featureExpectations1;
	double [] featureExpectations2;
	featureExpectations1.length = reward.dim();
	featureExpectations2.length = reward.dim();
	featureExpectations1[] = 0;
	featureExpectations2[] = 0;
	
	double [] lastWeights1 = new double[reward_weights.length];
	for (int i = 0; i < lastWeights1.length; i ++)
		lastWeights1[i] = uniform(.01, .1);

	double [] lastWeights2 = new double[reward_weights.length];
	for (int i = 0; i < lastWeights2.length; i ++)
		lastWeights2[i] = uniform(.01, .1);

	
//	lastWeights1[] = -10.00001;
//	lastWeights2[] = -10.00001;
	
/*	lastWeights1[0] = 1;
	lastWeights1[1] = 0.55;
	lastWeights2[0] = 1;
	lastWeights2[1] = 0.55;*/

//	lastWeights1 = [1, -1, .1, 0, 0];
//	lastWeights2 = [1, -1, .1, 0, 0];


	
	double [][] foundWeights;
	double [][] lastWeights;

	
	lastWeights ~= lastWeights1;
	lastWeights ~= lastWeights2;
	
	double val;

	
	auto interaction_delegate = delegate bool(State s1, State s2) {
		return s1.samePlaceAs(s2);
	};
	
		
	MaxEntIrlApproxEMPartialVisibilityMultipleAgentsUnknownNE irl = new MaxEntIrlApproxEMPartialVisibilityMultipleAgentsUnknownNE(100, new ValueIteration(), 150, .1, .1, .09, observableStatesList, interaction_delegate);
	
	sar [][][] trajectories;
	trajectories.length = 2;
	trajectories[0] = samples1;
	trajectories[1] = samples2;
	
	size_t [] traj_lengths;
	traj_lengths ~= samples1[0].length;
	traj_lengths ~= samples2[0].length;

	double [] found_NE_weights;

	
	Agent [] policies = irl.solve2(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, NEs, interactionLength, policy_priors, NE_prior, val, foundWeights, found_NE_weights);
	
	policy1 = policies[0];
	policy2 = policies[1];
	
	
	size_t chosenEquilibrium =  (minPos!("a > b")(found_NE_weights)).ptr - &(found_NE_weights[0]);

	debug {
	writeln(val, " ", foundWeights);
	}
	
	foreach (State s; models[0].S()) {
		foreach (Action a, double chance; policy1.actions(s)) {
			BoydState ps = cast(BoydState)s;
			writeln( ps.getLocation(), " = ", a);
		}
	}
	
	writeln("ENDPOLICY");

	foreach (State s; models[0].S()) {
		foreach (Action a, double chance; policy2.actions(s)) {
			BoydState ps = cast(BoydState)s;
			writeln( ps.getLocation(), " = ", a);
		}
	}	
	writeln("ENDPOLICY");
	 
	double[Action][] equilibria = NEs[chosenEquilibrium];

	foreach (key, value; equilibria[0]) {
		writeln(key, " = ", value);
	}
	writeln("ENDE");
	 
	foreach (key, value; equilibria[1]) {
		writeln(key, " = ", value); 
	}	 
	writeln("ENDE");
	
	return 0;
}
