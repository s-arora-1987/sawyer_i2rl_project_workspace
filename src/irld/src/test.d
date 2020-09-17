import mdp;
import std.stdio;
import irl;
import std.math;
import std.random;
import attacker;

version = boydright;

int main() {

version (boydright) {
	import boydright;
	
	// implement old mdp for performance compare
	// implement attacker mdp
	// create interface to python, needs to accept percepts and return a policy 
	
	// replace policy comparison with a comparison of the mu results, these should converge as policies become better
	
	double p_fail = 0.05;
	
	PatrolModel model = new PatrolModel(p_fail, null);
	
	PatrolReward reward = new PatrolReward(model);
	
	double [6] reward_weights = [1, -.1, 0, 0, 0, 0];
	
	reward.setParams(reward_weights);
	
	model.setReward(reward);
	
	model.setGamma(0.99);
	
	ValueIteration vi = new ValueIteration();
	
	double[State] V = vi.solve(model, .1);

//	foreach (State s ; model.S()) {
//		writeln(s, " ", V[s]);
		
//	}
	
	Agent opt_policy = vi.createPolicy(model, V);
	
	
	foreach (State s ; model.S()) {
		writeln(s, " ", opt_policy.actions(s));
	}


	model = new PatrolModel(p_fail, null);
	
	reward = new PatrolReward(model);

	reward.setParams(reward_weights);
	
	
	model.setReward(reward);
	
	model.setGamma(0.99);


	double[State] initial;
	foreach (State s ; model.S()) {
		initial[s] = 1.0;
		
	}
	Distr!State.normalize(initial);

	writeln("\nSimulation:");
	sar [] traj = simulate(model, opt_policy, initial, 50);
	foreach (sar SAR ; traj) {
		
		writeln(SAR.s, " ", SAR.a, " ", SAR.r);
	}
	
	sar[][] samples;
	for (int i = 0; i < 2; i ++) {
		samples ~= simulate(model, opt_policy, initial, 50);
	}
	
	
	
	
	double[State][] initials;
	initials.length = 2;
	
	foreach (int num, sar [] temp; samples) {
		foreach(sar SAR; temp) {
			initials[num][SAR.s] = 1.0;
		}
		Distr!State.normalize(initials[num]);
		
	}
	
	
//	MaxEntIrl irl = new MaxEntIrl(20,new ValueIteration(), 50, .001, .1);
//	NgProjIrl irl = new NgProjIrl(800,new ValueIteration(), 50, .1);
	
	
	sar [][] samples1;
	samples1 ~= samples[0];
	sar [][] samples2;
	samples2 ~= samples[1];
	
	Agent policy1 = new RandomAgent(model.A(null));
	Agent policy2 = new RandomAgent(model.A(null));
	
	int counter = 0;

	double [] featureExpectations1;
	double [] featureExpectations2;
	featureExpectations1.length =  (new PatrolReward(model)).dim();
	featureExpectations2.length =  (new PatrolReward(model)).dim();
	featureExpectations1[] = 0;
	featureExpectations2[] = 0;
	
	double [] lastWeights1 = new double[featureExpectations1.length];
	for (int i = 0; i < lastWeights1.length; i ++)
		lastWeights1[i] = uniform(0.0, 1.0);
		
	double [] lastWeights2 = new double[featureExpectations2.length];
	for (int i = 0; i < lastWeights2.length; i ++)
		lastWeights2[i] = uniform(0.0, 1.0);
			
	while (true) {
		
		double [] foundWeights;
		
		model = new PatrolModel(p_fail, null);
		
		reward = new PatrolReward(model);
		
		model.setReward(reward);
		
		model.setGamma(0.99);
		
		double val1;
//		MaxEntIrlBothPatrollers irl = new MaxEntIrlBothPatrollers(20,new ValueIteration(), 50, .1, .001, .1);
		MaxEntIrlBothPatrollers irl = new MaxEntIrlBothPatrollers(20,new ValueIteration(), 50, .1, .1, .1);
		
		policy1 = irl.solve(model, initials[1], samples1, lastWeights1, val1, foundWeights, policy2);
		
		model = new PatrolModel(p_fail, null);
		
		reward = new PatrolReward(model);
		
		model.setReward(reward);
		
		model.setGamma(0.99);
		
		MaxEntIrlBothPatrollers irl2 = new MaxEntIrlBothPatrollers(20,new ValueIteration(), 50, .1, .1, .1);
		
		double [] foundWeights2;
		double val2;
		policy2 = irl2.solve(model, initials[1], samples2, lastWeights2, val2, foundWeights2, policy1);
		
		
		
		// compare the feature expectactions from the new policies to the old ones
		/*
		sar [][] sample1;
		
		for (int i = 0; i < 1000; i ++)
			sample1 ~= simulate(model, policy1, initials[0], 100);
		
		double [] newFeat1 = irl.feature_expectations(model, sample1);

		sar [][] sample2;
		
		for (int i = 0; i < 1000; i ++)
			sample2 ~= simulate(model, policy2, initials[1], 100);

		double [] newFeat2 = irl.feature_expectations(model, sample2);
		
		double [] diff1;
		diff1.length = newFeat1.length;
		double [] diff2;
		diff2.length = newFeat2.length;
		
		diff1[] = newFeat1[] - featureExpectations1[];
		diff2[] = newFeat2[] - featureExpectations2[];
		
		featureExpectations1 = newFeat1;
		featureExpectations2 = newFeat2;
		
		double fnorm1 = l2norm(diff1);
		double fnorm2 = l2norm(diff2);
		*/
		
		double [] diff1;
		diff1.length = foundWeights.length;
		double [] diff2;
		diff2.length = foundWeights2.length;
		diff1[] = lastWeights1[] - foundWeights[];
		diff2[] = lastWeights2[] - foundWeights2[];
		
		
		double fnorm1 = l2norm(diff1);
		double fnorm2 = l2norm(diff2);
		
		lastWeights1 = foundWeights;
		lastWeights2 = foundWeights2;

		counter ++;
        writeln("Step ", counter, " ", fnorm1, " : ", fnorm2 );
		
		
		if (fnorm1 < .01*lastWeights1.length && fnorm2 < .01*lastWeights2.length)
			break;	
	}
} else {
	
	import boyd;
	
		
	// implement old mdp for performance compare
	// implement attacker mdp
	// create interface to python, needs to accept percepts and return a policy 
	
	double p_fail = 0.05;
	int longHallway = 10;
	int shortSides = 2;
	int patrolAreaSize = longHallway + shortSides + shortSides;
	
	int [] farness;
	farness.length = patrolAreaSize;
	
	for (int i = 0; i < patrolAreaSize; i ++) {
		int sum = 0;
		for (int j = 0; j < patrolAreaSize; j ++) {
			sum += abs(i - j);
		}
		farness[i] = sum; 
	}
	
	PatrolModel model = new PatrolModel(p_fail, longHallway, shortSides);
	
	PatrolReward reward = new PatrolReward(model, patrolAreaSize, farness);
	
	double [3] reward_weights = [1, 0, 0];
	
	reward.setParams(reward_weights);
	
	model.setReward(reward);
	
	model.setGamma(0.99);
	
	ValueIteration vi = new ValueIteration();
	
	double[State] V = vi.solve(model, .1);

//	foreach (State s ; model.S()) {
//		writeln(s, " ", V[s]);
		
//	}
	
	Agent opt_policy = vi.createPolicy(model, V);
	
	
	foreach (State s ; model.S()) {
		writeln(s, " ", opt_policy.actions(s));
	}


	model = new PatrolModel(p_fail, longHallway, shortSides);
	
	reward = new PatrolReward(model, patrolAreaSize, farness);

	reward.setParams(reward_weights);
	
	
	model.setReward(reward);
	
	model.setGamma(0.99);


	double[State] initial;
	foreach (State s ; model.S()) {
		initial[s] = 1.0;
		
	}
	Distr!State.normalize(initial);

	writeln("\nSimulation:");
	sar [] traj = simulate(model, opt_policy, initial, 50);
	foreach (sar SAR ; traj) {
		
		writeln(SAR.s, " ", SAR.a, " ", SAR.r);
	}
	
	sar[][] samples;
	for (int i = 0; i < 2; i ++) {
		samples ~= simulate(model, opt_policy, initial, 50);
	}
	
	
	double[State][] initials;
	initials.length = 2;
	
	foreach (int num, sar [] temp; samples) {
		foreach(sar SAR; temp) {
			initials[num][SAR.s] = 1.0;
		}
		Distr!State.normalize(initials[num]);
		
	}
	

	
	
//	MaxEntIrl irl = new MaxEntIrl(20,new ValueIteration(), 50, .001, .1);
//	NgProjIrl irl = new NgProjIrl(800,new ValueIteration(), 50, .1);
	
	
	sar [][] samples1;
	samples1 ~= samples[0];
	sar [][] samples2;
	samples2 ~= samples[1];
	
	Agent policy1 = new RandomAgent(model.A(null));
	Agent policy2 = new RandomAgent(model.A(null));
	
	int counter = 0;
	
	double [] featureExpectations1;
	double [] featureExpectations2;
	featureExpectations1.length =  (new PatrolReward(model, patrolAreaSize, farness)).dim();
	featureExpectations2.length =  (new PatrolReward(model, patrolAreaSize, farness)).dim();
	featureExpectations1[] = 0;
	featureExpectations2[] = 0;
	
	
	
	while (true) {
		
		double [] foundWeights;
		
		model = new PatrolModel(p_fail, longHallway, shortSides);
		
		reward = new PatrolReward(model, patrolAreaSize, farness);
		
		model.setReward(reward);
		
		model.setGamma(0.99);
		
		MaxEntIrlBothPatrollers irl = new MaxEntIrlBothPatrollers(20,new ValueIteration(), 50, .5, .1, 1);
		
		double val1;
		policy1 = irl.solve(model, initials[0], samples1, val1, foundWeights, policy2);
		
		
		model = new PatrolModel(p_fail, longHallway, shortSides);
		
		reward = new PatrolReward(model, patrolAreaSize, farness);
		
		model.setReward(reward);
		
		model.setGamma(0.99);
		
		MaxEntIrlBothPatrollers irl2 = new MaxEntIrlBothPatrollers(20,new ValueIteration(), 50, .5, .1, 1);
		
		double val2;
		policy2 = irl2.solve(model, initials[1], samples2, val2, foundWeights, policy1);
		
		counter ++;
 
 
		// compare the feature expectactions from the new policies to the old ones
		
		sar [][] sample1;
		
		for (int i = 0; i < 1000; i ++)
			sample1 ~= simulate(model, policy1, initials[0], 100);
		
		double [] newFeat1 = irl.feature_expectations(model, sample1);

		sar [][] sample2;
		
		for (int i = 0; i < 1000; i ++)
			sample2 ~= simulate(model, policy2, initials[1], 100);

		double [] newFeat2 = irl.feature_expectations(model, sample2);
		
		double [] diff1;
		diff1.length = newFeat1.length;
		double [] diff2;
		diff2.length = newFeat2.length;
		
		diff1[] = newFeat1[] - featureExpectations1[];
		diff2[] = newFeat2[] - featureExpectations2[];
		
		featureExpectations1 = newFeat1;
		featureExpectations2 = newFeat2;
		
		double fnorm1 = l2norm(diff1);
		double fnorm2 = l2norm(diff2);

        writeln("Step ", counter, " ", fnorm1, " : ", fnorm2 );
		
		
		if (fnorm1 < newFeat1.length && fnorm2 < newFeat2.length)
			break;
		
	}
}



	byte[][]	map = [[1, 1, 1, 1, 1, 1, 1, 1, 1], 
				     [1, 0, 1, 1, 1, 1, 0, 1, 0],
				     [1, 0, 0, 0, 1, 1, 1, 1, 1],
				     [1, 0, 0, 0, 0, 0, 1, 1, 1],
				     [1, 1, 1, 1, 1, 0, 0, 1, 0],
				     [0, 0, 0, 0, 1, 1, 1, 1, 0]];
    Agent policy = policy1;
	
	int badCount = 0;
	foreach (State s; model.S()) {
		if (opt_policy.actions(s) != policy.actions(s))
			badCount ++;
    	writeln(s, " - ", opt_policy.actions(s), " ", policy.actions(s));
    	
    }

    writeln();
    
    policy = policy2;
	
	int badCount2 = 0;
	foreach (State s; model.S()) {
		if (opt_policy.actions(s) != policy.actions(s))
			badCount2 ++;
    	writeln(s, " - ", opt_policy.actions(s), " ", policy.actions(s));
    	
    }
    
	
	writeln();
	writeln("Policy1 disagrees on: ", badCount, " out of ", model.S().length, " states");
	writeln("Policy2 disagrees on: ", badCount2, " out of ", model.S().length, " states");
	
	
	
		
		
	AttackerModel aModel = new AttackerModel(p_fail, map, new AttackerState([4, 2],0), 30);
	
	AttackerRewardPatrollerPolicyBoydRight aReward = new AttackerRewardPatrollerPolicyBoydRight(new AttackerState([4, 2],0), aModel, 10, -10, model, [policy1, policy2], [Distr!State.sample(initial), Distr!State.sample(initial)], [0,0], 3, 30, true);
	
    aReward.setParams([1]);

	aModel.setReward(aReward);
	
	aModel.setGamma(0.99);
	
	
	double[State] Va = vi.solve(aModel, .1);
	Agent attacker_policy = vi.createPolicy(aModel, Va);
		 
	
	return 0;
	
}
