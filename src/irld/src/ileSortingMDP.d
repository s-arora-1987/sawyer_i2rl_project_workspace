import mdp;
import std.stdio;
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
import sortingMDP;
import std.bitmanip;
import std.datetime;

void main(){

	// Inputs: Learned WEights, number of demonstrated behaviors, True weights for each 
	// returns ILE values computed for single learned weights vector applied to sorting MDP model
	// Do Not extend code to accommodate multiple learned weight vectors! 

    //writeln("entered main");
    auto stattime = Clock.currTime();
	string buf;
    string buf2, st;
    int dim;
    double [] WeightsIRL;
    int num_trueWeights;
    LinearReward opt_reward;
    LinearReward reward;
    double VI_threshold;
    double saFreqThresh; // original value 0.0001 took too long

    Model model = new sortingModel(0.05,null);
    dim = 8;
    opt_reward = new sortingReward2(model,dim);         
    reward = new sortingReward2(model,dim);         
    model.setGamma(0.99); 

    buf = readln(); 
    formattedRead(buf,"[%s]",&buf2);
    WeightsIRL.length = reward.dim();
    for (int j = 0; j < reward.dim()-1; j++) {
        formattedRead(buf2,"%s, ",&WeightsIRL[j]); 
    } 
    formattedRead(buf2,"%s",&WeightsIRL[reward.dim()-1]);
    //writeln("finished reading WeightsIRL ",WeightsIRL);

    buf = readln();
    formattedRead(buf, "%s", &num_trueWeights);
    //writeln("num_trueWeights ",num_trueWeights);
    
    double [][] trueWeights;
    trueWeights.length = num_trueWeights;

    for (int i = 0; i < num_trueWeights; i++) {
        trueWeights[i].length = reward.dim();
        trueWeights[i][] = 0.0;
    }

    for (int i = 0; i < num_trueWeights; i++) {
        buf2 = readln();
        formattedRead(buf2,"[%s]",&st);
        for (int j = 0; j < reward.dim()-1; j++) {
            formattedRead(st,"%s, ",&trueWeights[i][j]);
        }
        formattedRead(st,"%s",&trueWeights[i][reward.dim()-1]);
        //writeln("trueWeights[i] ",trueWeights[i]);
    } 

    buf = readln();
    formattedRead(buf, "%s", &VI_threshold);
    buf = readln();
    formattedRead(buf, "%s", &saFreqThresh);

	ValueIteration vi = new ValueIteration();
	double[State] V;
	Agent opt_policy;
	double[State] initial;
    foreach (s; model.S()) {
        initial[s] = 1.0;
    }
    Distr!State.normalize(initial);

	Agent learned_policy;
    reward.setParams(WeightsIRL);
    model.setReward(reward);
    V = vi.solve(model, VI_threshold);
    learned_policy = vi.createPolicy(model, V);
    auto learned_policy_optrew_value = policyValueOptRewardSingleReward(model, learned_policy, opt_reward, initial, saFreqThresh);
    writeln("computed learned_policy_optrew_value vec ");
    double[] diff;
    diff.length = learned_policy_optrew_value.length;

    // for each demonstratted behavior
    for (int i = 0; i < num_trueWeights; i++) {

        opt_reward.setParams(trueWeights[i]);
	    model.setReward(opt_reward);
	    V = vi.solve(model, VI_threshold);
	    opt_policy = vi.createPolicy(model, V);
	    auto opt_policy_optrew_value = policyValueOptRewardSingleReward(model, opt_policy, opt_reward, initial, saFreqThresh);

        if (opt_policy_optrew_value.length == learned_policy_optrew_value.length) {

	        diff[] = opt_policy_optrew_value[] - learned_policy_optrew_value[];
	    	double denom = l1norm(opt_policy_optrew_value);
	        double ile = l1norm(diff)/denom;// ILE*scaling of 10
	        ile= 100.0000*ile;
	        writeln(ile);

        } else {
            debug {
                writeln(" value array length mismatch ");
            }

        }

	}    



}

double [] policyValueOptRewardSingleReward(Model model, Agent policy, LinearReward true_reward, double[State] initial_states, double saFreqThresh) {

	// the s-a value given a policy is the state visitation frequency times the reward for the state
	double [] returnval;

	auto sa_freq = calcStateActionFreqSortingMDP(policy, initial_states, model, saFreqThresh);

	double [] val;

	foreach (sa, freq; sa_freq) {
		val ~= freq * true_reward.reward(sa.s, sa.a);
	}
    returnval.length = val.length;
	returnval = val[];

	return returnval;

}

// WARNING, only works for deterministic policies!
// copy of mdp.calcStateActionFreq with input threshold
double[StateAction] calcStateActionFreqSortingMDP(Agent policy, double[State] initial, Model m, double saFreqThresh) {
	
	double[StateAction] returnval;

	writeln("calcStateActionFreqSortingMDP: at calcStateFreqExact ");	
	double[State] tempState = calcStateFreqExact(policy, initial, m, saFreqThresh);

	writeln("calcStateActionFreqSortingMDP: after calcStateFreqExact ");	
	
	foreach (s; m.S()) {
		foreach(a, p; policy.actions(s)) {

			returnval[new StateAction(s,a)] = tempState[s];
		}
	}
	
	
	return returnval;

}