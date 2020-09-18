import mdp;
import sortingMDP;
import std.stdio;
import std.math;
import std.string;
import std.format;

int main() {

	// receives weights and returns sorting policy
	// double [] params_manualTuning_pickinspectplace = [ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12]; 
	// double [] params_manualTuning_rolling = [0.15082956259426847, -0.075414781297134234, -0.11312217194570136, 
	// 0.19607843137254902, -0.030165912518853699, 0.0, 0.28355957767722473, -0.15082956259426847]; 
	// double [] params_neg_pickinpectplace = [ 0.0, 0.10, 0.22, 0.0, -0.12, 0.44, 0.0, -0.12]; 

	string buf, st;
	double [] reward_weights;
	//int dim = 8;
	//int dim = 9;
	//int dim = 10;
	int dim = 11;
	LinearReward reward;
	Model model;
	//model = new sortingModel(0.05,null);
	//model = new sortingModelbyPSuresh(0.05,null);
	model = new sortingModelbyPSuresh2(0.05,null);
	//model = new sortingModelbyPSuresh3(0.05,null);
	//model = new sortingModelbyPSuresh4(0.05,null);
	//model = new sortingModel2(0.05,null);

	//reward = new sortingReward2(model,dim); 
	//reward = new sortingReward3(model,dim); 
	//reward = new sortingReward4(model,dim); 
	//reward = new sortingReward5(model,dim); 
	//reward = new sortingReward6(model,dim); 
	reward = new sortingReward7(model,dim); 
	
    reward_weights = new double[dim];
	reward_weights[] = 0;

	buf = readln();
	formattedRead(buf, "[%s]", &st); 
    for (int j = 0; j < dim-1; j++) {
        formattedRead(st,"%s, ",&reward_weights[j]);
    } 
    formattedRead(st,"%s",&reward_weights[dim-1]);

	reward.setParams(reward_weights);
	debug {
		writeln("reward_weights ",reward_weights);

	}
    model.setReward(reward);
    model.setGamma(0.99);

	//ValueIteration vi = new ValueIteration();
	int vi_duration_thresh_secs = 30;
	TimedValueIteration vi = new TimedValueIteration(int.max,false,vi_duration_thresh_secs); 
	Agent opt_policy; 

    double vi_threshold;
    //vi_threshold = 0.2; 
    //vi_threshold = 0.1; // 100% success for simulating roll-pcik-place, but low for pick-inspect-place 
    vi_threshold = 0.15; // 80% success for simulating pick-inspect-place, but lower for roll-pick-place 
    vi_threshold = 0.25; 

    double[State] V; 
    V = vi.solve(model, vi_threshold); 
    opt_policy = vi.createPolicy(model, V); 

    double max_chance;
    Action action;
	foreach (State s; model.S()) {
		sortingState ss = cast(sortingState)s;
		max_chance = 0.0;
		action = null;
		foreach (Action act, double chance; opt_policy.actions(ss)) {
			if (chance > max_chance) {
				action = act;	
			} 
		}
        writeln("[",ss._onion_location,",", 
		ss._prediction,",",ss._EE_location, 
		",",ss._listIDs_status,"]", " = ", action); 
	}

	return 0;
}