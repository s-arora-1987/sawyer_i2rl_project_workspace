import mdp;
import boydmdp;
import std.stdio;
import std.math;
import std.string;
import std.format;

int main() {
	
	string mapToUse;
	double[State][Action][State] T;
	
	string buf, st;
	buf = readln();
	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);

	while ((buf = readln()) != null) {
    	buf = strip(buf);
    	
    	if (buf == "ENDT") {
    		break;
    	}
    	
    	State s;
    	Action a;
    	State s_prime;
    	double p;

        p = parse_transitions(mapToUse, buf, s, a, s_prime);
	    debug {
	    	//writeln(" transition ",s, a, s_prime,p,". "); 
	    }

    	T[s][a][s_prime] = p;
    	
    }

 //   debug {
 //   	writeln("transition parsed."); 
 //   }

	double [] reward_weights;
	int dim = 6;
    reward_weights = new double[dim];
	reward_weights[] = 0;

	buf = readln();
	formattedRead(buf, "[%s]", &st); 
    for (int j = 0; j < dim-1; j++) {
        formattedRead(st,"%s, ",&reward_weights[j]);
    } 
    formattedRead(st,"%s",&reward_weights[dim-1]);

 //   debug {
 //   	writeln("reward_weights ",reward_weights); 
 //   }

	byte[][] map;
	LinearReward reward;
	Model model;
	ValueIteration vi = new ValueIteration();
	Agent a;

	if (mapToUse == "boyd2") {
	    debug {
	    	//writeln("starting to solve mdp."); 
	    }

	    // BoydModel model; 
		map = boyd2PatrollerMap();
		model = new BoydModel(null, map, T, 1, &simplefeatures);
		//reward = new Boyd2RewardGroupedFeaturesTestMTIRL(model); 
		reward = new Boyd2RewardGroupedFeatures(model);
		//double [6] reward_weights = [0, 0, 0, 0, 0, 1];
		reward.setParams(reward_weights);
        model.setReward(reward);
        model.setGamma(0.99);

        //writeln("starting V = vi.solve");
        double[State] V = vi.solve(model, .1);

        debug {
                //writeln(V);
        }
        a = vi.createPolicy(model, V);

	} 
	
	foreach (State s; model.S()) {
		foreach (Action act, double chance; a.actions(s)) {
            BoydState ps = cast(BoydState)s;
            writeln(ps.getLocation(), " = ", act);

		}
	}
	
	 
	return 0;	
	
}

