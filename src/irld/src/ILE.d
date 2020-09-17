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
import boydmdp;
import std.bitmanip;

void main(){

	string mapToUse;
	double[State][Action][State] T;

	string buf;
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

    	if (mapToUse == "largeGridPatrol") {
            p = parse_transitions2(mapToUse, buf, s, a, s_prime);
    	} else {
            p = parse_transitions(mapToUse, buf, s, a, s_prime);
    	}
    	T[s][a][s_prime] = p;

    }

    //writeln("reading trans success");
    //change this acc to choice of reward model
    string [2] st;
    buf = readln();
    formattedRead(buf, "[[%s], [%s]]", &st[0], &st[1]);
    double [][] WeightsIRL;
    WeightsIRL.length = 2;
    int dim;

    for (int i = 0; i < 2; i++)
    {
        if (mapToUse == "boyd2") {
            dim = 6;

        } else {

            if (mapToUse == "boydright") {
                dim = 5;

            } else {

                if (mapToUse == "boydright2") {
                    dim = 7;

                } else {
                    dim = 8;

                }

            }
        }

        WeightsIRL[i].length = dim;
        WeightsIRL[i][] = 0.0;
    }

    // change this acc to choice of reward model
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < dim-1; j++) {
            formattedRead(st[i],"%s, ",&WeightsIRL[i][j]);
        }

        formattedRead(st[i],"%s",&WeightsIRL[i][dim-1]);
    }

    //if (mapToUse == "largeGridPatrol") {
    //    WeightsIRL=[[0.0,0.0],[0.0,0.0]];
    //    for (int i = 0; i < 2; i++) {
    //        formattedRead(st[i], "%s, %s", &WeightsIRL[i][0], &WeightsIRL[i][1]);
    //    }
    //} else {
    //    WeightsIRL=[[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]];
    //    for (int i = 0; i < 2; i++) {
    //        formattedRead(st[i], "%s, %s, %s, %s, %s, %s", &WeightsIRL[i][0], &WeightsIRL[i][1], &WeightsIRL[i][2], &WeightsIRL[i][3], &WeightsIRL[i][4], &WeightsIRL[i][5]);
    //    }
    //}
    //writeln("reading weights success");

	byte[][] map;
	LinearReward opt_reward;
    Model model;

    if (mapToUse == "largeGridPatrol") {

        map = largeGridPatrollerMap();
        model =new BoydExtendedModel2(new BoydExtendedState2([-1,-1,-1],0), map, T, 1, &simplefeatures);
        opt_reward = new largeGridRewardGroupedFeatures2(model);
        double [2] reward_weights = [0.1, 1];
        opt_reward.setParams(reward_weights);

    } else {

        if (mapToUse == "boyd2") {

            map = boyd2PatrollerMap();
            model = new BoydModel(null, map, T, 1, &simplefeatures);
            opt_reward = new Boyd2RewardGroupedFeatures(model);
            double [6]  reward_weights = [1, -1, -1, -1, 0.5, -1];
    //		reward_weights = [0.465577137, 0.063009014, 0.063009014, 0.063009014, 0.282386808, 0.063009014]; // softmax of the above
            opt_reward.setParams(reward_weights);

        } else {

            if (mapToUse == "boydright2") {

                map = boydright2PatrollerMap();
                model = new BoydModel(null, map, T, 1, &simplefeatures);
                opt_reward = new BoydRight2Reward(model);
                double [7] reward_weights = [1, 0, 0, 0, 0, 0, 0.5];
                opt_reward.setParams(reward_weights);

            } else {

                map = boydrightPatrollerMap();
                model = new BoydModel(null, map, T, 1, &simplefeatures);
                opt_reward = new BoydRightReward(model);
                double [5] reward_weights = [1, -1, .1, 0, 0];
                opt_reward.setParams(reward_weights);

            }

        }
	}

//	writeln(reward);
	model.setReward(opt_reward);
	model.setGamma(0.99);
	ValueIteration vi = new ValueIteration();

	double[State] V = vi.solve(model, .1);

	debug {
        writeln(V);
	}
	Agent opt_policy = vi.createPolicy(model, V);

	double[State] initial;
    foreach (s; model.S()) {
        initial[s] = 1.0;
    }
    Distr!State.normalize(initial);

    auto opt_policy_value = policyValueOptReward([model], [opt_policy], [opt_reward], [initial], 5);
    //writeln("opt pol  values");
    //writeln(opt_policy_value[0]);

	Model [] models;

    if (mapToUse == "largeGridPatrol") {
        models ~= new BoydExtendedModel2(new BoydExtendedState2([-1,-1,-1],0), map, T, 1, &simplefeatures);
        models ~= new BoydExtendedModel2(new BoydExtendedState2([-1,-1,-1],0), map, T, 1, &simplefeatures);

    } else {

        models ~= new BoydModel(null, map, T, 1, &simplefeatures);
        models ~= new BoydModel(null, map, T, 1, &simplefeatures);

    }

	//models ~= new BoydModel(null, map, T, 1, &simplefeatures);
	//models ~= new BoydModel(null, map, T, 1, &simplefeatures);

	double [] reward_weights;
	LinearReward reward;
	foreach(i;0..models.length) {
		if (mapToUse == "boyd2") {

				reward = new Boyd2RewardGroupedFeatures(models[i]);
				reward_weights = new double[reward.dim()];
				reward_weights[] = 0;
    			reward.setParams(reward_weights);
			}
        else {
            if (mapToUse == "largeGridPatrol") {
                reward = new largeGridRewardGroupedFeatures2(models[i]);
                reward_weights = new double[reward.dim()];
                reward_weights[] = 0;
                reward.setParams(reward_weights);

            } else {

                if (mapToUse == "boydright2") {

                        reward = new BoydRight2Reward(models[i]);
                        reward_weights = new double[reward.dim()];
                        reward_weights[] = 0;
                        reward.setParams(reward_weights);

                } else {

                        reward = new BoydRightReward(models[i]);
                        reward_weights = new double[reward.dim()];
                        reward_weights[] = 0;
                        reward.setParams(reward_weights);
                }
			}
		}
		models[i].setReward(reward);
		models[i].setGamma(0.95);
	}

	// policy using WeightsIRL from LatentMaxEntIrlZiebartApproxMultipleAgentsBlockedGibbs
    ValueIteration solver = new ValueIteration();
    double solverError = .1;
    Agent [] learnedpolicies = new Agent[models.length];
    foreach(j, ref o; WeightsIRL) {
        LinearReward r = cast(LinearReward)models[j].getReward();
        r.setParams(o);

        learnedpolicies[j] = solver.createPolicy(models[j], solver.solve(models[j], solverError));

        auto learnedpolicy_value = policyValueOptReward([models[j]], [learnedpolicies[j]], [opt_reward], [initial], 5);
        //writeln("learnedpolicy_value j");
        //writeln(learnedpolicy_value[0]);

        double[] diff;
        diff.length = opt_policy_value[0].length;

        if (opt_policy_value[0].length == learnedpolicy_value[0].length) {

        //diff[] = opt_policy_value[0][] - learnedpolicy_value[0][];
        //double ile = (l1norm(opt_policy_value[0])-l1norm(learnedpolicy_value[0]))
        // /l1norm(opt_policy_value[0]);// ILE

        diff[] = opt_policy_value[0][] - learnedpolicy_value[0][];
    	double denom = l1norm(opt_policy_value[0]);
        //diff[] = diff[]/opt_policy_value[0][];

        //writeln(diff);
        double ile = l1norm(diff)/denom;// ILE*scaling of 10

        //scaling magnitude and subtracting offset
        if (mapToUse == "boyd2") {
            ile= (100*(ile-0.7986)/(1.3325-0.7986))+5;
        }
        else {
            ile= 100.0000*ile+5.0000;
        }
        writeln(ile);

        }
        else {
            debug {
                writeln(" value array length mismatch ");
            }

        }

    }
}

double [][] policyValueOptReward(Model [] models, Agent [] policies, LinearReward [] true_rewards, double[State][] initial_states, int length) {

	// the s-a value given a policy is the state visitation frequency times the reward for the state
	double [][] returnval;
	returnval.length = models.length;

	foreach (i, model; models) {
		auto sa_freq = calcStateActionFreq(policies[i], initial_states[i], model, length);

		double [] val;
        //val[]= 0.0;
		foreach (sa, freq; sa_freq) {
			val ~= freq * true_rewards[i].reward(sa.s, sa.a);
		}
        returnval[i].length = val.length;
		returnval[i] = val[];
	}

	return returnval;

}
