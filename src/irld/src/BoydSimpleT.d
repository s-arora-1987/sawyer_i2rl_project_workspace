import boydmdp;
import mdp;
import std.stdio;
import std.random;
import std.math;
import std.range;
import std.traits;
import std.numeric;
import std.format;
import std.algorithm;
import std.string;
import std.c.stdlib;
alias std.string.indexOf indexOf;

int main() {

	double pfail;
	sar [][][] SAR;
	string mapToUse;
	string buf;
	
	buf = readln();
	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	buf = readln();
	formattedRead(buf, "%s", &pfail);

	int curPatroller = 0;
	SAR.length = 1;
	
	byte [][] themap;
	//Model model;

	//BoydExtendedModel modelE = new BoydModel(null, themap, null, 1, &simplefeatures);

	if (mapToUse == "boyd2") {
		themap = boyd2PatrollerMap();

		BoydModel model = new BoydModel(null, themap, null, 1, &simplefeatures);
		double [] weights = [pfail];
        model.setT(model.createTransitionFunction(weights, & stopErrorModel));

        foreach (s; model.S()) {
            foreach(a; model.A()) {
                auto s_primes = model.T(s, a);

                foreach(s_prime, pr_s_prime; s_primes) {
                    writeln( (cast(BoydState)s).getLocation(), ":", a, ":", (cast(BoydState)s_prime).getLocation(), ":", pr_s_prime);
                }
            }
        }

	} else {

		if (mapToUse == "largeGridPatrol") {

			themap = largeGridPatrollerMap();
			//BoydExtendedModel  model = new BoydExtendedModel(new BoydExtendedState(null,null,0), themap, null, 1, &simplefeatures);
			BoydExtendedModel2  model = new BoydExtendedModel2(new BoydExtendedState2(null,0),
			themap, null, 1, &simplefeatures);
            double p_success = 0.99;
            //MoveForwardAction mvfd = new MoveForwardAction();

            //auto newT = createTransitionFunctionSimple(model, p_success);
            auto newT = model.createTransitionFunctionSimple2(p_success);
            //auto s_primes2 = newT[bes1][tl];
            ////if (bes.location == [8, 4, 0] && bes.last_location == [8,4,3] && bes.current_goal == 0 && a==TL) {
            //writeln("legal s_primes2",s_primes2);
            //}
            model.setT(newT);

            bool flag;
            foreach (s; model.S()) {

                //writeln((cast(BoydExtendedState)s).getLocation(), ",", (cast(BoydExtendedState)s).getAction());
                //BoydExtendedState bes = cast(BoydExtendedState)s;
    			BoydExtendedState2 bes = cast(BoydExtendedState2)s;
                foreach(a; model.A()) {
                    //if (bes.location == [6, 1, 2] && bes.current_goal == 2 && (a==mvfd)) {
                    //    flag = 1;
                    //} else {
                    //    flag = 0;
                    //}
                    auto s_primes = model.T(s, a);
                    //if (bes.location == [8, 4, 0] && bes.last_location == [8,4,3] && bes.current_goal == 0 && a==TL) {
                    //    writeln("legal s_primes",s_primes);
                    //}
                    foreach(s_prime, pr_s_prime; s_primes) {
                        //BoydExtendedState bes2 = cast(BoydExtendedState)s_prime;
                        BoydExtendedState2 bes2 = cast(BoydExtendedState2)s_prime;
                        writeln( bes.getLocation(), ",", bes.getCurrentGoal(), ":", a, ":",
                            bes2.getLocation(), ",", bes2.getCurrentGoal(), ":", pr_s_prime);
                        //writeln(bes.getLocation(),",",bes.getLastLocation(),",",bes.getCurrentGoal(),":", a, ":",
                        //    bes2.getLocation(),",",bes2.getLastLocation(),",",bes2.getCurrentGoal(), ":", pr_s_prime);
                    }
                }
            }

		} else {

            if (mapToUse == "boydright2") {
                themap = boydright2PatrollerMap();
                BoydModel model = new BoydModel(null, themap, null, 1, &simplefeatures);
                double [] weights = [pfail];
                //writeln("here1");
                double p_success = 0.99;
                auto newT = model.createTransitionFunctionSimple(p_success);
                model.setT(newT);//model.createTransitionFunction(weights, & stopErrorModel));
                //writeln("here2");
                foreach (s; model.S()) {
                    foreach(a; model.A()) {
                        auto s_primes = model.T(s, a);

                        foreach(s_prime, pr_s_prime; s_primes) {
                            writeln( (cast(BoydState)s).getLocation(), ":", a, ":", (cast(BoydState)s_prime).getLocation(), ":", pr_s_prime);
                        }
                    }
                }

            } else {

                themap = boydrightPatrollerMap();
                BoydModel model = new BoydModel(null, themap, null, 1, &simplefeatures);
                double [] weights = [pfail];
                //writeln("here1");
                double p_success = 0.99;
                auto newT = model.createTransitionFunctionSimple(p_success);
                model.setT(newT);//model.createTransitionFunction(weights, & stopErrorModel));
                //writeln("here2");
                foreach (s; model.S()) {
                    foreach(a; model.A()) {
                        auto s_primes = model.T(s, a);

                        foreach(s_prime, pr_s_prime; s_primes) {
                            writeln( (cast(BoydState)s).getLocation(), ":", a, ":", (cast(BoydState)s_prime).getLocation(), ":", pr_s_prime);
                        }
                    }
                }
            }

		}
	}

/*	foreach (a; model.A()) {
		errorModel[a] = 1.0 / model.A().length;
	}*/

	return 0;
}

public double[State][Action][State] createTransitionFunctionSimple(Model model, double success) {

    double[State][Action][State] returnval;
    auto states = model.S();
    double success_t = success;

    foreach(State state; states) { // for each state

        if (model.is_terminal(state)) { // if it's terminal, use 1.0 as transition prob
            returnval[state][new NullAction()][state] = 1.0;
            continue;
        }

        foreach(Action action; model.A(state)) { // for each action

            State next_st = action.apply(state); // compute intended next state

            if (! model.is_legal(next_st) || (cast(BoydState)next_st).location == (cast(BoydState)state).location) { // if intended one not legal, stay at same state
                returnval[state][action][state] = 1.0;
            } else {
                returnval[state][action][next_st] = success_t;
                returnval[state][action][state] = 1-success_t;
            }

        }
    }

    return returnval;

}

public double[State][Action][State] createTransitionFunctionSimple2(Model model, double success) {

    double[State][Action][State] returnval;
    auto states = model.S();
    double success_t = success;

    foreach(State state; states) { // for each state

        if (model.is_terminal(state)) { // if it's terminal, use 1.0 as transition prob
            returnval[state][new NullAction()][state] = 1.0;
            continue;
        }

        BoydExtendedState2 bes = cast(BoydExtendedState2)state;
        foreach(Action action; model.A(state)) { // for each action
            State next_st = action.apply(state); // compute intended next state
            BoydExtendedState2 bes2 = cast(BoydExtendedState2)next_st;

            if (! model.is_legal(bes2) || bes2.location==bes.location) { // if intended one not legal, stay at same state
                returnval[state][action][state] = 1.0;
                //if (bes.location == [8,4,0] && bes.last_location == [8,4,3] && bes.current_goal == 0 && action == tl) {
                //    writeln("illegal",state,action,next_st);
                //}
            } else {
                returnval[state][action][next_st] = success_t;
                returnval[state][action][state] = 1-success_t;
                //if (bes.location == [8, 4, 0] && bes.last_location == [8,4,3] && bes.current_goal == 0 && action == tl) {
                //    writeln("legal",state,action,next_st);
                //}
            }
        }
    }

    return returnval;

}
