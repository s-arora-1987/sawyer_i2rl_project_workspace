import std.random;
import std.algorithm;
import std.math;
import std.numeric;
import std.format;
import std.array;
import std.datetime;
//import boydmdp;

pure double l2norm(double [] vector) {
	return sqrt(dotProduct(vector, vector));
}

pure double l1norm(double [] vector) {
	double returnval = 0;
	foreach(v; vector)
		returnval += abs(v);
	return returnval;
}

class State {
	
	public bool extension;
	public abstract bool samePlaceAs(State s); 
	
}

class Action {
	
	public abstract State apply(State state);
	
	public override abstract bool opEquals(Object o);
	
}

class NullAction : Action {
	

	public override State apply(State state) {
		return state;
		
	}
	
	public override string toString() {
		return "NullAction"; 
	}


	override hash_t toHash() {
		return 100;
	}	
	
	override bool opEquals(Object o) {
		NullAction p = cast(NullAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		NullAction p = cast(NullAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
	
}

class Reward {
	
	public abstract double reward(State state, Action action); 
}

class LinearReward : Reward {
	
	protected double [] params;
	
	public double [] getParams() {
		return params;
	}
	
	public void setParams(double [] p) {
		params = p;
	}
	
	public abstract int dim();
	
	public abstract double [] features(State state, Action action);
	
	public override double reward (State state, Action action) {
		return dotProduct( params, features(state, action));
	}
	
}

class UniqueFeatureReward : LinearReward {
	
	size_t [Action][State] feature_index; 
	size_t count = 0;
	
	public this(Model model) {
		size_t idx = 0;
		foreach (s; model.S()) {
			
			if (model.is_terminal(s)) {
				feature_index[s][new NullAction()] = idx ++;
			} else {	
				foreach (a; model.A()) {
					feature_index[s][a] = idx ++;
				}
			}	
			
		}
		count = idx;
		
	}
	
	public override int dim() {
		return cast(int)count;
	}
	
	public override double [] features(State state, Action action) {
		double [] returnval;
		returnval.length = count;
		returnval[] = 0;
		
		auto i = feature_index[state][action];
		returnval[i] = 1;
		
		return returnval;
		
	}
	
}


class Model {

	public double gamma;
	protected Reward reward;
	
	public abstract int numTFeatures();
	public abstract int [] TFeatures(State state, Action action);
	
	public abstract double[State] T(State state, Action action);

	public double[State][Action][State] createTransitionFunction(double [] featureWeights,
	double[State] function (Model model, State curState, Action action, State intendedState, double remainder) error_model) {
		double[State][Action][State] returnval;
		bool flag;
		//MoveForwardAction mvfd = new MoveForwardAction();
		auto states = S();
		
		foreach(state; states) { // for each state

			if (is_terminal(state)) { // if it's terminal, use 1.0 as transition prob
				returnval[state][new NullAction()][state] = 1.0;
				continue;
			}
			//BoydExtendedState bes = cast(BoydExtendedState)state;
			foreach(action; A(state)) { // for each action
                //if (bes.location == [6, 1, 2] && bes.current_goal == 2 && (action==mvfd)) {
                //    flag = 1;
                //    writeln(bes.location.dup, bes.current_goal,action.toString());
                //
                //    foreach (s_prime, pr_s_prime; returnval[state][action]) {
                //        writeln ("s_prime, pr_s_prime ",s_prime, pr_s_prime);
                //    }
                //    writeln("flag = 1; createTransitionFunction");
                //} else {
                //    flag = 0;
                //}
				double success = 1.0;
                //if (flag == 1) {
                //    double[State] s_primes = returnval[state][action];
                //    foreach (s_prime, pr_s_prime; s_primes) {
                //        writeln ("before features and apply s_prime, pr_s_prime ",s_prime, pr_s_prime);
                //    }
                //}
				foreach(i,f; TFeatures(state,action)) {
				// TFeatures(state,action) = [1] for simplefeatures, featureweights = [pfail] = [1]
				// compute success prob for (s,a) using featureweights, [pfail]
					if (f != 0) {
						success *= featureWeights[i] * f;				
					}
				}
				//if (flag==1) {
				//    writeln("success foreach(i,f; TFeatures(state,action))",success);
				//}
				if (success > 1) {
					success = 1;
				}
				if (success < 0) {
					success = 0;
				}	
				auto s1 = action.apply(state); // compute intended next state
				//if (flag==1) {
				//    writeln("s1 ",s1);
				//}
				if (! is_legal(s1)) { // if intended one not legal, stay at same state
					s1 = state;
				}
                //if (flag == 1) {
                //    double[State] s_primes = returnval[state][action];
                //    foreach (s_prime, pr_s_prime; s_primes) {
                //        writeln ("before any addition s_prime, pr_s_prime ",s_prime, pr_s_prime);
                //    }
                //}
				if (state in returnval && action in returnval[state] && s1 in returnval[state][action]) {
				    // if intended state is already in returnval, then add to probability
					returnval[state][action][s1] += success;
                    //if (flag==1) {
                    //    writeln("s1 ",s1);
                    //    writeln("returnval[state][action][s1] = ",returnval[state][action][s1],
                    //    " if intended state is already in returnval");
                    //}
				} else {
				    // else crete a new entry
					returnval[state][action][s1] = success;
                    //if (flag==1) {
                    //    writeln("returnval[state][action][s1] = ",returnval[state][action][s1]," else");
                    //}
				}
                //if (flag == 1) {
                //    double[State] s_primes = returnval[state][action];
                //    foreach (s_prime, pr_s_prime; s_primes) {
                //        writeln ("before errorDistr s_prime, pr_s_prime ",s_prime, pr_s_prime);
                //    }
                //}

				double error = 0.0;

				// apply leftover prob mass to error model
				double[State] errorDistr = error_model(this, state, action, s1, 1.0 - success);
				// if error model is stoperrormodel, allot errorDistr[stopaction.apply(state)] =  1-success

				foreach (s, prob; errorDistr) {
				    // for each state-prob in error distribution
					if (state in returnval && action in returnval[state] && s in returnval[state][action]) {
					    // if state is already in returnval, then add to probability
						returnval[state][action][s] += prob;
					} else {
					    // else create entry
						returnval[state][action][s] = prob;
					}	
					error += prob;
                    //if (flag==1) {
                    //    writeln("s errorDistr",s," returnval[state][action][s] = ",returnval[state][action][s]);
                    //}
				}
                //if (flag==1) {
                //    writeln("success+error",success+error);
                //}

				// we've still got some prob mass left, apply equally to all actions
				if (success + error < 1.0) {

                    //if (flag==1) {
                    //    writeln("inside success+error < 1.0 ");
                    //}
					foreach (a; A(state)) {
						if (a != action) {
							auto s = a.apply(state);
							if (! is_legal(s)) {
								s = state;
							}
							double temp = (1.0 - (success + error)) / (A(state).length - 1);
							if (state in returnval && action in returnval[state] && s in returnval[state][action])
								returnval[state][action][s] += temp;
							else
								returnval[state][action][s] = temp;
						}
					}
				}
                //
                //if (flag == 1) {
                //    auto s_primes = returnval[state][action];
                //    foreach (s_prime, pr_s_prime; s_primes) {
                //        writeln ("s_prime, pr_s_prime ",s_prime, pr_s_prime);
                //    }
                //}
				Distr!State.normalize(returnval[state][action]);
                //if (flag == 1) {
                //    auto s_primes = returnval[state][action];
                //    foreach (s_prime, pr_s_prime; s_primes) {
                //        writeln ("s_prime, pr_s_prime ",s_prime, pr_s_prime);
                //    }
                //}
			}
		}
		
		return returnval;
	}


	public double R(State state, Action action) {
		
		return reward.reward(state, action);
	}
	
	public abstract State[] S();
	
	public abstract Action[] A(State state = null);
	
	public double getGamma() {
		return gamma;
	}
	
	public void setGamma(double g) {
		gamma = g;
	}
	
	public Reward getReward() {
		return reward;
	}
	
	public void setReward(Reward r) {
		reward = r;
	}
	
	public abstract bool is_terminal(State state);
	
	public abstract bool is_legal(State state);
	
}

// Applies remainder equally among other actions
public double[State] otherActionsErrorModel(Model model, State curState, Action action, State intendedState, double remainder) {

	double[State] returnval;
	auto actions = model.A(curState); 
	
	foreach (a; actions) {
		if (a != action) {

			double temp = remainder / (actions.length - 1); // assuming that action is a member of actions (or else, why are we considering this combo?)
			
			auto s = a.apply(curState);
			if (! model.is_legal(s)) {
				s = curState;
			}	
			if (s in returnval)
				returnval[s] += temp;
			else
				returnval[s] = temp;
		}
		
	}
	
	return returnval;
}

public template Distr(T1) {
	
	T1 sample(double[T1] distr) {
		
		auto r = uniform(0.0, .999);
		
		auto total = 0.0;
		
		auto keys = distr.keys;
		randomShuffle(keys);
		
		foreach (T1 s; keys) {
			total += distr[s];
			if (total >= r) {
				return s;
			}
			
		}
		throw new Exception("Distribution not normalized!");		
		
	}
	
	void normalize(ref double[T1] distr) {
		
		double total = 0;
		
		foreach (key, val ; distr) {
			total += val;
			
		} 
		
		if (total == 0) {
			throw new Exception("Cannot normalize, zero valued distribution"); 
			
		}
		
		foreach (key, val ; distr) {
			distr[key] = val / total;
			
		} 
			
	}

	T1 argmax (double[T1] distr) {
		if (distr.length == 0)
			throw new Exception("Cannot take argmax without any choices");
			
		
		double max = -double.max;
		T1 returnval = null;
		foreach (k, v ; distr) {
			if (v > max) {
				max = v;
				returnval = k;
			} 
		}
		return returnval;
	}
	
	T1 argmin (double[T1] distr) {
		if (distr.length == 0)
			throw new Exception("Cannot take argmin without any choices");
			
		
		double min = double.max;
		T1 returnval = null;
		
		foreach (k, v ; distr) {
			
			if (v < min) {
				min = v;
				returnval = k;
			} 
		}
		return returnval;
	}
}


unittest {
	double[string] test = ["hi" : 1.5, "middle" : 2, "best": 3];
	assert(Distr!string.argmax(test) == "best");	
	assert(Distr!string.argmin(test) == "hi");
}


class Agent {
  	
  	public abstract double[Action] actions(State state);
  	
  	public Action sample(State state) {
  		
  		return Distr!Action.sample(actions(state));
  	}
}
  
class MapAgent : Agent {
  	
  	private Action[State] policy;
  	private hash_t hash;
    private hash_t sh;
    private hash_t ah;

  	public this( Action[State] policy) {
  		
  		this.policy = policy;
  		hash = 0;
  		foreach (s, a; this.policy) {
  		    //writeln("hash += (s.toHash() + 1) * (a.toHash() + 1);");
  		    //writeln("s ",s);
  		    sh = s.toHash();
  		    //writeln("s.toHash()");
  		    //writeln(a);
  		    ah = a.toHash();
  		    //writeln("a.toHash()");
  		    hash += (sh + 1) * (ah + 1);

  			//hash += (s.toHash() + 1) * (a.toHash() + 1);
  		}
  	}
  	
  	public override double[Action] actions(State state) {
  		double[Action] returnval;
  		returnval[policy[state]] = 1.0;
  		return returnval;
  		
  	}
  	
  	public Action[State] getPolicy() {
  		return policy;
  	}
 
	override hash_t toHash() {
		return hash;
	}	
	
	override bool opEquals(Object o) {
		MapAgent p = cast(MapAgent)o;
		if( !p)
			return false;
			
		foreach(s,a; policy) {
			if (a != p.policy[s])
				return false;
			
		}
		return true;
		
	}
	
	override int opCmp(Object o) {
		MapAgent p = cast(MapAgent)o;
		
		if (!p) 
			return -1;
		
		int returnval = cast(int)( cast(long)(hash) - cast(long)p.hash);
		
		if (returnval == 0) {
			if ( opEquals(p) )
				return 0;
				
			// hmm, hashes are equal but the two agents are not.  Need to decide on an ordering
			
			// state with the lowest hash that disagrees on action is the winner, hash difference of the action is the returnval
			hash_t lowestState;
			bool initialized = false;
			foreach (s,a; policy) {
				if (a != p.policy[s]) {
					hash_t tempHash = s.toHash();
					if (!initialized || tempHash < lowestState) {
						initialized = true;
						
						lowestState = tempHash;
						returnval = cast(int)(cast(long)(a.toHash()) - cast(long)p.policy[s].toHash());
					}
					
				}
				
			}
			 
		}
		
		
		return returnval;
		
	}  
	
	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "%s", policy);
		return writer.data;
		
	}	
}

  
class StochasticAgent : Agent {
  	
  	private double[Action][State] policy;
  	
  	public this( double[Action][State] policy) {
  		
  		this.policy = policy;
  	}
  	
  	public override double[Action] actions(State state) {
  		return policy[state];
  		
  	}
  	
  	public double[Action][State] getPolicy() {
  		return policy;
  	}
  	
	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "%s", policy);
		return writer.data;
		
	}	
}

class RandomAgent : Agent {
	
	private double[Action] action_distr;
	
	public this (Action [] actions) {
		foreach (Action a; actions) {
			action_distr[a] = 1.0f;
		}
		Distr!Action.normalize(action_distr);
		
	}
	
	public override double[Action] actions(State state) {
		return action_distr;
	}
  	
}

public struct sar {
	
	public this(State s1, Action a1, double r1) {
		s = s1;
		a = a1;
		r = r1;
	}
	
	State s;
	Action a;
	double r;
	alias r p;
	
}

// creates a stochastic agent from the provided samples
class BayesStochasticAgent : StochasticAgent {
  	
  	private double[Action][State] policy;
  	
  	public this( sar[][] samples, Model model) {
  		super(policy);
  		// initialize entire policy to zero
  		
  		foreach (state; model.S()) {
  			foreach (action; model.A()) {
  				policy[state][action] = 0;
  			}
  		}
  		
  		foreach (traj; samples) {
  			foreach (SAR; traj) {
  				if (cast(NullAction)SAR.a)
  					continue;
  					
				policy[SAR.s][SAR.a] += SAR.p;
  				
  			}	
  		}

		auto size = model.A().length;
  		
  		foreach (state; model.S()) {
  			if (model.is_terminal(state)) {
  				policy[state] = null;
        		policy[state][new NullAction] = 1.0;
        		continue;
        	} 
  			
  			double sum = 0;
  			
  			foreach (action; model.A()) {
  				sum += policy[state][action];
  			}
  			
  			if (sum == 0) {
  				// no data, select uniform model
  				foreach (action; model.A()) {
  					policy[state][action] = 1.0/size;
  				}
  			} else {
	  			foreach (action; model.A()) {
	  				policy[state][action] = policy[state][action] / sum;
	  			}  				
  			}
  		}
  		debug {
  			write("BayesStochasticAgent: ");
  			writeln(policy);
  		}
  	}
  	
  	public override double[Action] actions(State state) {
  		return policy[state];
  		
  	}
  	
  	public override double[Action][State] getPolicy() {
  		return policy;
  	}
  	
}

import std.stdio;
public sar [] simulate(Model model, Agent agent, double[State] initial, size_t t_max) {
	
	State s = Distr!State.sample(initial);
	
	sar [] returnval;
	if (t_max < 0)
		return returnval;
	returnval.length = t_max;
	size_t t = 0;
	
	while (t < t_max && ! model.is_terminal(s)) {
		Action a = agent.sample(s);
		if (! a) {
			writeln("Null Action", s, " ", agent);
			
		}
		double r = model.R(s, a);
		
		sar temp;
		temp.s = s;
		temp.a = a;
		temp.r = r;
		returnval[t] = temp;
//		returnval ~= temp;
		
		s = Distr!State.sample(model.T(s, a));
		t += 1;
	}
	
	while (t < t_max && model.is_terminal(s)) {
		double r = model.R(s, new NullAction());
		
		sar temp;
		temp.s = s;
		temp.a = new NullAction();
		temp.r = r;
		returnval[t] = temp;
		t += 1;		
//#		returnval ~= temp;
	}
	
	return returnval;
}



public sar [][] multi_simulate(Model [] models, Agent [] agents, double[State][] initials, int t_max, double[Action] [] equilibria, int interactionLength = 1) {
	
	sar [][] returnval;
	returnval.length = models.length;
	if (t_max < 0)
		return returnval;
	for (int i = 0; i < returnval.length; i ++)	
		returnval[i].length = t_max;
		
	int t = 0;

	State [] Ss;
	foreach (double[State] initial; initials) {
		Ss ~= Distr!State.sample(initial);
	}
    //writeln("Ss ~= Distr!State.sample(initial); done");

	bool bothAtTerminal = false;
	long [] atTerminals;
	atTerminals.length = models.length;
	atTerminals[] = -1;

	foreach (i, model; models)
		if (model.is_terminal(Ss[i]))
			atTerminals[i] = 0; 

	
	int [] interactionCountdown = new int[Ss.length];
	interactionCountdown[] = -1;

	while (t < t_max && ! bothAtTerminal) {   
		
		// check if an agent is at the same place as another
		// if so, sample from the game for each
		// if not, sample from the agent
		Action [] actions;
		actions.length = models.length;
		
		bool allBelowZero = true;
		foreach (a ; interactionCountdown) {
			if (a >=0) {
				allBelowZero = false;
				break;
			}
				
		}
		
		if (! (equilibria is null) && allBelowZero) {
			foreach (int i, State s; Ss) {
				foreach (j; i+1 .. Ss.length) {
					State s2 = Ss[j];
					
					if (s2.samePlaceAs(s) || (t > 0 && (returnval[i][t - 1].s.samePlaceAs(s2) || s.samePlaceAs(returnval[j][t - 1].s)))) {
						// we have a conflict, run the game for both

						interactionCountdown[i] = interactionLength;
						interactionCountdown[j] = interactionLength;
					}
				}
			}
		}
		
		for( int i = 0; i < actions.length; i ++ ) {
			if (interactionCountdown[i] <= 0 || atTerminals[i] >= 0) {
				actions[i] = agents[i].sample(Ss[i]);
			} else if (interactionCountdown[i] > 1) {
				actions[i] = new NullAction();
			} else {

				actions[i] = Distr!Action.sample(equilibria[i]);				
				if (actions[i].toString() == "MoveForwardAction") {
					actions[i] = agents[i].sample(Ss[i]);
				} 

			}
			if (! actions[i]) {
				writeln("No Action", Ss[i], " ", agents[i]);
			
			}
		}
		
		interactionCountdown[] -= 1; 
		
		
		foreach (int i, Model model; models) {
			double r = model.R(Ss[i], actions[i]);
			
			sar temp;
			temp.s = Ss[i];
			temp.a = actions[i];
			temp.r = r;
			returnval[i][t] = temp; 

			if (actions[i] != new NullAction()) {
                //writeln(Ss[i],actions[i],model.T(Ss[i], actions[i]));
				Ss[i] = Distr!State.sample(model.T(Ss[i], actions[i]));
                //writeln("Ss[i] = Distr!State.sample(model.T(Ss[i], actions[i])); done");
			}
			
			if (model.is_terminal(Ss[i]) && atTerminals[i] < 0)
				atTerminals[i] = t;

		}

		t += 1;
		
		bothAtTerminal = true;
		foreach(a; atTerminals) {
			if (a < 0) {
				bothAtTerminal = false;
				break;
			}
		}
	}
	foreach (int i, Model model; models) {
		if (atTerminals[i] >= 0) {
			Action a = agents[i].sample(Ss[i]);
			double r = model.R(Ss[i], a);
			
			sar temp;
			temp.s = Ss[i];
			temp.a = a;
			temp.r = r;
/*			debug {
				writeln("AtTerminals: ", i, " t: ", atTerminals[i], " length: ", returnval[i].length );
				
			}*/
			returnval[i][atTerminals[i]] = temp;
			returnval[i].length = atTerminals[i] + 1;
		}
	}		
	
	
	return returnval;
}

public interface MDPSolver {
	
	public double[State] solve(Model model, double err);
	
	public Agent createPolicy(Model model, double[State] V);
}

public class ValueIteration : MDPSolver {
	int max_iter;
	bool terminalIsInfinite;
	
	public this(int maxiter = int.max, bool terminalIsInfiniteAction = false) {

		max_iter = maxiter;
		terminalIsInfinite = terminalIsInfiniteAction;
	}
	
	public double[State] solve(Model model, double err) {

		double[State] V;


		foreach (State s ; model.S()) {
			V[s] = 0.0;
		}
		//writeln("reached ValueIteration solve");
		double delta = 0;
		int i = 0;

		while (true) {
			delta = 0;
			double[State] V_next;

			foreach (State s ; model.S()) {

			    //writeln("starting foreach", s.toString());
				if (model.is_terminal(s) && ! terminalIsInfinite) {
	                double max_R = -double.max;
                    foreach (Action a; model.A(s)) {
                        if (model.R(s, a) > max_R) max_R = model.R(s,a);
                    }
                    V_next[s] = max_R;
				    //V_next[s] = model.R(s, new NullAction());
					delta = max(delta, abs(V[s] - V_next[s]));
					continue; 
				}
                //writeln("foreach");
				double[Action] q;
				if (model.is_terminal(s) && terminalIsInfinite) {

					Action a = new NullAction();
					//q.length = 1;
					double r = model.R(s, a);
					//writeln("getting reward value");
					
					q[a] = r + model.getGamma()*V[s];
				} else {
					//q.length = model.A(s).length;
			        //writeln("model.A(s):",model.A(s));
					foreach (Action a; model.A(s)) {
				        //writeln("a ",a);
						double r = model.R(s, a);
				        //writeln("r ",r);
						double[State] T = model.T(s, a);
	
						double expected_rewards = 0;
						foreach (s_prime, p; T){
					        //writeln("s_prime, p ",s_prime, p);
							if (s_prime in V) expected_rewards += p*V[s_prime];
						}
						
				        //writeln("q[a] = r");
						q[a] = r + model.getGamma()*expected_rewards;
					}
				}	
				double m = -double.max;
				foreach (double v; q.values)
					if (v > m)
						m = v;
				V_next[s] = m;
				//writeln("delta = max");
				delta = max(delta, abs(V[s] - V_next[s]));
			}
			V = V_next.dup;
			debug {
				//writeln("Current Iteration ", i," delta: ", delta);
			}
			i ++;

			if (delta < err || i > max_iter ) {
				//writeln("return V;");
				return V;
			}
		}
	}
	
	public Agent createPolicy(Model model, double[State] V) {
		
		Action[State] V_next;
		foreach (State s ; model.S()) {
			
			if (model.is_terminal(s)) {
				//V_next[s] = new NullAction();
              	Action act_max_R;
                double max_R = -double.max;
                foreach (Action a; model.A(s)) {
                	if (model.R(s, a) > max_R) act_max_R = a;
                }
                V_next[s] = act_max_R;

				continue;
			}
			
			double[Action] q;
			foreach (Action a; model.A(s)) {
				double r = model.R(s, a);
				double[State] T = model.T(s, a);
				
				double expected_rewards = 0;
				foreach (s_prime, p; T){
					writeln("s,a,s_prime,p");
					writeln(s,a,s_prime,p);
					expected_rewards += p*V[s_prime];
				}
				//writeln("q[a] = r");
				q[a] = r + model.getGamma()*expected_rewards;

			}
            //writeln("q for s ",s, ": ",q);
            V_next[s] = Distr!Action.argmax(q);
			
		}
		
        //writeln("(V_next)");
        //writeln(V_next);
		return new MapAgent(V_next);
	}		
	
	
}


public class TimedValueIteration : MDPSolver {
	int max_iter;
	bool terminalIsInfinite;
	auto duration_threshold = dur!"seconds"(30);
	
	public this(int maxiter = int.max, bool terminalIsInfiniteAction = false,int duration_threshold_int=30) {

		duration_threshold = dur!"seconds"(duration_threshold_int);
		max_iter = maxiter;
		terminalIsInfinite = terminalIsInfiniteAction;
	}
	
	public double[State] solve(Model model, double err) {

		double[State] V;

		auto stattimep = Clock.currTime();

		foreach (State s ; model.S()) {
			V[s] = 0.0;
		}
		//writeln("reached ValueIteration solve");
		double delta = 0;
		int i = 0;

		while (true) {
			delta = 0;
			double[State] V_next;

			foreach (State s ; model.S()) {

			    //writeln("starting foreach", s.toString());
				if (model.is_terminal(s) && ! terminalIsInfinite) {
	                double max_R = -double.max;
                    foreach (Action a; model.A(s)) {
                        if (model.R(s, a) > max_R) max_R = model.R(s,a);
                    }
                    V_next[s] = max_R;
				    //V_next[s] = model.R(s, new NullAction());
					delta = max(delta, abs(V[s] - V_next[s]));
					continue; 
				}
                //writeln("foreach");
				double[Action] q;
				if (model.is_terminal(s) && terminalIsInfinite) {

					Action a = new NullAction();
					//q.length = 1;
					double r = model.R(s, a);
					//writeln("getting reward value");
					
					q[a] = r + model.getGamma()*V[s];
				} else {
					//q.length = model.A(s).length;
			        //writeln("model.A(s):",model.A(s));
					foreach (Action a; model.A(s)) {
				        //writeln("a ",a);
						double r = model.R(s, a);
				        //writeln("r ",r);
						double[State] T = model.T(s, a);
	
						double expected_rewards = 0;
						foreach (s_prime, p; T){
					        //writeln("s_prime, p ",s_prime, p);
							if (s_prime in V) expected_rewards += p*V[s_prime];
						}
						
				        //writeln("q[a] = r");
						q[a] = r + model.getGamma()*expected_rewards;
					}
				}	
				double m = -double.max;
				foreach (double v; q.values)
					if (v > m)
						m = v;
				V_next[s] = m;
				//writeln("delta = max");
				delta = max(delta, abs(V[s] - V_next[s]));
			}
			V = V_next.dup;
			debug {
				//writeln("Current Iteration ", i," delta: ", delta);
			}
			i ++;
			auto endttimep = Clock.currTime();
			auto durationp = endttimep - stattimep;

			if (delta < err || i > max_iter || durationp < duration_threshold) {
				//writeln("return V;");
				return V;
			}
		}
	}
	
	public Agent createPolicy(Model model, double[State] V) {
		
		Action[State] V_next;
		foreach (State s ; model.S()) {
			
			if (model.is_terminal(s)) {
				//V_next[s] = new NullAction();
              	Action act_max_R;
                double max_R = -double.max;
                foreach (Action a; model.A(s)) {
                	if (model.R(s, a) > max_R) act_max_R = a;
                }
                V_next[s] = act_max_R;

				continue;
			}
			
			double[Action] q;
			foreach (Action a; model.A(s)) {
				double r = model.R(s, a);
				double[State] T = model.T(s, a);
				
				double expected_rewards = 0;
				foreach (s_prime, p; T){
					writeln("s,a,s_prime");
					writeln(s,a,s_prime);
					expected_rewards += p*V[s_prime];
				}
				//writeln("q[a] = r");
				q[a] = r + model.getGamma()*expected_rewards;

			}
            //writeln("V_next[s] = Distr!");
			V_next[s] = Distr!Action.argmax(q);
			
		}
		
        //writeln("(V_next)");
        //writeln(V_next);
		return new MapAgent(V_next);
	}		
	
	
}


/*
 * Note this class has state, it's intended to be repeated called with the same model (but different reward weights).
 * Don't re-use the same policy iteration object with different models, you've been warned.
*/
public class PolicyIteration : MDPSolver {
	
	private Agent lastPolicy;

	public double[State] solve(Model model, double err) {

		if (lastPolicy is null) 
			lastPolicy = new RandomAgent(model.A(null));
			
        double[State] v_pi;
        
        int i = 0;
        int last_different = 0;
        while (true) {
        	
        	v_pi = evalPolicy(model, lastPolicy, err);
        	
        	Agent newPolicy = oneStepLookahead(model, v_pi);
        	
        	int n_different = policy_difference(model, lastPolicy, newPolicy);
        	if (n_different == 0) {
        		break;	
        	}
        	
        	if (n_different == last_different) {
        		// lower error tolerance to prevent infinite looping
        		err /= 10;
        	}
        	last_different = n_different;
        	
        	i ++;
        	writeln("PI Iteration ", i, " differences: ", n_different);
        	
        	lastPolicy = newPolicy;
        	
        }
        
        
        return v_pi;

	}
	
	public Agent createPolicy(Model model, double[State] V) {
		// just return last calculated Policy
		return lastPolicy;
	}
	
	private double[State] evalPolicy(Model model, Agent policy, double err) {

		double[State] returnval;
		foreach (State s; model.S()) {
			returnval[s] = 0;
		}
		double delta = 0;
		
		do {
			double[State] VV;
			delta = 0;
			foreach (State s; model.S()) {
				double[Action] pi = policy.actions(s);
				double vv = 0;
				
				foreach (Action a, double t_pi; pi) {
					double v = model.R(s, a);
					double[State] T = model.T(s, a);
					
					double sum = 0;
					foreach (s_prime, t; T) {
						sum += t * returnval[s_prime];
						
					}
					
					v += model.getGamma()* sum;
					
					vv += t_pi*v;
					
				}
				VV[s] = vv;
				
				delta = max(delta, abs(returnval[s] - VV[s]));
			}
			returnval = VV;
		
		} while (delta > err);
		
		return returnval;


	}
	private double[State] samplingEvalPolicy(Model model, Agent policy, double err) {
		double[State] returnval;
		
		int num_samples = 100;
		int t_max = 30;
		
		foreach (s; model.S()) {
			
			double v = 0;
			
			double[State] initial;
			initial[s] = 1.0;
			
			for (int i = 0; i < num_samples; i ++ ){
				sar [] sample = simulate(model, policy, initial, t_max);
				foreach (int t, sar SAR; sample) {
					v += (pow(model.getGamma(), t)) * SAR.r;
					
				}
				
			}
			returnval[s] = v / num_samples;
			
		}
		return returnval;
	}
		
	private Agent oneStepLookahead(Model model, double[State] v_pi) {

		Action[State] policy;
		foreach	 (s; model.S() ) {
			double[Action] actions;
			
			foreach (a; model.A(s)) {
				double v = model.R(s, a);
				double[State] T = model.T(s, a);
				
				double sum = 0;
				foreach (s_prime, t; T) {
					sum += t * v_pi[s_prime];
					
				}
				
				v += model.getGamma()* sum;
				
				actions[a] = v;	
			}
			policy[s] = Distr!(Action).argmax(actions);
			
		}
		return new MapAgent(policy);
	}
	
	private int policy_difference(Model model, Agent policy1, Agent policy2) {
		int diffCount = 0;
		foreach (s; model.S()) {
			
			double[Action] p2 = policy2.actions(s);
			foreach (k, v; policy1.actions(s)) {
				if (! (k in p2)) {
					diffCount ++;
				}
			}
		}
		return diffCount;
	}	

}


class  StateAction {
	this(State s1, Action a1) {
		s = s1;
		a = a1;
	}
	
	State s;
	Action a;
	
	override hash_t toHash() {
		return (s.toHash() * 10) + a.toHash();
	}
	
	override bool opEquals(Object o) {
		StateAction other = cast(StateAction)o;
		return s.opEquals(other.s) && a.opEquals(other.a);
	}
	
	override int opCmp(Object o) {
		StateAction other = cast(StateAction)o;
		int q =s.opCmp(other.s); 
		if (q == 0)
			return a.opCmp(other.a); 
		return q;
	}
	
	public override string toString() {
		return s.toString() ~ " - " ~ a.toString();
		
	}
}

class JointStateAction {
	this(State s1, Action a1, State s2, Action a2) {
		s = s1;
		a = a1;
		this.s2 = s2;
		this.a2 = a2;
	}
	
	State s;
	Action a;
	State s2;
	Action a2;
	
	override hash_t toHash() {
		return (s.toHash() * 1000) + (a.toHash() * 100) + (s2.toHash() * 10) + a2.toHash();
	}
	
	override bool opEquals(Object o) {
		JointStateAction other = cast(JointStateAction)o;
		return s.opEquals(other.s) && a.opEquals(other.a) && s2.opEquals(other.s2) && a2.opEquals(other.a2);
	}
	
	override int opCmp(Object o) {
		JointStateAction other = cast(JointStateAction)o;
		int q = s.opCmp(other.s); 
		if (q == 0)
			q = a.opCmp(other.a);
		if (q == 0)
			q = s2.opCmp(other.s2);
		if (q == 0)
			q = a2.opCmp(other.a2);
		return q;
	}
	
	public override string toString() {
		return s.toString() ~ " - " ~ a.toString() ~ " : " ~ s2.toString() ~ " - " ~ a2.toString();
		
	}
}

public double[StateAction] QValueSoftMaxSolve(Model model, double err, size_t max_iter = size_t.max, bool terminalIsInfinite = true) {
	
		double[State] V;
		double[Action][State] Q;
		foreach (State s ; model.S()) {
			V[s] = 0.0;
			if (model.is_terminal(s)) {
				Q[s][new NullAction()] = 0;
			} else {
				foreach (a; model.A(s)) {
					Q[s][a] = 0;
				}
			}
		}
		V = V.rehash;
		Q = Q.rehash;
		double delta = 0;
		size_t iteration = 0;
		while (true) {
			delta = 0;
			
			foreach (State s ; model.S()) {
				if (model.is_terminal(s) && ! terminalIsInfinite) {
					Q[s][new NullAction()] = model.R(s, new NullAction());
					continue; 
				}	
				if (model.is_terminal(s) && terminalIsInfinite) {
					auto a = new NullAction();
					double r = model.R(s, a);
	
					Q[s][a] = r + model.getGamma()*V[s]*model.A().length; 
	
				} else {						
					foreach (Action a; model.A(s)) {
						double r = model.R(s, a);
						double[State] T = model.T(s, a);
	
						double expected_rewards = 0;
						foreach (s_prime, p; T){
							expected_rewards += p*V[s_prime];
						}
						
						Q[s][a] = r + model.getGamma()*expected_rewards; 
	
					}
				}
				
			}
			double[State] v;
			v = V.dup;
			foreach (State s ; model.S()) {
				double maxx = -double.max;
				if (model.is_terminal(s)) {
					maxx = max(maxx, Q[s][new NullAction()]);
				} else {
					foreach (Action a; model.A(s)) {
						maxx = max(maxx, Q[s][a]);
					}
				}
				double e_sum = double.min_normal;
				if (model.is_terminal(s)) {
					e_sum += exp(Q[s][new NullAction()] - maxx);
				} else {	
					foreach (Action a; model.A(s)) {
						e_sum += exp(Q[s][a] - maxx);
					}
				}
				v[s] = maxx - log(e_sum);
				delta = max(delta, abs(V[s] - v[s]));
			}


			V = v;
//			writeln("Current Iteration: ", delta);
			
			if (delta < err || iteration > max_iter) {
				double[StateAction] returnval;
				
				foreach(state, actvalue; Q) {
					foreach(action, value; actvalue) {
						returnval[new StateAction(state, action)] = value - v[state];
					}
				}
//				writeln("Finished Q-Value");
				
				return returnval;
			}
				
			iteration ++;
		}
		
		
}

/*  TODO: This doesn't work anymore, need to redo with a special implmentation just for the unittest
unittest {
	
	double [6] weights = [0.77223, -0.278873, -1.31235, -2.67311, -0.493401, -0.131478];
//	double [6] weights = [1.12691, -0.377004, -1.18167, -1.50622, -2.20608, 0.564132]
	import boydmdp;
	
	double p_fail = 0.05;
	
	PatrolModel model = new PatrolModel(p_fail, null);
	
	PatrolReward reward = new PatrolReward(model);
	
	reward.setParams(weights);
	
	model.setReward(reward);
	
	model.setGamma(0.99);
	
	double[StateAction] results = QValueSoftMaxSolve(model, 0.1);
	
	foreach (sa , v; results) {
		assert(! isnan(v));
		
	}
	
}
*/

public Agent CreatePolicyFromQValue(Model model, double[StateAction] Q_value) { 

        Action[State] V;
        foreach (State s; model.S()) {
        	double[Action] actions;
        	if (model.is_terminal(s)) {
        		actions[new NullAction()] = Q_value[new StateAction(s,new NullAction())];
        	} else {
        		foreach (Action a; model.A(s)) {
        			actions[a] = Q_value[new StateAction(s,a)];
        		}
        	}
        	V[s] = Distr!Action.argmax(actions);
        	if (V[s] is null) {
        		writeln("Got a null action");
        		foreach (SA, v; Q_value) {
        			writeln(SA.s, " ", SA.a, " -> ", v);
        		}
        		writeln(s); 
        		assert (false);
        		
        	}
        }
        return new MapAgent(V);
}

public Agent CreateStochasticPolicyFromQValue(Model model, double[StateAction] Q_value) { 

        double[Action][State] policy;
        foreach (State s; model.S()) {
        	double[Action] actions;
        	if (model.is_terminal(s)) {
        		actions[new NullAction()] = exp(Q_value[new StateAction(s,new NullAction())]);
        	} else {
        		foreach (Action a; model.A(s)) {
        			actions[a] = exp(Q_value[new StateAction(s,a)]);
        		}
        	}
        	
        	Distr!Action.normalize(actions);
        	
        	policy[s] = actions;
        }
        return new StochasticAgent(policy);
}

public double[StateAction] QValueSolve(Model model, double err, bool terminalIsInfinite = false) {

	double[State] V;
	double[Action][State] Q;
	foreach (State s ; model.S()) {
		V[s] = 0.0;
		if (model.is_terminal(s)) {
			Q[s][new NullAction()] = 0;
		} else {
			foreach (a; model.A(s)) {
				Q[s][a] = 0;
			}
		}
		Q[s] = Q[s].rehash;
	}
	V = V.rehash;
	Q = Q.rehash;
	double delta = 0;
	
	while (true) {
		delta = 0;
		
		
		foreach (State s ; model.S()) {
			if (model.is_terminal(s) && ! terminalIsInfinite) {
				Q[s][new NullAction()] = model.R(s, new NullAction());
				continue; 
			}
			
			double[Action] q;
			if (model.is_terminal(s) && terminalIsInfinite) {
				auto a = new NullAction();
				double r = model.R(s, a);

				Q[s][a] = r + model.getGamma()*V[s]; 

			} else {			
				foreach (Action a; model.A(s)) {
					double r = model.R(s, a);
					double[State] T = model.T(s, a);
	
					double expected_rewards = 0;
					foreach (s_prime, p; T){
						expected_rewards += p*V[s_prime];
					}
					
					Q[s][a] = r + model.getGamma()*expected_rewards; 
	
				}
			}
		}
		
		double[State] v;
		foreach (State s ; model.S()) {
			v[s] = -double.max;
			if (model.is_terminal(s)) {
				v[s] = Q[s][new NullAction()];
			} else {
				foreach (Action a; model.A(s)) {
					double val = Q[s][a];
					if (val > v[s])
						v[s] = val;
				}
			}
			
			delta = max(delta, abs(V[s] - v[s]));
		}
		


		V = v;
/*		debug {
			writeln("Current Iteration: ", delta);
		}*/
		
		if (delta < err) {
			double[StateAction] returnval;
			
			foreach(state, actvalue; Q) {
				foreach(action, value; actvalue) {
					returnval[new StateAction(state, action)] = value - v[state];
				}
			}
//			writeln("Finished Q-Value");
			
			return returnval;
		}
			
		
	}
	
		
	
}


void assignDistance(Model model, State startState, ref int[State] initial, int val = 0) {
	
	if (! (startState in initial) || val <= initial[startState]) {
		initial[startState] = val;
	
		foreach (a; model.A(startState)) {
			
			
			auto newState = a.apply(startState);
			if (model.is_legal(newState) && (! (newState in initial) || val  + 1 <= initial[newState])) {
				initial[newState] = val + 1;
			}
			
		}
		foreach (a; model.A(startState)) {
			
			auto newState = a.apply(startState);
			
			if (model.is_legal(newState)) {
				assignDistance(model, newState, initial, val + 1);
			}
		}		
	}

}

double[StateAction] calcStateActionFreq2(Agent policy, double[StateAction] q_value, Model m, int length, double[State] initial) {

	double[StateAction] returnval;

	foreach (s; m.S()) {
		foreach (a; m.A()) {
			auto key = new StateAction(s, a);
			double R = m.R(s,a);
			if (R != 0) {
				returnval[key] = exp(q_value[key]) / m.R(s, a)* initial[s];
			} else {
				returnval[key] = 0;
			}
			
		}
	}


	return returnval;
	

}

double[State] calcStateFreqExact(Agent policy, double[State] initial, Model m, double error) {

/*	foreach t, s, a
	= pi(s,a) * sum s' sum a' returnval^0(s') * T(s', a', s) * pi(s',a')

	then returnval^t is used for returnval^0*/
	
	double[State] tempState = initial;

	while(true) {
		double[State] newTempState;
		newTempState = initial.dup;
		foreach (s; m.S()) {
				
			double sum = 0;

			foreach(s_prime; m.S()) { 
				double[Action] a_pi = policy.actions(s_prime);

				foreach (a_prime, pi; a_pi) {
					
					double[State] t = m.T(s_prime, a_prime);
				
					if (s in t) {
						sum += tempState[s_prime] * t[s] * pi;
					}
				
				}
			}
			newTempState[s] += m.gamma * sum;
		}
		
		double max = -double.max;
		foreach (s, v; newTempState) {
			auto d = abs(tempState[s] - v);
			if (d > max) {
				max = d; 
			} 
		}
		
		tempState = newTempState;
//		writeln("CalcStateFreqAlt ", max);
		if (max < error)
			break;
		
	}
	return tempState;
}

double[State] calcStateFreq(Agent policy, double[State] initial, Model m, size_t length) {

/*	foreach t, s, a
	= pi(s,a) * sum s' sum a' returnval^0(s') * T(s', a', s) * pi(s',a')

	then returnval^t is used for returnval^0*/
	
	double[State] tempState = initial;

	foreach (i; 1 .. length) {
		double[State] newTempState;
		newTempState = initial.dup;
		foreach (s; m.S()) {
			foreach(s_prime; m.S()) { 
				double[Action] a_pi = policy.actions(s_prime);

				foreach (a_prime, pi; a_pi) {
					
					double[State] t = m.T(s_prime, a_prime);
				
					if (s in t) {
						newTempState[s] += tempState[s_prime] * t[s] * pi;
					}
				
				}
			}
		}
		tempState = newTempState;
	}

	return tempState;
}

// WARNING, only works for deterministic policies!
double[StateAction] calcStateActionFreq(Agent policy, double[State] initial, Model m, size_t length) {
	
	double[StateAction] returnval;

	double[State] tempState = calcStateFreqExact(policy, initial, m, 0.0001);
	//calcStateFreq(policy, initial, m, length);
	
//	Distr!State.normalize(tempState);
	
	foreach (s; m.S()) {
		foreach(a, p; policy.actions(s)) {
//				returnval[new StateAction(s,a)] = tempState[s] * length;
			returnval[new StateAction(s,a)] = tempState[s];
		}
	}
	
	
	return returnval;

} 

// WARNING, only works for deterministic policies!
double[StateAction] calcStateActionFreqExact(Agent policy, double[State] initial, Model m, double error) {
	
	double[StateAction] returnval;

	double[State] tempState = calcStateFreqExact(policy, initial, m, error);

	foreach (s; m.S()) {
		foreach(a; m.A(s)) {
			if (a in policy.actions(s))
				returnval[new StateAction(s,a)] = tempState[s];
		}
	}
		
	return returnval;

} 
