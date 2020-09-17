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

alias std.string.indexOf indexOf;


int main() {


	sar [][][] SAR;
	string mapToUse;
	string buf;
	bool useSimpleFeatures;
	double statesVisible;

	buf = readln();
	formattedRead(buf, "%s", &statesVisible);
	
	buf = readln();
	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	buf = readln();
	formattedRead(buf, "%s", &useSimpleFeatures);
	
	int curPatroller = 0;
	SAR.length = 1;
	
    while ((buf = readln()) != null) {
    	buf = strip(buf);
    
    	if (buf == "ENDTRAJ") {
    		curPatroller ++;
    		SAR.length = SAR.length + 1;
    		
    	} else {
    		sar [] newtraj;
    		
    		while (buf.indexOf(";") >= 0) {
    			string percept = buf[0..buf.indexOf(";")];
    			buf = buf[buf.indexOf(";") + 1 .. buf.length];
    			
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
   				
   				
   				newtraj ~= sar(new BoydState([x, y, z]), a, p);

    		}
    		
    		SAR[curPatroller] ~= newtraj;
    		
    	}
    	
    }
	SAR.length = SAR.length - 1;
	
	byte [][] themap;
	
	if (mapToUse == "boyd2") {
		themap = boyd2PatrollerMap();
		
	} else {
		themap = boydrightPatrollerMap();
		
	}
	
	BoydModel model = new BoydModel(null, themap, null, 0, null);

	State [] observableStatesList;

	foreach (s; model.S()) {
		if (mapToUse == "boyd2") {
			
			if (boyd2isvisible(cast(BoydState)s, statesVisible))
				observableStatesList ~= s;
		} else {	
			if (boydrightisvisible(cast(BoydState)s, statesVisible))
				observableStatesList ~= s;
		}
	}
	
	EM em = new EM();
	
	model.setT(em.calculate(model, SAR[0], 15, observableStatesList));
	
	
	foreach (s; model.S()) {
		foreach(a; model.A()) {
			auto s_primes = model.T(s, a);
			
			foreach(s_prime, pr_s_prime; s_primes) {
				writeln( (cast(BoydState)s).getLocation(), ":", a, ":", (cast(BoydState)s_prime).getLocation(), ":", pr_s_prime);
			}
		}
	}
	writeln("ENDT");

	model.setT(em.calculate(model, SAR[1], 15, observableStatesList));
	
	
	foreach (s; model.S()) {
		foreach(a; model.A()) {
			auto s_primes = model.T(s, a);
			
			foreach(s_prime, pr_s_prime; s_primes) {
				writeln( (cast(BoydState)s).getLocation(), ":", a, ":", (cast(BoydState)s_prime).getLocation(), ":", pr_s_prime);
			}
		}
	}
	writeln("ENDT");
	

	return 0;
}


class weighted_trajectory {
	public double weight;
	public sar [][] traj;
	
}

class EM {

	size_t [State] s_index;
	size_t [Action] a_index;
	size_t num_samples;
	size_t S_len;
	
	double[State][Action][State] calculate(BoydModel model, sar [][] traj, int iterations, State [] observableStates, int samples = 100) {
/*		# generate a random transition table
		# go in a loop calling E() and then M()
		# when out of iterations, return the result */
		
		BoydState [] S = cast(BoydState[])model.S();
		Action [] A = model.A();
		S_len = S.length;
		
//		uint num_samples = samples;
		num_samples = 10 * traj.length;
		
		foreach( i, s ; S ) {
			s_index[s] = i;
		}
		
		foreach( i, a; A){
			a_index[a] = i;
		}	 
		
		double [][] transitions;
		foreach (i; 0..(S.length * A.length)) {
			double [] temp;
			foreach (j; 0..(S.length)) {
				temp ~= uniform(0.0, 1.0); 
				
			}
			
			transitions ~= temp;
			
		}
		
//		# normalize so all transitions sum to one
		foreach( i, row; transitions) {
			double thesum = 0.0;
			foreach( entry; row) {
				thesum += entry;
			}

			foreach( ref entry; row) {
				entry /= thesum;
			}
			
		}
		auto prev_transitions = transitions;
		
//		# remove blank entries at the beginning of the trajectory (would need to sample from a uniform prior to handle these, which requires a buttload more sampling
		foreach( i, t; traj) {
			if( ! ( t is null)) {
				traj = traj[i..$-1];
				break;
			}	
		}		
		
		bool [] validStates;
		validStates.length = S_len;
		validStates[] = true;
		foreach (s; observableStates) {
			validStates[s_index[s]] = false;
		}

		Agent uniform = new RandomAgent(model.A());
		Agent policy = uniform;
		
		weighted_trajectory [] weighted_trajs;
		double lastdelta = 2.0;
		int i = 0;
		while(i < iterations){
			debug {
				writeln("Iteration number ", i, " samples: ", num_samples);
			}
			i += 1;
			weighted_trajs = E(model, traj, transitions, validStates, policy);
			debug {
				writeln("Finished E step");
			}	
			transitions = M(model, weighted_trajs, 0.001, uniform, policy);
			debug {
				writeln("Finished M step");
			}
				
			double delta = -1;
			
			foreach( r, row; transitions) {
				double [] newrow;
				foreach (j, entry; row) {
					newrow ~= abs(prev_transitions[r][j] - entry);
				}
				
				foreach (entry; newrow) {
					if( entry > delta) {
						delta = entry;
					}
				}		
			}
			
			debug {
				writeln("Delta = ", delta);
			}	
			if (delta < 0.04) {
				break;
			}	
			prev_transitions = transitions;
			if (delta > lastdelta) {
				num_samples = cast(int)(num_samples * 1.25);
			}	
			
			lastdelta = delta;
			
		}	
		
		auto finaltransitions = M(model, weighted_trajs, 0.001, uniform, policy);
		
		bool displayAll = true;
		
		debug {
			displayAll = false;
			
			writeln((cast(StochasticAgent)policy).policy);
		}	
		double[State][Action][State] returnval;
		foreach(s, i; s_index) {
			foreach(a, j; a_index) {
				foreach(s_pr, k; s_index) {					
					if (finaltransitions[S_len * j + i][k] > 0 || displayAll) {
						returnval[s][a][s_pr] = finaltransitions[S_len * j + i][k];
					}	
				}
			}	
		}
		
		return returnval;		
	}			
	
	weighted_trajectory [] E(BoydModel model, sar [][] traj, double [][] transitions, bool [] validStates, Agent policy) {
/*		# generate a bunch of trajectories by filling in the missing entries, use simulate for this
		# find a weight for each generated trajectory
		# normalize the weights*/
				
		weighted_trajectory [] weighted_trajs;
				
		foreach (i; 0..num_samples) {
			sar [][] traj_copy = traj.dup;
			
			foreach (t, entry; traj_copy) {
				
				if (entry is null || entry.length == 0) {
/*					# found a point to insert into, find the ending
					
					# the ending could be somewhere in the middle of the trajectory, or at the end
					# the starting could be at position 0 */
					size_t endpoint = t + 1;
					if (endpoint < traj_copy.length) {
						foreach (t2, endentry; traj_copy[t + 1..$ - 1]) {
							endpoint = t + 1 + t2;
							if (! (endentry is null) && endentry.length > 0){
								break;
							}
						}
					}
					double[State] initial;
					
					if (t > 0) {
						initial[traj_copy[t - 1][0].s] = 1.0;
					} else {
						auto S = model.S();
						foreach ( s; S) {
							initial[s] = 1.0 / S.length;
						}	
					}
					traj_copy[t..endpoint] = simulate(model, transitions, policy, initial, endpoint - t, validStates);
					
				}	
			}

//			# calc total probability of trajectory
			double prob = 1.0;
			
			debug {
				foreach ( t, entry; traj_copy) {
					if (t > 0) {
						auto start_s = s_index[traj_copy[t - 1][0].s];
						auto end_s = s_index[traj_copy[t][0].s];
						auto action = a_index[traj_copy[t - 1][0].a];
						
						prob *= transitions[model.S().length*action + start_s][end_s];
						
						auto actions = policy.actions(traj_copy[t - 1][0].s);
						
						prob *= actions.get(traj_copy[t - 1][0].a, 0.000000000000001);
					}	
				}
			}	
			weighted_trajectory wt = new weighted_trajectory();
			wt.weight = prob;
			wt.traj = traj_copy;
			
			weighted_trajs ~= wt;
		}	

		debug {
	//		# normalize weights		
			double sum_weights = 0;
			foreach( t; weighted_trajs) {
				sum_weights += t.weight;
			}
			
			writeln("Avg LogP of trajectories: ", log(sum_weights / num_samples));
				
			foreach ( ref t; weighted_trajs) 
				t.weight /= sum_weights;
		}

		return weighted_trajs;
	}
	
	double [][] M(BoydModel model, weighted_trajectory [] weighted_trajs, double prior, Agent uniform, out Agent newPolicy) {
/*		# generate a new transition table by finding the transitions for each generated trajectory
		# then combining them together according to the transition weight */  
		

		// initialize storage
		double [][] transitions;
		foreach (i; 0..(model.S().length * model.A().length)) {
			double [] temp;
			temp.length = model.S().length;
			temp[] = prior;

			transitions ~= temp;

		}

		double [][] policyCount;
		foreach (i; 0..model.S().length) {
			double [] temp;
			temp.length = model.A().length;
			temp[] = 0; // assume we're trying for a deterministic policy here, no prior

			policyCount ~= temp;

		}
		
		
		
		foreach( wt; weighted_trajs) {
			
/*			# for each transition, add a weighted amount to the total transitions
			# then normalize the rows*/
			
			sar preventry = sar (null, null, 0);
			foreach( timestep; wt.traj) {
				if (! (preventry.s is null)) {
					auto a = preventry.a;
					
					wt.weight = 1;	
					
					transitions[S_len * a_index[a] + s_index[preventry.s]][s_index[timestep[0].s]] += wt.weight;	
					
					// do the same for the policy
					
					policyCount[s_index[preventry.s]][a_index[a]] += wt.weight;
				}	
				preventry = timestep[0];
			}	
		}	
			
		
//		# normalize so all transitions sum to one
		foreach (i, row; transitions){
			double thesum = 0.0;
			foreach (entry; row) {
				thesum += entry;
			}
			if (thesum > 0) {
				foreach (ref entry; row) {
					entry /= thesum;
				}
			}
						
		}

		double [Action][State] policy;
		// create policy
		foreach (s, i; s_index) {
			
			double sum = 0.0;
			
			foreach (a, j; a_index) {
				sum += policyCount[i][j];
			}
			
			if (sum > 0) {
				foreach (a, j; a_index) {
					policy[s][a] = policyCount[i][j] / sum;
				}
			}
		}
		
		newPolicy = new StochasticAgent(policy, uniform);
		

		return transitions;
	}			
				

	sar [][] simulate(BoydModel model, double [][] transitions, Agent agent, double [State] initial, size_t t_max, bool [] validStates){
/*		'''
		Simulate an MDP for t_max timesteps or until the
		a terminal state is reached.  Returns a list
			[ (s_0, a_0, r_0), (s_1, a_1, r_1), ...]
		'''*/
		
		auto s = Distr!State.sample(initial);

		sar [][] result;
		size_t t = 0;
		uint retry_counter = 0; 
		
		while( t < t_max && ! model.is_terminal(s)) { 
			auto a = agent.sample(s);
			
			double[State] T;
			
			foreach ( key, value; s_index) {
				// don't allow visible states to be added to the trajectory unless we have to
				if (validStates[value] || t_max <= 2 || retry_counter > 50) // arbitrary choices, for your health
					T[key] = transitions[S_len * a_index[a] + s_index[s]][value];
				
			}
			if (T.length == 0) {
				retry_counter ++;
				continue;
			}
			retry_counter = 0;
			Distr!State.normalize(T);
			
			auto s_p = Distr!State.sample( T );
								
			
			sar temp;
			temp.s = s;
			temp.a = a;
			
			result ~= [ temp ];
			s = s_p;
			t += 1;
		}	
		if (model.is_terminal(s)) {
			auto a = agent.sample(s);

			sar temp;
			temp.s = s;
			temp.a = a;
			
			result ~= [temp];
		}	
		return result;				

	}
}

class StochasticAgent : Agent {
  	
  	private double [Action][State] policy;
  	private Agent uniform;
  	
  	public this( double [Action][State] policy, Agent uniform) {
  		
  		this.policy = policy;
  		this.uniform = uniform;
  	}
  	
  	public override double[Action] actions(State state) {
  		if (! ( state in policy)) {
  			return uniform.actions(state);
  		}
  		return policy[state];
  		
  	}
  	
}
