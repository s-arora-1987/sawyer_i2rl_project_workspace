import rsmdp;
import mdp;
import std.array;
import std.format;
import std.stdio;

class State : mdp.State {
	
	private int [] location;
	private int battery;
	
	public this ( int [] location = [0,0], int battery = 0 ) {
		
		setLocation(location);
		setBattery(battery);
	}
	
	public int[] getLocation() {
		return location;
	}
	
	public void setLocation(int [] l) {
		assert(l.length == 2);
		
		this.location = l;
		
	}
	
	public int getBattery() {
		return battery;
	}
	
	public void setBattery(int  l) {
		this.battery = l;
		
	}
	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "State: [%(%s, %)] @ %s", this.location, this.battery);
		return writer.data; 
	}


	override hash_t toHash() const {
		return location[0] * 100 + location[1] * 10 + battery;
	}	
	
	override bool opEquals(Object o) {
		if (this is o)
			return true;
		State p = cast(State)o;
		
		return p && p.location[0] == location[0] && p.location[1] == location[1] && p.battery == battery;
		
	}
	
	override public bool samePlaceAs(mdp.State o) {
		if (this is o)
			return true;
		State p = cast(State)o;
		
		return p && p.location[0] == location[0] && p.location[1] == location[1];
		
	}
	
	override int opCmp(Object o) const {
		State p = cast(State)o;

		if (!p) 
			return -1;
			
			
		for (int i = 0; i < location.length; i ++) {
			if (p.location[i] < location[i])
				return 1;
			else if (p.location[i] > location[i])
				return -1;
			
		}
		
		if (p.battery < battery)
			return 1;
		else if (p.battery > battery)
			return -1;
		
		return 0;
		
	}
}



public class MoveUpAction : mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		State p = cast(State)state;
		
		int [] s = p.getLocation().dup;
		s[0] -= 1;
		
		return new State(s, p.battery - 1);
		
	}
	
	public override string toString() {
		return "MoveUpAction"; 
	}


	override hash_t toHash() {
		return 0;
	}	
	
	override bool opEquals(Object o) {
		MoveUpAction p = cast(MoveUpAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		MoveUpAction p = cast(MoveUpAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}



public class MoveDownAction : mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		State p = cast(State)state;
		
		int [] s = p.getLocation().dup;
		s[0] += 1;
		
		return new State(s, p.battery - 1);
		
	}
	
	public override string toString() {
		return "MoveDownAction"; 
	}


	override hash_t toHash() {
		return 1;
	}	
	
	override bool opEquals(Object o) {
		MoveDownAction p = cast(MoveDownAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		MoveDownAction p = cast(MoveDownAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}



public class MoveLeftAction : mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		State p = cast(State)state;
		
		int [] s = p.getLocation().dup;
		s[1] -= 1;
		
		return new State(s, p.battery - 1);
		
	}
	
	public override string toString() {
		return "MoveLeftAction"; 
	}


	override hash_t toHash() {
		return 2;
	}	
	
	override bool opEquals(Object o) {
		MoveLeftAction p = cast(MoveLeftAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		MoveLeftAction p = cast(MoveLeftAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class MoveRightAction : mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		State p = cast(State)state;
		
		int [] s = p.getLocation().dup;
		s[1] += 1;
		
		return new State(s, p.battery - 1);
		
	}
	
	public override string toString() {
		return "MoveRightAction"; 
	}


	override hash_t toHash() {
		return 3;
	}	
	
	override bool opEquals(Object o) {
		MoveRightAction p = cast(MoveRightAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		MoveRightAction p = cast(MoveRightAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class Model : rsmdp.RiskSensitiveModel {
	
	private State [] states;
	private Action [] actions;
	private double [mdp.State][Action][mdp.State] Transitions;
	private State [] terminals;

	double[mdp.State] uniform;

	
	public this (State [] S, Action [] A, double [mdp.State][Action][mdp.State] T, double gamma, State [] terminal) {
		states = S;
		actions = A;
		Transitions = T;
		this.gamma = gamma;
		this.terminals = terminal;
		
		foreach (s; states) {
			uniform[s] = 1.0/states.length;
		}
	}
	
	
		
	public override int numTFeatures() {
		return 2;
	}
		
	public override int [] TFeatures(mdp.State state, Action action) {
		int [] returnval;
		State s = cast(State)state;
		if (s.getLocation()[1] == 0 || (s.getLocation()[1] == 1 && s.getLocation()[0] == 3 && action.toHash() != 0))
			returnval ~= 1;
		else 
			returnval ~= 0;
			
		if (s.getLocation()[1] == 1 && (s.getLocation()[0] < 3 || action.toHash() == 0))
			returnval ~= 1;
		else 
			returnval ~= 0;
			
		
		return returnval;
	}
	
	public void setT(double[mdp.State][Action][mdp.State] newT) {
		Transitions = newT;
	}
	
	
	public override double[mdp.State] T(mdp.State state, Action action) {
		return (state in Transitions && action in Transitions[state]) ? Transitions[state][action] : uniform ;
	}
	
	public override mdp.State [] S () {
		return cast(mdp.State[])states;
	}
	
	public override Action[] A(mdp.State state = null) {
		return actions;
		
	}
	
	public override bool is_terminal(mdp.State state) {
		foreach (s ; terminals) {
			if (s == state)
				return true;
		} 
		return false;
	}
	
	public override bool is_legal(mdp.State state) {
		foreach (s ; states) {
			if (s == state)
				return true;
		} 
		return false;
	}
	
	public override double getRiskSensitivity(mdp.State s) {
		State p = cast(State)s;
		
		return -1.5 * (1- (p.battery / 10.0));
		
	}
	
}

public class Reward : mdp.Reward {
	
	private State g;
	private State [] c;
	
	public this (State goal, State [] cliffs) {
		g = goal;
		c = cliffs;
	}
	
	public override double reward(mdp.State state, Action action) {
		if (state.samePlaceAs(g))
			return 1;
			
		foreach (s ; c) {
			if (s.samePlaceAs(state))
				return -1;
			
		}
		
		State s = cast(State) state;
		if (s.battery == 0)
			return -10.0;
		
		return -0.1;	
		
	} 
	
}

public class Reward2 : mdp.Reward {
	
	private State g;
	private State [] c;
	
	public this (State goal, State [] cliffs) {
		g = goal;
		c = cliffs;
	}
	
	public override double reward(mdp.State state, Action action) {
		if (state.samePlaceAs(g))
			return 1;
			
		foreach (s ; c) {
			if (s.samePlaceAs(state))
				return -1;
			
		}
		
		return -0.1;	
		
	} 
	
}
int main() {
	
	// create 4 x 3 gridworld with one cliff and 10 battery levels
	
	State [] states;
	
	for (int i = 0; i < 4; i ++) {
		for (int j = 0; j < 3; j ++) {
			if (j == 2 && i != 1)
				continue;
			for (int b = 0; b < 10; b ++) {
				states ~= new State( [i, j], b);
			}
		}
	}
	
	State [] terminals;
	for (int i = 0; i < 4; i ++) {
		for (int j = 0; j < 3; j ++) {
			terminals ~= new State( [i, j], 0);
		}
	}
	for (int b = 0; b < 10; b ++) {
		terminals ~= new State( [0, 1], b);
		terminals ~= new State( [1, 2], b);		
	}
	
	
	Action [] actions;
	
	actions ~= new MoveUpAction();
	actions ~= new MoveDownAction();
	actions ~= new MoveLeftAction();
	actions ~= new MoveRightAction();
	
	double [mdp.State][Action][mdp.State] T;
	
	foreach (s; states) {
		foreach (a; actions) {
			State testState = cast(State)a.apply(s);
			
			bool islegal = false;
			
			foreach (tests; states) {
				if (tests == testState) {
					islegal = true;
					break;
				}
			}
			
			if (! islegal) {
				T[s][a][s] = 1;
				continue;
			}
			
			foreach (sp; states) {
				
				
				if (testState == sp) {
					T[s][a][sp] = 1;
				} else {
					T[s][a][sp] = 0;
				}
			}
		}
		
	}
	
	
	
	Model m = new Model(states, actions, T, 1, terminals);
	
	m.setT(m.createTransitionFunction([1.0, 0.9], &otherActionsErrorModel));
	
/*	foreach (s; m.S()) {
		foreach(a; m.A()) {
			auto s_primes = m.T(s, a);
			
			foreach(s_prime, pr_s_prime; s_primes) {
				writeln( s, ":", a, ":", s_prime, ":", pr_s_prime);
			}
		}
	}*/
		
	m.setReward(new Reward2(new State([0, 1], 0), [ new State([1, 2], 0) ] ));
	
	mdp.ValueIteration vi = new mdp.ValueIteration();
	
	double[mdp.State] value = vi.solve(m, 0.0001);
	
//	writeln(value);
	Agent policy = vi.createPolicy(m, value);
	
	vi = new rsmdp.ValueIteration();

	m.setReward(new Reward2(new State([0, 1], 0), [ new State([1, 2], 0) ] ));
	
	double[mdp.State] value2 = vi.solve(m, 0.0001);
	
//	writeln(value);
	Agent policy2 = vi.createPolicy(m, value2);
	
	
	foreach (s; m.S()) {
		if (! m.is_terminal(s))
			writeln(s, " - RN ", policy.actions(s), " for: ", value[s], "  RS ", policy2.actions(s), " for: ", value2[s]);
    	else 
			writeln(s, " - TERMINATE for: ", value[s]);    	
    }
	
	return 0;
}
