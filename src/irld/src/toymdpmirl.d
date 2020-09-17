import mdp;
import std.array;
import std.format;
import std.math;
import std.numeric;


protected class ToyState : mdp.State {
	
	private size_t [] location;
	
	public this ( size_t [] location = [0,0] ) {
		
		setLocation(location);
	}
	
	public size_t[] getLocation() {
		return location;
	}
	
	public void setLocation(size_t [] l) {
		assert(l.length == 2);
		
		this.location = l;
		
	}
	
	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "State: [%(%s, %)]", this.location);
		return writer.data; 
	}


	override hash_t toHash() const {
		return location[0] + location[1];
	}	
	
	override bool opEquals(Object o) {
		if (this is o)
			return true;
		ToyState p = cast(ToyState)o;
		
		return p && p.location[0] == location[0] && p.location[1] == location[1];
		
	}
	
	override public bool samePlaceAs(mdp.State o) {
		if (this is o)
			return true;
		ToyState p = cast(ToyState)o;
		
		return p && p.location[0] == location[0] && p.location[1] == location[1];
		
	}
	
	override int opCmp(Object o) const {
		ToyState p = cast(ToyState)o;

		if (!p) 
			return -1;
			
			
		for (int i = 0; i < location.length; i ++) {
			if (p.location[i] < location[i])
				return 1;
			else if (p.location[i] > location[i])
				return -1;
			
		}
		
		return 0;
		
	}
}


public class MoveUpAction : mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		ToyState p = cast(ToyState)state;
		
		size_t [] s = p.getLocation().dup;
		s[0] -= 1;
		
		return new ToyState(s);
		
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
		ToyState p = cast(ToyState)state;
		
		size_t [] s = p.getLocation().dup;
		s[0] += 1;
		
		return new ToyState(s);
		
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
		ToyState p = cast(ToyState)state;
		
		size_t [] s = p.getLocation().dup;
		s[1] -= 1;
		
		return new ToyState(s);
		
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
		ToyState p = cast(ToyState)state;
		
		size_t [] s = p.getLocation().dup;
		s[1] += 1;
		
		return new ToyState(s);
		
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



protected class ToyRewardSimple : mdp.LinearReward {
	Model model;
	public this(Model m) {
		model = m;
	}
	public override int dim() {
		return 3;
	}
	
			
	public override double [] features(State state, Action action) {

		/* bias (matches everything)
		   goal
		   penalty
		   moving to the right
		*/
		
		
		double [] returnval = new double[dim()];
		returnval[] = 0;
		
		if (action !is null) {
			State nextState = action.apply(state);
			
			if (model.is_legal(nextState)) {
				returnval[0] = 1;
			}
		
		} 
		
		ToyState ts = cast(ToyState)state;
		
		if (ts.location[0] == 0 && ts.location[1] == 3)
			returnval[1] = 1;
			
		if (ts.location[0] == 1 && ts.location[1] == 3)
			returnval[2] = 1;

//		if (action == new MoveRightAction() && ts.location[1] < 3)
	//		returnval[3] = 1;
			
		return returnval;
	}
}

protected class ToyModel : mdp.Model {


	Action [] actions;
	State [] states;
	State [] terminals;
	size_t savedX;
	size_t savedY;
	double [State][Action][State] t;
	double[State] uniform;

	public this( State [] terminals, size_t x, size_t y, size_t [][] invalid_states) {
		this.terminals = terminals;
		
		this.actions ~= new MoveUpAction();
		this.actions ~= new MoveDownAction();
		this.actions ~= new MoveLeftAction();
		this.actions ~= new MoveRightAction();
		
		foreach(i; 0 .. x) {
			foreach(j; 0 .. y) {
				bool skip = false;
				foreach (inv; invalid_states) {
					if (inv[0] == i && inv[1] == j) {
						skip = true;
						break;
					}
				}
				if (skip) continue;
				
				states ~= new ToyState([i, j]);
			}
		}
		
		savedX = x;
		savedY = y;
				
		foreach (s; states) {
			uniform[s] = 1.0/states.length;
		}
	}
	
	public override int numTFeatures() {
		return 1;
	}
	
	public override int [] TFeatures(State state, Action action) {
		return [1];	
	}
	
	public void setT(double[State][Action][State] newT) {
		t = newT;
	}
	
	public override double[State] T(State state, Action action) {
		return (state in t && action in t[state]) ? t[state][action] : uniform ;
	}
	
	public override State [] S () {
		return states;
	}
	
	public override Action[] A(State state = null) {
		return actions;
		
	}
	
	public override bool is_terminal(State state) {
		foreach(terminal; terminals) {
		 	if (state == terminal) {
		 		return true;
		 	}
		 }		
		return false;
	}
	
	public override bool is_legal(State state) {
		ToyState s = cast(ToyState)state;
		
		foreach (s2; states) {
			if (s == s2)
				return true;
			
		}
		return false;
	}

}
