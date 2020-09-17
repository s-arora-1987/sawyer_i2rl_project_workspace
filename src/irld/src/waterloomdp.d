import mdp;
import std.array;
import std.format;
import std.math;
import std.numeric;

public const int WaterlooActionTimeSlots = 12;

class WaterlooState : mdp.State {
	
	public int position;
	public int ballType;
	public int previousActionTime;
	public bool holdingBall;
	
	public this (int pos, int ball, int previousActionTime, bool holding) {
		this.position = pos;
		this.ballType = ball;
		this.previousActionTime = previousActionTime;
		this.holdingBall = holding;
		
	}
	
	
	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "State: [%s, %s, %s, %s]", this.position, this.ballType, this.previousActionTime, this.holdingBall);
		return writer.data; 
	}


	override hash_t toHash() const {
		return position + ballType + previousActionTime + cast(int)holdingBall;
	}	
	
	override bool opEquals(Object o) {
		if (this is o)
			return true;
		WaterlooState p = cast(WaterlooState)o;
		
		return p && p.position == position && p.ballType == ballType && p.previousActionTime == previousActionTime && p.holdingBall == holdingBall;
		
	}
	
	override public bool samePlaceAs(mdp.State o) {
		return opEquals(o);
	}
	
	override int opCmp(Object o) const {
		WaterlooState p = cast(WaterlooState)o;

		if (!p) 
			return -1;
			
		if (p.position < position)
			return 1;
		else if (p.position > position)
			return -1;
		
		if (p.ballType < ballType)
			return 1;
		else if (p.ballType > ballType)
			return -1;
		
		if (p.previousActionTime < previousActionTime)
			return 1;
		else if (p.previousActionTime > previousActionTime)
			return -1;
		
		if (p.holdingBall && ! holdingBall)
			return 1;
		else if (! p.holdingBall && holdingBall)
			return -1;
		
		
		return 0;
		
	}	
}	


class WaterlooMoveCenterAction: mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		WaterlooState p = cast(WaterlooState)state;
		
		int newPos = 1;

		return new WaterlooState(newPos, p.ballType, 0, p.holdingBall);
		
	}
	
	public override string toString() {
		return "MoveCenterAction"; 
	}


	override hash_t toHash() {
		return 0;
	}	
	
	override bool opEquals(Object o) {
		WaterlooMoveCenterAction p = cast(WaterlooMoveCenterAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		WaterlooMoveCenterAction p = cast(WaterlooMoveCenterAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}	
}


class WaterlooMoveBin1Action: mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		WaterlooState p = cast(WaterlooState)state;
		
		int newPos = 2;

		return new WaterlooState(newPos, p.ballType, 0, p.holdingBall);
		
	}
	
	public override string toString() {
		return "MoveBin1Action"; 
	}


	override hash_t toHash() {
		return 1;
	}	
	
	override bool opEquals(Object o) {
		WaterlooMoveBin1Action p = cast(WaterlooMoveBin1Action)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		WaterlooMoveBin1Action p = cast(WaterlooMoveBin1Action)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}	
}

class WaterlooMoveBin2Action: mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		WaterlooState p = cast(WaterlooState)state;
		
		int newPos = 3;

		return new WaterlooState(newPos, p.ballType, 0, p.holdingBall);
		
	}
	
	public override string toString() {
		return "MoveBin2Action"; 
	}


	override hash_t toHash() {
		return 2;
	}	
	
	override bool opEquals(Object o) {
		WaterlooMoveBin2Action p = cast(WaterlooMoveBin2Action)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		WaterlooMoveBin2Action p = cast(WaterlooMoveBin2Action)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}	
}

class WaterlooGrabHardAction: mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		WaterlooState p = cast(WaterlooState)state;
		
		bool holdingBall = true;
		bool ballIsDamaged = (p.ballType == 2 || p.ballType == 4) ? true : false;

		return new WaterlooState(p.position, p.ballType, 0, holdingBall);
		
	}
	
	public override string toString() {
		return "GrabHardAction"; 
	}


	override hash_t toHash() {
		return 3;
	}	
	
	override bool opEquals(Object o) {
		WaterlooGrabHardAction p = cast(WaterlooGrabHardAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		WaterlooGrabHardAction p = cast(WaterlooGrabHardAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

class WaterlooGrabSoftAction: mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		WaterlooState p = cast(WaterlooState)state;
		
		bool holdingBall = true;

		return new WaterlooState(p.position, p.ballType, 0, holdingBall);
		
	}
	
	public override string toString() {
		return "GrabSoftAction"; 
	}


	override hash_t toHash() {
		return 4;
	}	
	
	override bool opEquals(Object o) {
		WaterlooGrabSoftAction p = cast(WaterlooGrabSoftAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		WaterlooGrabSoftAction p = cast(WaterlooGrabSoftAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}	
}

class WaterlooReleaseHardAction: mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		WaterlooState p = cast(WaterlooState)state;
		
		bool holdingBall = false;
		bool ballIsDamaged = (p.ballType == 2 || p.ballType == 4) ? true : false;

		return new WaterlooState(p.position, p.ballType, 0, holdingBall);
		
	}
	
	public override string toString() {
		return "ReleaseHardAction"; 
	}


	override hash_t toHash() {
		return 5;
	}	
	
	override bool opEquals(Object o) {
		WaterlooReleaseHardAction p = cast(WaterlooReleaseHardAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		WaterlooReleaseHardAction p = cast(WaterlooReleaseHardAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}

}

class WaterlooReleaseSoftAction: mdp.Action {
	
	public override mdp.State apply(mdp.State state) {
		WaterlooState p = cast(WaterlooState)state;
		
		bool holdingBall = false;

		return new WaterlooState(p.position, p.ballType, 0, holdingBall);
		
	}
	
	public override string toString() {
		return "ReleaseSoftAction"; 
	}


	override hash_t toHash() {
		return 6;
	}	
	
	override bool opEquals(Object o) {
		WaterlooReleaseSoftAction p = cast(WaterlooReleaseSoftAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		WaterlooReleaseSoftAction p = cast(WaterlooReleaseSoftAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}		
}

class WaterlooReward : mdp.LinearReward {
	
	public override int dim() {
		return 12;
	}
		
	public override double [] features(State state, Action action) {
		
		WaterlooState ws = cast(WaterlooState)state;
		
		double [] returnval = new double[dim()];
		returnval[] = 0;

		if (ws.position == 2 && ws.holdingBall == true ) {
			// released in bin 1
			returnval[ws.ballType - 1] = 1.0;
		} else if (ws.position == 3 && ws.holdingBall == true ) {
			// released in bin 2
			returnval[4 + ws.ballType - 1] = 1.0;
		} 
		
		// handle each balltype soft
		
		WaterlooGrabSoftAction wgsa = cast(WaterlooGrabSoftAction)action;
		WaterlooReleaseSoftAction wrsa = cast(WaterlooReleaseSoftAction)action;
		
		if (wgsa || wrsa) {
			returnval[8 + ws.ballType - 1] = 1.0;
		}
		
		return returnval;
	}	

}

class WaterlooModel : mdp.Model {


	Action [][State] actions;
	State [] states;
	State [] terminals;
	double [State][Action][State] t;
	double[State] uniform;

	public this() {
		
		foreach(j; 1 .. 5) {
			State s;
			
			foreach (i; 0 .. WaterlooActionTimeSlots) {
				s = new WaterlooState(0, j, i, false);
				states ~= s;
				actions[s] ~= new WaterlooGrabHardAction();
				actions[s] ~= new WaterlooGrabSoftAction();
			}
			
				
			foreach (i; 0 .. WaterlooActionTimeSlots) {
				s = new WaterlooState(0, j, i, true);
				states ~= s;
				actions[s] ~= new WaterlooMoveCenterAction();
			}	
			
		}

		foreach(j; 1 .. 5) {
			State s;
			
			foreach (i; 0 .. WaterlooActionTimeSlots) {	
				s = new WaterlooState(1, j, i, true);
				states ~= s;
				actions[s] ~= new WaterlooMoveBin1Action();
				actions[s] ~= new WaterlooMoveBin2Action();
			}	
			
			foreach (i; 0 .. WaterlooActionTimeSlots) {
				s = new WaterlooState(1, j, i, false);
				states ~= s;
				terminals ~= s;
				actions[s] ~= new NullAction();
			}				
		}
		
		foreach(j; 1 .. 5) {
			State s;
			
			foreach (i; 0 .. WaterlooActionTimeSlots) {
				s = new WaterlooState(2, j, i, false);
				states ~= s;
				terminals ~= s;
				actions[s] ~= new NullAction();
			}	

			foreach (i; 0 .. WaterlooActionTimeSlots) {
				s = new WaterlooState(2, j, i, true);
				states ~= s;
				actions[s] ~= new WaterlooReleaseHardAction();
				actions[s] ~= new WaterlooReleaseSoftAction();
			}	
			
		}
		
		foreach(j; 1 .. 5) {
			State s;
			
			foreach (i; 0 .. WaterlooActionTimeSlots) {
				s = new WaterlooState(3, j, i, false);
				states ~= s;
				terminals ~= s;
				actions[s] ~= new NullAction();
			}

			foreach (i; 0 .. WaterlooActionTimeSlots) {
				s = new WaterlooState(3, j, i, true);
				states ~= s;
				actions[s] ~= new WaterlooReleaseHardAction();
				actions[s] ~= new WaterlooReleaseSoftAction();
			}
			
		}
		
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
		return actions[state];
		
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
		WaterlooState s = cast(WaterlooState)state;
		
		foreach (s2; states) {
			if (s == s2)
				return true;
			
		}
		return false;
	}
	
}	