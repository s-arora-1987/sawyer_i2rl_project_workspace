
import mdp;
import std.array;
import std.format;
import std.math;
import std.stdio;
import std.string;


byte [][] boyd2PatrollerMap() {
	return [[0, 1, 1, 1, 1, 1, 1, 1, 1], 
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 1, 1, 1, 1, 1, 1, 1]];
}

byte [][] boyd2AttackerMap() {
	return [[1, 1, 1, 1, 1, 1, 1, 1, 1], 
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 0, 0, 0, 0, 0, 0, 0],
			     [0, 1, 1, 1, 1, 1, 1, 1, 1]];
}

byte [][] boydrightPatrollerMap() {
	/*
	return [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0], 
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0]];
	*/
	return [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
			[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0]];
}

byte [][] boydright2PatrollerMap() {
	return [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0]];
}

byte [][] boydright2AttackerMap() {
	return [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
		   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0]];
}

byte [][] boydrightAttackerMap() {
	/*
	return [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0]];
	*/
	return [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0]];
}

byte [][] largeGridPatrollerMap() {
	return [[0, 0, 0, 0, 1, 1, 1, 1, 1],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 1, 1, 1, 1],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [1, 1, 1, 1, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [1, 1, 1, 1, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0]];
    /*
	return [[1, 1, 1, 0, 0, 1, 1, 1, 1],
			     [1, 1, 1, 0, 0, 1, 1, 1, 1],
                 [1, 1, 1, 1, 1, 1, 1, 1, 1],
                 [0, 1, 0, 0, 0, 0, 0, 1, 0],
                 [1, 1, 1, 0, 0, 1, 1, 1, 1],
                 [1, 1, 1, 0, 0, 1, 1, 1, 1],
                 [1, 1, 1, 1, 1, 1, 1, 1, 1],
			     [1, 1, 1, 0, 0, 1, 1, 1, 1]];
    */
}

byte [][] largeGridAttackerMap() {
	return [[0, 0, 0, 0, 1, 1, 1, 1, 1],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 1, 1, 1, 1],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [1, 1, 1, 1, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [1, 1, 1, 1, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0]];
}

byte [][] reducedGridPatrollerMap() {
	//return [[0, 0, 0, 0, 0, 0, 0, 0, 0],
     //          [0, 0, 0, 0, 0, 0, 0, 0, 0],
     //          [0, 0, 0, 0, 0, 0, 0, 0, 0],
     //          [0, 0, 0, 0, 0, 0, 0, 0, 0],
     //          [0, 0, 0, 0, 0, 0, 0, 0, 0],
     //          [0, 0, 0, 0, 0, 0, 0, 0, 0],
     //          [1, 1, 1, 1, 1, 0, 0, 0, 0],
     //          [0, 0, 0, 0, 1, 0, 0, 0, 0],
     //          [0, 0, 0, 0, 1, 0, 0, 0, 0],
     //          [1, 1, 1, 1, 1, 0, 0, 0, 0],
     //          [0, 0, 0, 0, 0, 0, 0, 0, 0]];
	return [[0, 0, 0, 0, 1, 1, 1, 1, 1],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0],
               [1, 1, 1, 1, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 1, 0, 0, 0, 0]];
}

double[State] stopErrorModel(Model model, State curState, Action action, State intendedState, double remainder) {

	State nextState = new StopAction().apply(curState);
	double[State] returnval;
	returnval[nextState] = remainder;
	return returnval;
}


double[State] realisticErrorModel(Model model, State curState, Action action, State intendedState, double remainder) {
	// all states within distances of 1 from the intended get an equal share of the remaineder
	State [] possibleStates;
	foreach(s; model.S()) {
		BoydState bs = cast(BoydState)s;
		if (s != intendedState && bs.distanceTo(cast(BoydState)intendedState) <= 1) {
			possibleStates ~= s;
		}
		
	}
	
	double[State] returnval;
	foreach(s; possibleStates) {
		returnval[s] = remainder/possibleStates.length;
	}
	
	return returnval;
}

class BoydState : mdp.State {
	
	public int [] location;
	public this ( int [] location = [0,0,0] ) {
		extension = false;
		setLocation(location);
	}
	
	public int[] getLocation() {
		return location;
	}
	
	public void setLocation(int [] l) {
		assert(l.length == 3);
		
		this.location = l;
		
	}
	
	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "BoydState: [location = %(%s, %) ]", this.location);
		return writer.data; 
	}


	override hash_t toHash() const {
		//return location[0] + location[1] + location[2];
		return (location[0]*100 + location[1])*4 + location[2];
	}	
	
	override bool opEquals(Object o) {
		if (this is o)
			return true;
		BoydState p = cast(BoydState)o;
		
		return p && p.location[0] == location[0] && p.location[1] == location[1] && p.location[2] == location[2];
		
	}
	
	override public bool samePlaceAs(State o) {
		BoydState p = cast(BoydState)o;
		
		return p && p.location[0] == location[0] && p.location[1] == location[1];
		
	}
	
	override int opCmp(Object o) const {
		BoydState p = cast(BoydState)o;

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
	
	public double distanceTo(BoydState otherState) {
		return sqrt( cast(double)(location[0]-otherState.location[0])*(location[0]-otherState.location[0]) + (location[1]-otherState.location[1])*(location[1]-otherState.location[1]));
		
	}
}


class BoydExtendedState : mdp.State {

	public int [] location;
	//public Action last_action;
	public int [] last_location;
    public int current_goal;
	//public this ( int [] location = [0,0,0], int [] last_location = [0,0,0]) { //Action last_action=null ) {
	//	extension = true;
    //   setLocationLastLocation(location, last_location);
	//	//setLocationLastAction(location, last_action);
	//}

	public this (int [] location = [0,0,0], int [] last_location = [0,0,0], int current_goal = 0) { //Action last_action=null ) {
		extension = true;
        setLocationGoalLastLocation(location, last_location, current_goal);
        //setLocationGoal(location, current_goal);
		//setLocationLastAction(location, last_action);
	}

	public int[] getLocation() {
		return location;
	}

	public int getCurrentGoal() {
		return current_goal;
	}

	public int[] getLastLocation() {
		return last_location;
	}

	//public Action getAction() {
	//	return last_action;
	//}

	//public void setLocationLastLocation(int [] l, int [] l_l) {
	//	assert(l.length == 3);
    //
	//	this.location = l;
	//	this.last_location = l_l;
    //
	//}

	//public void setLocationGoal(int [] l, int cg) {
	//	assert(l.length == 3);
    //
	//	this.location = l;
	//	this.current_goal = cg;
    //
	//}

	//public void setLocationLastAction(int [] l, Action l_a) {
	//	assert(l.length == 3);
    //
	//	this.location = l;
	//	this.last_action = l_a;
    //
	//}

	public void setLocationGoalLastLocation(int [] l, int [] l_l, int cg) {
		assert(l.length == 3);

		this.location = l;
		this.last_location = l_l;
		this.current_goal = cg;

	}

	public override string toString() {
		auto writer = appender!string();
		//formattedWrite(writer, "BoydExtendedState: ([location = %(%s, %) ], %s )", this.location, this.last_action.toString());
		//formattedWrite(writer, "BoydExtendedState: ([location = %(%s, %) ], %s )", this.location, this.last_location);
		formattedWrite(writer, "BoydExtendedState: ([location = %(%s, %) ], %s, %s )", this.location, this.last_location, this.current_goal);
		return writer.data;

	}


	override hash_t toHash() const {
		//return location[0] + location[1] + location[2] + current_goal;
		return 2*(4*(9*(10*(4*(9*location[0] + location[1]) + location[2]) + last_location[0]) + last_location[1]) + last_location[2]) + current_goal;
	}

	//override bool opEquals(Object o) {
	//	if (this is o)
	//		return true;
	//	BoydExtendedState p = cast(BoydExtendedState)o;
    //
	//	return p && p.location[0] == location[0] && p.location[1] == location[1] && p.location[2] == location[2]
	//	&& last_action.opEquals(p.last_action);
    //
	//}

	override bool opEquals(Object o) {
		if (this is o)
			return true;
		BoydExtendedState p = cast(BoydExtendedState)o;

		return p && p.location[0] == location[0] && p.location[1] == location[1] && p.location[2] == location[2]
		&& p.current_goal == current_goal && p.last_location[0] == last_location[0] &&
		p.last_location[1] == last_location[1] && p.last_location[2] == last_location[2];
		//return p && p.location[0] == location[0] && p.location[1] == location[1] && p.location[2] == location[2]
		//&& p.last_location[0] == last_location[0] && p.last_location[1] == last_location[1] && p.last_location[2] == last_location[2];

	}

	override public bool samePlaceAs(State o) {
		BoydExtendedState p = cast(BoydExtendedState)o;

		return p && p.location[0] == location[0] && p.location[1] == location[1];

	}

	override int opCmp(Object o) const {
		BoydExtendedState p = cast(BoydExtendedState)o;

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

	public double distanceTo(BoydExtendedState otherState) {
		return sqrt( cast(double)(location[0]-otherState.location[0])*(location[0]-otherState.location[0]) + (location[1]-otherState.location[1])*(location[1]-otherState.location[1]));

	}

}

class BoydExtendedState2 : mdp.State {

	public int [] location;
    public int current_goal;

	public this (int [] location = [0,0,0], int current_goal = 0) {
	    extension = true;
        setLocationGoal(location, current_goal);
	}

	public int[] getLocation() {
		return location;
	}

	public int getCurrentGoal() {
		return current_goal;
	}

	public void setLocationGoal(int [] l, int cg) {
		assert(l.length == 3);

		this.location = l;
		this.current_goal = cg;

	}

	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "BoydExtendedState: ([location = %(%s, %) ], %s )", this.location, this.current_goal);
		return writer.data;

	}


	override hash_t toHash() const {
		//return location[0] + location[1] + location[2] + current_goal;
		return 2*(4*(9*location[0] + location[1]) + location[2]) + current_goal;
	}

	override bool opEquals(Object o) {
		if (this is o)
			return true;
		BoydExtendedState2 p = cast(BoydExtendedState2)o;

		return p && p.location[0] == location[0] && p.location[1] == location[1] && p.location[2] == location[2]
		&& p.current_goal == current_goal;

	}

	override public bool samePlaceAs(State o) {
		BoydExtendedState2 p = cast(BoydExtendedState2)o;

		return p && p.location[0] == location[0] && p.location[1] == location[1];

	}

	override int opCmp(Object o) const {
		BoydExtendedState2 p = cast(BoydExtendedState2)o;

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

	public double distanceTo(BoydExtendedState otherState) {
		return sqrt( cast(double)(location[0]-otherState.location[0])*(location[0]-otherState.location[0]) + (location[1]-otherState.location[1])*(location[1]-otherState.location[1]));

	}

}

public int check_goalprogress_and_increment(int [] next_loc, int prev_goal) {

    int result = prev_goal;
    if ((next_loc == [6,0,2]) && ((prev_goal == 4))) {
        result = 1;
    }
    if ((next_loc == [0,8,0]) && ((prev_goal == 1))) {
        result = 2;
    }
    if ((next_loc == [3,8,0]) && ((prev_goal == 2))) {
        result = 3;
    }
    if ((next_loc == [9,0,2]) && ((prev_goal == 3))) {
        result = 4;
    }

    return result;
}

public class MoveForwardAction : Action {
	
	public override State apply(State state) {

	    if (state.extension) {

	        //BoydExtendedState p = cast(BoydExtendedState)state;
	        BoydExtendedState2 p = cast(BoydExtendedState2)state;
            int prevcurr_goal = p.current_goal;
            int [] ll = p.location.dup;
            int orientation = ll[2];
            int [] s = ll.dup;
            if (orientation == 0)
                s[1] += 1;
            if (orientation == 1)
                s[0] -= 1;
            if (orientation == 2)
                s[1] -= 1;
            if (orientation == 3)
                s[0] += 1;

            int nextcurr_goal = check_goalprogress_and_increment(s,prevcurr_goal);
            return new BoydExtendedState2(s, nextcurr_goal);
            //return new BoydExtendedState(s, ll, nextcurr_goal);
            //return new BoydExtendedState(s, new MoveForwardAction());

	    } else {

	        //writeln("MoveForwardAction else ");
	        BoydState p = cast(BoydState)state;

            int orientation = p.getLocation()[2];

            int [] s = p.getLocation().dup;
            if (orientation == 0)
                s[1] += 1;
            if (orientation == 1)
                s[0] -= 1;
            if (orientation == 2)
                s[1] -= 1;
            if (orientation == 3)
                s[0] += 1;

            return new BoydState(s);

	    }

	}
	
	public override string toString() {
		return "MoveForwardAction"; 
	}


	override hash_t toHash() {
		return 0;
	}	
	
	override bool opEquals(Object o) {
		MoveForwardAction p = cast(MoveForwardAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		MoveForwardAction p = cast(MoveForwardAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public int check_goalprogress_and_incrementR(int [] next_loc, int prev_goal) {

    int result = prev_goal;
    if ((next_loc == [0,8,0]) && (prev_goal == 4)) {
        result = 2;
    }
    if ((next_loc == [9,0,2]) && (prev_goal == 2)) {
        result = 4;
    }

    return result;
}


public class MoveForwardActionR : Action {

	public override State apply(State state) {

	    if (state.extension) {

	        //BoydExtendedState p = cast(BoydExtendedState)state;
	        BoydExtendedState2 p = cast(BoydExtendedState2)state;
            int prevcurr_goal = p.current_goal;
            int [] ll = p.location.dup;
            int orientation = ll[2];
            int [] s = ll.dup;
            if (orientation == 0)
                s[1] += 1;
            if (orientation == 1)
                s[0] -= 1;
            if (orientation == 2)
                s[1] -= 1;
            if (orientation == 3)
                s[0] += 1;

            int nextcurr_goal = check_goalprogress_and_incrementR(s,prevcurr_goal);
            return new BoydExtendedState2(s, nextcurr_goal);
            //return new BoydExtendedState(s, ll, nextcurr_goal);
            //return new BoydExtendedState(s, new MoveForwardAction());

	    } else {

	        //writeln("MoveForwardAction else ");
	        BoydState p = cast(BoydState)state;

            int orientation = p.getLocation()[2];

            int [] s = p.getLocation().dup;
            if (orientation == 0)
                s[1] += 1;
            if (orientation == 1)
                s[0] -= 1;
            if (orientation == 2)
                s[1] -= 1;
            if (orientation == 3)
                s[0] += 1;

            return new BoydState(s);

	    }

	}

	public override string toString() {
		return "MoveForwardAction";
	}


	override hash_t toHash() {
		return 0;
	}

	override bool opEquals(Object o) {
		MoveForwardAction p = cast(MoveForwardAction)o;

		return p && true;

	}

	override int opCmp(Object o) {
		MoveForwardAction p = cast(MoveForwardAction)o;

		if (!p)
			return -1;

		return 0;

	}
}

public class StopAction : Action {
	
	public override State apply(State state) {

	    if (state.extension) {

	        //BoydExtendedState p = cast(BoydExtendedState)state;
	        BoydExtendedState2 p = cast(BoydExtendedState2)state;

            return new BoydExtendedState2(p.getLocation().dup, p.getCurrentGoal());
            //return new BoydExtendedState(p.getLocation().dup, p.getLocation().dup, p.getCurrentGoal());

	    } else {

            BoydState p = cast(BoydState)state;

            return new BoydState(p.getLocation().dup);

	    }

	}
	
	public override string toString() {
		return "StopAction"; 
	}


	override hash_t toHash() {
		return 3;
	}	
	
	override bool opEquals(Object o) {
		StopAction p = cast(StopAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		StopAction p = cast(StopAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class TurnLeftAction : Action {
	
	public override State apply(State state) {

	    if (state.extension) {

	        //BoydExtendedState p = cast(BoydExtendedState)state;
	        BoydExtendedState2 p = cast(BoydExtendedState2)state;

            int [] ll = p.location;
            int cg = p.current_goal;
            int orientation = ll[2] + 1;

            if (orientation > 3)
                orientation = 0;

            int [] s = ll.dup;
            s[2] = orientation;

            //return new BoydExtendedState(s, new TurnLeftAction());
            return new BoydExtendedState2(s, cg);
            //return new BoydExtendedState(s, ll, cg);

	    } else {

            BoydState p = cast(BoydState)state;

            int orientation = p.getLocation()[2] + 1;

            if (orientation > 3)
                orientation = 0;

            int [] s = p.getLocation().dup;
            s[2] = orientation;

            return new BoydState(s);

	    }

	}
	
	public override string toString() {
		return "TurnLeftAction"; 
	}


	override hash_t toHash() {
		return 1;
	}	
	
	override bool opEquals(Object o) {
		TurnLeftAction p = cast(TurnLeftAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		TurnLeftAction p = cast(TurnLeftAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class TurnAroundAction : Action {
	
	public override State apply(State state) {

	    if (state.extension) {

	        //BoydExtendedState p = cast(BoydExtendedState)state;
	        BoydExtendedState2 p = cast(BoydExtendedState2)state;

            int orientation = p.getLocation()[2] + 2;

            if (orientation > 3)
                orientation -= 4;

            int [] s = p.getLocation().dup;
            s[2] = orientation;

            //return new BoydExtendedState(s, new TurnAroundAction());
            return new BoydExtendedState2(s, p.getCurrentGoal());
            //return new BoydExtendedState(s, p.getLocation().dup, p.getCurrentGoal());

	    } else {

	        //writeln("TurnAroundAction else ");
            BoydState p = cast(BoydState)state;

            int orientation = p.getLocation()[2] + 2;

            if (orientation > 3)
                orientation -= 4;

            int [] s = p.getLocation().dup;
            s[2] = orientation;

            return new BoydState(s);

	    }

	}
	
	public override string toString() {
		return "TurnAroundAction"; 
	}


	override hash_t toHash() {
		return 4;
	}	
	
	override bool opEquals(Object o) {
		TurnAroundAction p = cast(TurnAroundAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		TurnAroundAction p = cast(TurnAroundAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class TurnRightAction : Action {
	
	public override State apply(State state) {

	    if (state.extension) {

	        //BoydExtendedState p = cast(BoydExtendedState)state;
	        BoydExtendedState2 p = cast(BoydExtendedState2)state;

            int [] ll = p.location;
            int cg = p.current_goal;
            int orientation = ll[2] - 1;

            if (orientation < 0)
                orientation = 3;

            int [] s = p.getLocation().dup;
            s[2] = orientation;

            //return new BoydExtendedState(s, new TurnRightAction());
            return new BoydExtendedState2(s, cg);
            //return new BoydExtendedState(s, ll, cg);

	    } else {

	        //writeln("TurnRightAction else ");
            BoydState p = cast(BoydState)state;

            int orientation = p.getLocation()[2] - 1;

            if (orientation < 0)
                orientation = 3;

            int [] s = p.getLocation().dup;
            s[2] = orientation;

            return new BoydState(s);
	    }

	}
	
	public override string toString() {
		return "TurnRightAction"; 
	}


	override hash_t toHash() {
		return 2;
	}	
	
	override bool opEquals(Object o) {
		TurnRightAction p = cast(TurnRightAction)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		TurnRightAction p = cast(TurnRightAction)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

int [] simplefeatures(State state, Action action) {
	return [1];
}


class BoydModel : mdp.Model {
	
	byte [][] map;
	Action [] actions;
	BoydState [] states;
	State terminal;
	double [State][Action][State] t;
	int [] function(State, Action) ff;
	int numFeatures;
	double[State] uniform;
	
	public this( State terminal, byte [][] themap, double [State][Action][State] newT, int numFeatures, int [] function(State, Action) ff) {
		this.t = newT;
		this.terminal = terminal;
		this.map = themap;
		this.numFeatures = numFeatures;
		this.ff = ff;
		
		this.actions ~= new MoveForwardAction();
		this.actions ~= new TurnLeftAction();
		this.actions ~= new TurnRightAction();
		this.actions ~= new StopAction();
//		this.actions ~= new TurnAroundAction();

		for (int i = 0; i < map.length; i ++) {
			
			for (int j = 0; j < map[i].length; j ++) {
				if (map[i][j] == 1) {
					states ~= new BoydState([i, j, 0]);
					states ~= new BoydState([i, j, 1]);
					states ~= new BoydState([i, j, 2]);
					states ~= new BoydState([i, j, 3]);
				}
			}
		}
		
		
		foreach (s; states) {
			uniform[s] = 1.0/states.length;
		}
		  
	}
	
	public override int numTFeatures() {
		return numFeatures;
	}
		
	public override int [] TFeatures(State state, Action action) {
		return ff(state, action);
	}
	
	public void setT(double[State][Action][State] newT) {
		t = newT;
	}
	
	
	public override double[State] T(State state, Action action) {
		return (state in t && action in t[state]) ? t[state][action] : uniform ;
	}
	
	public override State [] S () {
		return cast(State[])states;
	}
	
	public override Action[] A(State state = null) {
		return actions;
		
	}
	
	public override bool is_terminal(State state) {

		return state == terminal;
	}
	
	public override bool is_legal(State state) {
		BoydState s = cast(BoydState)state;
		int [] l = s.location;
		
		return l[0] >= 0 && l[0] < map.length && l[1] >= 0 && l[1] < map[0].length && map[l[0]][l[1]] == 1; 
	}


}

class BoydModelWdObsFeatures : BoydModel {
	// This class takes has as its member an observation feature function 
	// for estimating observation model
	int numObFeatures;
	// observation model to be estimated 
	double [StateAction][StateAction] obsMod;

	public this( State terminal, byte [][] themap, double [State][Action][State] newT, int inpNumObFeatures)
	//, int [] function(State, Action) of) 
		{
		super(terminal, themap, newT, 1, &simplefeatures);
		// observation features
		this.numObFeatures = inpNumObFeatures;
			
	}

	public override int [] obsFeatures(State state, Action action) {
		
		//if (cast(TurnRightAction)action) { 
		//	return [0, 1, 1, 1];
		//} 

		//if (cast(TurnLeftAction)action) { 
		//	return [1, 0, 1, 1];
		//} 
		
		//if (cast(StopAction)action) {
		//	return [1, 1, 0, 1];
		//} 
		//return [1, 1, 1, 1];

		//if (cast(TurnLeftAction)action) { 
		//	return [1,0];
		//} 
		//return [1,1];

		//if (cast(MoveForwardAction)action && (cast(BoydState)state).getLocation()[1]==0) { 
		//	return [1,0,0];
		//} 
		//if (cast(TurnLeftAction)action ) { 
		//	return [0,1,0];
		//} 
		//return [0,0,1];

		// location at y=0		
		int [] result;
		// This is where number of features is decided 
		result.length = 4;
		result[] = 0;

		if (cast(MoveForwardAction)action) result[0] = 1;

		if (cast(TurnLeftAction)action )  result[1] = 1;

		if ((cast(BoydState)state).getLocation()[1]==0) result[2] = 1;

		if ((cast(BoydState)state).getLocation()[0]==0) result[3] = 1;

		return result;

	}

	override public void setNumObFeatures(int inpNumObFeatures) {
		this.numObFeatures = inpNumObFeatures;
	}

	override public int getNumObFeatures() {
		return numObFeatures;
	}
		
	//public int [] obFeatures(State state, Action action) {
	//	return of(state, action);
	//}

	override public void setObsMod(double [StateAction][StateAction] newObsMod) {
		obsMod = newObsMod;
	}

}


class BoydExtendedModel : mdp.Model {

	byte [][] map;
	Action [] actions;
	BoydExtendedState [] states;
	BoydExtendedState terminal;
	double [State][Action][State] t;
	int [] function(State, Action) ff;
	int numFeatures;
	double[State] uniform;

	public this( BoydExtendedState terminal, byte [][] themap, double [State][Action][State] newT, int numFeatures, int [] function(State, Action) ff) {
		this.t = newT;
		this.terminal = terminal;
		this.map = themap;
		this.numFeatures = numFeatures;
		this.ff = ff;

		this.actions ~= new MoveForwardAction();
		this.actions ~= new TurnLeftAction();
		this.actions ~= new TurnRightAction();
		this.actions ~= new StopAction();
//		this.actions ~= new TurnAroundAction();
        BoydState [] states2;

		for (int i = 0; i < map.length; i ++) {

			for (int j = 0; j < map[i].length; j ++) {
				if (map[i][j] == 1) {
					states2 ~= new BoydState([i, j, 0]);
					states2 ~= new BoydState([i, j, 1]);
					states2 ~= new BoydState([i, j, 2]);
					states2 ~= new BoydState([i, j, 3]);
				}
			}
		}

        bool nextstatelegal, validgoal, validgoal0, validgoal4, validgoal1, validgoal2, validgoal3;
        State ns;
        int [] l;
        //foreach (Action act; actions) {
        //    foreach (BoydState bdst; states2) {
        //
        //        ns = act.apply(bdst);
        //        l= (cast(BoydState)ns).location;
        //        pair_valid = l[0] >= 0 && l[0] < map.length && l[1] >= 0 && l[1] < map[0].length && map[l[0]][l[1]] == 1;
        //
        //        if (pair_valid) {
        //            //states ~= new BoydExtendedState(l, act);
        //            states ~= new BoydExtendedState(l, bdst.location);
        //        }
        //    }
        //}

        //int [] goals = [0,1,2,3,4];
        //foreach (int g; goals) {
        //    foreach (BoydState bdst; states2) {
        //        l= bdst.location.dup;
        //        states ~= new BoydExtendedState(l, g);
        //    }
        //}

        int [] goals = [0,1,2,3,4];
        foreach (int g; goals) {
            foreach (Action act; actions) {
                foreach (BoydState bdst; states2) {

                    ns = act.apply(bdst);
                    l= (cast(BoydState)ns).location;
                    nextstatelegal = l[0] >= 0 && l[0] < map.length && l[1] >= 0 && l[1] < map[0].length && map[l[0]][l[1]] == 1;

                    validgoal0 = g==0 && l[0]>=6 && l[0 .. 2] != [6,0];
                    validgoal4 = g==4 && l[0]>=6 && l[0 .. 2] != [6,0];
                    validgoal1 = g==1 && l[0]<=6 && !(l[0] == 3 && l[1] > 4) && l[0 .. 2] != [0,8];
                    validgoal2 = g==2 && l[0]<=3 && l[0 .. 2] != [3,8];
                    validgoal3 = g==3 && l[0]>=3 && !(l[0] == 6 && l[1] < 4) && l[0 .. 2] != [9,0];
                    validgoal = validgoal0 || validgoal4 || validgoal1 || validgoal2 || validgoal3;

                    if (nextstatelegal && validgoal) {
                        states ~= new BoydExtendedState(l, bdst.location, g);
                    }
                }
            }
        }
        //writeln(states.length);
		foreach (s; states) {
			uniform[s] = 1.0/states.length;
		}

	}

	public override int numTFeatures() {
		return numFeatures;
	}

	public override int [] TFeatures(State state, Action action) {
		return ff(state, action);
	}

	public void setT(double[State][Action][State] newT) {
		t = newT;
	}


	public override double[State] T(State state, Action action) {
		return (state in t && action in t[state]) ? t[state][action] : uniform ;
	}

	public override State [] S () {
		return cast(State[])states;
	}

	public override Action[] A(State state = null) {
		return actions;

	}

	public override bool is_terminal(State state) {
		return (cast(BoydExtendedState)state).getLocation() == terminal.getLocation();
	}

	public override bool is_legal(State state) {
		BoydExtendedState s = cast(BoydExtendedState)state;
		int [] l = s.location;
        int [] ll = s.last_location;
        int cg = s.current_goal;
		bool legal_l = l[0] >= 0 && l[0] < map.length && l[1] >= 0 && l[1] < map[0].length && map[l[0]][l[1]] == 1;
		bool legal_ll = ll[0] >= 0 && ll[0] < map.length && ll[1] >= 0 && ll[1] < map[0].length && map[ll[0]][ll[1]] == 1;
		bool legal_cg = cg >= 0 && cg <= 4;
		return legal_l && legal_ll && legal_cg;
	}
}

class BoydExtendedModel2 : mdp.Model {

	byte [][] map;
	Action [] actions;
	BoydExtendedState2 [] states;
	BoydExtendedState2 terminal;
	double [State][Action][State] t;
	int [] function(State, Action) ff;
	int numFeatures;
	double[State] uniform;

	public this( BoydExtendedState2 terminal, byte [][] themap, double [State][Action][State] newT, int numFeatures, int [] function(State, Action) ff) {
		this.t = newT;
		this.terminal = terminal;
		this.map = themap;
		this.numFeatures = numFeatures;
		this.ff = ff;

		this.actions ~= new MoveForwardAction();
		this.actions ~= new TurnLeftAction();
		this.actions ~= new TurnRightAction();
		this.actions ~= new StopAction();
//		this.actions ~= new TurnAroundAction();
        BoydState [] states2;

		for (int i = 0; i < map.length; i ++) {

			for (int j = 0; j < map[i].length; j ++) {
				if (map[i][j] == 1) {
					states2 ~= new BoydState([i, j, 0]);
					states2 ~= new BoydState([i, j, 1]);
					states2 ~= new BoydState([i, j, 2]);
					states2 ~= new BoydState([i, j, 3]);
				}
			}
		}

        bool legal_l, validgoal, validgoal0, validgoal4, validgoal1, validgoal2, validgoal3;
        State ns;
        int [] l;

        int [] goals = [0,1,2,3,4];
        foreach (int g; goals) {
            foreach (BoydState bdst; states2) {

                l= bdst.location;
                legal_l = l[0] >= 0 && l[0] < map.length && l[1] >= 0 && l[1] < map[0].length && map[l[0]][l[1]] == 1;
                //validgoal0 = g==0;
                //validgoal0 = g==0 && ((l[0]>=6 && l[0 .. 2] != [6,0]) || (l[1]==4)) ;
                validgoal4 = g==4 && l[0]>=6 && l[0 .. 2] != [6,0];
                validgoal1 = g==1 && l[0]<=6 && !(l[0] == 3 && l[1] > 4) && l[0 .. 2] != [0,8];
                validgoal2 = g==2 && l[0]<=3 && l[0 .. 2] != [3,8];
                validgoal3 = g==3 && l[0]>=3 && !(l[0] == 6 && l[1] < 4) && l[0 .. 2] != [9,0];
                //validgoal = validgoal0 || validgoal4 || validgoal1 || validgoal2 || validgoal3;
                validgoal = validgoal4 || validgoal1 || validgoal2 || validgoal3;

                if (legal_l && validgoal) {
                    states ~= new BoydExtendedState2(l, g);
                }
            }
        }
        //writeln("states.length :",states.length);
		foreach (s; states) {
			uniform[s] = 1.0/states.length;
		}

	}

	public override int numTFeatures() {
		return numFeatures;
	}

	public override int [] TFeatures(State state, Action action) {
		return ff(state, action);
	}

    public double[State][Action][State] createTransitionFunctionSimple2(double success) {

        double[State][Action][State] returnval;
        auto states = S();
        double success_t = success;

        foreach(State state; states) { // for each state

            if (is_terminal(state)) { // if it's terminal, use 1.0 as transition prob
                returnval[state][new NullAction()][state] = 1.0;
                continue;
            }

            BoydExtendedState2 bes = cast(BoydExtendedState2)state;
            foreach(Action action; A(state)) { // for each action
                State next_st = action.apply(state); // compute intended next state
                BoydExtendedState2 bes2 = cast(BoydExtendedState2)next_st;

                if (! is_legal(bes2) || bes2.location==bes.location) { // if intended one not legal, stay at same state
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

	public void setT(double[State][Action][State] newT) {
		t = newT;
	}


	public override double[State] T(State state, Action action) {
		return (state in t && action in t[state]) ? t[state][action] : uniform ;
	}

	public override State [] S () {
		return cast(State[])states;
	}

	public override Action[] A(State state = null) {
		return actions;

	}

	public override bool is_terminal(State state) {
		return (cast(BoydExtendedState2)state).getLocation() == terminal.getLocation();
	}

	public override bool is_legal(State state) {
		BoydExtendedState2 s = cast(BoydExtendedState2)state;
		int [] l = s.location;
        int g = s.current_goal;
		bool legal_l = l[0] >= 0 && l[0] < map.length && l[1] >= 0 && l[1] < map[0].length && map[l[0]][l[1]] == 1;
		bool legal_cg = g >= 0 && g <= 4;
		bool validgoal, validgoal0, validgoal4, validgoal1, validgoal2, validgoal3;
        //validgoal0 = g==0;
        validgoal4 = g==4 && l[0]>=6 && l[0 .. 2] != [6,0];
        validgoal1 = g==1 && l[0]<=6 && !(l[0] == 3 && l[1] > 4) && l[0 .. 2] != [0,8];
        validgoal2 = g==2 && l[0]<=3 && l[0 .. 2] != [3,8];
        validgoal3 = g==3 && l[0]>=3 && !(l[0] == 6 && l[1] < 4) && l[0 .. 2] != [9,0];
        //validgoal = validgoal0 || validgoal4 || validgoal1 || validgoal2 || validgoal3;
        validgoal = validgoal4 || validgoal1 || validgoal2 || validgoal3;
		return legal_l && legal_cg && validgoal;
	}
}

class BoydExtendedModel2R : mdp.Model {

	byte [][] map;
	Action [] actions;
	BoydExtendedState2 [] states;
	BoydExtendedState2 terminal;
	double [State][Action][State] t;
	int [] function(State, Action) ff;
	int numFeatures;
	double[State] uniform;

	public this( BoydExtendedState2 terminal, byte [][] themap, double [State][Action][State] newT, int numFeatures, int [] function(State, Action) ff ) {
		this.t = newT;
		this.terminal = terminal;
		this.map = themap;
		this.numFeatures = numFeatures;
		this.ff = ff;

		this.actions ~= new MoveForwardActionR();
		this.actions ~= new TurnLeftAction();
		this.actions ~= new TurnRightAction();
		this.actions ~= new StopAction();
//		this.actions ~= new TurnAroundAction();
        BoydState [] states2;

		for (int i = 0; i < map.length; i ++) {

			for (int j = 0; j < map[i].length; j ++) {
				if (map[i][j] == 1) {
					states2 ~= new BoydState([i, j, 0]);
					states2 ~= new BoydState([i, j, 1]);
					states2 ~= new BoydState([i, j, 2]);
					states2 ~= new BoydState([i, j, 3]);
				}
			}
		}

        bool legal_l, validgoal, validgoal2, validgoal4, validgoal1;
        State ns;
        int [] l;

        int [] goals = [2,4];
        foreach (int g; goals) {
            foreach (BoydState bdst; states2) {

                l= bdst.location;
                legal_l = l[0] >= 0 && l[0] < map.length && l[1] >= 0 && l[1] < map[0].length && map[l[0]][l[1]] == 1;
                //validgoal0 = g==0;
                //validgoal0 = g==0 && ((l[0]>=6 && l[0 .. 2] != [6,0]) || (l[1]==4)) ;
                validgoal4 = g==4 && l[0 .. 2] != [0,8];
                validgoal2 = g==2 && l[0 .. 2] != [9,0];
                //validgoal = validgoal0 || validgoal4 || validgoal1 || validgoal2 || validgoal3;
                validgoal = validgoal4 || validgoal2;

                if (legal_l && validgoal) {
                    states ~= new BoydExtendedState2(l, g);
                }
            }
        }
        //writeln(states);
        //writeln("states.length :",states.length);
		foreach (s; states) {
			uniform[s] = 1.0/states.length;
		}

	}

	public override int numTFeatures() {
		return numFeatures;
	}

	public override int [] TFeatures(State state, Action action) {
		return ff(state, action);
	}

    public double[State][Action][State] createTransitionFunctionSimple2(double success) {

        double[State][Action][State] returnval;
        auto states = S();
        double success_t = success;

        foreach(State state; states) { // for each state

            if (is_terminal(state)) { // if it's terminal, use 1.0 as transition prob
                returnval[state][new NullAction()][state] = 1.0;
                continue;
            }

            BoydExtendedState2 bes = cast(BoydExtendedState2)state;
            foreach(Action action; A(state)) { // for each action
                State next_st = action.apply(state); // compute intended next state
                BoydExtendedState2 bes2 = cast(BoydExtendedState2)next_st;

                if (! is_legal(bes2) || bes2.location==bes.location) { // if intended one not legal, stay at same state
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

	public void setT(double[State][Action][State] newT) {
		t = newT;
	}


	public override double[State] T(State state, Action action) {
		return (state in t && action in t[state]) ? t[state][action] : uniform ;
	}

	public override State [] S () {
		return cast(State[])states;
	}

	public override Action[] A(State state = null) {
		return actions;

	}

	public override bool is_terminal(State state) {
		return (cast(BoydExtendedState2)state).getLocation() == terminal.getLocation();
	}

	public override bool is_legal(State state) {
		BoydExtendedState2 s = cast(BoydExtendedState2)state;
		int [] l = s.location;
        int g = s.current_goal;
		bool legal_l = l[0] >= 0 && l[0] < map.length && l[1] >= 0 && l[1] < map[0].length && map[l[0]][l[1]] == 1;
		bool legal_cg = g >= 0 && g <= 4;
		bool validgoal, validgoal4, validgoal1, validgoal2;
        //validgoal0 = g==0;
        validgoal4 = g==4 && l[0 .. 2] != [0,8];
        validgoal2 = g==2 && l[0 .. 2] != [9,0];
        //validgoal = validgoal0 || validgoal4 || validgoal1 || validgoal2 || validgoal3;
        validgoal = validgoal4 || validgoal2;
		return legal_l && legal_cg && validgoal;
	}
}


class BoydRightReward : LinearReward {
	
	Model model;
	
	public this (Model model) {
		
		this.model = model;
	}
	
	public override int dim() {
		return 5;
	}
	
	
	
			
	public override double [] features(State state, Action action) {
		State newState = action.apply(state);
		
		bool moved = true;
		
		if (! model.is_legal(newState) || newState.samePlaceAs(state)) 
			moved = false;
		
		double [] result = new double[dim()];
		result[] = 0;
		
		
//		MoveForwardAction test = cast(MoveForwardAction) action;
//		if (test && moved)
		if (moved)
			result[0] = 1;
		else
			result[0] = 0;
		
        	BoydState s = cast(BoydState)state;
        
		// the reward for not being in the hallway
		if ((s.location[1] < 16 && s.location[0] > 0 && s.location[0] < 13)) 
			result[1] = 1;
		else
			result[1] = 0;
    
		// reward for turning around in the hallways
		result[2] = cast(TurnLeftAction)action !is null  && ( (s.location[0] == 0) ||
		(s.location[1] == 16))? 1 : 0;
    	
		if (moved) {

/*
	    	if (s.location[0] == 0 && s.location[1] < 7) {
	    		result[2] = 1;
	    	}
	        
	    	if (s.location[1] == 7) {
	    		result[3] = 1;
	    	} */ 
        	
/*
        	if ((s.location[0] == 0 && s.location[1] == 0) || (s.location[0] == 5 && s.location[1] == 7)) {
        		result[2] = 1;
        	} */
        	
        	// Reward for moving into the rooms from the hallway
        	BoydState news = cast(BoydState)newState;
        	if (!(s.location[1] < 16 && s.location[0] > 0 && s.location[0] < 13)) {
        		if ((news.location[1] < 16 && news.location[0] > 0 && news.location[0] < 13)) {
        			result[3] = 1;
        		} 
        		
        	}
        	
        	// reward for moving into the hallway from the rooms
        	if (!(news.location[1] < 16 && news.location[0] > 0 && news.location[0] < 13)) {
        		if ((s.location[1] < 16 && s.location[0] > 0 && s.location[0] < 13)){
        			result[4] = 1;
        		} 
        		
        	} 

/*	        // distance from top left
	        result[2] = sqrt(cast(double)(s.location[1] * s.location[1]) + (s.location[0]*s.location[0]));	
	        // distance from top right
	        result[3] = sqrt(cast(double)((s.location[1]-8) * (s.location[1]-8)) + (s.location[0]*s.location[0]));
	        // distance from bottom left
	        result[4] = sqrt(cast(double)(s.location[1] - 0)*(s.location[1] - 0) + (s.location[0] - 5)*(s.location[0] - 5));
	        // distance from bottom right
	        result[5] = sqrt(cast(double)(s.location[1] - 8)*(s.location[1] - 8) + (s.location[0] - 5)*(s.location[0] - 5)); */
        }
        	
		return result;
	}

}

class BoydRight2Reward : LinearReward {
	
	Model model;
	MoveForwardAction moveForwardAction;
	Action turnAroundAction;
	Action otherTurnAction;

	public this (Model model) {
		this.model = model;

		this.moveForwardAction = new MoveForwardAction();
		this.turnAroundAction = new TurnLeftAction();
		this.otherTurnAction = new TurnRightAction();
	}

	public override int dim() {
		return 7;
	}

	public override double [] features(State state, Action action) {
		State newState = action.apply(state);

		bool moved = true;

		if (! model.is_legal(newState) || newState.samePlaceAs(state))
			moved = false;

		double [] result;
		result.length = dim();
		result[] = 0;

		if (moved)
			result[0] = 1;

		/* we want features that represent a turn around at a certain distance value
		 * but we only have single turn actions (when the turnaround takes at least two
		 * so maybe we can represent a turn around by a turn that results in face a direction we can move (since boyd2 is a hallway, this
		 * feature won't be activated when turning to face a wall)  This in combination with a moveforward feature should let me
		 * specify a point to turn around at (and have this learned from observations too)
		 */
		// if you could move forward, then choose the turnleft action only (to bias to one side)
		// otherwise this feature applies to both left and right

		bool couldTurnAround = model.is_legal(moveForwardAction.apply(turnAroundAction.apply(turnAroundAction.apply(state)))) && model.is_legal(moveForwardAction.apply(state));

		if (model.is_legal(newState) && ! moved ) {
				// when action is turning, activate a feature based on current region of state space
				if (action == turnAroundAction || action == otherTurnAction) {
					BoydState bs = cast(BoydState)state;
					// (s.location[1] < 16 && s.location[0] > 0 && s.location[0] < 13)
					if (bs.location[0] >= 1 && bs.location[0] <= 13)
						result[1] = 1;
					if ((bs.location[0] <= 1 || bs.location[0] >= 13) && (bs.location[1] >= 9 && bs.location[1] <= 11))
						result[2] = 1;
					if (bs.location[1] >= 0 && bs.location[1] <= 4)
						result[3] = 1;
					if (bs.location[1] >= 4 && bs.location[1] <= 9)
						result[4] = 1;
					if (bs.location[1] >= 11 && bs.location[1] <= 13)
						result[5] = 1;
					if (bs.location[1] >= 14 && bs.location[1] <= 16)
						result[6] = 1;
				}

		}

		return result;
	}

}


class Boyd2Reward : LinearReward {

	Model model;
	int[State] distance;
	int maxDistance;
	MoveForwardAction moveForwardAction;
	Action turnAroundAction;
	Action otherTurnAction;

	public this (Model model, int[State] distance) {
		this.model = model;
		this.distance = distance;

		if (distance.length > 0) {
			int [] distances = distance.values;
			foreach (d; distances) {
				if (d > maxDistance)
					maxDistance = d;
			}
		}

		this.moveForwardAction = new MoveForwardAction();
		this.turnAroundAction = new TurnLeftAction();
		this.otherTurnAction = new TurnRightAction();
	}

	public override int dim() {
		return 2 + maxDistance;
	}

	public override double [] features(State state, Action action) {
		State newState = action.apply(state);

		bool moved = true;

		if (! model.is_legal(newState) || newState.samePlaceAs(state))
			moved = false;

		double [] result;
		result.length = dim();
		result[] = 0;

		if (moved)
			result[0] = 1;

		auto stateDistance = distance[state];

		/* we want features that represent a turn around at a certain distance value
		 * but we only have single turn actions (when the turnaround takes at least two
		 * so maybe we can represent a turn around by a turn that results in face a direction we can move (since boyd2 is a hallway, this
		 * feature won't be activated when turning to face a wall)  This in combination with a moveforward feature should let me
		 * specify a point to turn around at (and have this learned from observations too)
		 */
		// if you could move forward, then choose the turnleft action only (to bias to one side)
		// otherwise this feature applies to both left and right

		bool couldTurnAround = model.is_legal(moveForwardAction.apply(turnAroundAction.apply(turnAroundAction.apply(state)))) && model.is_legal(moveForwardAction.apply(state));

		if (model.is_legal(newState) && ! moved ) {

			if (couldTurnAround) {
				// require the turn around action

				if (action == turnAroundAction) {
					result[stateDistance + 1] = 1;
				}


			} else {

				if (action == turnAroundAction || action == otherTurnAction) {
					result[stateDistance + 1] = 1;
				}

			}
		}


		return result;
	}

}

class Boyd2RewardAlt : LinearReward {
	
	Model model;
	int[State] distance;
	int maxDistance;
	MoveForwardAction moveForwardAction;
	Action turnAroundAction;
	Action otherTurnAction;
	BoydState centerPoint;
	
	public this (Model model, BoydState centerPoint) {
		this.model = model;
		this.centerPoint = centerPoint;
		
		// assign distance based on manhattan distance
		
		
		foreach(s; model.S()) {
			BoydState bs = cast(BoydState)s;
			
			distance[s] = abs(bs.location[0] - centerPoint.location[0]) + abs(bs.location[1] - centerPoint.location[1]);
			
		}
		
		
/*		int counter = 0;
		foreach(s; model.S()) {
			bool found = false;
			foreach(already_seen, d; distance) {
				if (already_seen.samePlaceAs(s) ){
					distance[s] = d;
					found = true;
					break;
				}	
			}
			if (! found) {
				distance[s] = counter;
				counter ++;
			}
			
		}
		this.distance = dist;*/
		
		if (distance.length > 0) {
			int [] distances = distance.values;
			foreach (d; distances) {
				if (d > maxDistance)
					maxDistance = d;
			}
		}
		
		this.moveForwardAction = new MoveForwardAction();
		this.turnAroundAction = new TurnLeftAction();
		this.otherTurnAction = new TurnRightAction();
	}
	
	public override int dim() {
		return 3 + maxDistance;
	}
			
	public override double [] features(State state, Action action) {
		State newState = action.apply(state);
		
		bool moved = true;
		
		if (! model.is_legal(newState) || newState.samePlaceAs(state)) 
			moved = false;
		
		double [] result;
		result.length = dim();
		result[] = 0;
		
		if (moved) 
			result[0] = 1;
		
		auto stateDistance = distance[state];
		
		/* we want features that represent a turn around at a certain distance value
		 * but we only have single turn actions (when the turnaround takes at least two
		 * so maybe we can represent a turn around by a turn that results in face a direction we can move (since boyd2 is a hallway, this 
		 * feature won't be activated when turning to face a wall)  This in combination with a moveforward feature should let me 
		 * specify a point to turn around at (and have this learned from observations too)
		 */
		// if you could move forward, then choose the turnleft action only (to bias to one side)
		// otherwise this feature applies to both left and right


		// need to have all turns at the same location count as the same feature

		//fuck it, just hard code this shit
		
		if (action == turnAroundAction || action == otherTurnAction) {
		
			BoydState bs = cast(BoydState)state;
			BoydState nbs = cast(BoydState)newState;
			
			if (bs.location[0] < 0 && bs.location[0] < centerPoint.location[0]) {
				if (bs.location[2] == 1) {
					result[stateDistance + 1] = 1;
				} else if (bs.location[2] == 0) {
					result[stateDistance + 1] = 1;
				} else if (bs.location[2] == 2) {
					result[stateDistance + 1] = 1;					
				}
				
			} else if (bs.location[0] > centerPoint.location[0] && bs.location[0] < 16) {
				if (bs.location[2] == 3) {
					result[stateDistance + 1] = 1;
				} else if (bs.location[2] == 0) {
					result[stateDistance + 1] = 1;
				} else if (bs.location[2] == 2) {
					result[stateDistance + 1] = 1;					
				}
				
			} else if (bs.location[1] > 1) {
				if (bs.location[2] == 0) {
					result[stateDistance + 1] = 1;
				} else if (bs.location[2] == 1 && action == turnAroundAction) {
					result[stateDistance + 1] = 1;
				} else if (bs.location[2] == 3 && action == otherTurnAction) {
					result[stateDistance + 1] = 1;					
				}
				
			} else if (bs.location[0] == 0) {
				if (bs.location[2] == 0 || bs.location[2] == 3) {
					result[stateDistance + 1] = 1;
				} else if (bs.location[2] == 2 && action == otherTurnAction) {
					result[stateDistance + 1] = 1;					
				}				
			} else if (bs.location[0] == 16) {
				if (bs.location[2] == 1 || bs.location[2] == 0) {
					result[stateDistance + 1] = 1;
				} else if (bs.location[2] == 2 && action == turnAroundAction) {
					result[stateDistance + 1] = 1;					
				}				
			}			
				
		}
	
		// all other actions

		if (l2norm(result) == 0)
			result[dim() - 1] = 1;

		return result;
	}

}

class Boyd2RewardGroupedFeatures : LinearReward {
	
	Model model;
	MoveForwardAction moveForwardAction;
	Action turnAroundAction;
	Action otherTurnAction;
	
	public this (Model model) {
		this.model = model;
		
		this.moveForwardAction = new MoveForwardAction();
		this.turnAroundAction = new TurnLeftAction();
		this.otherTurnAction = new TurnRightAction();
	}
	
	public override int dim() {
		return 6;
	}
			
	public override double [] features(State state, Action action) {
		State newState = action.apply(state);
		
		bool moved = true;
		
		if (! model.is_legal(newState) || newState.samePlaceAs(state)) 
			moved = false;
		
		double [] result;
		result.length = dim();
		result[] = 0;
		
		if (moved) 
			result[0] = 1;
		
		/* we want features that represent a turn around at a certain distance value
		 * but we only have single turn actions (when the turnaround takes at least two
		 * so maybe we can represent a turn around by a turn that results in face a direction we can move (since boyd2 is a hallway, this 
		 * feature won't be activated when turning to face a wall)  This in combination with a moveforward feature should let me 
		 * specify a point to turn around at (and have this learned from observations too)
		 */
		// if you could move forward, then choose the turnleft action only (to bias to one side)
		// otherwise this feature applies to both left and right

		bool couldTurnAround = model.is_legal(moveForwardAction.apply(turnAroundAction.apply(turnAroundAction.apply(state)))) && model.is_legal(moveForwardAction.apply(state));
		
		if (model.is_legal(newState) && ! moved ) {
			// when action is turning, activate a feature based on current region of state space 
				if (action == turnAroundAction || action == otherTurnAction) {
					BoydState bs = cast(BoydState)state;
					if (bs.location[0] >= 1 && bs.location[0] <= 15)
						result[1] = 1;
					if ((bs.location[0] <= 1 || bs.location[0] >= 15) && bs.location[1] <= 2)
						result[2] = 1;
					if (bs.location[1] >= 2 && bs.location[1] <= 3)
						result[3] = 1;
					if (bs.location[1] >= 4 && bs.location[1] <= 5)
						result[4] = 1;
					if (bs.location[1] >= 6 && bs.location[1] <= 8)
						result[5] = 1;
				}
				
		}

		return result;
	}

}

class Boyd2RewardGroupedFeaturesTestMTIRL : LinearReward {
	
	Model model;
	MoveForwardAction moveForwardAction;
	Action turnAroundAction;
	Action otherTurnAction;
	
	public this (Model model) {
		this.model = model;
		
		this.moveForwardAction = new MoveForwardAction();
		this.turnAroundAction = new TurnLeftAction();
		this.otherTurnAction = new TurnRightAction();
	}
	
	public override int dim() {
		return 6;
	}
			
	public override double [] features(State state, Action action) {
		State newState = action.apply(state);
		
		bool moved = true;
		
		if (! model.is_legal(newState) || newState.samePlaceAs(state)) 
			moved = false;
		
		double [] result;
		result.length = dim();
		result[] = 0;
		
		if (moved) 
			result[0] = 1;
		
		/* we want features that represent a turn around at a certain distance value
		 * but we only have single turn actions (when the turnaround takes at least two
		 * so maybe we can represent a turn around by a turn that results in face a direction we can move (since boyd2 is a hallway, this 
		 * feature won't be activated when turning to face a wall)  This in combination with a moveforward feature should let me 
		 * specify a point to turn around at (and have this learned from observations too)
		 */
		// if you could move forward, then choose the turnleft action only (to bias to one side)
		// otherwise this feature applies to both left and right

		bool couldTurnAround = model.is_legal(moveForwardAction.apply(turnAroundAction.apply(turnAroundAction.apply(state)))) && model.is_legal(moveForwardAction.apply(state));
		
		if (model.is_legal(newState) && ! moved ) {
			// when action is turning, activate a feature based on current region of state space 
				if (action == turnAroundAction || action == otherTurnAction) {
					BoydState bs = cast(BoydState)state;
					if (bs.location[0] >= 1 && bs.location[0] <= 15)
						result[1] = 1;
					if ((bs.location[0] <= 1 || bs.location[0] >= 15) && bs.location[1] <= 2)
						result[2] = 1;
					if (bs.location[1] >= 2 && bs.location[1] <= 3)
						result[3] = 1;
					if ((bs.location[1] >= 6 && bs.location[1] <= 8) && bs.location[0] == 0)
						result[4] = 1;
					if ((bs.location[1] >= 6 && bs.location[1] <= 8) && bs.location[0] == 16)
						result[5] = 1;
				}
				
		}

		return result;
	}

}

class largeGridRewardGroupedFeatures : LinearReward {
	
	Model model;
	Action moveForwardAction;
	Action turnLeftAction;
	Action turnRightAction;

	public this (Model model) {
		this.model = model;
		this.moveForwardAction = new MoveForwardAction();
		this.turnLeftAction = new TurnLeftAction();
		this.turnRightAction = new TurnRightAction();
	}
	
	public override int dim() {
		return 10;
	}

	public override double [] features(State state, Action action) {
	    //writeln("inside features");
		State newState = action.apply(state);
		bool moved = true;
		if (! model.is_legal(newState) || newState.samePlaceAs(state))
			moved = false;

		double [] result;
		result.length = dim();
		result[] = 0;
		// current action is move forward
		if (moved) {
			result[0] = 1;
        }
        //writeln("if (moved)");

        BoydExtendedState s = cast(BoydExtendedState)state;
        BoydExtendedState last_bs = new BoydExtendedState(s.last_location,null);
        BoydExtendedState newBES = cast(BoydExtendedState)newState;

        bool turnleft_movingfrwd = model.is_legal(moveForwardAction.apply(turnLeftAction.apply(s)));
        bool cur_act_left = (action==turnLeftAction);
        //bool turnleft_movingfrwd_ls = model.is_legal(moveForwardAction.apply(turnLeftAction.apply(last_bs)));
        //bool movingfrwd_ls = model.is_legal(moveForwardAction.apply(last_bs));

        bool movingfrwd = model.is_legal(moveForwardAction.apply(s));
        bool cur_act_frwd = moved; //is(typeof(action)==MoveForwardAction);
        bool lstactmvfrwd = (model.is_legal(s) && !s.samePlaceAs(last_bs));
        bool turnright_movingfrwd = model.is_legal(moveForwardAction.apply(turnRightAction.apply(s)));

        // with last action moveforward, going left side is possible, and left is chosen
        // turn left is chosen only if last action movefrward
        int [] loc = s.location;
        int [] last__loc = s.last_location;
        int [] next_loc = newBES.location;
        int curr_goal = s.current_goal;
        //if (lstactmvfrwd && turnleft_movingfrwd && !moved && cur_act_left) {
        //((loc == [6,4,1] &&
        //(curr_goal == 0 || curr_goal == 4)) || (loc==[3,4,3] && curr_goal == 2))
        if (lstactmvfrwd && turnleft_movingfrwd && !moved && !turnright_movingfrwd && cur_act_left) {
            if ((loc == [6,4,1] && (curr_goal == 0 || curr_goal == 4)) || (loc==[3,4,3] && curr_goal == 2)) {
                    result[1] = 1;
            }
        //    if ((curr_goal == 0 || curr_goal == 4 || curr_goal == 2))
            //writeln("result[9] = 1;");
        }
        if (lstactmvfrwd && turnleft_movingfrwd && !moved && cur_act_left &&
        (last__loc[1] !=4 && next_loc[1]==4)) {
            result[2] = 1;
            //writeln("result[9] = 1;");
        }


        /*
        with last action moveforward, going left side and forward is not possible, right is chosen
        instead of turn around
        */
        bool cur_act_tnrt = (action==turnRightAction);
        //bool turnright_movingfrwd_ls = model.is_legal(moveForwardAction.apply(turnRightAction.apply(last_bs)));
        //bool turnright_movingfrwd_ls = model.is_legal(moveForwardAction.apply(turnRightAction.apply(last_bs)));

        //if (!moved && lstactmvfrwd && !turnleft_movingfrwd && !movingfrwd && turnright_movingfrwd && cur_act_tnrt) {
        //&& ((loc == [0,4,1] && curr_goal == 1) || (loc==[9,4,3] && curr_goal==3))
        if (lstactmvfrwd && !moved && !turnleft_movingfrwd && !movingfrwd && turnright_movingfrwd && cur_act_tnrt
        && (curr_goal == 1 || curr_goal == 3)) {
            result[3] = 1;
            //writeln("result[10] = 1;");
        }
         //last state location in cell after turning right at these locations, and move forward is chosen
        //if (moved && !turnleft_movingfrwd_ls && turnright_movingfrwd_ls && movingfrwd && cur_act_frwd) {
        //    result[8] = 1;
        //    writeln("result[8] = 1;");
        //}

        // if agent is not at a junction and current action is turning
        if (model.is_legal(newState) && !moved  && !movingfrwd && (cur_act_left || cur_act_tnrt)) {
        // if agent is at end locations not facing open side
        // [6,0,2])  [0,8,0])  [3,8,0]) [9,0,2])
        //    bool at_end = ((loc[0] == 6) && (loc[1] == 0) && (loc[2] != 0)) || ((loc[0] == 0) && (loc[1] == 8) && (loc[2] != 2))
        //           || ((loc[0] == 3) && (loc[1] == 8) && (loc[2] != 2)) || ((loc[0] == 9) && (loc[1] == 0) && (loc[2] != 0));
            bool at_end = ((loc[0] == 6) && (loc[1] == 0)) || ((loc[0] == 0) && (loc[1] == 8))
                   || ((loc[0] == 3) && (loc[1] == 8)) || ((loc[0] == 9) && (loc[1] == 0));

            if (at_end) {
                result[4] = 1;
            }
        }

        bool inregiongoal1 = next_loc[0] == 6 && next_loc[1]<=2 && curr_goal == 1;
        bool inregiongoal2 = next_loc[0] == 0 && next_loc[1]>=7 && curr_goal == 2;
        bool inregiongoal3 = next_loc[0] == 3 && next_loc[1]>=7 && curr_goal == 3;
        bool inregiongoal4 = next_loc[0] == 9 && next_loc[1]<=2 && curr_goal == 4;

        if (inregiongoal1 || inregiongoal2 || inregiongoal3 || inregiongoal4) {
            result[5] = 1;
        }

        int next_goal = newBES.current_goal;
        if (next_goal != curr_goal) {
            //if (curr_goal == 0 || curr_goal == 4 && next_goal == 1) {
            //    for (int number = 6; number < 7; ++number) {
            //        result[number] = 1;
            //    }
            //}
            //if (next_goal == 2) {
            //    for (int number = 7; number < 8; ++number) {
            //        result[number] = 1;
            //    }
            //}
            //if (next_goal == 3) {
            //    for (int number = 8; number < 9; ++number) {
            //        result[number] = 1;
            //    }
            //}
            //if (next_goal == 4) {
            //    for (int number = 9; number < 10; ++number) {
            //        result[number] = 1;
            //    }
            //}
            for (int number = 6; number < 10; ++number) {
                result[number] = 1;
            }
        }

		return result;
        // after turning left, forward is chosen.
        //if (turnleft_movingfrwd_ls && movingfrwd && moved && cur_act_frwd) {
        //    result[6] = 1;
        //    writeln("result[6] = 1;");
        //}

        // with last action moveforward, going left side is not possible, forward is possible and forward is chosen
        // but this is forced by moved



        //if (model.is_legal(newState) && !moved && !is(typeof(action)==StopAction) && !turnleft_movingfrwd && !turnright_movingfrwd ) {
        //if (model.is_legal(newState) && !moved && !turnleft_movingfrwd && !turnright_movingfrwd && !is(typeof(action)==StopAction)) {
        //    // when action is turning
        //    if (is(typeof(action)==TurnRightAction) || is(typeof(action)==TurnLeftAction)) {
        //        // preferred places are ends of sub pathsss
        //        // end regions in left
        //        if (s.location[1] >= 0 && s.location[1] <= 2 && s.location[0] == 6 || s.location[0] == 9) {//s.location[0] >= 4 && s.location[0] <= 5 && s.location[1] == 0) {
        //            result[5] = 1;
        //        } else {
        //        // end regions in right
        //        if (s.location[1] >= 6 && s.location[1] <= 8 && s.location[0] == 0 || s.location[0] == 3) {
        //            result[6] = 1;
        //        }
        //        else {
        //        // other non junction points
        //            result[7] = 1;
        //        }
        //        }
        //
        //    }
        //}

        /*
        if (s.location[2] == 0)
            loc_array = [s.location[0]-1, s.location[1], s.location[2]];
        else if (s.location[2] == 1)
            loc_array = [s.location[0], s.location[1]-1, s.location[2]];
        else if (s.location[2] == 2)
            loc_array = [s.location[0]+1, s.location[1], s.location[2]];
        else
            loc_array = [s.location[0], s.location[1]+1, s.location[2]];
        */
        //MoveForwardAction mvfd = new MoveForwardAction();
        //TurnLeftAction tnlt = new TurnLeftAction();
        //writeln("if (moved)");

        /*
        // Is lst_action move_forward?
        bool lstactmvfrwd = is(typeof(s.last_action)==MoveForwardAction); //.opEquals(mvfd);
        bool lstactturnrght = is(typeof(s.last_action)==TurnRightAction);
        // Is lst_action turn left?
        bool lstactturnleft = is(typeof(s.last_action)==TurnLeftAction); //.opEquals(tnlt);
        //writeln("checked lstactmvfrwd");

        // is it possible to turn left and move_forward?
        bool turnleft_movingfrwd = model.is_legal(moveForwardAction.apply(turnLeftAction.apply(s)));
        bool turnright_movingfrwd = model.is_legal(moveForwardAction.apply(turnRightAction.apply(s)));
        // is it possible to move_forward?
        bool movingfrwd = model.is_legal(moveForwardAction.apply(s));

        bool cur_act_left = is(typeof(action)==TurnLeftAction);
        bool cur_act_frwd = is(typeof(action)==MoveForwardAction);
        bool cur_act_tnrt = is(typeof(action)==TurnRightAction);
        //writeln("turnleft_movingfrwd");
        // with last action moveforward, if going either way is possible, and
        // if going other than left is not possible,left is preferred
        // after turning left (even if turn right and turn left are possible), frwrd is preferrable
        if (model.is_legal(newState) && (!moved && lstactmvfrwd && turnleft_movingfrwd && cur_act_left)) {
        // ||        (lstactturnleft && movingfrwd && cur_act_frwd)){
            result[1] = 1;
        }

        */
        //int [] ll = s.getLastLocation().dup;
        //int [] l = s.getLocation().dup;
        //int [] nl = newBES.getLocation().dup;
        //if ((l == [6, 4, 1] && nl == [6,4,2]) || (l == [3,4,3] && nl == [3,4,0])) {
        //    result[1] = 1;
        //}
        //if ((ll == [6, 4, 1] && nl == [6,3,2]) || (ll == [3,4,3] && nl == [3,5,0])) {
        //    result[1] = 1;
        //}
        /*
        last state location was in small horizontal hallways that needs left turns
        and current one is in vertical hallway
        */
        //if ((nl[0] == 6 && nl[1] == 4 && nl[2] == 1 && ll[0] == 6 && ll[1] == 3 && ll[2] == 0) || (nl[0] == 3 &&
        //nl[1] == 4 && nl[2] == 3 && ll[0] == 3 && ll[1] == 5 && ll[2] == 2)) {
        //    result[2] = 1;
        //}

        // If learner has to choose between move forward and turn right, first feature will force move forward

        // If turn left movfrwrd and move frwrd are not possible
        // but turn right move frwrd is possible
        // then turn right is preferrable over other actions: turn left for turning around (stop is taken care of by first feature)
        // after turning right (at cell where moveforward was not option), move forward is preferrable over turn left or turn right
        //if (model.is_legal(newState) && (!moved && lstactmvfrwd && !turnleft_movingfrwd && !movingfrwd
        //&& turnright_movingfrwd && cur_act_tnrt)) { //|| (!turnleft_movingfrwd && lstactturnrght && movingfrwd && cur_act_frwd)) {
        //    //result[0] = 1;
        //    result[2] = 1;
        //}
        /*
		BoydState bs = cast(BoydState)state;
        foreach(i, st; model.S()) {
            BoydState s = cast(BoydState)st;
            if (s.location[0] == bs.location[0] && s.location[1] == bs.location[1] && s.location[2] == bs.location[2])
                result[i] = 1;
        }
        */
        //writeln("finished features");
	}
}

class largeGridRewardGroupedFeatures2 : LinearReward {

	Model model;

	public this (Model model) {
		this.model = model;
	}

	public override int dim() {
		return 8;
	}

	public override double [] features(State state, Action action) {

		State newState = action.apply(state);
		bool moved = true;
		if (! model.is_legal(newState) || newState.samePlaceAs(state))
			moved = false;

		double [] result;
		result.length = dim();
		result[] = 0;
		// current action is move forward
		if (moved) {
			//result[1] = 1;
        }
        //writeln("if (moved)");

        BoydExtendedState2 s = cast(BoydExtendedState2)state;
        BoydExtendedState2 newBES = cast(BoydExtendedState2)newState;
        int curr_goal = s.current_goal;
        int next_goal = newBES.current_goal;
        if (next_goal != curr_goal) {
            if (next_goal == 1) {
                result[0] = 1;
                result[1] = 1;
            }
            if (next_goal == 2) {
                result[2] = 1;
                result[3] = 1;
            }
            if (next_goal == 3) {
                result[4] = 1;
                result[5] = 1;
            }
            if (next_goal == 4) {
                result[6] = 1;
                result[7] = 1;
            }
            //
            //for (int number = 1; number < 10; ++number) {
            //    result[number] = 1;
            //}
        }

		return result;
    }
}

class largeGridRewardGroupedFeatures2R : LinearReward {

	Model model;

	public this (Model model) {
		this.model = model;
	}

	public override int dim() {
		return 8;
	}

	public override double [] features(State state, Action action) {

		State newState = action.apply(state);
		bool moved = true;
		if ( ! model.is_legal(newState) || newState.samePlaceAs(state) )
			moved = false;

		double [] result;
		result.length = dim();
		result[] = 0;

        BoydExtendedState2 s = cast(BoydExtendedState2)state;
        BoydExtendedState2 newBES = cast(BoydExtendedState2)newState;
        int curr_goal = s.current_goal;
        int next_goal = newBES.current_goal;
        if (next_goal != curr_goal) {
            if (next_goal == 2) {
                //writeln("(next_goal == 1)");
                result[0] = 1;
                result[1] = 1;
                result[2] = 1;
                result[3] = 1;
            }
            if (next_goal == 4) {
                //writeln("(next_goal == 4)");
                result[4] = 1;
                result[5] = 1;
                result[6] = 1;
                result[7] = 1;
            }
        }

		return result;
    }

}

class largeGridRewardGroupedFeatures3 : LinearReward {

	Model model;
    BoydState [] regions;

	public this (Model model) {
		this.model = model;
		// create regions by not taking orientation into acount
		bool flag = 1; // region for current state not included
        foreach(i, st; model.S()) {
            flag= 1;
            BoydState s = cast(BoydState)st;
            foreach(j, bs; this.regions){

                if (s.location[0] == bs.location[0] && s.location[1] == bs.location[1]){
                    flag = 0;
                }
            }
            if (flag == 1) {
                // append state if it's region is not already included
                this.regions ~= new BoydState([s.location[0], s.location[1], 0]);
            }

        }

	}

	public override int dim() {
		return cast(int)this.regions.length;
	}

	public override double [] features(State state, Action action) {

		double [] result;
		result.length = dim();
		result[] = 0;
		BoydState bs = cast(BoydState)state;

        foreach(i, st; this.regions) {
            BoydState s = cast(BoydState)st;
            if (s.location[0] == bs.location[0] && s.location[1] == bs.location[1] )
                result[i] = 1;
        }

		return result;
	}

}

class Boyd2RewardReducedFeatures : LinearReward {
	
	Model model;
	int[State] distance;
	int maxDistance;
	MoveForwardAction moveForwardAction;
	Action turnAroundAction;
	Action otherTurnAction;	
	
	public this (Model model, int[State] distance) {
		this.model = model;
		this.distance = distance;
		
		if (distance.length > 0) {
			int [] distances = distance.values;
			foreach (d; distances) {
				if (d > maxDistance)
					maxDistance = d;
			}
		}
		
		this.moveForwardAction = new MoveForwardAction();
		this.turnAroundAction = new TurnLeftAction();
		this.otherTurnAction = new TurnRightAction();
	}
	
	public override int dim() {
		return 4 + maxDistance;
	}
	
	public int num_weights() {
		return 2;
	}
			
	public override double [] features(State state, Action action) {
		State newState = action.apply(state);
		
		bool moved = true;
		
		if (! model.is_legal(newState) || newState.samePlaceAs(state)) 
			moved = false;
		
		double [] result;
		result.length = dim();
		result[] = 0;
		
		if (moved) {
			BoydState bs = cast(BoydState)state;
			if (bs.getLocation()[1] <= 1) {
				result[0] = 1;
			}
			if (bs.getLocation()[0] == 0) {
				result[1] = 1;
			}
			if (bs.getLocation()[0] == 16) {
				result[2] = 1;
			}				
			
		}
				
		auto stateDistance = distance[state];
		
		/* we want features that represent a turn around at a certain distance value
		 * but we only have single turn actions (when the turnaround takes at least two
		 * so maybe we can represent a turn around by a turn that results in face a direction we can move (since boyd2 is a hallway, this 
		 * feature won't be activated when turning to face a wall)  This in combination with a moveforward feature should let me 
		 * specify a point to turn around at (and have this learned from observations too)
		*/
		bool couldTurnAround = model.is_legal(moveForwardAction.apply(turnAroundAction.apply(turnAroundAction.apply(state)))) && model.is_legal(moveForwardAction.apply(state));
			
		if (model.is_legal(newState) && ! moved ) {
			
			if (couldTurnAround) {
				// require the turn around action
				
				if (action == turnAroundAction) {
					result[stateDistance + 3] = 1;
				}
				
				
			} else {
				
				if (action == turnAroundAction || action == otherTurnAction) {
					result[stateDistance + 3] = 1;
				}
				
			}
		}
		
		
		return result;
	}

	
	public override void setParams(double [] p) {
	
		// build the params vector from p;
		
		params = new double[dim()];
		params[0] = p[0];
		params[1] = p[0];
		params[2] = p[0];
		
		int turnAroundsStartAt = 3;
		
		//immutable variance = 0.591607978;
		//immutable variance = 0.632455532;
		immutable variance = 1;
		static denom = variance * sqrt(2 * 3.1415); 
		static denom2 = (-2 * variance * variance);
		
		uint turnAroundAt = cast(uint)(maxDistance - (abs(p[1] * (dim() - turnAroundsStartAt))));
		foreach (i; turnAroundsStartAt..dim()) {
			// get params from a normal distribution centered at p[1]
			if ((i-turnAroundsStartAt) == turnAroundAt || (i-turnAroundsStartAt) == turnAroundAt + 1) {
				params[i] = p[0] * 0.6;
			} else {
				params[i] = -1;
			}
		}
		debug {
			writeln(params);
		}
	}
}



double[Action][][] genEquilibria() {
	
	double[Action][][] returnval;
	returnval.length = 5;  // five equilibria (<Go,Go>, <Stop, Go>, <Go, Stop>, <Turn, Go>, <Go, Turn>)
	
	double[Action][] one;
	one.length = 2;
	
	one[0][new MoveForwardAction()] = 1.0;
	one[1][new MoveForwardAction()] = 1.0;
	returnval[0] = one;
	
	
	double[Action][] two;
	two.length = 2;

	two[0][new StopAction()] = 1.0;
	two[1][new MoveForwardAction()] = 1.0;
	returnval[1] ~= two;


	double[Action][] three;
	three.length = 2;

	three[0][new MoveForwardAction()] = 1.0;
	three[1][new StopAction()] = 1.0;
	returnval[2] ~= three;


	double[Action][] four;
	four.length = 2;

	four[0][new TurnLeftAction()] = 1.0;
	four[1][new MoveForwardAction()] = 1.0;
	returnval[3] ~= four;


	double[Action][] five;
	five.length = 2;

	five[0][new MoveForwardAction()] = 1.0;
	five[1][new TurnLeftAction()] = 1.0;
	returnval[4] ~= five;

	
	return returnval;
	
}

bool boyd2isvisible(BoydState state, double visiblepercent) {
	if (visiblepercent > 10.5 / 14.0)
		return true;
	
	int [] location = state.getLocation();
	
	if (visiblepercent > 6.5 / 14.0) {
		if (location[0] <= 14) {
			return true;
		}	
	}
	
	if (visiblepercent > 4.5 / 14.0) {
		if (location[0] <= 5) {
			return true;
		}
	}

		
	if (visiblepercent > 2.5 / 14.0) {
		if (location[0] <= 2 && location[1] <= 6) {
			return true;
		}
	}

	if (visiblepercent > 1.5 / 14.0) {
		if (location[0] == 0 && location[1] <= 4) {
			return true;
		}
	}

			
	if (location[0] == 0 && location[1] == 1)
		return true;
	if (location[0] == 0 && location[1] == 2)
		return true;
		
	return false;
	
}

bool largeGridisvisible(BoydExtendedState2 state, double visiblepercent) {
	if (visiblepercent > 10.5 / 14.0)
		return true;

	int [] location = state.getLocation();

	//if (visiblepercent > 6.5 / 14.0) {
	//	if (location[1] <= 5 || location[1] >= 8) {
	//		return true;
	//	}
	//}

	if (visiblepercent > 4.5 / 14.0) {
		if (location[1] == 4) {
			return true;
		}
	}

	if (visiblepercent > 2.5 / 14.0) {
		if (location[1] == 4 && location[0] >= 3) {
			return true;
		}
	}
	//if (visiblepercent > 1.5 / 14.0) {
	//	if (location[1] <= 1 || location[1] >= 8) {
	//		return true;
	//	}
	//}
    //
    //
	//if (location[0] == 0 && location[1] == 1)
	//	return true;
	//if (location[0] == 0 && location[1] == 2)
	//	return true;

	return false;

}

bool boydrightisvisible(BoydState state, double visiblepercent) {
	if (visiblepercent > 10.5 / 14.0) // total 52 locations
		return true;
	
	int [] location = state.getLocation();
	
	if (visiblepercent > 6.5 / 14.0) {
		if (location[0] == 0 && location[1] >= 6)
			return true;
		if (location[0] <= 4 && location[1] >= 10)
			return true;
		if (location[0] == 5 && location[1] >= 12)
			return true;
		if (location[0] <= 7 && location[1] >= 14)
			return true;
		if (location[0] <= 11 && location[1] >= 15)
			return true	;
	}
	if (visiblepercent > 4.5 / 14.0) {
		if (location[0] == 0 && location[1] >= 8)
			return true;
		if (location[0] == 1 && location[1] >= 10)
			return true;
		if (location[0] == 2 && location[1] >= 9)
			return true;
		if (location[0] == 5 && location[1] >= 14)
			return true	;	
		if (location[0] <= 7 && location[1] == 16)
			return true	;	
	}

	// 32%  4.5*52/14 = 16, 15 locations visible along with room doors
	if (visiblepercent > 2.5 / 14.0) {
		if (location[0] == 0 && location[1] >= 10) // 7
			return true;
		if (location[0] == 1 && location[1] >= 10) // 2
			return true;
		if (location[0] <= 6 && ( location[1] == 15 || location[1] == 16)) // 6
			return true;
	}

	// 18%  2.5*52/14 = 9, 7 locations visible but room doors not visible
	// 8 locations with one room door visible
	if (visiblepercent > 1.5 / 14.0) {
		/*
		if (location[0] == 0 && location[1] >= 13)
			return true;
		if (location[0] <= 3 && location[1] == 16)
			return true;
		*/
		if (location[0] == 0 && location[1] >= 10)
			return true;
		if (location[0] == 1 && location[1] == 16)
			return true;
	}

	// 10.7%  1.5*52/14 = 5.5, 4 locations visible but room doors not visible
	if (location[0] == 0 && location[1] >= 14)
		return true;
	if (location[0] == 1 && location[1] == 16)
		return true;
		
	return false;
	
}

bool boydright2isvisible(BoydState state, double visiblepercent) {
	if (visiblepercent > 10.5 / 14.0)
		return true;
	int [] location = state.getLocation();

	if (visiblepercent > 6.5 / 14.0) {
		if (location[0] <= 14 && location[0] >= 1 && location[1] == 10)
			return true;
		if (location[0] == 1 && location[1] >= 6 && location[1] <= 9)
			return true;
		if (location[0] == 14 && location[1] >= 11 && location[1] <= 13)
			return true;
	}

	if (visiblepercent > 4.5 / 14.0) {
		if (location[0] <= 9 && location[0] >= 1 && location[1] == 10)
			return true;
		if (location[0] == 1 && location[1] >= 6 && location[1] <= 10)
			return true;
	}

	if (visiblepercent > 2.5 / 14.0) {
		if (location[0] <= 9 && location[1] >= 1 && location[1] == 10)
			return true;
	}
	return false;
}

double parse_transitions(string mapToUse, string line, out State s, out Action a, out State s_prime) {

	string state;
	string action;
	string state_prime;
	double p;
	
	formattedRead(line, "%s:%s:%s:%s", &state, &action, &state_prime, &p);
	
	int x;
	int y;
	int z;

	if (mapToUse == "largeGridPatrol") {

    	//string last_action;
        int lx;
        int ly;
        int lz;
        int cg;
        state = state[1..state.length];
        //formattedRead(state, "%s, %s, %s], [%s, %s, %s", &x, &y, &z, &last_action);
        //formattedRead(state, "%s, %s, %s], [%s, %s, %s]", &x, &y, &z, &lx, &ly, &lz);
        formattedRead(state, "%s, %s, %s], %s", &x, &y, &z, &cg);
        //formattedRead(state, "%s, %s, %s], [%s, %s, %s], %s", &x, &y, &z, &lx, &ly, &lz, &cg);
        //Action la;
        //writeln(x,y,z,last_action);
        //if (last_action == "MoveForwardAction") {
        //    la = cast(MoveForwardAction)la;
        //} else if (action == "StopAction") {
        //    la = cast(StopAction)la;
        //} else if (action == "TurnLeftAction") {
        //    la = cast(TurnLeftAction)la;
        //} else if (action == "TurnAroundAction") {
        //    la = cast(TurnAroundAction)la;
        //} else {
        //    la = cast(TurnRightAction)la;
        //}

        //s = new BoydExtendedState([x, y, z], la);
        //s = new BoydExtendedState([x, y, z], [lx, ly, lz], cg);
        s = new BoydExtendedState2([x, y, z], cg);

        state_prime = state_prime[1..state_prime.length];
        //formattedRead(state_prime, "%s, %s, %s], %s", &x, &y, &z, &last_action);
        formattedRead(state, "%s, %s, %s], %s", &x, &y, &z, &cg);
        //formattedRead(state, "%s, %s, %s], [%s, %s, %s], %s", &x, &y, &z, &lx, &ly, &lz, &cg);
        //writeln(x,y,z,last_action);
        //if (last_action == "MoveForwardAction") {
        //    la = cast(MoveForwardAction)la;
        //} else if (action == "StopAction") {
        //    la = cast(StopAction)la;
        //} else if (action == "TurnLeftAction") {
        //    la = cast(TurnLeftAction)la;
        //} else if (action == "TurnAroundAction") {
        //    la = cast(TurnAroundAction)la;
        //} else {
        //    la = cast(TurnRightAction)la;
        //}
        s_prime = new BoydExtendedState2([x, y, z], cg);
        //s_prime = new BoydExtendedState([x, y, z], [lx, ly, lz], cg);

	} else {

        state = state[1..state.length];
        formattedRead(state, "%s, %s, %s]", &x, &y, &z);

        s = new BoydState([x, y, z]);

        state_prime = state_prime[1..state_prime.length];
        formattedRead(state_prime, "%s, %s, %s]", &x, &y, &z);

        s_prime = new BoydState([x, y, z]);
	}


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

	return p;

}

double parse_transitions2(string mapToUse, string line, out State s, out Action a, out State s_prime) {

	string state;
	string action;
	string state_prime;
	double p;

	formattedRead(line, "%s:%s:%s:%s", &state, &action, &state_prime, &p);

	int x;
	int y;
	int z;

	if (mapToUse == "largeGridPatrol") {

        int cg;
        state = state[1..state.length];
        formattedRead(state, "%s, %s, %s], %s", &x, &y, &z, &cg);

        s = new BoydExtendedState2([x, y, z], cg);

        state_prime = state_prime[1..state_prime.length];
        formattedRead(state_prime, "%s, %s, %s], %s", &x, &y, &z, &cg);

        s_prime = new BoydExtendedState2([x, y, z], cg);

	} else {

        state = state[1..state.length];
        formattedRead(state, "%s, %s, %s]", &x, &y, &z);

        s = new BoydState([x, y, z]);

        state_prime = state_prime[1..state_prime.length];
        formattedRead(state_prime, "%s, %s, %s]", &x, &y, &z);

        s_prime = new BoydState([x, y, z]);
	}


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

	return p;

}
