import mdp;
import std.array;
import std.format;
import std.math;
import std.stdio;
import std.string;
import std.random;

int num_objects = 8;

class sortingState : mdp.State {
	
    public int [] status;
    public int _onion_location;
    public int _prediction;
    public int _EE_location;
    public int _listIDs_status;

	public this ( int [] status = [0,2,0,2] ) {
		this._onion_location = status[0];
		this._prediction = status[1];
		this._EE_location = status[2];
		this._listIDs_status = status[3];
		this.status = status;
	}
	
	//public int[] getLocation() {
	//	return location;
	//}
	
	//public void setLocation(int [] l) {
	//	assert(l.length == 3);
		
	//	this.location = l;
		
	//}
	
	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "[%(%s, %) ]", this.status);
		return writer.data; 
	}

	override hash_t toHash() const {
		return (status[0] + 5*(status[1] + 3*(status[2]+4*status[3])));
	}	
	
	override bool samePlaceAs(State o) {
		if (this is o)
			return true;
		sortingState p = cast(sortingState)o;
		
		return p && p.status[0] == status[0] && p.status[1] == status[1] && p.status[2] == status[2] && p.status[3] == status[3];
		
	}

	override bool opEquals(Object o) {
		if (this is o)
			return true;
		sortingState p = cast(sortingState)o;
		
		return p && p.status[0] == status[0] && p.status[1] == status[1] && p.status[2] == status[2] && p.status[3] == status[3];
		
	}


	/*
	override int opCmp(Object o) const {
		sortingState p = cast(sortingState)o;

		if (!p) 
			return -1;
			
		for (int i = 0; i < status.length; i ++) {
			if (p.status[i] < status[i])
				return 1;
			else if (p.status[i] > status[i])
				return -1;
			
		}
		
		return 0;
		
	}
	*/		
	
}


class sortingModel : mdp.Model { 
	
	Action [] actions;
	sortingState [] states;
	State terminal;
	double p_fail; 

	public this( double p_fail, State terminal) {
		this.p_fail = p_fail;
		this.terminal = terminal;
		
		this.actions ~= new InspectAfterPicking();
		this.actions ~= new InspectWithoutPicking();
		this.actions ~= new Pick();
		this.actions ~= new PlaceOnConveyor();
		this.actions ~= new PlaceInBin();
		//this.actions ~= new GoHome();
		this.actions ~= new ClaimNewOnion();
		this.actions ~= new ClaimNextInList();

		for (int ol = 0; ol < 5; ol ++) {
			for (int pr = 0; pr < 3; pr ++) {
				for (int el = 0; el < 4; el ++) {
					for (int le = 0; le < 3; le ++) {
						// invalid if onion infront/ athome and EE loc not same
						//if ((ol == 4 && pr == 2) || 
                        if ((ol == 1 && el != 1) || 
                        (ol == 3 && el != 3)) {						
							continue;
						}
						states ~= new sortingState([ol,pr,el,le]);
					}
				}
			}
		}
	}
	
	public override State [] S () {
		return cast(State[])states;
	}
	
	public override Action[] A(State st = null) {

		if (st is null) { 
			return [new InspectAfterPicking(),new PlaceOnConveyor(),new PlaceInBin(),
			new Pick(),new ClaimNewOnion(),new InspectWithoutPicking(),new ClaimNextInList()];
		}

		sortingState state = cast(sortingState)st;
		if (state._onion_location == 1 || state._onion_location == 3) {
			// home or front, onion is picked
			if (state._listIDs_status == 2) {
				//return [new InspectAfterPicking(),new PlaceOnConveyor(),new PlaceInBin()];
				if (state._onion_location == 1) { // no inspect after inspecting
					return [new PlaceOnConveyor(), new PlaceInBin()];
				} else {
					return [new InspectAfterPicking(),new PlaceInBin()];
				}
			} else {
				// do not re-inspect if list exists
				return [new PlaceInBin()];
			}
			//return [new InspectAfterPicking(),new PlaceOnConveyor(),new PlaceInBin()];
		}
		if (state._onion_location == 0 || state._onion_location == 4) {
			// # on conveyor (not picked yet or already placed) 
			if (state._listIDs_status == 2) {
				// # can not claim from list if list not available 
				return [new Pick(),new ClaimNewOnion(),new InspectWithoutPicking()];
			} else {
				// # can not create list again (InspectWithoutPicking) if a list is already available 
				// # sorter can claim new onion only when a list of predictions has not been pending 
				//# if we allow ClaimNewOnion with a list available
    //            # then it will do *,0,2,1 ClaimNewOnion 0,2,2,1 ClaimNextInList 0,0,2,1
    //            # and will assume onion is bad without inspection
				return [new Pick(),new ClaimNextInList()];
			}
		}
		if (state._onion_location == 2) {
			// # in bin, can't pick from bin because not reachable 
			if (state._listIDs_status == 2) {
				// # sorter can claim new onion only when a list of predictions has not been pending 
				return [new ClaimNewOnion(),new InspectWithoutPicking()];
			} else {
				return [new ClaimNextInList()];
			}
		}
		return null;
	}

	public override bool is_terminal(State state) {

		return false;
	}

	public override bool is_legal(State state) {
		return true; 
	}

	public override double[State] T(State state, Action action) {
		double[State] returnval;
        //sortingState st = cast(sortingState)state;
        //sortingState next_st = cast(sortingState)(action.apply(state));
        State st = state;
        State next_st = action.apply(state);
        if (! is_legal(st) || next_st.opEquals(st)) { 
            returnval[st] = 1.0;
        } else {
            returnval[next_st] = 1.0-this.p_fail;
            returnval[st] = this.p_fail;
        }

		return returnval;
	}

	public override int numTFeatures() {
		return 0;
	}

	public override int [] TFeatures(State state, Action action){
		return null;
	}
	
}


class sortingModelbyPSuresh : sortingModel { 

	public this(double p_fail, State terminal) {
		super(p_fail, terminal);

		sortingState [] states2;
		for (int ol = 0; ol < 5; ol ++) {
			for (int pr = 0; pr < 3; pr ++) {
				for (int el = 0; el < 4; el ++) {
					for (int le = 0; le < 3; le ++) {

                        if ((ol == 1 && el != 1) || 
                        (ol == 2 && el != 2) ||
                        (ol == 3 && el != 3)) {						
							continue;
						}
						states2 ~= new sortingState([ol,pr,el,le]);
					}
				}
			}
		}

		this.states = states2;
		
	}

	public override Action[] A(State st = null) {

		if (st is null) { 
			return [new InspectAfterPicking(),new PlaceOnConveyor(),
			new PlaceInBin(),
			new Pick(),new ClaimNewOnion(),
			new InspectWithoutPicking(),
			new ClaimNextInList()];
		}

		sortingState state = cast(sortingState)st;

		if (state._onion_location == 0 ) {
			if (state._listIDs_status == 2) {
				return [new Pick()]; 
			} else if (state._listIDs_status == 0) {
				//return [new InspectWithoutPicking()]; 
				return [new InspectWithoutPicking(),new Pick()]; 
			} else {
				if ( state._prediction == 2) {
					return [new ClaimNextInList()]; 
				} else {
					return [new Pick()]; 
				}
			}
		} else if (state._onion_location == 1 ) {
			if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else if (state._prediction == 1) {
				return [new PlaceOnConveyor()]; 
			} else {
				return [new InspectAfterPicking()];
			}
		} else if (state._onion_location == 2 ) {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		} else if (state._onion_location == 3 ) {
			if (state._prediction == 2) {
				return [new InspectAfterPicking()]; 
			} else if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else {
				return [new PlaceOnConveyor()]; 
			}
		} else {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		}

	}

}


class sortingModelbyPSuresh2 : sortingModel { 

	public this(double p_fail, State terminal) {
		super(p_fail, terminal);

        int [][] statesList = [[ 0, 2, 0, 0],
        [ 3, 2, 3, 0],
        [ 1, 0, 1, 2],
        [ 2, 2, 2, 2],
        [ 0, 2, 2, 2],
        [ 3, 2, 3, 2],
        [ 1, 1, 1, 2],
        [ 4, 2, 0, 2],
        [ 0, 0, 0, 1],
        [ 3, 0, 3, 1],
        [ 2, 2, 2, 1],
        [ 0, 0, 2, 1],
        [ 2, 2, 2, 0],
        [0, 2, 0, 2],
        [0, 2, 2, 0],
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],
        [3,1,3,0],[0,0,1,1],[0,0,3,1]];

        writeln("created statesList");

		sortingState [] states2;
		foreach(ls; statesList){

			states2 ~= new sortingState(ls);
		}
        writeln("created states ");

		this.states = states2;
		
	}

	public override Action[] A(State st = null) {

		if (st is null) { 
			return [new InspectAfterPicking(),new PlaceOnConveyor(),
			new PlaceInBin(),
			new Pick(),new ClaimNewOnion(),
			new InspectWithoutPicking(),
			new ClaimNextInList()];
		}

		sortingState state = cast(sortingState)st;

		if (state._onion_location == 0 ) {
			if (state._listIDs_status == 2) {
				return [new Pick()]; 
			} else if (state._listIDs_status == 0) {
				//return [new InspectWithoutPicking()]; 
				return [new InspectWithoutPicking(),new Pick()]; 
			} else {
				if ( state._prediction == 2) {
					return [new ClaimNextInList()]; 
				} else {
					return [new Pick()]; 
				}
			}
		} else if (state._onion_location == 1 ) {
			if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else if (state._prediction == 1) {
				return [new PlaceOnConveyor()]; 
			} else {
				return [new InspectAfterPicking()];
			}
		} else if (state._onion_location == 2 ) {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		} else if (state._onion_location == 3 ) {
			if (state._prediction == 2) {
				return [new InspectAfterPicking()]; 
			} else if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else {
				return [new PlaceOnConveyor()]; 
			}
		} else {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		}
	}
}


class sortingModelbyPSuresh2WOPlaced : sortingModel { 

	public this(double p_fail, State terminal) {
		super(p_fail, terminal);

        int [][] statesList = [[ 0, 2, 0, 0],
        [ 3, 2, 3, 0],
        [ 1, 0, 1, 2],
        [ 2, 2, 2, 2],
        [ 0, 2, 2, 2],
        [ 3, 2, 3, 2],
        [ 1, 1, 1, 2],
        [ 0, 0, 0, 1],
        [ 3, 0, 3, 1],
        [ 2, 2, 2, 1],
        [ 0, 0, 2, 1],
        [ 2, 2, 2, 0],
        [0, 2, 0, 2],
        [0, 2, 2, 0],
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],
        [3,1,3,0],[0,0,1,1],[0,0,3,1] //,
        //[0,0,0,0],[0,0,1,0],[0,0,2,0],[0,0,3,0],
        //[3,0,3,0]
        ];

        writeln("created statesList");

		sortingState [] states2;
		foreach(ls; statesList){

			states2 ~= new sortingState(ls);
		}
        writeln("created states ");

		this.states = states2;
		
	}

	public override Action[] A(State st = null) {

		if (st is null) { 
			return [new InspectAfterPicking(),new PlaceOnConveyor(),
			new PlaceInBin(),
			new Pick(),new ClaimNewOnion(),
			new InspectWithoutPicking(),
			new ClaimNextInList()];
		}

		sortingState state = cast(sortingState)st;

		if (state._onion_location == 0 ) {
			if (state._listIDs_status == 2) {
				return [new Pick()]; 
			} else if (state._listIDs_status == 0) {
				//return [new InspectWithoutPicking()]; 
				return [new InspectWithoutPicking(),new Pick()]; 
			} else {
				if ( state._prediction == 2) {
					return [new ClaimNextInList()]; 
				} else {
					return [new Pick()]; 
				}
			}
		} else if (state._onion_location == 1 ) {
			if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else if (state._prediction == 1) {
				return [new PlaceOnConveyor()]; 
			} else {
				return [new InspectAfterPicking()];
			}
		} else if (state._onion_location == 2 ) {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		} else if (state._onion_location == 3 ) {
			if (state._prediction == 2) {
				return [new InspectAfterPicking()]; 
			} else if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else {
				return [new PlaceOnConveyor()]; 
			}
		} else {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		}
	}
}

class sortingModelbyPSuresh3 : sortingModel { 

	public this(double p_fail, State terminal) {
		super(p_fail, terminal);

        int [][] statesList = [[ 0, 2, 0, 0],
        [ 3, 2, 3, 0],
        [ 1, 0, 1, 2],
        [ 2, 2, 2, 2],
        [ 0, 2, 2, 2],
        [ 3, 2, 3, 2],
        [ 1, 1, 1, 2],
        [ 4, 2, 0, 2],
        [ 0, 0, 0, 1],
        [ 3, 0, 3, 1],
        [ 2, 2, 2, 1],
        [ 0, 0, 2, 1],
        [ 2, 2, 2, 0],
        [0, 2, 0, 2],
        [0, 2, 2, 0],
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],
        [3,1,3,0],[0,0,1,1],[0,0,3,1],
        [0,2,1,2],[0,2,3,2]];

        writeln("created statesList");

		sortingState [] states2;
		foreach(ls; statesList){

			states2 ~= new sortingState(ls);
		}
        writeln("created states ");

		this.states = states2;
		
	}

	public override Action[] A(State st = null) {

		if (st is null) { 
			return [new InspectAfterPicking(),new PlaceOnConveyor(),
			new PlaceInBin(),
			new Pick(),new ClaimNewOnion(),
			new InspectWithoutPicking(),
			new ClaimNextInList()];
		}

		sortingState state = cast(sortingState)st;

		if (state._onion_location == 0 ) {
			if (state._listIDs_status == 2) {
				return [new Pick(), new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				//return [new InspectWithoutPicking()]; 
				return [new InspectWithoutPicking(),new Pick(), new ClaimNewOnion()]; 
			} else {
				if ( state._prediction == 2) {
					return [new ClaimNextInList()]; 
				} else {
					return [new Pick()]; 
				}
			}
		} else if (state._onion_location == 1 ) {
			if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else if (state._prediction == 1) {
				return [new PlaceOnConveyor()]; 
			} else {
				return [new InspectAfterPicking()];
			}
		} else if (state._onion_location == 2 ) {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		} else if (state._onion_location == 3 ) {
			if (state._prediction == 2) {
				return [new InspectAfterPicking()]; 
			} else if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else {
				return [new PlaceOnConveyor()]; 
			}
		} else {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		}
	}
}

class sortingModelbyPSuresh3multipleInit : sortingModel { 

	public this(double p_fail, State terminal) {
		super(p_fail, terminal);

        int [][] statesList = [[ 0, 2, 0, 0],
        [ 3, 2, 3, 0],
        [ 1, 0, 1, 2],
        [ 2, 2, 2, 2],
        [ 0, 2, 2, 2],
        [ 3, 2, 3, 2],
        [ 1, 1, 1, 2],
        [ 4, 2, 0, 2],
        [ 0, 0, 0, 1],
        [ 3, 0, 3, 1],
        [ 2, 2, 2, 1],
        [ 0, 0, 2, 1],
        [ 2, 2, 2, 0],
        [0, 2, 0, 2],
        [0, 2, 2, 0],
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],
        [3,1,3,0],[0,0,1,1],[0,0,3,1],
        [0,2,1,2],[0,2,3,2]];

        writeln("created statesList");

		sortingState [] states2;
		foreach(ls; statesList){

			states2 ~= new sortingState(ls);
		}
        writeln("created states ");

		this.states = states2;
		
	}

	public override Action[] A(State st = null) {

		if (st is null) { 
			return [new InspectAfterPicking(),new PlaceOnConveyor(),
			new PlaceInBin(),
			new Pick(),new ClaimNewOnion(),
			new InspectWithoutPicking(),
			new ClaimNextInList()];
		}

		sortingState state = cast(sortingState)st;

		if (state._onion_location == 0 ) {
			if (state._listIDs_status == 2) {
				return [new Pick(), new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				//return [new InspectWithoutPicking()]; 
				return [new InspectWithoutPicking(),new Pick(), new ClaimNewOnion()]; 
			} else {
				if ( state._prediction == 2) {
					return [new ClaimNextInList()]; 
				} else {
					return [new Pick()]; 
				}
			}
		} else if (state._onion_location == 1 ) {
			if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else if (state._prediction == 1) {
				return [new PlaceOnConveyor()]; 
			} else {
				return [new InspectAfterPicking()];
			}
		} else if (state._onion_location == 2 ) {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		} else if (state._onion_location == 3 ) {
			if (state._prediction == 2) {
				return [new InspectAfterPicking()]; 
			} else if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else {
				return [new PlaceOnConveyor()]; 
			}
		} else {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		}
	}
}

class sortingModelbyPSuresh4 : sortingModel { 

	public this(double p_fail, State terminal) {
		super(p_fail, terminal);

        int [][] statesList = [[ 0, 2, 0, 0],
        [ 3, 2, 3, 0],
        [ 1, 0, 1, 2],
        [ 2, 2, 2, 2],
        [ 0, 2, 2, 2],
        [ 3, 2, 3, 2],
        [ 1, 1, 1, 2],
        [ 4, 2, 0, 2],
        [ 0, 0, 0, 1],
        [ 3, 0, 3, 1],
        [ 2, 2, 2, 1],
        [ 0, 0, 2, 1],
        [ 2, 2, 2, 0],
        [0, 2, 0, 2],
        [0, 2, 2, 0],
        [0,1,0,0],[0,1,1,0],[0,1,2,0],[0,1,3,0],[0,2,1,0],[0,2,3,0],
        [3,1,3,0],[0,0,1,1],[0,0,3,1],
        [0,2,1,2],[0,2,3,2]];

        writeln("created statesList");

		sortingState [] states2;
		foreach(ls; statesList){

			states2 ~= new sortingState(ls);
		}
        writeln("created states ");

		this.states = states2;
		
	}

	public override Action[] A(State st = null) {

		if (st is null) { 
			return [new InspectAfterPicking(),new PlaceOnConveyor(),
			new PlaceInBin(),
			new Pick(),new ClaimNewOnion(),
			new InspectWithoutPicking(),
			new ClaimNextInList()];
		}

		sortingState state = cast(sortingState)st;

		if (state._onion_location == 0 ) {
			if (state._listIDs_status == 2) {
				return [new Pick(), new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				//return [new InspectWithoutPicking()]; 
				return [new InspectWithoutPicking(),new Pick(), new ClaimNewOnion()]; 
			} else {
				if ( state._prediction == 2) {
					return [new ClaimNextInList()]; 
				} else {
					return [new Pick()]; 
				}
			}
		} else if (state._onion_location == 1 ) {
			if (state._listIDs_status == 2) {
				if (state._prediction == 0) {
					return [new PlaceInBin(),new PlaceOnConveyor()]; 
				} else if (state._prediction == 1) {
					return [new PlaceOnConveyor(),new PlaceInBin()]; 
				} else {
					return [new InspectAfterPicking()];
				} 
			} else {
				if (state._prediction == 0) {
					return [new PlaceInBin()]; 
				} else if (state._prediction == 1) {
					return [new PlaceOnConveyor()]; 
				} else {
					return [new InspectAfterPicking()];
				} 
			}
		} else if (state._onion_location == 2 ) {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		} else if (state._onion_location == 3 ) {
			if (state._prediction == 2) {
				return [new InspectAfterPicking()]; 
			} else if (state._prediction == 0) {
				return [new PlaceInBin()]; 
			} else {
				return [new PlaceOnConveyor()]; 
			}
		} else {
			if (state._listIDs_status == 2) {
				return [new ClaimNewOnion()]; 
			} else if (state._listIDs_status == 0) {
				return [new InspectWithoutPicking()]; 
			} else {
				return [new ClaimNextInList()]; 
			}
		}
	}
}


class sortingModel2 : sortingModel { 
	
	public this(double p_fail, State terminal) {
		super(p_fail, terminal);

		sortingState [] states2;

		for (int ol = 0; ol < 5; ol ++) {
			for (int pr = 0; pr < 3; pr ++) {
				for (int el = 0; el < 4; el ++) {
					for (int le = 0; le < 3; le ++) {
						// invalid if onion infront/ athome and EE loc not same
						//if ((ol == 4 && pr == 2) || 
                        if ((ol == 1 && el != 1) || 
                        (ol == 3 && el != 3) ||
                        (ol == 2 && (le == 0 || le == 1))) {						
							continue;
						}
						states2 ~= new sortingState([ol,pr,el,le]);
					}
				}
			}
		}

		this.states = states2;
		
	}

	public override Action[] A(State st = null) {

		if (st is null) { 
			return [new InspectAfterPicking(),new PlaceOnConveyor(),new PlaceInBin(),
			new Pick(),new ClaimNewOnion(),new InspectWithoutPicking(),new PlaceInBinClaimNextInList()];
		}

		sortingState state = cast(sortingState)st;
		if (state._onion_location == 1 || state._onion_location == 3) {
			// home or front, onion is picked
			if (state._listIDs_status == 2) {
				//return [new InspectAfterPicking(),new PlaceOnConveyor(),new PlaceInBin()];
				if (state._onion_location == 1) { // no inspect after inspecting
					return [new PlaceOnConveyor(), new PlaceInBin()];
				} else {
					return [new InspectAfterPicking(),new PlaceInBin()];
				}
			} else {
				// new action specific to rolling
				return [new PlaceInBinClaimNextInList()];
			}
			//return [new InspectAfterPicking(),new PlaceOnConveyor(),new PlaceInBin()];
		}

		if (state._onion_location == 0 || state._onion_location == 4) {
			// # on conveyor (not picked yet or already placed) 
			if (state._listIDs_status == 2) {
				// # can not claim from list if list not available 
				return [new Pick(),new ClaimNewOnion(),new InspectWithoutPicking()];
			} else {
				// # can not create list again (InspectWithoutPicking) if a list is already available 
				// # sorter can claim new onion only when a list of predictions has not been pending 
				//# if we allow ClaimNewOnion with a list available
    //            # then it will do *,0,2,1 ClaimNewOnion 0,2,2,1 ClaimNextInList 0,0,2,1
    //            # and will assume onion is bad without inspection
				return [new Pick()];
			}
		}

		if (state._onion_location == 2) {
			// # in bin, can't pick from bin because not reachable 
			if (state._listIDs_status == 2) {
				// # sorter can claim new onion only when a list of predictions has not been pending 
				return [new ClaimNewOnion(),new InspectWithoutPicking()];
			} else { // with new action, these states aren't valid anymore.
				return null;
			}
		}
		return null;
	}

}


public class InspectAfterPickingold : Action {
	
	public override State apply(State state) {
		if ((cast(sortingState)state)._prediction == 2) {
	        double pp = 0.5*0.95;
	        auto pred = dice(pp, 1-pp);
	        return new sortingState( [1, cast(int)pred, 1, (cast(sortingState)state)._listIDs_status ]);
		} else return new sortingState( [1, (cast(sortingState)state)._prediction, 1, (cast(sortingState)state)._listIDs_status ]);

	}
	
	public override string toString() {
		return "InspectAfterPicking"; 
	}

	override hash_t toHash() {
		return 0;
	}	
	
	override bool opEquals(Object o) {
		InspectAfterPicking p = cast(InspectAfterPicking)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		InspectAfterPicking p = cast(InspectAfterPicking)o;
		if (!p) 
			return -1;
		return 0;
	}
}


public class InspectAfterPicking : Action {
	
	public override State apply(State state) {
		if ((cast(sortingState)state)._prediction == 2) {
	        double pp = 0.5;
	        auto pred = dice(pp, 1-pp);
	        return new sortingState( [1, cast(int)pred, 1, 2 ]);
		} else return new sortingState( 
		[1, (cast(sortingState)state)._prediction, 1, 
		(cast(sortingState)state)._listIDs_status ]); 
		//	} else return new sortingState( 
		//[(cast(sortingState)state)._onion_location, 
		//(cast(sortingState)state)._prediction, 1, 
		//(cast(sortingState)state)._listIDs_status ]);

	}
	
	public override string toString() {
		return "InspectAfterPicking"; 
	}

	override hash_t toHash() {
		return 8;
	}	
	
	override bool opEquals(Object o) {
		InspectAfterPicking p = cast(InspectAfterPicking)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		InspectAfterPicking p = cast(InspectAfterPicking)o;
		if (!p) 
			return -1;
		return 0;
	}
}

public class InspectWithoutPickingold : Action {
	
	public override State apply(State state) {
        // # 95 % chance of actually bad onion present, and 70% chance of detecting them correctly
        //# 95 X 70 / 100 % chance of non-empty list
        //# if list is non-empty, it gives first index from list and location is on conveyor
        double pp = 0.95*(1 - pow((1-0.7),(num_objects/2)) );
        auto ls = dice(1-pp,pp);
        int pred;
        if (cast(int)ls == 0) pred = 2;
        else pred = 0;
        return new sortingState([ 0, pred, (cast(sortingState)state)._EE_location, cast(int)ls ]);

	}
	
	public override string toString() {
		return "InspectWithoutPicking"; 
	}


	override hash_t toHash() {
		return 1;
	}	
	
	override bool opEquals(Object o) {
		InspectWithoutPicking p = cast(InspectWithoutPicking)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		InspectWithoutPicking p = cast(InspectWithoutPicking)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class InspectWithoutPicking : Action {
	
	public override State apply(State state) {
        // # 100 % chance of actually bad onion present, and 70% chance of detecting them correctly
        //# 100 X 70 / 100 % chance of non-empty list
        // if this probability is not high enough, mixed behaviors are learnt
        double pp = 0.5;
        pp = 1*0.95;
        auto ls = dice(1-pp,pp);
        int pred;
        if (cast(int)ls == 0) pred = 2;
        else pred = 0;
        return new sortingState([ 0, pred, 
       	(cast(sortingState)state)._EE_location, cast(int)ls ]);

	}
	
	public override string toString() {
		return "InspectWithoutPicking"; 
	}


	override hash_t toHash() {
		return 13;
	}	
	
	override bool opEquals(Object o) {
		InspectWithoutPicking p = cast(InspectWithoutPicking)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		InspectWithoutPicking p = cast(InspectWithoutPicking)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class Pick : Action {
	
	public override State apply(State state) {
        return new sortingState([3, (cast(sortingState)state)._prediction, 3, (cast(sortingState)state)._listIDs_status]);
	}
	
	public override string toString() {
		return "Pick"; 
	}


	override hash_t toHash() {
		return 2;
	}	
	
	override bool opEquals(Object o) {
		Pick p = cast(Pick)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		Pick p = cast(Pick)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class Pickpip : Action {
	
	public override State apply(State state) {
        return new sortingState([3, (cast(sortingState)state)._prediction, 3, (cast(sortingState)state)._listIDs_status]);
	}
	
	public override string toString() {
		return "Pickpip"; 
	}


	override hash_t toHash() {
		return 20;
	}	
	
	override bool opEquals(Object o) {
		Pickpip p = cast(Pickpip)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		Pickpip p = cast(Pickpip)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class PlaceOnConveyorold : Action {
	
	public override State apply(State state) {
        return new sortingState([ 4, (cast(sortingState)state)._prediction, 0, (cast(sortingState)state)._listIDs_status ]);
	}
	
	public override string toString() {
		return "PlaceOnConveyor"; 
	}


	override hash_t toHash() {
		return 3;
	}	
	
	override bool opEquals(Object o) {
		PlaceOnConveyor p = cast(PlaceOnConveyor)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		PlaceOnConveyor p = cast(PlaceOnConveyor)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class PlaceOnConveyorWPlaced : Action {
	
	public override State apply(State state) {
        return new sortingState([ 4, 2, 0, 2 ]);
	}
	
	public override string toString() {
		return "PlaceOnConveyor"; 
	}


	override hash_t toHash() {
		return 9;
	}	
	
	override bool opEquals(Object o) {
		PlaceOnConveyor p = cast(PlaceOnConveyor)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		PlaceOnConveyor p = cast(PlaceOnConveyor)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class PlaceOnConveyor : Action {
	
	public override State apply(State state) {
        return new sortingState([ 0, 2, 0, 2 ]);
	}
	
	public override string toString() {
		return "PlaceOnConveyor"; 
	}


	override hash_t toHash() {
		return 24;
	}	
	
	override bool opEquals(Object o) {
		PlaceOnConveyor p = cast(PlaceOnConveyor)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		PlaceOnConveyor p = cast(PlaceOnConveyor)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class PlaceInBinold : Action {
	
	public override State apply(State state) {
/*        # most of attempts won't make list empty if it is not already empty or unavailable
        # if list is available and 50% of objects are bad, then 1 out of 6 attempts make 
        # list empty
*/

        int ls1 = cast(int)(cast(sortingState)state)._listIDs_status ;
        int ls2;
        if (ls1 == 1) {
            double pp = 1-(2/cast(double)num_objects);
            ls2 = cast(int)(dice(1-pp,pp));
        } else ls2 = ls1;

        return new sortingState( [2, (cast(sortingState)state)._prediction, 2, ls2] );
	}
	
	public override string toString() {
		return "PlaceInBin"; 
	}


	override hash_t toHash() {
		return 4;
	}	
	
	override bool opEquals(Object o) {
		PlaceInBin p = cast(PlaceInBin)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		PlaceInBin p = cast(PlaceInBin)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class PlaceInBin : Action {
	
	public override State apply(State state) {
/*        # most of attempts won't make list empty if it is not already empty or unavailable
        # if list is available and 50% of objects are bad, then 1 out of 6 attempts make 
        # list empty
*/

        int ls1 = cast(int)(cast(sortingState)state)._listIDs_status ;
        int ls2;
        if (ls1 == 1) {
            double pp = 0.5;
            ls2 = cast(int)(dice(1-pp,pp));
        } else ls2 = ls1;

        return new sortingState( [2, 2, 2, ls2] );
	}
	
	public override string toString() {
		return "PlaceInBin"; 
	}


	override hash_t toHash() {
		return 10;
	}	
	
	override bool opEquals(Object o) {
		PlaceInBin p = cast(PlaceInBin)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		PlaceInBin p = cast(PlaceInBin)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class PlaceInBinpip : Action {
	
	public override State apply(State state) {
/*        # most of attempts won't make list empty if it is not already empty or unavailable
        # if list is available and 50% of objects are bad, then 1 out of 6 attempts make 
        # list empty
*/

        int ls1 = cast(int)(cast(sortingState)state)._listIDs_status ;
        int ls2;
        if (ls1 == 1) {
            double pp = 0.5;
            ls2 = cast(int)(dice(1-pp,pp));
        } else ls2 = ls1;

        return new sortingState( [2, 2, 2, ls2] );
	}
	
	public override string toString() {
		return "PlaceInBinpip"; 
	}


	override hash_t toHash() {
		return 21;
	}	
	
	override bool opEquals(Object o) {
		PlaceInBinpip p = cast(PlaceInBinpip)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		PlaceInBinpip p = cast(PlaceInBinpip)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

//public class GoHome : Action {
	
//	public override State apply(State state) {
//		// NOt USED
//        return null;
//	}
	
//	public override string toString() {
//		return "GoHome"; 
//	}


//	override hash_t toHash() {
//		return 5;
//	}	
	
//	override bool opEquals(Object o) {
//		GoHome p = cast(GoHome)o;
		
//		return p && true;
		
//	}
	
//	override int opCmp(Object o) {
//		GoHome p = cast(GoHome)o;
		
//		if (!p) 
//			return -1;
			
//		return 0;
		
//	}
//}


public class ClaimNewOnionold : Action {
	
	public override State apply(State state) {
		// NOt USED
        return new sortingState([0, 2, (cast(sortingState)state)._EE_location, (cast(sortingState)state)._listIDs_status]);
	}
	
	public override string toString() {
		return "ClaimNewOnion"; 
	}


	override hash_t toHash() {
		return 5;
	}	
	
	override bool opEquals(Object o) {
		ClaimNewOnion p = cast(ClaimNewOnion)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		ClaimNewOnion p = cast(ClaimNewOnion)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class ClaimNewOnion : Action {
	
	public override State apply(State state) {
		// NOt USED
        return new sortingState([0, 2, 
        (cast(sortingState)state)._EE_location, 2]);
	}
	
	public override string toString() {
		return "ClaimNewOnion"; 
	}


	override hash_t toHash() {
		return 12;
	}	
	
	override bool opEquals(Object o) {
		ClaimNewOnion p = cast(ClaimNewOnion)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		ClaimNewOnion p = cast(ClaimNewOnion)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class ClaimNextInListold : Action {
	
	public override State apply(State state) {

        if ((cast(sortingState)state)._listIDs_status == 1) {
            //# if list not empty, then 
            return new sortingState( [0, 0, (cast(sortingState)state)._EE_location, 1] );

        } else {
            //# else make onion unknown and list not available
            return new sortingState( [0, 2, (cast(sortingState)state)._EE_location, 2] );
        }

	}
	
	public override string toString() {
		return "ClaimNextInList"; 
	}


	override hash_t toHash() {
		return 6;
	}	
	
	override bool opEquals(Object o) {
		ClaimNextInList p = cast(ClaimNextInList)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		ClaimNextInList p = cast(ClaimNextInList)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}


public class ClaimNextInList : Action {
	
	public override State apply(State state) {

        if ((cast(sortingState)state)._listIDs_status == 1) {
            //# if list not empty, then 
            return new sortingState( [0, 0, (cast(sortingState)state)._EE_location, 1] );

        } else {
            //# else make onion unknown and list not available
            return new sortingState( [0, 2, 
            (cast(sortingState)state)._EE_location, 
            (cast(sortingState)state)._listIDs_status] );
        }

	}
	
	public override string toString() {
		return "ClaimNextInList"; 
	}


	override hash_t toHash() {
		return 14;
	}	
	
	override bool opEquals(Object o) {
		ClaimNextInList p = cast(ClaimNextInList)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		ClaimNextInList p = cast(ClaimNextInList)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

public class PlaceInBinClaimNextInList : Action {
	
	public override State apply(State state) {
		// placeinbin part
        int ls1 = cast(int)(cast(sortingState)state)._listIDs_status ;
        int ls2;
        if (ls1 == 1) {
            double pp = 1-(2/cast(double)num_objects);
            ls2 = cast(int)(dice(1-pp,pp));
        } else ls2 = ls1;

		// claimnextinlist part

        if (ls2 == 1) {
            //# if list not empty, then 
            return new sortingState( [0, 0, (cast(sortingState)state)._EE_location, 1] );

        } else {
            //# else make onion unknown and list not available
            return new sortingState( [0, 2, (cast(sortingState)state)._EE_location, 2] );
        }

	}
	
	public override string toString() {
		return "PlaceInBinClaimNextInList"; 
	}


	override hash_t toHash() {
		return 7;
	}	
	
	override bool opEquals(Object o) {
		PlaceInBinClaimNextInList p = cast(PlaceInBinClaimNextInList)o;
		
		return p && true;
		
	}
	
	override int opCmp(Object o) {
		PlaceInBinClaimNextInList p = cast(PlaceInBinClaimNextInList)o;
		
		if (!p) 
			return -1;
			
		return 0;
		
	}
}

class sortingReward1 : LinearReward {
	
	Model model;
	int _dim;

	public this (Model model, int dim) {
		this.model = model;
		this._dim = dim;
	}
	
	public override int dim() {
		return this._dim;
	}
			
	public override double [] features(State st, Action action) {
		sortingState state = cast(sortingState)st;
		sortingState next_state = cast(sortingState)(action.apply(st));
		
		double [] result;
		result.length = dim();
		result[] = 0;
        if (next_state._prediction == 1 && next_state._onion_location == 4) result[0] = 1;
        if (next_state._prediction == 0 && next_state._onion_location == 4) result[1] = 1;
        if (next_state._prediction == 1 && next_state._onion_location == 2) result[2] = 1;
        if (next_state._prediction == 0 && next_state._onion_location == 2) result[3] = 1;
        //# keep inspecting
        if (state._prediction == 2 && next_state._prediction != 2) result[4] = 1;

        //# don't stay still, keep changing EE location
        if ( (! (state._onion_location == next_state._onion_location)) &&
        state._prediction == next_state._prediction &&
        state._EE_location == next_state._EE_location &&
        state._listIDs_status == next_state._listIDs_status) result[5] = 1; 

        // keep claiming new onions
        if (! (state._prediction == next_state._prediction)) result[6] = 1;

        // change in EE location 
        if (! (state._EE_location == next_state._EE_location)) result[7] = 1;

        // empty the list 
        if (state._listIDs_status == 1 && next_state._listIDs_status == 0) result[8] = 1;

        // create the list 
        if (state._listIDs_status == 2 && next_state._listIDs_status != 2) result[9] = 1;

        // make empty list unavailable 
        if (state._listIDs_status == 0 && next_state._listIDs_status == 2) result[10] = 1 ;

        // "keeping prediction from unknown in current state to unknown in next state" cycle
        if (state._prediction == 2 && next_state._prediction == 2) result[11] = 1; 

        // inspecting an onion that is already inspected 
        if (state._prediction != 2 && next_state._EE_location == 1) result[12] = 1;

        // pick a good (pick-place-pick cycles)
        if (state._prediction == 1 && next_state._EE_location == 1) result[13] = 1;

		return result;
	}

}


class sortingReward2 : LinearReward {
	
	Model model;
	int _dim;

	public this (Model model, int dim) {
		this.model = model;
		this._dim = dim;
	}
	
	public override int dim() {
		return this._dim;
	}
			
	public override double [] features(State st, Action action) {
		sortingState state = cast(sortingState)st;
		sortingState next_state = cast(sortingState)(action.apply(st));
		
		double [] result;
		result.length = dim();
		result[] = 0;

		// good placed on belt
        if (next_state._prediction == 1 && next_state._onion_location == 4) result[0] = 1;

		// bad placed on belt
        if (next_state._prediction == 0 && next_state._onion_location == 4) result[1] = 1;

		// good placed in bin
        if (next_state._prediction == 1 && next_state._onion_location == 2) result[2] = 1;

		// bad placed in bin
        if (next_state._prediction == 0 && next_state._onion_location == 2) result[3] = 1;

        // staying still
        if ( state._onion_location == next_state._onion_location &&
        state._prediction == next_state._prediction &&
        state._EE_location == next_state._EE_location &&
        state._listIDs_status == next_state._listIDs_status) result[4] = 1; 

        // claim new onion 
        if ((! (state._prediction == next_state._prediction)) && state._prediction == 2 ) result[5] = 1;

        // create the list 
        if (state._listIDs_status == 2 && next_state._listIDs_status != 2) result[6] = 1;

        // pick a good or pick already placed one (pick-place-pick cycles)
        //if ((state._onion_location == 4 || state._prediction == 1) && next_state._EE_location == 3) result[7] = 1;
        // pick a good one
        //if (state._prediction == 1 && next_state._EE_location == 3) result[7] = 1;
        // pick a placed one
        if (state._onion_location == 4 && next_state._EE_location == 3) result[7] = 1;

		return result;
	}
}


class sortingReward3 : LinearReward {
	
	Model model;
	int _dim;

	public this (Model model, int dim) {
		this.model = model;
		this._dim = dim;
	}
	
	public override int dim() {
		return this._dim;
	}
			
	public override double [] features(State st, Action action) {
		sortingState state = cast(sortingState)st;
		sortingState next_state = cast(sortingState)(action.apply(st));
		
		double [] result;
		result.length = dim();
		result[] = 0;

		// good placed on belt
        if (next_state._prediction == 1 && next_state._onion_location == 4) result[0] = 1;

		// bad placed on belt
        if (next_state._prediction == 0 && next_state._onion_location == 4) result[1] = 1;

		// good placed in bin
        if (next_state._prediction == 1 && next_state._onion_location == 2) result[2] = 1;

		// bad placed in bin
        if (next_state._prediction == 0 && next_state._onion_location == 2) result[3] = 1;

        // staying still
        if ( state._onion_location == next_state._onion_location &&
        state._prediction == next_state._prediction &&
        state._EE_location == next_state._EE_location &&
        state._listIDs_status == next_state._listIDs_status) result[4] = 1; 

        // classify after picking
        if ((! (state._prediction == next_state._prediction)) && 
        	(state._prediction == 2 && state._onion_location!=0) ) result[5] = 1;

        // create the list 
        if (state._listIDs_status == 2 && next_state._listIDs_status != 2) result[6] = 1;

        // pick a good or pick already placed one (pick-place-pick cycles)
        //if ((state._onion_location == 4 || state._prediction == 1) && next_state._EE_location == 3) result[7] = 1;
        // pick a good one
        //if (state._prediction == 1 && next_state._EE_location == 3) result[7] = 1;
        // pick a placed one
        if (state._onion_location == 4 && next_state._EE_location == 3) result[7] = 1;

        // classify without picking
        if ((! (state._prediction == next_state._prediction)) && 
        	(state._prediction == 2 && state._onion_location==0) ) result[8] = 1;

        // place an uninspected in bin
        if (state._prediction == 2 && next_state._EE_location == 2) result[9] = 1;

		return result;
	}
}

class sortingReward4 : LinearReward {
	
	Model model;
	int _dim;

	public this (Model model, int dim) {
		this.model = model;
		this._dim = dim;
	}
	
	public override int dim() {
		return this._dim;
	}
			
	public override double [] features(State st, Action action) {

		sortingState state = cast(sortingState)st;
		sortingState next_state = cast(sortingState)(action.apply(st));
		
		double [] result;
		result.length = dim();
		result[] = 0;

		// good placed on belt
        if (next_state._prediction == 1 && next_state._onion_location == 4) result[0] = 1;

		// not placing bad on belt
        if (next_state._prediction == 0 && next_state._onion_location != 4) result[1] = 1;

		// not placing good in bin
        if (next_state._prediction == 1 && next_state._onion_location != 2) result[2] = 1;

		// bad placed in bin
        if (next_state._prediction == 0 && next_state._onion_location == 2) result[3] = 1;

        // not staying still
        if (!( state._onion_location == next_state._onion_location &&
        state._prediction == next_state._prediction &&
        state._EE_location == next_state._EE_location &&
        state._listIDs_status == next_state._listIDs_status)) result[4] = 1; 

        // classify after picking
        if ( (!(state._prediction == next_state._prediction)) && 
        	(state._prediction == 2 && state._onion_location!=0) ) result[5] = 1;

        // create the list 
        if (state._listIDs_status == 2 && next_state._listIDs_status != 2) result[6] = 1;

        // not picking a placed one
        if (state._onion_location == 4 && next_state._EE_location != 3) result[7] = 1;

        // classify without picking
        if ((! (state._prediction == next_state._prediction)) && 
        	(state._prediction == 2 && state._onion_location==0) ) result[8] = 1;

        // not placing uninspected in bin
        if (state._prediction == 2 && next_state._EE_location != 2) result[9] = 1;

		return result;
	}
}


class sortingReward5 : LinearReward {
	
	Model model;
	int _dim;

	public this (Model model, int dim) {
		this.model = model;
		this._dim = dim;
	}
	
	public override int dim() {
		return this._dim;
	}
			
	public override double [] features(State st, Action action) {

		sortingState state = cast(sortingState)st;
		sortingState next_state = cast(sortingState)(action.apply(st));
		
		double [] result;
		result.length = dim();
		result[] = 0;

		// good placed on belt
        if (next_state._prediction == 1 && next_state._onion_location == 4) result[0] = 1;

		// not placing bad on belt
        if (next_state._prediction == 0 && next_state._onion_location != 4) result[1] = 1;

		// not placing good in bin
        if (next_state._prediction == 1 && next_state._onion_location != 2) result[2] = 1;

		// bad placed in bin
        if (next_state._prediction == 0 && next_state._onion_location == 2) result[3] = 1;

        // not staying still
        if (!( state._onion_location == next_state._onion_location &&
        state._prediction == next_state._prediction &&
        state._EE_location == next_state._EE_location &&
        state._listIDs_status == next_state._listIDs_status)) result[4] = 1; 

        // claim new onion 
        if (state._prediction == 2 &&
        (state._onion_location == 2 || state._onion_location == 4)
        && state._EE_location == 3) result[5] = 1;

        // create the list 
        if (state._listIDs_status == 0 && 
        	next_state._listIDs_status == 1) result[6] = 1;

        // picking an unknown
        if (state._onion_location == 0 && state._prediction == 2 
        && ((next_state._prediction == 2 && next_state._EE_location == 3))) result[7] = 1;

		return result;
	}
}

class sortingReward6 : LinearReward {
	
	Model model;
	int _dim;

	public this (Model model, int dim) {
		this.model = model;
		this._dim = dim;
	}
	
	public override int dim() {
		return this._dim;
	}
			
	public override double [] features(State st, Action action) {

		sortingState state = cast(sortingState)st;
		sortingState next_state = cast(sortingState)(action.apply(st));
		
		double [] result;
		result.length = dim();
		result[] = 0;

		// good placed on belt
        if (next_state._prediction == 1 && next_state._onion_location == 4) result[0] = 1;

		// not placing bad on belt
        if (next_state._prediction == 0 && next_state._onion_location != 4) result[1] = 1;

		// not placing good in bin
        if (next_state._prediction == 1 && next_state._onion_location != 2) result[2] = 1;

		// bad placed in bin
        if (next_state._prediction == 0 && next_state._onion_location == 2) result[3] = 1;

        // not staying still
        if (!( state._onion_location == next_state._onion_location &&
        state._prediction == next_state._prediction &&
        state._EE_location == next_state._EE_location &&
        state._listIDs_status == next_state._listIDs_status)) result[4] = 1; 

        // claim new onion 
        if (state._prediction == 2 &&
        (state._onion_location == 2 || state._onion_location == 4)
        && state._EE_location == 3) result[5] = 1;

        // create the list 
        if (state._listIDs_status == 0 && 
        	next_state._listIDs_status == 1) result[6] = 1;

        // picking an unknown
        if (state._onion_location == 0 && state._prediction == 2 
        && ((next_state._prediction == 2 && next_state._EE_location == 3))) result[7] = 1;

        // picking an onion with known pred - blemished  
        if (state._onion_location == 0 && state._prediction == 0
        && ((next_state._prediction == 0 && next_state._EE_location == 3))) result[8] = 1;

        // finish the list 
        if (state._listIDs_status == 1 && 
        next_state._listIDs_status == 0) result[9] = 1;

        // inspect picked onion 
        if (state._onion_location == 3 && state._prediction == 2 && 
        next_state._prediction != 2) result[10] = 1;

		return result;
	}
}


class sortingReward7WPlaced : LinearReward { // corrected phi[0,1,2,3,5]
	
	Model model;
	int _dim;

	public this (Model model, int dim) {
		this.model = model;
		this._dim = dim;
	}
	
	public override int dim() {
		return this._dim;
	}
			
	public override double [] features(State st, Action action) {

		sortingState state = cast(sortingState)st;
		sortingState next_state = cast(sortingState)(action.apply(st));
		
		double [] result;
		result.length = dim();
		result[] = 0;

		// good placed on belt
        if (state._prediction == 1 && next_state._onion_location == 4) result[0] = 1;

		// not placing bad on belt
        if (state._prediction == 0 && next_state._onion_location != 4) result[1] = 1;

		// not placing good in bin
        if (state._prediction == 1 && next_state._onion_location != 2) result[2] = 1;

		// bad placed in bin
        if (state._prediction == 0 && next_state._onion_location == 2) result[3] = 1;

        // not staying still
        if (!( state._onion_location == next_state._onion_location &&
        state._prediction == next_state._prediction &&
        state._EE_location == next_state._EE_location &&
        state._listIDs_status == next_state._listIDs_status)) result[4] = 1; 

        // claim new onion 
        if (next_state._prediction == 2 &&
        next_state._onion_location == 0) result[5] = 1;

        // create the list 
        if (state._listIDs_status == 0 && 
        	next_state._listIDs_status == 1) result[6] = 1;

        // picking an unknown
        if (state._onion_location == 0 && state._prediction == 2 
        && ((next_state._prediction == 2 && next_state._EE_location == 3))) result[7] = 1;

        // picking an onion with known pred - blemished  
        if (state._onion_location == 0 && state._prediction == 0
        && ((next_state._prediction == 0 && next_state._EE_location == 3))) result[8] = 1;

        // finish the list 
        if (state._listIDs_status == 1 && 
        next_state._listIDs_status == 0) result[9] = 1;

        // inspect picked onion 
        if (state._onion_location == 3 && state._prediction == 2 && 
        next_state._prediction != 2) result[10] = 1;

		return result;
	}
}

class sortingReward7 : LinearReward { // corrected phi[0,1,2,3,5]
	
	Model model;
	int _dim;

	public this (Model model, int dim) {
		this.model = model;
		this._dim = dim;
	}
	
	public override int dim() {
		return this._dim;
	}
			
	public override double [] features(State st, Action action) {

		sortingState state = cast(sortingState)st;
		sortingState next_state = cast(sortingState)(action.apply(st));
		
		double [] result;
		result.length = dim();
		result[] = 0;

		// good placed on belt
        if (state._prediction == 1 && next_state._onion_location == 0) result[0] = 1;

		// not placing bad on belt
        if (state._prediction == 0 && next_state._onion_location != 0) result[1] = 1;

		// not placing good in bin
        if (state._prediction == 1 && next_state._onion_location != 2) result[2] = 1;

		// bad placed in bin
        if (state._prediction == 0 && next_state._onion_location == 2) result[3] = 1;

        // not staying still
        if (!( state._onion_location == next_state._onion_location &&
        state._prediction == next_state._prediction &&
        state._EE_location == next_state._EE_location &&
        state._listIDs_status == next_state._listIDs_status)) result[4] = 1; 

        // claim new onion 
        if (next_state._prediction == 2 &&
        next_state._onion_location == 0) result[5] = 1;

        // create the list 
        if (state._listIDs_status == 0 && 
        	next_state._listIDs_status == 1) result[6] = 1;

        // picking an unknown
        if (state._onion_location == 0 && state._prediction == 2 
        && ((next_state._prediction == 2 && next_state._EE_location == 3))) result[7] = 1;

        // picking an onion with known pred - blemished  
        if (state._onion_location == 0 && state._prediction == 0
        && ((next_state._prediction == 0 && next_state._EE_location == 3))) result[8] = 1;

        // finish the list 
        if (state._listIDs_status == 1 && 
        next_state._listIDs_status == 0) result[9] = 1;

        // inspect picked onion 
        if (state._onion_location == 3 && state._prediction == 2 && 
        next_state._prediction != 2) result[10] = 1;

		return result;
	}
}



int print_FormattedOutput(sar pair) {
	//s = pair.s;
	sortingState s = cast(sortingState)(pair.s);
	writeln("[",s._onion_location,",",
		s._prediction,",",s._EE_location,
		",",s._listIDs_status,"]:",pair.a,":1;");
	return 0;
}

//int xyz() {

//	// receives weights and returns sorting policy
//	// double [] params_manualTuning_pickinspectplace = [ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12];
//	// double [] params_manualTuning_rolling = [0.15082956259426847, -0.075414781297134234, -0.11312217194570136, 
//	// 0.19607843137254902, -0.030165912518853699, 0.0, 0.28355957767722473, -0.15082956259426847]; 
//	// double [] params_neg_pickinpectplace = [ 0.0, 0.10, 0.22, 0.0, -0.12, 0.44, 0.0, -0.12];

//	LinearReward reward;
//	Model model;
//	ValueIteration vi = new ValueIteration();

//	Agent opt_policy1, opt_policy2, opt_policy3, opt_policy4;

//	model = new sortingModel(0.05,null);
//	//int dim = 14;
//	//reward = new sortingReward1(model,dim); 
//	int dim = 8;
//	reward = new sortingReward2(model,dim); 
//	double [] params = new double[dim]; 

//	writeln("\n behavior method pick-inspect-place:");
//	double [] params_manualTuning_pickinspectplace = [ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12];
//	// correct simulated trajs with last version of 8th reward feature 
//	params = params_manualTuning_pickinspectplace; 

//	reward.setParams(params);
//    model.setReward(reward);
//    model.setGamma(0.99);

//    //writeln("starting V = vi.solve");
//    double vi_threshold = 0.2;
//    double[State] V;
//    V = vi.solve(model, vi_threshold);
//    //writeln(V);

//    opt_policy1 = vi.createPolicy(model, V);
//    //writeln("finished V = vi.solve");

//    int ol, pr, el, ls;

//	foreach (State s; model.S()) {
//		foreach (Action act, double chance; opt_policy1.actions(s)) {
//            sortingState s = cast(sortingState)s;
//            string str_s="";
//			if (s._onion_location == 0) str_s=str_s~"Onconveyor,";
//			if (s._onion_location == 1) str_s=str_s~"Infront,";
//			if (s._onion_location == 2) str_s=str_s~"Inbin,";
//			if (s._onion_location == 3) str_s=str_s~"Picked/AtHomePose,";
//			if (s._onion_location == 4) str_s=str_s~"Placed,";

//			if (s._prediction == 0) str_s=str_s~"bad,";
//			if (s._prediction == 1) str_s=str_s~"good,";
//			if (s._prediction == 2) str_s=str_s~"unknown,";

//			if (s._EE_location == 0) str_s=str_s~"Onconveyor,";
//			if (s._EE_location == 1) str_s=str_s~"Infront,";
//			if (s._EE_location == 2) str_s=str_s~"Inbin,";
//			if (s._EE_location == 3) str_s=str_s~"Picked/AtHomePose,";

//			if (s._listIDs_status == 0) str_s=str_s~"Empty";
//			if (s._listIDs_status == 1) str_s=str_s~"NotEmpty"; 
//			if (s._listIDs_status == 2) str_s=str_s~"Unavailable";

//            //writeln(str_s," = ", act);
//		}
//	}

//	double[State] initial;
//	foreach (State st ; model.S()) {
//		sortingState s = cast(sortingState)st;
//		//if (s._onion_location == 2 && s._prediction == 0 && s._EE_location == 2 && s._listIDs_status ==0) initial[s] = 1.0;		
//		initial[s] = 1.0;
//	}
//	Distr!State.normalize(initial);

//	sar [] traj;
//	//writeln("\nSimulation behavior:");
//	//for(int i = 0; i < 15; i++) {
//	//	traj = simulate(model, opt_policy1, initial, 50);
//	//	foreach (sar pair ; traj) {
//	//		print_FormattedOutput(pair);
//			//writeln(pair.s, " ", pair.a, " ", pair.r);
//	//	}
//	//	writeln("ENDTRAJ");
//	//}

//	//writeln("\n behavior method roll-pick-place:");
//	double [] params_manualTuning_rolling = [0.15082956259426847, -0.075414781297134234, -0.11312217194570136, 
//	0.19607843137254902, -0.030165912518853699, 0.0, 0.28355957767722473, -0.15082956259426847]; 
//	// best simulated trajs with last version of 8th reward feature
//	params = params_manualTuning_rolling; 

//	reward.setParams(params);

//    //writeln("starting V = vi.solve");
//    V = vi.solve(model, vi_threshold);
//    opt_policy2 = vi.createPolicy(model, V);

//	foreach (State s; model.S()) {
//		foreach (Action act, double chance; opt_policy2.actions(s)) {
//            sortingState s = cast(sortingState)s;
//            string str_s="";
//			if (s._onion_location == 0) str_s=str_s~"Onconveyor,";
//			if (s._onion_location == 1) str_s=str_s~"Infront,";
//			if (s._onion_location == 2) str_s=str_s~"Inbin,";
//			if (s._onion_location == 3) str_s=str_s~"Picked/AtHomePose,";
//			if (s._onion_location == 4) str_s=str_s~"Placed,";

//			if (s._prediction == 0) str_s=str_s~"bad,";
//			if (s._prediction == 1) str_s=str_s~"good,";
//			if (s._prediction == 2) str_s=str_s~"unknown,";

//			if (s._EE_location == 0) str_s=str_s~"Onconveyor,";
//			if (s._EE_location == 1) str_s=str_s~"Infront,";
//			if (s._EE_location == 2) str_s=str_s~"Inbin,";
//			if (s._EE_location == 3) str_s=str_s~"Picked/AtHomePose,";

//			if (s._listIDs_status == 0) str_s=str_s~"Empty";
//			if (s._listIDs_status == 1) str_s=str_s~"NotEmpty"; 
//			if (s._listIDs_status == 2) str_s=str_s~"Unavailable";

//            //writeln(str_s," = ", act);
//		}
//	}

//	//writeln("\nSimulation behavior :");	
//	//for(int i = 0; i < 15; i++) {
//	//	traj = simulate(model, opt_policy2, initial, 50);
//	//	foreach (sar pair ; traj) {
//	//		print_FormattedOutput(pair);
//			//writeln(pair.s, " ", pair.a, " ", pair.r);
//	//	}
//	//	writeln("ENDTRAJ");
//	//}

//	writeln("\n behavior method negative pick-inspect-place:");
//	double [] params_neg_pickinpectplace = [ 0.0, 0.10, 0.22, 0.0, -0.12, 0.44, 0.0, -0.12];
//	// correct simulated trajs with last version of 8th reward feature 
//	params = params_neg_pickinpectplace; 

//	reward.setParams(params);

//    //writeln("starting V = vi.solve");
//    V = vi.solve(model, vi_threshold);
//    //writeln(V);

//    opt_policy3 = vi.createPolicy(model, V);
//    //writeln("finished V = vi.solve");

//	writeln("\nSimulation behavior :");	
//	for(int i = 0; i < 15; i++) {
//		traj = simulate(model, opt_policy3, initial, 50);
//		foreach (sar pair ; traj) {
//			print_FormattedOutput(pair);
//		}
//		writeln("ENDTRAJ");
//	}

//	return 0;
//}
/*
	foreach (State s; model.S()) {
		foreach (Action a; model.A(s)) {
			double[State] T = model.T(s, a);
			foreach (s_prime, p; T){
				writeln(((s_prime),p));
			}
		}
	}
*/
