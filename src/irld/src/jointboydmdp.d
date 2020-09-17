import mdp;
import std.array;
import std.format;
import std.math;
import std.stdio;
import std.string;
import std.conv;

class JointBoydState : mdp.State {
	
	private int [] location1;
	private int [] location2;
	private int interactionStep;
	
	public this ( int [] location1 = [0,0,0], int [] location2 = [0,0,0], int interactionStep = 0 ) {
		
		setLocation1(location1);
		setLocation2(location2);
		setInteractionStep(interactionStep);
		
	}
	
	public int[] getLocation1() {
		return location1;
	}
	
	public void setLocation1(int [] l) {
		assert(l.length == 3);
		
		this.location1 = l;
		
	}

	public int[] getLocation2() {
		return location2;
	}
	
	public void setLocation2(int [] l) {
		assert(l.length == 3);
		
		this.location2 = l;
		
	}
	
	public int getInteractionStep() {
		return interactionStep;
	}


	public void setInteractionStep(int i) {
		interactionStep = i;
	}
	
	public override string toString() {
		auto writer = appender!string();
		formattedWrite(writer, "JointBoydState: [location = %(%s, %) , %(%s, %) interactionStep = %s]", this.location1, this.location2, this.interactionStep);
		return writer.data; 
	}


	override hash_t toHash() {
		return location1[0] + location1[1] + location1[2] + location2[0] + location2[1] + location2[2] + interactionStep;
	}	

	override bool opEquals(Object o) {
		JointBoydState p = cast(JointBoydState)o;
		
		return p && p.location1[0] == location1[0] && p.location1[1] == location1[1] && p.location1[2] == location1[2] &&
		 			p.location2[0] == location2[0] && p.location2[1] == location2[1] && p.location2[2] == location2[2] &&
		 			interactionStep == p.getInteractionStep();
		
	}
	
	public static bool isConflict(int [] location1start, int [] location1end, int [] location2start, int [] location2end) {
	
		return (location1end[0] == location2end[0] && location1end[1] == location2end[1]) ||
				(location1start[0] == location2end[0] && location1start[1] == location2end[1]) ||
				(location1end[0] == location2start[0] && location1end[1] == location2start[1]);
	
	}
	
	public static bool isSamePlace(int [] loc1, int [] loc2) {
		return loc1[0] == loc2[0] && loc1[1] == loc2[1];
	}
	
	override public bool samePlaceAs(State o) {
		throw new Exception("Not Implemented");
		
	}
	
	override int opCmp(Object o) {
		JointBoydState p = cast(JointBoydState)o;

		if (!p) 
			return -1;

		for (int i = 0; i < location1.length; i ++) {
			if (p.location1[i] < location1[i])
				return 1;
			else if (p.location1[i] > location1[i])
				return -1;
			
		}

		for (int i = 0; i < location2.length; i ++) {
			if (p.location2[i] < location2[i])
				return 1;
			else if (p.location2[i] > location2[i])
				return -1;
			
		}

		return interactionStep - p.interactionStep;
	}

}

public class JointBoydAction : Action {
	
	protected int action1;
	protected int action2;
	protected int interactionLength;
	
	public this(int a1, int a2, int interactionLength = 0) {
		this.action1 = a1;
		this.action2 = a2;
		this.interactionLength = interactionLength;
		
	}
	
	
	public int getAction1() {
		return action1;
	}
	
	public int getAction2() {
		return action2;
	}
	
	public override State apply(State state) {
		JointBoydState p = cast(JointBoydState)state;
		int [] newLocation1;
		int [] newLocation2;
		
		int newInteraction;
		
		switch(action1) {
			case 0:
				newLocation1 = MoveForward(p.getLocation1());
				break;
			case 1:
				newLocation1 = Stop(p.getLocation1());
				break;
			case 2:
				newLocation1 = TurnLeft(p.getLocation1());
				break;
			case 3:
				newLocation1 = TurnRight(p.getLocation1());
				break;
			case 4:
				newLocation1 = TurnAround(p.getLocation1());
				break;
			default:
				throw new Exception("Invalid Action specified: " ~ to!string(action1));
		}

		switch(action2) {
			case 0:
				newLocation2 = MoveForward(p.getLocation2());
				break;
			case 1:
				newLocation2 = Stop(p.getLocation2());
				break;
			case 2:
				newLocation2 = TurnLeft(p.getLocation2());
				break;
			case 3:
				newLocation2 = TurnRight(p.getLocation2());
				break;
			case 4:
				newLocation2 = TurnAround(p.getLocation2());
				break;
			default:
				throw new Exception("Invalid Action specified: " ~ to!string(action2));
		}
		
		
		if (JointBoydState.isConflict(p.getLocation1(), newLocation1, p.getLocation2(), newLocation2) && p.getInteractionStep() == 0) {
			newInteraction = interactionLength;
		} else {
			newInteraction = p.getInteractionStep() - 1;
		}
		
		if (newInteraction < 0)
			newInteraction = 0;
		
		return new JointBoydState(newLocation1, newLocation2, newInteraction);
	}
	
	protected int [] MoveForward(int [] location) {
		
		int orientation = location[2];
		
		int [] s = location.dup;
		if (orientation == 0) 
			s[1] += 1;
		if (orientation == 1) 
			s[0] -= 1;
		if (orientation == 2) 
			s[1] -= 1;
		if (orientation == 3) 
			s[0] += 1;
		
		return s;
		
	}
	
	protected int [] Stop(int [] location) {
		return location.dup;
		
	}
	
	
	protected int [] TurnLeft(int [] location) {
		
		int orientation = location[2] + 1;
		
		if (orientation > 3)
			orientation = 0;
			
		int [] s = location.dup;
		s[2] = orientation;
		
		return s;
		
	}

	protected int [] TurnRight(int [] location) {
		
		int orientation = location[2] - 1;
		
		if (orientation < 0)
			orientation = 3;
			
		int [] s = location.dup;
		s[2] = orientation;
		
		return s;
		
	}
	
	protected int [] TurnAround(int [] location) {
		
		int orientation = location[2] + 2;
		
		if (orientation > 3)
			orientation -= 4;
			
		int [] s = location.dup;
		s[2] = orientation;
		
		return s;
		
	}
		
	public override string toString() {
		return printAction(action1) ~ ":" ~ printAction(action2); 
	}


	override hash_t toHash() {
		return action1 + action2;
	}	
	
	override bool opEquals(Object o) {
		JointBoydAction p = cast(JointBoydAction) o;
		
		return p && action1 == p.action1 && action2 == p.action2;
		
	}
	
	override int opCmp(Object o) {
		JointBoydAction p = cast(JointBoydAction)o;
		
		if (!p) 
			return -1;
		

		if (p.action1 < action1)
			return 1;
		else if (p.action1 > action1)
			return -1;
			
		if (p.action2 < action2)
			return 1;
		else if (p.action2 > action2)
			return -1;
					
		return 0;
		
	}
	
	
	public static string printAction(int a) {
		switch (a) {
			case 0:
				return "MoveForward";
				break;
			case 1:
				return "Stop";
				break;
			case 2:
				return "TurnLeft";
				break;
			case 3:
				return "TurnRight";
				break;
			case 4:
				return "TurnAround";
				break;
			default:
				throw new Exception("Invalid action specified: " ~ to!string(a));
		}
	}
		
}



class JointBoydModel : mdp.Model {
	
	byte [][] map;
	double p_fail;
	Action [] actions;
	State [] states;
	State terminal;
	JointBoydAction ne;
	JointBoydAction delayNe;
	int interactionLength;
	
	public this( double p_fail, State terminal, byte [][] themap, JointBoydAction nashEquilibrium, int interactionLength) {
		this.p_fail = p_fail;
		this.terminal = terminal;
		this.map = themap;
		this.ne = nashEquilibrium;
		this.interactionLength = interactionLength;
		this.delayNe = new JointBoydAction(1,1);
		
		foreach (i; 0..5) {
			foreach(j; 0..5) {
				actions ~= new JointBoydAction(i, j);
			}
		}
		
		
		foreach (int i; 0.. map.length) {
			foreach (int j; 0..map[i].length) {			
				foreach (int k; 0.. map.length) {
					foreach (int l; 0..map[k].length) {			
						if (map[i][j] == 1 && map[k][l] == 1) {
							foreach (o; 0..4) {
								foreach (o2; 0..4) {
									// if we're more than distance one apart in  x + y then we don't need to worry about interaction
									states ~= new JointBoydState([i, j, o], [k, l, o2], 0);
									if (abs(i - k) <= 1 && abs(j - l) <= 1) {
										foreach (interaction; 1..interactionLength + 1) {
											states ~= new JointBoydState([i, j, o], [k, l, o2], interaction);
										}
									}
								}
							}
						}
					}
				}
			}
		}
		
	}
	
	
	public override double[State] T(State state, Action act) {
		
		// is the given state a conflict? if not, just apply the action normally
		// if so, then see if we are still in the interaction length, and if so apply the delayNe
		// otherwise, apply the Game to resolve the conflict (converting to the correct action if necessary)
		
		
		JointBoydState s = cast(JointBoydState)state;
		JointBoydAction action = cast(JointBoydAction)act;
		
		
		if (s.getInteractionStep() > 0) {
		
			JointBoydState newState;
			if (s.getInteractionStep() == 1) {
				// check the given ne
				
				int action1 = ne.getAction1();
				int action2 = ne.getAction2();
				
				if (action1 == 0) {
					action1 = action.getAction1();
				}
				if (action2 == 0) {
					action2 = action.getAction2();
				}
				
				action = new JointBoydAction(action1, action2);
			
			} else {
				action = delayNe;
			}
		
		}
		double[State] returnval;
		
		Action [] actions = A(state);
		
		double totalP = 0;
		foreach (a ; actions) {
			double p = 0;
			if (a == action) {
				p = 1.0 - p_fail;
			} else {
				p = p_fail / (actions.length - 1);
			}
			
			auto s_prime = a.apply(state);
			
			if (! is_legal(s_prime) ) {
				returnval[state] += p;
			} else {
				returnval[s_prime] += p;
			}
			totalP += p;
		}
		
		returnval[state] += 1.0 - totalP;
		
		return returnval;
	}
	
	public override State [] S () {
		
		return states;
	}
	
	public override Action[] A(State state = null) {
		return actions;
		
	}
	
	public override bool is_terminal(State state) {
		return state == terminal;
	}
	
	public override bool is_legal(State state) {
		JointBoydState s = cast(JointBoydState)state;
		auto l1 = s.getLocation1();
		auto l2 = s.getLocation2();
		if (s.getInteractionStep() > 0) {
			if (abs(l1[0] - l2[0]) > 1 || abs(l1[1] - l2[1]) > 1) {
				return false;
			}
		}
		
		
		return l1[0] >= 0 && l1[0] < map.length && l1[1] >= 0 && l1[1] < map[0].length && map[l1[0]][l1[1]] == 1 &&
				l2[0] >= 0 && l2[0] < map.length && l2[1] >= 0 && l2[1] < map[0].length && map[l2[0]][l2[1]] == 1 &&
				s.getInteractionStep() <= interactionLength && s.getInteractionStep() >= 0;
	}
}


class JointBoyd2Reward : LinearReward {
	
	Model model;
	int[][][] distance;
	int maxDistance;
	int turnAroundActionNum;
	
	public this (Model model, int[][][] distance) {
		this.model = model;
		this.distance = distance;
		
		foreach (i; distance) {
			foreach (j ; i) {
				foreach (k; j) {
					if (k > maxDistance)
						maxDistance = k;
				}
			}
		}
		
		this.turnAroundActionNum = 4;
		
	}
	
	public override int dim() {
		return 2 * (2 + maxDistance);
	}
			
	public override double [] features(State state, Action action) {
		
		State newState = action.apply(state);

		double [] result = new double[dim()];
		result[] = 0;

		if (! model.is_legal(newState)) {
			return result;
		}
				
		// for each patroller:
		// 	check if the patroller's location changed in newState, if so give the appropriate feature
		//  lookup the distance for the state's location for this patroller, mark the appropriate feature
		
		JointBoydState jbs = cast(JointBoydState)state;
		JointBoydState newjbs = cast(JointBoydState)newState;
		JointBoydAction jba = cast(JointBoydAction)action;

		// first patroller
		 
		if (! JointBoydState.isSamePlace(jbs.getLocation1(), newjbs.getLocation1())) {
			result[0] = 1;
		}

		if (jba.getAction1() == this.turnAroundActionNum) {
			int [] loc = jbs.getLocation1();
			auto stateDistance = distance[loc[0]][loc[1]][loc[2]];
			result[stateDistance + 1] = 1;
		}


		// second patroller
		 
		if (! JointBoydState.isSamePlace(jbs.getLocation2(), newjbs.getLocation2())) {
			result[dim() / 2] = 1;
		}

		if (jba.getAction2() == this.turnAroundActionNum) {
			int [] loc = jbs.getLocation2();
			auto stateDistance = distance[loc[0]][loc[1]][loc[2]];
			result[(dim() / 2) + stateDistance + 1] = 1;
		}
		
		return result;
	}

}
