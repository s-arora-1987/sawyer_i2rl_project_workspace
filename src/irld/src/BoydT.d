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


int [] boyd2features(State state, Action action) {
	
	if (cast(TurnRightAction)action) { 
		return [0, 1, 1, 1];
	} 

	if (cast(TurnLeftAction)action) { 
		return [1, 0, 1, 1];
	} 
	
	if (cast(StopAction)action) {
		return [1, 1, 0, 1];
	} 
	return [1, 1, 1, 1];
}

int main() {


	double [] weights;
	weights.length = 4;
	sar [][][] SAR;
	string mapToUse;
	string buf;
	
	buf = readln();
	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	buf = readln();
	formattedRead(buf, "%s, %s, %s, %s", &(weights[0]), &(weights[1]), &(weights[2]), &(weights[3]));
	
	
	int curPatroller = 0;
	SAR.length = 1;
	
	byte [][] themap;
	
	if (mapToUse == "boyd2") {
		themap = boyd2PatrollerMap();
		
	} else {
		themap = boydrightPatrollerMap();
		
	}
	
	BoydModel model = new BoydModel(null, themap, null, 1, &boyd2features);
	
/*	foreach (a; model.A()) {
		errorModel[a] = 1.0 / model.A().length;
	}*/
	
	model.setT(model.createTransitionFunction(weights, &stopErrorModel));
	
	
	foreach (s; model.S()) {
		foreach(a; model.A()) {
			auto s_primes = model.T(s, a);
			
			foreach(s_prime, pr_s_prime; s_primes) {
				writeln( (cast(BoydState)s).getLocation(), ":", a, ":", (cast(BoydState)s_prime).getLocation(), ":", pr_s_prime);
			}
		}
	}
	

	return 0;
}
