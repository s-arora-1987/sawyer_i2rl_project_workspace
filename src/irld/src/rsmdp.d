import mdp;
import std.math;
import std.algorithm;
import std.stdio;



public class RiskSensitiveModel : mdp.Model {
	
	
	public double getRiskSensitivity(State s);
	
	
}


public class ValueIteration : mdp.ValueIteration {
	
	public override double[State] solve(Model mod, double err) {
		RiskSensitiveModel model = cast(RiskSensitiveModel) mod;
		
		double[State] V;
		foreach (State s ; model.S()) {
			V[s] = 0.0;
		}
		
		double delta = 0;
		int i = 0;
		while (true) {
			delta = 0;
			double[State] V_next;
			
			foreach (State s ; model.S()) {				
				if (model.is_terminal(s)) {
					V_next[s] = model.R(s, null);
					delta = max(delta, abs(V[s] - V_next[s]));
					continue; 
				}
				
				double[Action] q;
				foreach (Action a; model.A(s)) {
					double r = model.R(s, a);
					double[State] T = model.T(s, a);

					double expected_rewards = 0;
					foreach (s_prime, p; T){
						expected_rewards += p*exp(model.getRiskSensitivity(s) * V[s_prime]);
					}
					
					q[a] = model.getGamma()* r + log(expected_rewards) / model.getRiskSensitivity(s);
				}
				double m = -double.max;
				foreach (double v; q.values)
					if (v > m)
						m = v;
				V_next[s] = m;
				
				delta = max(delta, abs(V[s] - V_next[s]));
			}
			V = V_next.dup;
			debug {
				writeln("Current Iteration ", i,": ", delta);
			}
			i ++;
			if (delta < err || i > max_iter)
				return V;

		}
			
		
	}

	
	
} 
