import random
import util.functions
import mdp.agent

class DBNFULLSTATES:
	
	def __init__(self, model, traj ):
				
		temp = {}
		
		self.maxCount = len(model.S())
		self.model = model
		
		for a in model.A():
			
			t = {}
			for i in model.S():
				
				u = {}
				for q in model.S():
					u[q] = 0.001
				
				t[i] = u
			temp[a] = t
			
		self.counts = temp
		
		
		preventry = None
		for timestep in traj:
			if not timestep is None and not preventry is None:
				a = preventry[1]
					
				self.counts[a][preventry[0]][timestep[0]] += 1
				
				
			preventry = timestep
	
		
		temp = {}
		
		for action in model.A():
			a = {}
			for start_state in model.S():
				total = 0				
				for end_state in model.S():
					total += self.counts[action][start_state][end_state] 
		
				a[start_state] = total
			temp[action] = a
		self.totals = temp
	
	
	def getTransition(self, s, a, s_prime):
		
		count = self.counts[a][s][s_prime]
		
		total = self.totals[a][s]
		
		if (total < 1):
			return 0
		
		return float(count) / float(total)


class EM:
	
	def calculate(self, model, traj, iterations, samples = 100):
		# generate a random transition table
		# go in a loop calling E() and then M()
		# when out of iterations, return the result
		
		S = model.S()
		A = model.A()
		self.S_len = len(S)
		self.num_samples = samples
		self.num_samples = 10 * len(traj)
		
		self.s_index = {}
		for i, s in enumerate(S):
			self.s_index[s] = i
		
		self.a_index = {}
		for i, a in enumerate(A):
			self.a_index[a] = i 
		
		transitions = [[random.random() for x in xrange(len(S))] for x in xrange(len(S) * len(A))] 	
		
		# normalize so all transitions sum to one
		for i, row in enumerate(transitions):
			thesum = 0.0
			for entry in row:
				thesum += entry
			
			transitions[i] = [x / thesum for x in row]
		
		prev_transitions = transitions
		
		# remove blank entries at the beginning of the trajectory (would need to sample from a uniform prior to handle these, which requires a buttload more sampling
		for i, t in enumerate(traj):
			if t is not None:
				break
		traj = traj[i:]
		
		lastdelta = 2.0
		i = 0
		while(True):
			print("Iteration number " + str(i))
			i += 1
			weighted_trajs = self.E(model, traj, transitions)
			print("Finished E step")
			transitions = self.M(model, weighted_trajs, 0.001)
			print("Finished M step")

			delta = -1
			
			for r, row in enumerate(transitions):
				newrow = [abs(prev_transitions[r][x] - row[x]) for x in range(len(row))]
				
				for entry in newrow:
					if entry > delta:
						delta = entry
			
			print("Delta = " + str(delta))
			if (delta < 0.04):
				break
			prev_transitions = transitions
			if (delta > lastdelta):
				self.num_samples = int(self.num_samples * 1.1)
				
			
			lastdelta = delta
			
			
		self.transitions = self.M(model, weighted_trajs, 0.0)
			

	def getTransition(self, s, a, s_prime):
		# convert transitions to what the solvers are expecting
		
		return self.transitions[self.S_len * self.a_index[a] + self.s_index[s]][self.s_index[s_prime]]
	
	def E(self, model, traj, transitions):
		# generate a bunch of trajectories by filling in the missing entries, use simulate for this
		# find a weight for each generated trajectory
		# normalize the weights
		
		weighted_trajs = []
		
		uniform = mdp.agent.RandomAgent(model.A())
		
		for i in range(self.num_samples):
			traj_copy = traj[:]
			
			for t, entry in enumerate(traj_copy):
				
				if entry is None or len(entry) == 0:
					# found a point to insert into, find the ending
					
					# the ending could be somewhere in the middle of the trajectory, or at the end
					# the starting could be at position 0
					endpoint = t + 1
					for t2, endentry in enumerate(traj_copy[t + 1:]):
						endpoint = t + 1 + t2
						if endentry is not None and len(endentry) > 0:
							break
					
					initial = {}
					
					if (t > 0):
						initial[traj_copy[t - 1][0]] = 1.0
					else:
						S = model.S();
						for s in S:
							initial[s] = 1.0 / len(S)

					traj_copy[t:endpoint] = self.simulate(model, transitions, uniform, initial, endpoint - t)
			

			# calc total probability of trajectory
			prob = 1.0
			
			for t, entry in enumerate(traj_copy):
				if (t > 0):
					start_s = self.s_index[traj_copy[t - 1][0]]
					end_s = self.s_index[traj_copy[t][0]]
					action = self.a_index[traj_copy[t - 1][1]]
					
					prob *= transitions[len(model.S())*action + start_s][end_s]
					
			weighted_trajs.append((prob, traj_copy))
			

		# normalize weights		
		sum_weights = 0
		for t in weighted_trajs:
			sum_weights += t[0]
			
		for i, t in enumerate(weighted_trajs):
			weighted_trajs[i] = (t[0] / sum_weights, t[1])


				
		return weighted_trajs
	
	
	def M(self, model, weighted_trajs, prior):
		# generate a new transition table by finding the transitions for each generated trajectory
		# then combining them together according to the transition weight  

		transitions = [[prior for x in xrange(len(model.S()))] for x in xrange(len(model.S()) * len(model.A()))] 	
		
		for weight, traj in weighted_trajs:
			
			# for each transition, add a weighted amount to the total transitions
			# then normalize the rows
			
			preventry = None
			for timestep in traj:
				if not preventry is None:
					a = preventry[1]
					
					transitions[self.S_len * self.a_index[a] + self.s_index[preventry[0]]][self.s_index[timestep[0]]] += weight	
					
				preventry = timestep
			
			
		
		# normalize so all transitions sum to one
		for i, row in enumerate(transitions):
			thesum = 0.0
			for entry in row:
				thesum += entry
			
			if thesum > 0:
				transitions[i] = [x / thesum for x in row]		
			
			
#			dbn = DBNFULLSTATES(model, traj)
			
#			for s in model.S():
#				for a in model.A():
#					for s_prime in model.S():
#						start_s = self.s_index[s]
#						end_s = self.s_index[s_prime]
#						action = self.a_index[a]						
#						
#						transitions[len(model.S())*action + start_s][end_s] += weight * dbn.getTransition(s, a, s_prime)
		

		return transitions
				
				

	def simulate(self, model, transitions, agent, initial, t_max):
		'''
		Simulate an MDP for t_max timesteps or until the
		a terminal state is reached.  Returns a list
			[ (s_0, a_0, r_0), (s_1, a_1, r_1), ...]
		'''
		
		s = util.functions.sample(initial)
		result = []
		t = 0
		while t < t_max and not model.is_terminal(s): 
			a = agent.sample(s)
			
			T = {}
			
			for key, value in self.s_index.iteritems():
				T[key] = transitions[self.S_len * self.a_index[a] + self.s_index[s]][value]
			
			s_p = util.functions.sample( T )
	#		r = model.R(s,a)
			
			result.append( (s,a) )
			s = s_p
			t += 1
		if model.is_terminal(s):
			a = agent.sample(s)
	#		r = model.R(s,a)
			
			result.append( (s,a) )
			
		return result				