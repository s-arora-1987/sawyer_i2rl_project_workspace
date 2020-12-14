# -*- coding: utf-8 -*-
import util.classes
import mdp.agent
import patrol.rewardbayesian
import patrol.model
import random
import numpy
import math
import scipy.optimize

class QValueIteration(object):
    '''Use Q-Value Iteration to solve an MDP'''
    def __init__(self, max_iter):
        self._max_iter = max_iter
        
    def solve(self, model):
        '''Returns an Agent directed by a policy determined by this solver'''
        Q = util.classes.NumMap()
        for i in range(self._max_iter):

            print("Q-Value Iteration: " + str(i))
            Q = self.iter(model, Q)
            
        return Q
    
    @classmethod
    def iter(cls, model, Q):
        V = util.classes.NumMap()
        # Compute V(s) = max_{a} Q(s,a)
        for s in model.S():
            V_s = util.classes.NumMap()
            for a in model.A(s):
                V_s[a] = Q[ (s,a) ]
            if len(V_s) > 0:
                V[s] = V_s.max()
            else:
                V[s] = 0.0
        
        # QQ(s,a) = R(s,a) + gamma*sum_{s'} T(s,a,s')*V(s') 
        QQ = util.classes.NumMap()
        for s in model.S():
            for a in model.A(s):
                value = model.R(s,a)
                T = model.T(s,a)
                value += sum( [model.gamma*t*V[s_prime] for (s_prime,t) in  T.items()] )
                QQ[ (s,a) ] = value
        return QQ

        
class IRLBayesianSolver(object):
    '''Solves the inverse reinforcement learning problem'''
    def __init__(self, max_iter, mdp_solver, q_value, n_samples=500):
        '''
        max_iter: maximum number of times to iterate policies
        mdp_solver: class that implements self.solve(model)
        n_samples: number of samples used to estimate feature expectations
        '''
        self._max_iter = max_iter
        self._solver = mdp_solver
        self._n_samples = n_samples
        self._q_value_solver = q_value
        
    def solve(self, model, true_samples, s_r_prior, gridsize, max_states):
        '''
        Returns a pair (agent, weights) where the agent attempts to generalize
        from the behavior observed in samples and weights is what was combined with
        the MDP to generate agent.
        
        model: an MDP with a linear reward functions.  Parameters WILL be overwritten.
        initial: initial distribution over states
        samples: a list of sample trajectories [ (s_t,a_t) ] of the (supposedly) optimal
            policy.
        '''
        # Initial weight vector
#        w_0 = model.feature_function.params

        R = patrol.rewardbayesian.BayesianPatrolReward(len(model.S())*len(model.A()), gridsize)
        
        model.reward_function = R
        
        pi = self._solver.solve(model)
        
        for i in range(self._max_iter):
            R_tilde = R.randNeighbor()
            
            model.reward_function = R_tilde
            
            Q_pi = self._q_value_solver.solve(model)
            
            found_worse = False
            for history in true_samples:
                for (s, a) in history:
                    
                    if s.location[0] >= 0 and s.location[0] < max_states and Q_pi[(s, pi.actions(s).keys()[0])] < Q_pi[(s, a)]:
#                        print(a, Q_pi[(s, a)], pi.actions(s).keys()[0], Q_pi[(s, pi.actions(s).keys()[0])])
                        found_worse = True
                        break
                
            if found_worse:
                pi_tilde = self._solver.solve(model)
                
                chance = min(1, s_r_prior.prior(pi_tilde, R_tilde) / s_r_prior.prior(pi, R))

                if random.random() < chance:
                    pi = pi_tilde
                    R = R_tilde
            else:
                chance = min(1, s_r_prior.prior(pi, R_tilde) / s_r_prior.prior(pi, R))
                if random.random() < chance:
                    R = R_tilde
                
        model.reward_function = R
        return pi

        
class PatrolPrior:
    
    def __init__(self, all_samples, visibleLow, visibleHigh, patrolSize):
        self._samples = all_samples
        self._visibleLow = visibleLow
        self._visibleHigh = visibleHigh
        self._patrolSize = patrolSize
        totalLeft = 0
        totalRight = 0
        for history in self._samples:
            for (s, a) in history:
                if s.location[0] < 0:
                    totalLeft += 1
                elif s.location[0] >= patrolSize:
                    totalRight += 1
        
        self._totalLeft = totalLeft / 2
        self._totalRight = totalRight / 2
        
    def prior(self, policy, reward):
        
        infiniteTurnArounds = 0
        
        # count up the crap states (both actions are turnaround)
        for i in range(self._patrolSize):
            if policy.actions(patrol.model.PatrolState([i,0])).keys()[0].__class__.__name__ == "PatrolActionTurnAround" and policy.actions(patrol.model.PatrolState([i,1])).keys()[0].__class__.__name__ == "PatrolActionTurnAround":
                infiniteTurnArounds += 1
        
        visibleViolations = 0
        totalVisible = 0
        # count up the visible violations (visible actions of the expert don't match the given policy)
        for history in self._samples:
            for (s, a) in history:
                if (s.location[0] >= 0 and s.location[0] < self._patrolSize):
                    totalVisible += 1
                    
                    if ( not policy.actions(s).keys()[0].__class__.__name__ == a.__class__.__name__):
                        visibleViolations += 1
        
        # count up the time the expert spends outside of view, then count the number of turn arounds within the appropriate range of the visible
        leftTurnAroundViolations = 0
        rightTurnAroundViolations = 0
        for i in range(self._visibleLow, self._visibleLow - self._totalLeft):
            if i >= 0:
                if policy.actions(patrol.model.PatrolState([i,0])).keys()[0].__class__.__name__ == "PatrolActionTurnAround":
                    leftTurnAroundViolations += 1
                if policy.actions(patrol.model.PatrolState([i,1])).keys()[0].__class__.__name__ == "PatrolActionTurnAround":
                    leftTurnAroundViolations += 1
                    
        for i in range(self._visibleHigh, self._visibleHigh + self._totalRight):
            if i < self._patrolSize:
                if policy.actions(patrol.model.PatrolState([i,0])).keys()[0].__class__.__name__ == "PatrolActionTurnAround":
                    rightTurnAroundViolations += 1
                if policy.actions(patrol.model.PatrolState([i,1])).keys()[0].__class__.__name__ == "PatrolActionTurnAround":
                    rightTurnAroundViolations += 1
        
        
        return .01 + (1.0 - (infiniteTurnArounds / float(self._patrolSize)))*(1 - (visibleViolations / float(totalVisible)))*(1.0 - (leftTurnAroundViolations / self._totalLeft))*(1.0 - (rightTurnAroundViolations / self._totalRight))
        

    # This is apparently the projection algorithm from Ng 2004
class IRLApprximateSolver(object):
    '''Solves the inverse reinforcement learning problem'''
    def __init__(self, max_iter, mdp_solver, n_samples=500, error=.0001):
        '''
        max_iter: maximum number of times to iterate policies
        mdp_solver: class that implements self.solve(model)
        n_samples: number of samples used to estimate feature expectations
        '''
        self._max_iter = max_iter
        self._solver = mdp_solver
        self._n_samples = n_samples
        self._epsilon = error
        
    def solve(self, model, initial, true_samples):
        '''
        Returns a pair (agent, weights) where the agent attempts to generalize
        from the behavior observed in samples and weights is what was combined with
        the MDP to generate agent.
        
        model: an MDP with a linear reward functions.  Parameters WILL be overwritten.
        initial: initial distribution over states
        samples: a list of sample trajectories [ (s_t,a_t) ] of the (supposedly) optimal
            policy.
        '''
        # Initial weight vector
#        w_0 = model.feature_function.params
        
        # Compute feature expectations of agent = mu_E from samples
        mu_E = self.feature_expectations2(model, true_samples)
        print("True Samples", mu_E)
        # Pick random policy pi^(0)
        agent = mdp.agent.RandomAgent( model.A() )
        
        # Calculate feature expectations of pi^(0) = mu^(0)
        samples = self.generate_samples(model, agent, initial, len(true_samples[0]))
        mu = self.feature_expectations(model, samples )

#        mu = self.feature_expectations2(model, initial, agent )
        lastT = 0
								
        for i in range(self._max_iter):
            # Perform projections to new weights w^(i)
            if i == 0:
                mu_bar = mu
            else:
                mmmb = mu - mu_bar
                mu_bar = mu_bar + numpy.dot( mmmb, mu_E-mu_bar )/numpy.dot( mmmb,mmmb )*mmmb
            w = mu_E - mu_bar
            t = numpy.linalg.norm(mu_E - mu_bar)
            w[0] = abs(w[0])
            print(w)
            model.reward_function.params = w
            
            print 'IRLApproxSolver Iteration #{},t = {:4.4f}'.format(i,t)
            if t < self._epsilon:
                break
            if abs(t - lastT) < .000001:
                break

            lastT = t
            
            # Compute optimal policy used R(s,a) = dot( feature_f(s,a), w^(i) )
            if (numpy.linalg.norm(mu) == 0):
                agent = mdp.agent.RandomAgent( model.A() )
            else:
                agent = self._solver.solve(model)
                        
            # Compute feature expectations of pi^(i) = mu^(i)
            samples = self.generate_samples(model, agent, initial, len(true_samples[0]))
            mu = self.feature_expectations(model, samples)
            print(mu)
#            mu = self.feature_expectations2(model, initial, agent)
            
        # Restore initial weight vector
#        model.feature_function.params = w_0
        return (agent, w)
    
    def generate_samples(self, model, agent, initial, num_samples):
        '''
        Generate self.n_samples different histories of length t_max by
        following agent.  Each history of the form,
            [ (s_0,a_0), (s_1,a_1), ...]
        '''
        # t_max such that gamma^t_max = 0.01
        t_max = min(num_samples, math.ceil( math.log(0.01)/math.log(model.gamma) ))
        result = []
        for i in range(self._n_samples):
            hist = []
            for (s,a,r) in mdp.simulation.simulate(model, agent, initial, t_max):
                hist.append( (s,a) )
            result.append(hist)
        return result
            
    def feature_expectations(self, model, samples):
        '''
        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        '''
        ff = model.reward_function
        result = numpy.zeros( ff.dim )
#        count = 0
        for sample in samples:
            for (t,sa) in enumerate(sample):
                s = sa[0]
                a = sa[1]
                result += (model.gamma**t)*ff.features(s,a)
#                result += ff.features(s,a)
#                count += 1
        return (1.0/(len(samples)))*result
#        return result / float(count)
    
    def feature_expectations2(self, model, samples):
        '''
        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        
        This is for the expert only! we assume each timestep has multiple possible states!
        '''
        ff = model.reward_function
        result = numpy.zeros( ff.dim )
#        count = 0
        for sample in samples:
            for (t,sal) in enumerate(sample):
                for (s, a, p) in sal:
                    result += p*(model.gamma**t)*ff.features(s,a)
#                result += ff.features(s,a)
#                count += 1
        return (1.0/(len(samples)))*result


    # This is apparently the projection algorithm from Ng 2004
class IRLApprximateSolverBothPatrollers(object):
    '''Solves the inverse reinforcement learning problem'''
    def __init__(self, max_iter, mdp_solver, n_samples=500, error=.0001):
        '''
        max_iter: maximum number of times to iterate policies
        mdp_solver: class that implements self.solve(model)
        n_samples: number of samples used to estimate feature expectations
        '''
        self._max_iter = max_iter
        self._solver = mdp_solver
        self._n_samples = n_samples
        self._epsilon = error
        
    def solve(self, model, initial, true_samples, other_policy):
        '''
        Returns a pair (agent, weights) where the agent attempts to generalize
        from the behavior observed in samples and weights is what was combined with
        the MDP to generate agent.
        
        model: an MDP with a linear reward functions.  Parameters WILL be overwritten.
        initial: initial distribution over states
        samples: a list of sample trajectories [ (s_t,a_t) ] of the (supposedly) optimal
            policy.
        '''
        # Initial weight vector
#        w_0 = model.feature_function.params
        self.other_policy = other_policy
        self.full_initial = util.classes.NumMap()
        for s in model.S():
            self.full_initial[s] = 1.0
        self.full_initial = self.full_initial.normalize()
                
        # Compute feature expectations of agent = mu_E from samples
        mu_E = self.feature_expectations2(model, true_samples)
        print("True Samples", mu_E)
        # Pick random policy pi^(0)
        agent = mdp.agent.RandomAgent( model.A() )
        
        # Calculate feature expectations of pi^(0) = mu^(0)
        samples = self.generate_samples(model, agent, initial, len(true_samples[0]))
        mu = self.feature_expectations(model, samples )

#        mu = self.feature_expectations2(model, initial, agent )
        lastT = 0
								
        for i in range(self._max_iter):
            # Perform projections to new weights w^(i)
            if i == 0:
                mu_bar = mu
            else:
                mmmb = mu - mu_bar
                mu_bar = mu_bar + numpy.dot( mmmb, mu_E-mu_bar )/numpy.dot( mmmb,mmmb )*mmmb
            w = mu_E - mu_bar
            t = numpy.linalg.norm(mu_E - mu_bar)
            w[0] = abs(w[0])
            print(w)
            model.reward_function.params = w
            
            print 'IRLApproxSolver Iteration #{},t = {:4.4f}'.format(i,t)
            if t < self._epsilon:
                break
            if abs(t - lastT) < .000001:
                break

            lastT = t
            
            # Compute optimal policy used R(s,a) = dot( feature_f(s,a), w^(i) )
            if (numpy.linalg.norm(mu) == 0):
                agent = mdp.agent.RandomAgent( model.A() )
            else:
                agent = self._solver.solve(model)
                        
            # Compute feature expectations of pi^(i) = mu^(i)
            samples = self.generate_samples(model, agent, initial, len(true_samples[0]))
            mu = self.feature_expectations(model, samples)
            print(mu)
#            mu = self.feature_expectations2(model, initial, agent)
            
        # Restore initial weight vector
#        model.feature_function.params = w_0
        return (agent, w)
    
    def generate_samples(self, model, agent, initial, num_samples):
        '''
        Generate self.n_samples different histories of length t_max by
        following agent.  Each history of the form,
            [ (s_0,a_0), (s_1,a_1), ...]
        '''
        # t_max such that gamma^t_max = 0.01

        t_max = min(num_samples, math.ceil( math.log(0.01)/math.log(model.gamma) ))
        result = []
        for i in range(self._n_samples):
            otherHist = []
            for (s,a,r) in mdp.simulation.simulate(model, self.other_policy, self.full_initial, t_max):
                otherHist.append( (s,a) )
            
            hist = []
            for (s,a,r) in mdp.simulation.simulate(model, agent, initial, t_max):
                hist.append( (s,a) )
                
            (hist, otherHist) = self.add_delay(hist, otherHist)

            result.append(hist)
        return result

    def add_delay(self, hist1, hist2):
        newhist1 = []
        newhist2 = []
        for (i, sa) in enumerate(hist1):
            newhist1.append(sa)
            newhist2.append(hist2[i])
            if sa[0] == hist2[i][0]  or (i > 0 and ( all(hist1[i-1][0].location == hist2[i][0].location) or all(sa[0].location == hist2[i-1][0].location) ) ):
                newhist1.append(sa)
                newhist2.append(hist2[i])
                newhist1.append(sa)
                newhist2.append(hist2[i])
                
            
        newhist1 = newhist1[0:len(hist1)]
        newhist2 = newhist2[0:len(hist2)]
        return (newhist1, newhist2)
            
    def feature_expectations(self, model, samples):
        '''
        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        '''
        ff = model.reward_function
        result = numpy.zeros( ff.dim )
#        count = 0
        for sample in samples:
            for (t,sa) in enumerate(sample):
                s = sa[0]
                a = sa[1]
                result += (model.gamma**t)*ff.features(s,a)
#                result += ff.features(s,a)
#                count += 1
        return (1.0/(len(samples)))*result
#        return result / float(count)
    
    def feature_expectations2(self, model, samples):
        '''
        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        
        This is for the expert only! we assume each timestep has multiple possible states!
        '''
        ff = model.reward_function
        result = numpy.zeros( ff.dim )
#        count = 0
        for sample in samples:
            for (t,sal) in enumerate(sample):
                for (s, a, p) in sal:
                    result += p*(model.gamma**t)*ff.features(s,a)
#                result += ff.features(s,a)
#                count += 1
        return (1.0/(len(samples)))*result


# Maximum Entropy IRL from Ziebart 2004
class MaxEntIRLApprximateSolver(object):
    '''Solves the inverse reinforcement learning problem'''
    def __init__(self, max_iter, mdp_solver, n_samples=500, error=.0001, qValThresh=0.01):
        '''
        max_iter: maximum number of times to iterate policies
        mdp_solver: class that implements self.solve(model)
        n_samples: number of samples used to estimate feature expectations
        '''
        self._max_iter = max_iter
        self._solver = mdp_solver
        self._n_samples = n_samples
        self._epsilon = error
        self._qValThresh = qValThresh
        
    def solve(self, model, initial, true_samples):
        '''
        Returns a pair (agent, weights) where the agent attempts to generalize
        from the behavior observed in samples and weights is what was combined with
        the MDP to generate agent.
        
        model: an MDP with a linear reward functions.  Parameters WILL be overwritten.
        initial: initial distribution over states
        samples: a list of sample trajectories [ (s_t,a_t) ] of the (supposedly) optimal
            policy.
        '''
        # Initial weight vector
#        w_0 = model.feature_function.params
        
        # Compute feature expectations of agent = mu_E from samples
        (mu_E, sa_freq) = self.feature_expectations2(model, true_samples)
        print("True Samples", mu_E)
        
        self.Q_value = None
        w = numpy.random.random((model.reward_function.dim,))
#        w = numpy.zeros((model.reward_function.dim,))
        
        w_opt = scipy.optimize.fmin_bfgs(self.maxEntObjValue, w, self.maxEntObjGradient, (model, initial, mu_E, len(true_samples[0]), sa_freq), self._epsilon, maxiter=self._max_iter)
             
        print(w_opt)
        model.reward_function.params = w_opt
        agent = self._solver.solve(model)
        
        return (agent, w_opt)

    def maxEntObjValue(self, w, model, initial, mu_E, true_samples_len, sa_freq):
#        Do q value iteration
#        multiply by the observed frequencies of experts state/action pairs
#        sum and return (negated and log'd)

        model.reward_function.params = w
  #      q_iter = QValueIteration(30)
        Q_value = QValueSoftMaxSolve(model, self._qValThresh)
        
        sum = 0
        for (sa, count) in sa_freq.iteritems():
            
            sum += Q_value[sa] * count
            
        self.Q_value = Q_value
        return - sum
        
    def maxEntObjGradient(self, w, model, initial, mu_E, true_samples_len, sa_freq):
        
        if (self.Q_value == None):
            model.reward_function.params = w
            agent = self._solver.solve(model) #shouldn't be doing this!
            
        else:
            policy = {}
            for s in model.S():
                actions = util.classes.NumMap()
                for a in model.A(s):
                    actions[a] = self.Q_value[ (s,a) ]
                policy[s] = actions.argmax()
            agent = mdp.agent.MapAgent(policy)
            
        samples = self.generate_samples(model, agent, initial, true_samples_len)
        _mu = self.feature_expectations(model, samples)
                    
        print(w, mu_E - _mu)
        return -( mu_E - _mu)
    
    def generate_samples(self, model, agent, initial, num_samples):
        '''
        Generate self.n_samples different histories of length t_max by
        following agent.  Each history of the form,
            [ (s_0,a_0), (s_1,a_1), ...]
        '''
        # t_max such that gamma^t_max = 0.01
        t_max = min(num_samples, math.ceil( math.log(0.01)/math.log(model.gamma) ))
        result = []
        for i in range(self._n_samples):
            hist = []
            for (s,a,r) in mdp.simulation.simulate(model, agent, initial, t_max):
                hist.append( (s,a) )
            result.append(hist)
        return result
            
    def feature_expectations(self, model, samples):
        '''
        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        '''
        ff = model.reward_function
        result = numpy.zeros( ff.dim )
#        count = 0
        for sample in samples:
            for (t,sa) in enumerate(sample):
                s = sa[0]
                a = sa[1]
                result += ff.features(s,a)
#                result += ff.features(s,a)
#                count += 1
        return (1.0/(len(samples)))*result
#        return result / float(count)
    
    def feature_expectations2(self, model, samples):
        '''
        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        
        This is for the expert only! we assume each timestep has multiple possible states!
        '''
        
        sa_freq = util.classes.NumMap()
        
        ff = model.reward_function
        result = numpy.zeros( ff.dim )
#        count = 0
        for sample in samples:
            for (t,sal) in enumerate(sample):
                for (s, a, p) in sal:
                    result += p*ff.features(s,a)
                    sa_freq[(s,a)] += p
#                result += ff.features(s,a)
#                count += 1
        return ((1.0/(len(samples)))*result, sa_freq)
        

# Maximum Entropy IRL from Ziebart 2004
class MaxEntIRLApprximateSolverBothPatrollers(object):
    '''Solves the inverse reinforcement learning problem'''
    def __init__(self, max_iter, mdp_solver, n_samples=500, error=.0001, qValThresh=0.01):
        '''
        max_iter: maximum number of times to iterate policies
        mdp_solver: class that implements self.solve(model)
        n_samples: number of samples used to estimate feature expectations
        '''
        self._max_iter = max_iter
        self._solver = mdp_solver
        self._n_samples = n_samples
        self._epsilon = error
        self._qValThresh = qValThresh
        
    def solve(self, model, initial, true_samples, other_policy):
        '''
        Returns a pair (agent, weights) where the agent attempts to generalize
        from the behavior observed in samples and weights is what was combined with
        the MDP to generate agent.
        
        model: an MDP with a linear reward functions.  Parameters WILL be overwritten.
        initial: initial distribution over states
        samples: a list of sample trajectories [ (s_t,a_t) ] of the (supposedly) optimal
            policy.
        '''
        # Initial weight vector
#        w_0 = model.feature_function.params
        self.other_policy = other_policy
        self.full_initial = util.classes.NumMap()
        for s in model.S():
            self.full_initial[s] = 1.0
        self.full_initial = self.full_initial.normalize()
        
        # Compute feature expectations of agent = mu_E from samples
        (mu_E, sa_freq) = self.feature_expectations2(model, true_samples)
        print("True Samples", mu_E)
        
        self.Q_value = None
        w = numpy.random.random((model.reward_function.dim,))
#        w = numpy.zeros((model.reward_function.dim,))
        
        w_opt = scipy.optimize.fmin_bfgs(self.maxEntObjValue, w, self.maxEntObjGradient, (model, initial, mu_E, len(true_samples[0]), sa_freq), self._epsilon, maxiter=self._max_iter)
             
        print(w_opt)
        model.reward_function.params = w_opt
        agent = self._solver.solve(model)
        
        return (agent, w_opt)

    def maxEntObjValue(self, w, model, initial, mu_E, true_samples_len, sa_freq):
#        Do q value iteration
#        multiply by the observed frequencies of experts state/action pairs
#        sum and return (negated and log'd)

        model.reward_function.params = w
  #      q_iter = QValueIteration(30)
        Q_value = QValueSoftMaxSolve(model, self._qValThresh)
        
        sum = 0
        for (sa, count) in sa_freq.iteritems():
            
            sum += Q_value[sa] * count
            
        self.Q_value = Q_value
        return - sum
        
    def maxEntObjGradient(self, w, model, initial, mu_E, true_samples_len, sa_freq):
        
        if (self.Q_value == None):
            model.reward_function.params = w
            agent = self._solver.solve(model) #shouldn't be doing this!
            
        else:
            policy = {}
            for s in model.S():
                actions = util.classes.NumMap()
                for a in model.A(s):
                    actions[a] = self.Q_value[ (s,a) ]
                policy[s] = actions.argmax()
            agent = mdp.agent.MapAgent(policy)
            
        samples = self.generate_samples(model, agent, initial, true_samples_len)
        _mu = self.feature_expectations(model, samples)
                    
        print(w, mu_E - _mu)
        return -( mu_E - _mu)
    
    def generate_samples(self, model, agent, initial, num_samples):
        '''
        Generate self.n_samples different histories of length t_max by
        following agent.  Each history of the form,
            [ (s_0,a_0), (s_1,a_1), ...]
        '''
        # t_max such that gamma^t_max = 0.01
        t_max = min(num_samples, math.ceil( math.log(0.01)/math.log(model.gamma) ))
        result = []
        for i in range(self._n_samples):
            otherHist = []
            for (s,a,r) in mdp.simulation.simulate(model, self.other_policy, self.full_initial, t_max):
                otherHist.append( (s,a) )
            
            hist = []
            for (s,a,r) in mdp.simulation.simulate(model, agent, initial, t_max):
                hist.append( (s,a) )
                
            (hist, otherHist) = self.add_delay(hist, otherHist)

            result.append(hist)
        return result
            
    def feature_expectations(self, model, samples):
        '''
        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        '''
        ff = model.reward_function
        result = numpy.zeros( ff.dim )
#        count = 0
        for sample in samples:
            for (t,sa) in enumerate(sample):
                s = sa[0]
                a = sa[1]
                result += ff.features(s,a)
#                result += ff.features(s,a)
#                count += 1
        return (1.0/(len(samples)))*result
#        return result / float(count)

    def add_delay(self, hist1, hist2):
        newhist1 = []
        newhist2 = []
        for (i, sa) in enumerate(hist1):
            newhist1.append(sa)
            newhist2.append(hist2[i])
            if sa[0] == hist2[i][0]  or (i > 0 and ( all(hist1[i-1][0].location == hist2[i][0].location) or all(sa[0].location == hist2[i-1][0].location) ) ):
                newhist1.append(sa)
                newhist2.append(hist2[i])
                newhist1.append(sa)
                newhist2.append(hist2[i])
                
            
        newhist1 = newhist1[0:len(hist1)]
        newhist2 = newhist2[0:len(hist2)]
        return (newhist1, newhist2)
        
    
    def feature_expectations2(self, model, samples):
        '''
        Compute empirical feature expectations
        E[sum_t gamma^t phi(s_t,a_t)] ~~ (1/m) sum_i sum_t gamma^t phi(s^i_t, a^i_t)
        
        This is for the expert only! we assume each timestep has multiple possible states!
        '''
        
        sa_freq = util.classes.NumMap()
        
        ff = model.reward_function
        result = numpy.zeros( ff.dim )
#        count = 0
        for sample in samples:
            for (t,sal) in enumerate(sample):
                for (s, a, p) in sal:
                    result += p*ff.features(s,a)
                    sa_freq[(s,a)] += p
#                result += ff.features(s,a)
#                count += 1
        return ((1.0/(len(samples)))*result, sa_freq)
        

        
class QValueIteration2:
    '''Use Q-Value Iteration to solve an MDP'''
    def __init__(self, max_iter):
        self._max_iter = max_iter
        
    def solve(self, model):
        '''Returns a map of (state, action) => q-value determined by this solver'''
        Q = util.classes.NumMap()
        for i in range(self._max_iter):
            Q = self.iter(model, Q)
            
                    
        returnQ = util.classes.NumMap()
        
        V = util.classes.NumMap()
        # Compute V(s) = max_{a} Q(s,a)
        for s in model.S():
            V_s = util.classes.NumMap()
            for a in model.A(s):
                V_s[a] = Q[ (s,a) ]
            if len(V_s) > 0:
                V[s] = V_s.max()
            else:
                V[s] = 0.0
        
        for (sa, value) in Q.iteritems():
            returnQ[sa] = value - V[sa[0]]
        
        return returnQ
    
    @classmethod
    def iter(cls, model, Q):
        V = util.classes.NumMap()
        # Compute V(s) = max_{a} Q(s,a)
        for s in model.S():
            V_s = util.classes.NumMap()
            for a in model.A(s):
                V_s[a] = Q[ (s,a) ]
            if len(V_s) > 0:
                V[s] = V_s.max()
            else:
                V[s] = 0.0
        
        # QQ(s,a) = R(s,a) + gamma*sum_{s'} T(s,a,s')*V(s') 
        QQ = util.classes.NumMap()
        for s in model.S():
            for a in model.A(s):
                value = model.R(s,a)
                T = model.T(s,a)
                value += sum( [model.gamma*t*V[s_prime] for (s_prime,t) in  T.items()] )
                QQ[ (s,a) ] = value

        # to find the log policy, find the argmax at each state and then create a new Q with each (s,a) = oldQ - (max for that state)


        return QQ
        

    
def QValueSoftMaxSolve(model, thresh = 1):
    
    v = util.classes.NumMap()
    for s in model.S():
        v[s] = 0.0
        
        
    diff = 100.0
    
    while diff >= thresh:
        vp = v
        
        Q = util.classes.NumMap()
        for s in model.S():
            for a in model.A(s):
                value = model.R(s,a)
                T = model.T(s,a)
                value += sum( [model.gamma*t*v[s_prime] for (s_prime,t) in  T.items()] )
                Q[ (s,a) ] = value            
        
        v = util.classes.NumMap()

        # need the max action for each state!
        for s in model.S():
            maxx = None
            for a in model.A(s):
                if (maxx == None) or Q[(s,a)] > maxx:
                    maxx = Q[(s,a)]


            e_sum = 0
            for a in model.A(s):
                e_sum += math.exp(Q[(s,a)] - maxx)
                
            v[s] = maxx + math.log(e_sum)
        
        diff = max(abs(value - vp[s]) for (s, value) in v.iteritems())
        
        
    logp = util.classes.NumMap()
    for (sa, value) in Q.iteritems():
        logp[sa] = value - v[sa[0]]
    return logp
