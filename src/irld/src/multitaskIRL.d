import mdp;
import irlWithoutLBFGS;
import std.stdio;
import std.format;
import std.string;
import std.math;
import std.random;
import std.algorithm;
import std.algorithm.searching;
import sortingMDP;
import std.math;
import std.numeric;
import core.stdc.stdlib : exit;
import std.datetime;
import std.range;
import boydmdp;

    /*
    As per definitions, 
    - Any DPM model first generates assignment c_m and then trajectory Xm. Therefore, we compute vdm first and then fk(Xm).
    - E[phi] uses vdms over whole trajectory space and fk(Xm)'s for corresponding trajectories. 
    For any DPM, second can't be computed without first.
    - \hat(phi) uses vdms specific to trajectories in demonstration. We need a metric to match fk(Xm). 

    Solve method for easy understanding: 
    Inputs: Model class instaance, initial distribution over states, list of sequences of state-action pairs,
    length of a trajectory, initial weights of all clusters, best likelihood value, learned weights of all clusters,
    
    1 - initialize weights and policy for each cluster (random for batch irl, previous weights for i2rl) 
    2 - initialize assignment distribution as uniform
    3 - Generate trajectories. as the trajectories used for computing E[phi] is different in every iteration, vdms will change in
        each iteration. 
        a - sample an assignment value c_m from assignment distribution and then sample a 
        trajectory from policy of cluster c_m. Compute f_k(X_m) for this trajectory.
        b - For the trajectory, make vdm=1 for cluster indexed d=c_m and vdm=0 for other values of d. 
        c - Keep repeating above two steps until m == n_samples. 
        d - For each trajectory in demonstration, 
        find a f_k(X_m) in traj_space that is close to f_k(X_m) of input trajectory (may be normed diff between two). 
        Assign corresponding vdm of matching trajectory from traj_space to that demonstration trajectory.
    4 - concatenate exp(gradients) values in a matrix form. we can not concatenate gradients directly if we are 
        using exponentiated gradient ascent. 
    5 - implement gradient ascent for thetad's
        a - For each cluster d, compute hatd(phi) using vdms specific to f_k(X_m) in demonstration.
        b - For each cluster d, implement ascent update on theta_d
        c - if diff for D x K weights or cumulative likelihood is not converged, go back to b) 
    5 - update in all vdms corresponding to one d is same (do update within solve())
        a - Compute second term of gradient as input
        a - For each d, compute gradient using non-updated values of vdm
        b - implement ascent/descent on vdm using update equation
        c - if not converged, go back to a)
        d - update vdms_demonstration acc to new vdm_trajs
    6 - update (sum_m vdm)/n_samples and therefore DPM's assignment distribution. If 
    not converged (assignments vary), go to step 3. 

    ...........................................................................

    CHANGE 

    After every policy update, for next cycle, sample new assignment variables and 
    new trajectories simulating trajectory space. Append the demonstration and 
    updated corresponding assignment variables to sample set. Use them to 
    update weights again, and corresponding policy. 

    1 - initialize weights and policy for each cluster (random for batch irl, previous weights for i2rl) 
    2 - initialize assignment distribution as uniform
    3 - sample any $|\Xcal{}|$ v_{d,m}s for demonstration. 
    compute f_k(X_m) of all input trajectories.
    4 - Generate trajectories. as the trajectories used for computing E[phi] is different 
    in every iteration, vdms will change in each iteration. 
        a - sample an assignment value c_m from assignment distribution and then sample a 
        trajectory from policy of cluster c_m. Compute f_k(X_m) for this trajectory.
        b - For the trajectory, make vdm=1 for cluster indexed d=c_m and vdm=0 for other values of d. 
        c - Keep repeating above two steps until m == n_samples. 
        d - append the set of f_k(X_m)s and set of v_{d,m}s to those for demonstration. 
    5 - need (sum vdms) / |\X|. compute that first.  we can not concatenate gradients if 
    we are using exponentiated gradient ascent. we need to concatenate
    exp(gradients) for that. 
        a - For each cluster d, compute hatd(phi) using vdms specific to f_k(X_m) in demonstration.
        b - For each cluster d, implement ascent update on theta_d based on convergence on weights
        d - compute polices for each cluster 
    5 - update all vdms depending on whether or not they are from demonstration. this is where 
    demonstration vdm's are updated in synergy with the change in weights.  

    6 - update mixture proportion of assignment distribution to be (sum_m vdm)/(n_samples+|\Xcal{}|) 
    If not converged (assignments vary), go to step 4. 

    */ 

class NonParamMTIRL : MaxEntIrlZiebartApprox {

    string mapToUse;
    int max_d;
    double [] beta_ds;
    double [][] mu_Eds;
    double [][] theta_ds, Ed_phis; 
    double [][] init_weights_allclusters;
    LinearReward r;
    double Ephi_thresh;
    int descent_duration_thresh_secs; 
    double gradient_descent_step_size;
    int max_restarts;

    public this(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=200, 
        double error=0.01, int descent_duration_thresh_secs=4*60, double gradient_descent_step_size=0.0001,
        double solverError =0.1, int max_d=4, double Ephi_thresh=0.1, int max_restarts=3) {
        super(max_iter, solver, observableStates, n_samples, error, solverError);
        this.max_d = max_d;
        this.Ephi_thresh = Ephi_thresh;
        this.gradient_descent_step_size = gradient_descent_step_size;
        this.descent_duration_thresh_secs = descent_duration_thresh_secs;
        this.max_restarts = max_restarts;
    }

    // additional inputs for online mtirl are: reward weights learned in previous session, 
    // hat-phis for each cluster, total number of trajs assigned to each cluster until now,
    // current demo, 
    // weights of assignment distribution learned in previous session = #assignments[q]/totalAssignmentsExcludingCurrentDemo
    // additional outputs of online mtirl are: reward weights learned in current session, 
    // updated hat-phis for each cluster, total number of trajs assigned to each cluster until now,
    public double solve(string mapToUse, Model model, double[State] initial, sar[][] true_samples,
    size_t sample_length, double [][] init_weights_allclusters, 
    out double [][] opt_weights, int dim, double [][] trueWeights, 
    int[] true_assignments, bool useAdaptive) { 

        this.init_weights_allclusters = init_weights_allclusters;
        this.max_d = cast(int)init_weights_allclusters.length; 
        debug {
            //writeln("reached initialization of vdms");
        }
        // separating vdms because trajspace changes every cycle 
        double [][] vdms_trajspace;// {d x m} 
        vdms_trajspace.length = this.max_d; // D
        foreach (q, arr; vdms_trajspace) {
            vdms_trajspace[q].length = this.n_samples; // |\X|
            vdms_trajspace[q][] = 0; 
        }
        //writeln("model.getReward() - ",model.getReward().classinfo);

        this.model = model;
        this.initial = initial;
        if (mapToUse == "sorting") {
            this.r = cast(LinearReward)this.model.getReward();
            if (this.r is null) {
                writeln("could not fetch reward type for input model");
                return 0;
            }
            //this.r = new sortingReward6(this.model,dim);
            writeln("this.r fetched from model object - ",this.r.classinfo);

        } else {
            this.r = new Boyd2RewardGroupedFeatures(this.model);;
            //this.r = new Boyd2RewardGroupedFeaturesTestMTIRL(this.model);;
        }
        this.model.setReward(this.r);
        // policies for clusters  
        Agent [] policy_ds = new Agent[init_weights_allclusters.length];
        policy_ds.length = this.max_d;
        beta_ds.length = init_weights_allclusters.length;

        //writeln("reached initialization of beta_ds, beta_ds.length: ",beta_ds.length);

        foreach (q; 0 .. this.max_d) {
            //writeln("init_weights cluster ",q," ",(init_weights_allclusters[q]));
            r.setParams(init_weights_allclusters[q]);  
            policy_ds[q] = solver.createPolicy(model, solver.solve(model, solverError)); 
            
            // intialize assignment distribution as uniform
            beta_ds[q] = 1/cast(double)this.max_d; 
        } 
        //writeln("computed policy for all q ");

        sar [][] Xms_trajspace;
        Xms_trajspace.length = this.n_samples;
        double [][] fk_Xms_traj_space;
        fk_Xms_traj_space.length = this.n_samples;

        double [][] fk_Xms_demonstration;
        fk_Xms_demonstration.length = true_samples.length;
        double diff_fk_Xm, last_diff_fk_Xm;

        double [][] vdms_demonstration;// {d times m}
        vdms_demonstration.length = this.max_d; // D
        int [][] vdms_demonstration_bin;// {d times m}
        vdms_demonstration_bin.length = this.max_d; // D
        //writeln("reached initialization of vdms_demonstration");
        foreach (q, arr; vdms_demonstration) {
            vdms_demonstration[q].length = true_samples.length; // |\Xcal|
            vdms_demonstration[q][] = 0; 
            vdms_demonstration_bin[q].length = true_samples.length; // |\Xcal|
            vdms_demonstration_bin[q][] = 0;
        }

        theta_ds.length = this.max_d;
        mu_Eds.length = this.max_d;
        Ed_phis.length = this.max_d;
        //writeln("reached initialization of theta_ds");
        foreach (q, mu_Ed; mu_Eds) {
            mu_Eds[q].length = r.dim();
            //Ed_phis[i].length = r.dim();
            theta_ds[q].length = r.dim();
            foreach (k; 0 .. theta_ds[q].length) {
                theta_ds[q][k] = init_weights_allclusters[q][k];
            }
            writeln("thetad_s[",q,"]: ",theta_ds[q]);
        }

        double beta_ds_error = -double.max;//_normal;
        double temp_error = -double.max;//_normal;
        int [] learned_assignments;
        learned_assignments.length = true_assignments.length;
        foreach (j; 0 .. learned_assignments.length) {
            learned_assignments[j] = -1;
        }
        double avg_EVD = 0.0;

        //, approximate Omega as exp(average_d (estimated Ed[phi]*thetad)) 
        //double log_Omega = 0.0;
        //double[] product;
        //product.length = r.dim();
        //foreach (q; 0 .. this.max_d) {
        //    writeln("Ed_phis[q] ",Ed_phis[q]);
        //    product[] = Ed_phis[q][]*theta_ds[q][];
        //    log_Omega += beta_ds[q]*reduce!("a + b")(0.0, product);
        //    writeln("log_Omega ",log_Omega);
        //    //writeln(fk_Xms_demonstration[j][k]);
        //}
        //Omega = exp(log_Omega);
        //writeln("Omega ",Omega);
        // Omega = sum_q mix-prop sum_i exp(sum_k theta_ds[q][k]*beta_ds[q]*fk_Xms_traj_space[i][k])

        // vdms demo initialized only once
        foreach (i; 0 .. true_samples.length) { 
            //writeln("beta_ds ",beta_ds);
            auto c_m = dice(beta_ds); 
            foreach (q; 0 .. this.max_d) {
                if (q == c_m) vdms_demonstration[q][i] = 1;
                else vdms_demonstration[q][i] = 0;

            }
        }
        fk_Xms_demonstration = calc_feature_expectations_per_trajectory(model, true_samples);

        debug {
            //writeln("fk_Xms_demonstration:",fk_Xms_demonstration);
            //writeln("true_samples.length ",true_samples.length);
            //writeln("fk_Xms_demonstration.length ",fk_Xms_demonstration.length);
            //writeln("true_assignments ",true_assignments);
            //writeln("trueWeights ", trueWeights);
            
            // Used only for debuggin , not used in algorithm
            double [] mu_E_debugSingleTask;
            mu_E_debugSingleTask.length = r.dim();    
            mu_E_debugSingleTask[] = 0;
            foreach(traj_fe; fk_Xms_demonstration) {
                foreach (i; 0 .. mu_E_debugSingleTask.length) {
                    mu_E_debugSingleTask[i] += traj_fe[i]/fk_Xms_demonstration.length;
                }
                //mu_E_debugSingleTask[] += traj_fe[];
            }
            writeln("average feature counts of input ",mu_E_debugSingleTask);
            //exit(0);
        }

        //double desc_step_size = 0.05; 

        // true polices for cpmuting lba
        Agent [] truePolicies;
        truePolicies.length = trueWeights.length;
        int jk = 0;
        foreach (thetaT;trueWeights) {
            r.setParams(trueWeights[jk]);  
            truePolicies[jk] = solver.createPolicy(model, solver.solve(model, solverError));
            jk += 1;
        } 

        // how similar are behaviors?
        writeln("lba between rpp and pip ",this.computeLBA(cast(MapAgent)truePolicies[2],cast(MapAgent)truePolicies[0]));
        writeln("lba between pip and staystill ",this.computeLBA(cast(MapAgent)truePolicies[0],cast(MapAgent)truePolicies[1]));
        //writeln("rpp\n ",(cast(MapAgent)truePolicies[2]).getPolicy(),"\nstayStill",(cast(MapAgent)truePolicies[1]).getPolicy());
        writeln("lba between rpp and staystill ",this.computeLBA(cast(MapAgent)truePolicies[2],cast(MapAgent)truePolicies[1]));
        //exit(0);

        double n_corr_lrndass = 0.0;
        int mapped_lrndass = 0; 

        if (max_iter > 0) { 

            int iterations = 0;
            do {
                // for every upated set of policies, initialize fkXms and vdms for trajspace 
                // 4 - step creating new fk-Xm-s and new v_dms  
                foreach (i; 0 .. this.n_samples) {
                    auto c_m = dice(beta_ds); 
                    Xms_trajspace[i] = simulate(model, policy_ds[c_m], initial, sample_length);

                    foreach (q; 0 .. this.max_d) {
                        if (q == c_m) vdms_trajspace[q][i] = 1;
                        else vdms_trajspace[q][i] = 0;
                    } 
                }
                fk_Xms_traj_space = calc_feature_expectations_per_trajectory(model, Xms_trajspace); 
                //writeln("calc_feature_expectations_per_trajectory fk_Xms_traj_space:"); //,fk_Xms_traj_space);


                // 5 - update theta_ds; need (sum vdms)/|\X| = (sum along rows/ num of columns), for computing Edphi
                // using vdms from preivous iteration
                //writeln("beta_ds after re-computing vdms and fkXms");
                double normalizer_betas = 0.0;
                foreach (q; 0 .. beta_ds.length) {
                    normalizer_betas += (sum(vdms_trajspace[q])+sum(vdms_demonstration[q]));
                }
                foreach (q; 0 .. beta_ds.length) {
                    beta_ds[q] = (sum(vdms_trajspace[q])+sum(vdms_demonstration[q]))/normalizer_betas;
                    ///
                    //(vdms_trajspace[q].length+vdms_demonstration[q].length);
                    //writeln(beta_ds[q]);
                }
                //writeln("computing muE and new weights");
                // Correction: normalization should be done by number of trajectories assigned to cluster
                double divisor = 0.0;
                double grad_val;
                double optLL_val;
                double [] temp_weights_restarts; 
                double [] opt_weights_restarts; 
                int restarts;
                temp_weights_restarts.length = dim;
                opt_weights_restarts.length = dim;
                foreach (q; 0 .. this.max_d) {
                    mu_Eds[q][] = 0;
                    // Compute hat-phi_dk or muE[q][k]
                    foreach (k; 0 .. mu_Eds[q].length) {
                        //writeln(1/cast(double)fk_Xms_demonstration.length);
                        foreach (j; 0 .. fk_Xms_demonstration.length) { 
                            //writeln(cast(double)vdms_demonstration[q][j]);
                            //writeln(fk_Xms_demonstration[j][k]);
                            debug {
                                //writeln("iteration for computing muE ");
                                //writeln((1.0/cast(double)fk_Xms_demonstration.length)*(cast(double)vdms_demonstration[q][j])*cast(double)fk_Xms_demonstration[j][k]);
                            }
                            //mu_Eds[q][k] += (1/cast(double)fk_Xms_demonstration.length)*
                            //(cast(double)vdms_demonstration[q][j])*fk_Xms_demonstration[j][k];
                            mu_Eds[q][k] += 
                            (cast(double)vdms_demonstration[q][j])*fk_Xms_demonstration[j][k];
                        }
                        divisor = 0.0;
                        foreach (j; 0 .. fk_Xms_demonstration.length) { 
                            divisor += (cast(double)vdms_demonstration[q][j]);
                        }
                        if (divisor != 0.0) mu_Eds[q][k] /= divisor;
                    }
                    //writeln("mu_Eds[q] numerators: ",mu_Eds[q]);
                    //writeln("mu_Eds[q] denominator: ",divisor);
                    //writeln("mu_Eds[q]: ",mu_Eds[q]);

                    // trying IRL without restarts + populate corresponding Ed_Phis values
                    //theta_ds[q] = SingleAgentExponentiatedGradient(theta_ds[q].dup, 0.25, error, q);
                    //theta_ds[q] = SingleAgentExponentiatedGradient(theta_ds[q].dup, desc_step_size, error, q);

                    // as algorithm is fast with timed value iteration, try random restarts 
                    optLL_val = -double.max;
                    restarts = 0;
                    do {
                        temp_weights_restarts = theta_ds[q].dup;
                        auto starttime = Clock.currTime();
                        
                        //temp_weights_restarts = SingleAgentExponentiatedGradient(temp_weights_restarts.dup, this.gradient_descent_step_size, 
                        //error, q, this.Ephi_thresh, grad_val, this.descent_duration_thresh_secs);
                        //writeln("restarts ",restarts);
                        temp_weights_restarts = singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent(
                            temp_weights_restarts.dup, 0.01, 
                            error, 25, this.Ephi_thresh, q, useAdaptive, false, 1);
                        //writeln("time for descent ",Clock.currTime()-starttime);
                        restarts += 1;
                        double newQValue = SingleAgentcalcQ(temp_weights_restarts,this.Ephi_thresh);
                        if ((newQValue > optLL_val) ) {
                            optLL_val = newQValue;
                            auto i=0;
                            foreach(j, ref o2; opt_weights_restarts) {
                                o2 = temp_weights_restarts[i++];         
                            }
                            debug {
                                //writeln("Q(", restarts, ") = ", newQValue, " for weights: ", theta_ds[q]);                   
                            } 
                        } 
                    } while(restarts < this.max_restarts); 
                    theta_ds[q] = opt_weights_restarts.dup;

                    writeln("theta_ds[",q,"]: ",theta_ds[q]); 

                    r.setParams(theta_ds[q]);  
                    policy_ds[q] = solver.createPolicy(model, solver.solve(model, solverError)); 
                } 
                
                // 6 : Update vdms
                double common_step, step, Omega;
                Omega = 0;
                foreach (q; 0 .. this.max_d) {
                    double sum_exp = 0;
                    foreach (i; 0 .. fk_Xms_traj_space.length) {
                        double  sum_k = 0;
                        foreach (k; 0 .. theta_ds[q].length ){
                            sum_k += theta_ds[q][k]*beta_ds[q]*fk_Xms_traj_space[i][k];
                        } 
                        sum_exp += exp(sum_k);
                    }
                    Omega += sum_exp*beta_ds[q];
                } 
                //writeln("Omega ",Omega);

                double log_Omega = log(Omega);
                double buffer_division = 0.001;
                double learning_rate = 0.1;// 0.2
                double covergence_thresh = 0.1;// 0.0005
                foreach (q; 0 .. this.max_d) {
                    double sum_Pyicid = 0;
                    foreach (i; 0 .. fk_Xms_traj_space.length) {
                        double sum = 0;
                        foreach (k; 0 .. theta_ds[q].length ){
                            sum += theta_ds[q][k]*beta_ds[q]*fk_Xms_traj_space[i][k];
                        } 
                        //writeln("sum ",sum);
                        sum_Pyicid += exp(sum)/Omega;
                    }
                    //writeln("sum_Pyicid ",sum_Pyicid);

                    foreach (j; 0 .. vdms_demonstration[q].length) {
                        // compute gradient step
                        do {
                            common_step = (1/(sum(vdms_demonstration[q])+buffer_division))*(sum_Pyicid+1);
                            //writeln("common step ",common_step);
                            step = common_step -(1/(vdms_demonstration[q][j]+buffer_division))
                            *(fk_Xms_demonstration.length/(sum(vdms_demonstration[q])+buffer_division))*(1-log_Omega);
                            //writeln("step for vdms_demonstration ",step);
                            vdms_demonstration[q][j] = vdms_demonstration[q][j] + learning_rate*step;
                        } while (abs(step) > covergence_thresh);
                    } 
                    //writeln("descent on vdms_demonstration converged for q ",q);
                    foreach (j; 0 .. vdms_trajspace[q].length) {
                        do {
                            common_step = (1/(sum(vdms_demonstration[q])+buffer_division))*(sum_Pyicid+1);
                            step = common_step -(1/(vdms_trajspace[q][j]+buffer_division))
                            *(fk_Xms_demonstration.length/(sum(vdms_demonstration[q])+buffer_division))*(1-log_Omega);
                            //writeln("step for vdms_trajspace ",step);
                            vdms_trajspace[q][j] = vdms_trajspace[q][j] + learning_rate*step;
                        } while (abs(step) > covergence_thresh);
                    }
                    //writeln("descent on vdms_trajspace converged for q ",q);
                }

                foreach (j; 0 .. fk_Xms_demonstration.length) {
                    //convert them to corresponding binary values
                    //and normalize for bounding between [0,1] for contributing to betads to further calculations.
                    double highest_vdm = -double.max;
                    int chosen_cluster;
                    double sumoverclusters = 0;
                    foreach (q; 0 .. this.max_d) {
                        sumoverclusters += vdms_demonstration[q][j];
                        if (vdms_demonstration[q][j] > highest_vdm) {
                            highest_vdm = vdms_demonstration[q][j];
                            chosen_cluster = q;
                        }
                    }

                    foreach (q; 0 .. this.max_d) {
                        vdms_demonstration[q][j] = vdms_demonstration[q][j]/sumoverclusters;
                        if (q == chosen_cluster) vdms_demonstration_bin[q][j] = 1;
                        else vdms_demonstration_bin[q][j] = 0;
                    }
                } 
                //writeln("updated vdms demo  "); //vdms_demonstration_bin);
                
                // updated betads
                normalizer_betas = 0.0;
                foreach (q; 0 .. beta_ds.length) {
                  normalizer_betas += (sum(vdms_trajspace[q])+sum(vdms_demonstration[q]));
                }

                foreach (q; 0 .. this.max_d) {// Is assignment distribution changing?

                    temp_error = abs(beta_ds[q] - (sum(vdms_trajspace[q])+sum(vdms_demonstration[q]))/
                    normalizer_betas);

                    if (temp_error > beta_ds_error) beta_ds_error = temp_error;

                    beta_ds[q] = (sum(vdms_trajspace[q])+sum(vdms_demonstration[q]))/
                    normalizer_betas;
                }

                //foreach (j; 0 .. fk_Xms_traj_space.length) {
                //    double highest_vdm = -double.max;
                //    int chosen_cluster;
                //    foreach (q; 0 .. this.max_d) {
                //        if (vdms_trajspace[q][j] > highest_vdm) {
                //            highest_vdm = vdms_trajspace[q][j];
                //            chosen_cluster = q;
                //        }
                //    }

                //    foreach (q; 0 .. this.max_d) {
                //        if (q == chosen_cluster) vdms_trajspace[q][j] = 1;
                //        else vdms_trajspace[q][j] = 0;
                //    }
                //}
                //writeln("updated vdms trajs for ",vdms_trajspace);

                //writeln("beta_ds ",beta_ds);
                //writeln("for current iteration, %change in assignment distribution (beta_ds_error): ",beta_ds_error);
                iterations += 1;
                writeln ("iterations - ",iterations);

                //Compute average EVD
                double trajval_trueweight, trajval_learnedweight;
                avg_EVD = 0.0;
                //writeln("fk_Xms_demonstration.length ",fk_Xms_demonstration.length);
                foreach (j; 0 .. fk_Xms_demonstration.length) {
                    //wrt trueWeights
                    int chosen_cluster = -1;
                    //writeln("chosen_cluster ");
                    foreach (q; 0 .. this.max_d) {
                        if (vdms_demonstration_bin[q][j] == 1) chosen_cluster=q;
                    }
                    learned_assignments[j] = chosen_cluster;
                    trajval_learnedweight = dotProduct(theta_ds[chosen_cluster],fk_Xms_demonstration[j]);
                    //writeln("j ",j);
                    //writeln("true_assignments[j] ",true_assignments[j]);
                    //writeln("length trueWeights[true_assignments[j]] ",trueWeights[true_assignments[j]].length);
                    trajval_trueweight = dotProduct(trueWeights[true_assignments[j]],fk_Xms_demonstration[j]);
                    //writeln("value of traj w.r.t true weight ",trajval_trueweight);
                    
                    //if (trajval_trueweight != 0) {
                    //    avg_EVD += abs(trajval_trueweight - trajval_learnedweight)/(trajval_trueweight*cast(double)fk_Xms_demonstration.length);
                    //} else {
                    //    avg_EVD += abs(trajval_trueweight - trajval_learnedweight)/(cast(double)fk_Xms_demonstration.length);
                    //}

                    // DPM-BIRL doesn't compute it as a percentage, it is just a direct difference in state-value matrices
                    avg_EVD += abs(trajval_trueweight - trajval_learnedweight)/(cast(double)fk_Xms_demonstration.length);
                }
                writeln("learned_assignments ",learned_assignments);

                // for each cluster, compute LBA w.r.t true policies 
                double [] lbavals;
                double lba_max;
                // mapping indices of learned clusters to original indices 
                int [int] mapping_learned2original;
                //mapping_learned2original.length = max_d; 
                foreach (q; 0 .. this.max_d) {
                    lbavals.length = 0;
                    //writeln("lba values for learned cluster indexed ",q," w.r.t. policies for p-i-p, r-p-p, keep-claiming");
                    foreach (truepol;truePolicies) {
                        lbavals ~= this.computeLBA(cast(MapAgent)policy_ds[q],cast(MapAgent)truepol);
                    }

                    //writeln(lbavals);
                    //writeln(q);
                    
                    // base case: 2 clusters
                    // pick the indices of maximum lba for pip if that is higher rpp, else pick rpp
                    //if (lbavals[0] > lbavals[2]) mapping_learned2original[q] = 0;
                    //else mapping_learned2original[q] = 2;

                    // case of multiple clusters
                    int i = 0;
                    lba_max = -double.max;
                    foreach(lba; lbavals) {

                        if (lba > lba_max) {
                            mapping_learned2original[q] = i;
                            lba_max = lba;
                        }
                        i += 1;
                    }

                    //mapping_learned2original[q] = lbavals.maxIndex;
                    //exit(0);
                    //writeln("mapping_learned2original ",mapping_learned2original);
                }
                //writeln("mapping_learned2original ",mapping_learned2original);
                n_corr_lrndass = 0.0;
                mapped_lrndass = 0; 
                foreach (ci; 0 .. learned_assignments.length) {
                    //writeln("ci ",ci);
                    mapped_lrndass = mapping_learned2original[learned_assignments[cast(int)ci]];
                    //writeln("ci ",ci," mapping_learned2original[learned_assignments[cast(int)ci]] ",mapped_lrndass);
                    if (mapped_lrndass == true_assignments[cast(int)ci]) n_corr_lrndass += 1;
                }
                //writeln("n_corr_lrndass ",n_corr_lrndass," learned_assignments.length ",learned_assignments.length);
                //writeln("ass-accuracy ",iterations,":",cast(double)n_corr_lrndass/cast(double)learned_assignments.length);

                writeln("avg_EVD",iterations,"_",learned_assignments.length);
                writeln(avg_EVD,"endavg_EVD",iterations,"_",learned_assignments.length,"\n");

                // approx likelihood = (theta dot-product feature-expectations) - log(partition function)
                // sum over approximations of likelihoods w.r.t. true trajectory distributions of experts
                double LL_t=0.0;
                // sum over approximations of likelihoods w.r.t. estimated trajectory distributions of experts
                double LL_hat=0.0;
                foreach (q; 0 .. this.max_d) {
                    double sum_k1 = 0.0;
                    double sum_k2 = 0.0;
                    foreach (k; 0 .. theta_ds[q].length ){
                        sum_k1 += theta_ds[q][k]*Ed_phis[q][k];
                        sum_k2 += theta_ds[q][k]*mu_Eds[q][k];
                    }
                    LL_t += sum_k1;
                    LL_hat += sum_k2;
                }    
                LL_t -= log_Omega;
                LL_hat -= log_Omega;

                // compute log likelihoods using calc LLH maxent method in BIRL repo
                double LL_hat2_assignDistrPrior = 0.0;
                double LL_hat2_LLH = 0.0;
                // for each cluster, for each sa in traj assigned to that cluster
                // numerator(q,l_as,s,a) = visitation-freq for (s,a)*exp(thetads[q]*phi(s,a)) =
                // sum_{traj assigned to q} sum_(s,a) exp(thetads[q]*phi(s,a))
                // denominator (q) = exp(thetads[q]*phi(s,a) for all (s,a)* expected discounted visitation-freq for all (s,a))
                // = sum_{trajs in mdp} sum_(s,a while following policy_ds[q]) exp(thetads[q]*phi(s,a))  
                // approx exp(thetads[q]* Ed_phis[q])
                // LLH is sum over log (numerator(q,l_as,s,a)/ denominator(q)) over all clusters
                // instead of doing it over clusters, we do it over trajectories 
                // as dom_birl code add logClusterAssPDistr to LLH, we need ot directly multple beta value
                double [] omegas_ds;
                omegas_ds.length = this.max_d;
                foreach (q; 0 .. this.max_d) {
                    double sum_k1 = 0.0;
                    foreach (k; 0 .. theta_ds[q].length ){
                        sum_k1 += exp(theta_ds[q][k]*Ed_phis[q][k]);
                    }
                    omegas_ds[q] = sum_k1;
                }    
                foreach(i; 0 .. learned_assignments.length) {
                    int l_as = learned_assignments[i];
                    foreach (sar; true_samples[i]){
                        if (sar.s in (cast(MapAgent)policy_ds[l_as]).getPolicy()){
                            Action act = (cast(MapAgent)policy_ds[l_as]).sample(sar.s);
                            if (act == sar.a) {
                                double sum_num = 0.0;
                                foreach (k; 0 .. theta_ds[l_as].length ){
                                    sum_num += theta_ds[l_as][k]*this.r.features(sar.s,sar.a)[k];
                                }
                                LL_hat2_LLH += log( beta_ds[l_as]* exp(sum_num)/omegas_ds[l_as] );
                            }
                        }
                    }
                }
                double LL_hat2 = LL_hat2_assignDistrPrior+LL_hat2_LLH;

                //writeln("LL_hat:",LL_hat);
                writeln("LL_hat2:",LL_hat2);
            } while (iterations < max_iter); //(beta_ds_error > 0.0001 && iterations < 10);
        }


        writeln("assignmentAccuracy","_",learned_assignments.length);
        if (n_corr_lrndass == 0) {
            writeln("0.0endassignmentAccuracy","_",learned_assignments.length);
        } else {
            writeln(n_corr_lrndass/cast(double)learned_assignments.length,
                "endassignmentAccuracy","_",learned_assignments.length);
        }

        avg_EVD = cast(double)avg_EVD/(cast(double)fk_Xms_demonstration.length);

        writeln("finalavgEVD","_",learned_assignments.length);
        writeln(avg_EVD,"endfinalavgEVD","_",learned_assignments.length,"\n");
        //writeln("learned_assignments ",learned_assignments);

        //writeln("learned assignment dsitribution mixing proportions"); 
        foreach (q; 0 .. this.max_d) {
            //writeln(beta_ds[q]);
            ;
        }

        opt_weights.length = theta_ds.length;
        foreach(q, ref o; opt_weights) {
            o.length = theta_ds[q].length;
        }

        foreach(q, ref o; opt_weights) {
            auto i = 0;
            //writeln("learned theta_ds cluster ",q," ",theta_ds[q]);
            foreach(ref o2; o) {
                o2 = theta_ds[q][i++];
            }
            //writeln("opt_weights array cluster ",q," ",o);
        } 

        return avg_EVD;

    }
    
    double [] SingleAgentExponentiatedGradient(double [] w, double c, double err, int q,
        double Ephi_thresh, out double gradient_val, int dur_thresh_secs) {

        debug {
            //writeln("max_sample_length: ", max_sample_length);
        }
        
        double [] y = mu_Eds[q].dup;
        
        //y[] *= (1-this.model.gamma);

        //foreach(ref t; w)
        //    t = abs(t);

        // theta_d
        w[] /= l1norm(w);

        debug {
            //writeln("initial weights: ", w);
            //writeln("Y : ", y);
        }
                
        double diff = double.max;
        double lastdiff = double.max;
        double [] lastGradient = y.dup;
        double [] y_prime;
        double [] gradient;
        
        debug {
            //writeln("do while started ");
        }

        auto stattime = Clock.currTime();
        auto endttime = Clock.currTime();
        auto duration = dur!"seconds"(1);
        //writeln("dur! seconds (dur_thresh_secs) ",dur!"seconds"(dur_thresh_secs));

        do {
            
            y_prime = SingleAgentExpectedEdgeFrequencyFeatures(w,this.Ephi_thresh);

            // update Edphi for computing Omega needed for updating vdms
            this.Ed_phis[q] = y_prime.dup;

            //y_prime[] *= (1-this.model.gamma);

            // Ed_phi = mix-proportion * (sampling based estimate of E[phi] by using thetad) 
            //foreach (k, fc; y_prime)
            //  y_prime[k] = beta_ds[q]*y_prime[k];

            debug {
                //writeln("y_prime",y_prime);
            }

            
            debug {
                auto temp = y_prime.dup;
                temp[] -= y[];
                //writeln("GRADIENT: ", temp);
            }

            double [] next_w = new double[w.length]; 
            foreach(i; 0..w.length) {
                next_w[i] = w[i] * exp(-2*c*(y_prime[i] - y[i]) );
            }
            
            double norm = l1norm(next_w);
            if (norm != 0)
                next_w[] /= norm;
            
            double [] test = w.dup;
            test[] -= next_w[];
            diff = l2norm(test);

            gradient = y_prime.dup;
            gradient[] -= y[];
            /*double [] test = gradient.dup;
            test[] -= lastGradient[];
            diff = l2norm(test);*/
            
            w = next_w;
            //c /= 1.025;

            if (diff > lastdiff) {
                c /= 1.05;
            } else {
                c *= 1.025;
            }   

            lastdiff = diff;    
            lastGradient = gradient;
            
            endttime = Clock.currTime();
            duration = endttime - stattime;

            debug {
                //writeln("norm of GRADIENT vector: ", l1norm(gradient));
                //writeln("diff:",diff," err:",err," duration:",duration);
                //writeln("diff > err: ",(diff > err));
            }
            debug {
                //writeln(" (duration < dur (dur_thresh_secs)) : ",duration < dur!"seconds"(dur_thresh_secs));
            }

        } while (diff > err && duration < dur!"seconds"(dur_thresh_secs));

        debug {
            //writeln("descent converged");
            //writeln("diff < thresh: ",(diff < err));
            ////writeln("E[phi] / mu for learned weights ", y_prime);
        }

        gradient_val = l1norm(gradient);
        return w;
    }

    double [] singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent(double [] w,
        double nu, double err, size_t max_iter, double Ephi_thresh, int q, bool useAdaptive,  
        bool usePathLengthBounds = true, size_t moving_average_length = 5) {
          
        usePathLengthBounds = false;
        nu = 0.0075;
        //nu = 0.1;
        double diff;
        double lastdiff = double.max;
        err = 1;
        moving_average_length = 3;
        
        double [] expert_features = mu_Eds[q].dup;
        double [] beta = new double[expert_features.length];
        beta[] = - log(beta.length );
        
        double [] z_prev = new double [beta.length];
        z_prev[] = 0;
        double [] w_prev = new double [beta.length];
        w_prev[] = w[];

        size_t t = 0;
        size_t iterations = 0;
        double[][] moving_average_data;
        size_t moving_average_counter = 0;
        double [] err_moving_averages = new double[moving_average_length];
        foreach (ref e ; err_moving_averages) {
           e = double.max;
        }
        double err_diff = double.infinity;

        //writeln("starting singleTaskUnconstrainedAdaptiveExponentiatedStochasticGradientDescent");
        auto stattime = Clock.currTime();
        auto endttime = Clock.currTime();
        auto duration = dur!"seconds"(1);

        //while (iterations < max_iter && (err_diff > err || iterations < moving_average_length)) {
        while ((err_diff > err )) {
        //while ((err_diff > err ) && duration < dur!"seconds"(dur_thresh_secs)) {

            double [] m_t = z_prev.dup;

            if (! usePathLengthBounds && iterations > 0)
                m_t[] /= iterations;

            double [] weights = new double[beta.length];
            foreach (i ; 0 .. (beta.length)) {
                weights[i] = exp(beta[i] - nu*m_t[i]);
            }

            debug {

                //writeln("likelihood:",SingleAgentcalcQ(weights,Ephi_thresh));
            }


            double [] actual_weights = weights.dup;

            double [] z_t = SingleAgentExpectedEdgeFrequencyFeatures(actual_weights,Ephi_thresh);
            this.Ed_phis[q] = z_t.dup;

            z_t[] -= expert_features[];

            diff = l1norm(z_t);

            debug {
                //writeln("learned weights ",actual_weights);
                //writeln(" (lhs - rhs) of constraint: ", (z_t));
                //writeln("normed (lhs - rhs) of constraint: ", l1norm(z_t));
            }
                
            if (usePathLengthBounds) {
                z_prev = z_t;
            } else {
                z_prev[] += z_t[];
            }

            foreach(i; 0..(beta.length)) {
                beta[i] = beta[i] - nu*z_t[i] - nu*nu*(z_t[i] - m_t[i])*(z_t[i] - m_t[i]);
            }

            err_moving_averages[moving_average_counter] = diff;
            //writeln("err_moving_averages\n ",err_moving_averages);
            moving_average_counter ++;
            
            t ++;
            iterations ++;
            moving_average_counter %= moving_average_length;

            double sum = 0;
            foreach (entry; err_moving_averages) {
                sum += abs(entry);
            }
            sum /= err_moving_averages.length;
            err_diff = sum;

            err_diff = this.stddev(err_moving_averages);

            //writeln("err_diff ",err_diff);
            if (moving_average_counter == 0 && iterations > 0 && useAdaptive) {
                if (err_diff > lastdiff) {
                    nu /= 1.05;
                } else {
                    nu *= 1.05;
                } 
            }  

            w_prev = actual_weights;
            lastdiff = err_diff;


            endttime = Clock.currTime();
            duration = endttime - stattime;

        } 
        
        //writeln("iterations >= max_iter ",(iterations >= max_iter)," err_diff < err ",
        //    (err_diff < err)," iterations > moving_average_length ",(iterations > moving_average_length));
        
        return w_prev;
    }

    double [] SingleAgentExpectedEdgeFrequencyFeatures(double [] w, double threshold) {
        // approximate feature expectations to avoid computing partition function
        debug {
            //writeln("SingleAgentExpectedEdgeFrequencyFeatures started");
        }

        this.r.setParams(w);     

        Agent policy = this.solver.createPolicy(this.model, this.solver.solve(this.model, this.solverError));
        debug {
            //writeln("computed policy");
        }

        //Agent [] policies = getPoliciesFor(w);
        double [] returnval = new double[w.length];
        returnval[] = 0;
        debug {
            //writeln("SingleAgentExpectedEdgeFrequencyFeatures");
        }

        //double threshold = 0.5;//for patrolling task
        //threshold = 0.5;//sorting task both behaviors
        //threshold = 0.05;
        //threshold = 2.0;//for learning weights for individual sorting behaviors
        if (this.n_samples > 0) { 

               double [] total = returnval.dup;
               double [] last_avg = returnval.dup;
               size_t repeats = 0;
               while(true) {

                    debug {
                        //writeln("SingleAgentExpectedEdgeFrequencyFeatures started sampling traj space");
                    }
                    sar [][] samples = generate_samples(this.model, policy, this.initial, 
                        this.n_samples, this.sample_length);
                    debug {
                        //writeln("sampled traj space");
                    }

                    // cumulative feature count for samples
                    total[] += feature_expectations(this.model, samples)[];
                    debug {
                        //writeln("computed FEs");
                    }

                    repeats ++;

                    double [] new_avg = total.dup;
                    // running average
                    new_avg[] /= cast(double)repeats;

                    double max_diff = -double.max;

                    foreach(i, k; new_avg) {
                        // change in running average
                        auto tempdiff = abs(k - last_avg[i]);
                        if (tempdiff > max_diff) {
                            max_diff = tempdiff;

                        }
                    } 
                    debug {
                        //writeln("E[phi]/ mu max_diff for ", repeats," repeat is ", max_diff);
                    }

                    if (max_diff < threshold) {
                        debug {
                            //writeln("mu Converged after ", repeats, " repeats, ", n_samples * repeats, " simulations");
                            //writeln("mu Converged after ", repeats, " repeats, ", n_samples * repeats, " simulations");
                        }
                        break;
                    }         
                       
                    last_avg = new_avg;             
                       
               }
               // total/repeats , not last_avg/repeats
               returnval[] = total[] / repeats;
               //writeln("Edphi[q] running avg FE/n_samples ",returnval);
               
        } else {
//             auto Ds = calcStateFreq(policy, initial, model, sample_length);
               throw new Exception("Not Supported with n_samples < 0");
                       
        }
         
        return returnval;
    }

    override double[][] calc_feature_expectations_per_trajectory(Model model, sar[][] trajs) {
        
        double [][] returnval;
        LinearReward ff = cast(LinearReward)model.getReward();
        
        this.sample_length = 0;
        
        foreach (traj; trajs) {
            
            double [] temp_fe = new double[ff.dim()];
            temp_fe[] = 0;
            foreach(i, SAR; traj) {
                if (SAR.s ! is null)
                    temp_fe[] += ff.features(SAR.s, SAR.a)[];// * (1/cast(double)traj.length);
                if (i > this.sample_length)
                    this.sample_length = i;
            }
            
            returnval ~= temp_fe;
        }
        
        return returnval;
    } 

    double computeLBA(MapAgent agent1, MapAgent agent2) {
        double totalsuccess = 0.0;
        double totalstates = 0.0;
        // Mapagent is a deterministic policy by definition
        foreach (s; this.model.S()) {
            if (s in agent1.getPolicy()) {
                // check key existence 
                //writeln("number of actions in current state in learned policy",(agent1.policy[s]))

                Action action = agent1.sample(s);

                if (s in agent2.getPolicy()) {
                    totalstates += 1;
                    if (agent2.sample(s) == action) {
                        //writeln("found a matching action");
                        totalsuccess += 1;
                    } //else writeln("for state {},  action {} neq action {} ".format(ss2,action,truePol[ss2]));                    
                } else writeln("state ",s," not found in agent2");
            } else writeln("state ",s," not found in agent1");
        }

        //print("totalstates, totalsuccess: "+str(totalstates)+", "+str(totalsuccess))
        if (totalstates == 0.0) {
            writeln("Error: states in two policies are different");
            return 0;
        }
        double lba = (totalsuccess) / (totalstates);
        return lba;
    }

}   


int main() {
	
	// Read in stdin to get settings and trajectory
    //writeln("entered main");
    auto stattime = Clock.currTime();

	sar [][] SAR;
	string buf;
    string buf2, st;
	string algorithm; 
    int max_d;
    int trajCounter;
    int num_trueWeights;
    int max_iterations; // 7
    ////////////////////////////////////////////////////////////////////////////////////
    // Threshold on gradient descent of individual cluster decides the speed of convergence
    ////////////////////////////////////////////////////////////////////////////////////
    double descent_error; // 0.0001; //0.00005; // 0.00001; 
    int descent_duration_thresh_secs; // 3*60;
    double gradient_descent_step_size; // 0.0001
    double VI_threshold; // 0.4 // 0.2
    int vi_duration_thresh_secs; // 45; //30;
    double Ephi_thresh; // 0.1
    int max_restarts_descent; // 4
    bool useAdaptive;

    string mapToUse;
    buf = readln();
    formattedRead(buf, "%s", &mapToUse);
    mapToUse = strip(mapToUse);

	buf = readln();
	formattedRead(buf, "%s", &algorithm);
	algorithm = strip(algorithm); 

    buf = readln();
    formattedRead(buf, "%s", &max_d);
    //max_d = int(strip(st_max_d)); 

    buf = readln();
    formattedRead(buf, "%s", &max_iterations);
    buf = readln();
    formattedRead(buf, "%s", &descent_error);
    buf = readln();
    formattedRead(buf, "%s", &descent_duration_thresh_secs);
    buf = readln();
    formattedRead(buf, "%s", &gradient_descent_step_size);
    buf = readln();
    formattedRead(buf, "%s", &max_restarts_descent);
    buf = readln();
    formattedRead(buf, "%s", &VI_threshold);
    buf = readln();
    formattedRead(buf, "%s", &vi_duration_thresh_secs);
    buf = readln();
    formattedRead(buf, "%s", &Ephi_thresh);
    buf = readln();
    formattedRead(buf, "%s", &useAdaptive);
    writeln("\nuseAdaptive ",useAdaptive,"\n");

	SAR.length = 0; 
    trajCounter = 0;
    sar [] newtraj;
    while ((buf = readln()) != null) {
    	buf = strip(buf);
        //writeln("buf:",buf);
        
        if (buf == "ENDDEMO") {
            break;
    	} else {

            if (buf == "ENDTRAJ") {

                //writeln("appending ",newtraj);
                SAR ~= newtraj;         
                newtraj.length = 0;

                trajCounter++;                
                //SAR.length = SAR.length + 1;

                //writeln("trajCounter ",trajCounter);
        		
        		continue;
        	} else {

                //while (buf.countUntil(";") >= 0) {
                string percept = buf[0..buf.countUntil(";")];
                //writeln("percept:",percept);

                buf = buf[buf.countUntil(";") + 1 .. buf.length];
                
                string state;
                string action;
                double p;

                formattedRead(percept, "%s:%s:%s", &state, &action, &p);
                
                if (mapToUse == "sorting") {
                    ;

                    int ol;
                    int pr;
                    int el;
                    int ls;

                    state = state[1..state.length];
                    //writeln("state string:",state);
                    formattedRead(state, " %s, %s, %s, %s]", &ol, &pr, &el, &ls);
                    //writeln("state components: (",ol, pr, el, ls,")");

                    Action a;
                    if (action == "InspectAfterPicking") {
                        a = new InspectAfterPicking();
                    } else if (action == "InspectWithoutPicking" ) {
                        a = new InspectWithoutPicking();
                    } else if (action == "Pick" ) {
                        a = new Pick();
                    } else if (action == "PlaceOnConveyor" ) {
                        a = new PlaceOnConveyor();
                    } else if (action == "PlaceInBin" ) {
                        a = new PlaceInBin();
                    } else if (action == "ClaimNewOnion" ) {
                        a = new ClaimNewOnion();
                    } else {
                        a = new ClaimNextInList();
                    }
                    
                    newtraj ~= sar(new sortingState([ol, pr, el, ls]),a,p);

                    //writeln("newtraj ",newtraj);

                    debug {
                        //writeln("finished reading ",[ol, pr, el, ls],action);
                    }
                    //}

                } else {// boyd patrolling trajectory

                    int x;
                    int y;
                    int z;

                    state = state[1..state.length];
                    formattedRead(state, "%s, %s, %s]", &x, &y, &z);

                    Action a;
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

                    newtraj ~= sar(new BoydState([x, y, z]), a, p);
                    debug {
                        //writeln("finished reading ",[ol, pr, el, ls],action);
                    }


                }

        	}
        }    	
    }
	//SAR.length = SAR.length - 1;

    //writeln("Input trajs count ",SAR.length);

    double[State][Action][State] T;
    if (mapToUse == "boyd2") {
        while ((buf = readln()) != null) {
            buf = strip(buf);
            
            if (buf == "ENDT") {
                break;
            }
            
            State s;
            Action a;
            State s_prime;
            double p;

            p = parse_transitions(mapToUse, buf, s, a, s_prime);
            debug {
                //writeln(" transition ",s, a, s_prime,p,". "); 
            }

            T[s][a][s_prime] = p;
            
        }
    }

    int dim;
    Model model;
    LinearReward reward;
    if (mapToUse == "sorting") {
        
        //model = new sortingModel(0.05,null);
        //model = new sortingModelbyPSuresh(0.05,null);
        //model = new sortingModelbyPSuresh2(0.05,null);
        //model = new sortingModelbyPSuresh3(0.05,null);
        model = new sortingModelbyPSuresh2WOPlaced(0.05,null);
        //model = new sortingModelbyPSuresh3multipleInit(0.05,null);
        
        // dim = 8;
        // reward = new sortingReward2(model,dim);
        //dim = 10;
        //reward = new sortingReward4(model, dim); 
        dim = 11;
        //reward = new sortingReward6(model,dim); 
        //reward = new sortingReward7WPlaced(model,dim); 
        reward = new sortingReward7(model,dim); 

    } else {
        byte[][] map;

        map = boyd2PatrollerMap();
        model = new BoydModel(null, map, T, 1, &simplefeatures);
        dim = 6;
        reward = new Boyd2RewardGroupedFeatures(model);
        //reward = new Boyd2RewardGroupedFeaturesTestMTIRL(model);
    }

    model.setReward(reward);
    //writeln("model.getReward(); ",model.getReward());
    model.setGamma(0.99);

    //read actual assignments and weights for computing ILE/EVD_traj
    int [] true_assignments;
    true_assignments.length = SAR.length;

    buf = readln();
    //writeln("buf true_assignments. ",buf);
    formattedRead(buf, "[%s]", &buf2);
    for (int j = 0; j < true_assignments.length-1; j++) {
        formattedRead(buf2,"%s, ",&true_assignments[j]);
    }
    formattedRead(buf2,"%s",&true_assignments[true_assignments.length-1]);
    writeln("true_assignments ",true_assignments);

    // NO NEED 
    // assignment indices shifted back 
    // true_assignments = [1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2];
    // foreach (ref ass; true_assignments) ass = ass-1;
    //writeln("true_assignments ",true_assignments);
    
    buf = readln();
    formattedRead(buf, "%s", &num_trueWeights);
    writeln("num_trueWeights ",num_trueWeights);
    
    double [][] trueWeights;
    trueWeights.length = num_trueWeights;

    for (int i = 0; i < num_trueWeights; i++) {
        trueWeights[i].length = reward.dim();
        trueWeights[i][] = 0.0;
    }

    for (int i = 0; i < num_trueWeights; i++) {
        buf2 = readln();
        formattedRead(buf2,"[%s]",&st);
        for (int j = 0; j < reward.dim()-1; j++) {
            formattedRead(st,"%s, ",&trueWeights[i][j]);
        }
        formattedRead(st,"%s",&trueWeights[i][reward.dim()-1]);
        //writeln("trueWeights[i] ",trueWeights[i]);
    } 
    //writeln("trueWeights ", trueWeights);

    //// read last weight vector 
    //formattedRead(buf2,"[%s]",&st);
    //for (int j = 0; j < reward.dim()-1; j++) {
    //    formattedRead(st,"%s, ",&trueWeights[num_trueWeights-1][j]);
    //}
    //formattedRead(st,"%s",&trueWeights[num_trueWeights-1][reward.dim()-1]);

    //trueWeights[0] = [ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12]; 
    //trueWeights[1] = [ 0.0, 0.10, 0.22, 0.0, -0.12, 0.44, 0.0, -0.12]; 
    //trueWeights[2] = [ 0.13509862199405565, -0.067549310997027823, 
    //-0.10132396649554175, 0.17562820859227235, -0.027019724398811135, 0.0, 0.3582815455282356, -0.13509862199405565];

    //string [] st;
    //buf = readln();
    ////writeln("read buf");
    ////writeln(buf);
    //// change this acc to number of rewards used in demo
    //st.length = 2;
    //formattedRead(buf, "[[%s], [%s]]", &st[0], &st[1]);
    ////writeln("read st");
    ////change this acc to choice of reward model
    //for (int i = 0; i < 2; i++) {
    //    for (int j = 0; j < reward.dim()-1; j++) {
    //        formattedRead(st[i],"%s, ",&trueWeights[i][j]);
    //    }
    //    formattedRead(st[i],"%s",&trueWeights[i][reward.dim()-1]);
    //}

    //writeln(trueWeights);

	double[State] initial;
    if (mapToUse == "sorting") {
        // ALWAYS START FROm  0,2,0,2
        //sortingState iss;
        //iss = new sortingState([0,2,0,2]);
        //initial[iss] = 1.0;
        // suresh's mdp
        //iss = new sortingState([0,2,0,0]);
        //initial[iss] = 1.0;
        foreach (ms; model.S()) {
            sortingState s = cast(sortingState)ms;
            if ((s._onion_location == 0) && (s._listIDs_status == 0)){
                initial[s] = 1.0;
            } 
        }
        
    } else {//boyd patrol
    	foreach (s; model.S()) {
    		initial[s] = 1.0;
    	} 
    }
	Distr!State.normalize(initial); 

    double [][] foundWeightsGlbl;
    Agent[] policy_ds;    
    double [][] lastWeights;
    foundWeightsGlbl.length = max_d;
    lastWeights.length = max_d;
    policy_ds.length = max_d;

    for (int q = 0; q < max_d; q ++) {
        lastWeights[q].length = reward.dim(); 
        for (int i = 0; i < reward.dim(); i ++) {
            //foundWeightsGlbl[q][i] = uniform(-0.99, .99);
            lastWeights[q][i] = uniform(0.01, .99);
        }
        policy_ds[q] = new RandomAgent(model.A(null));
    } 
    debug {
        //writeln("intitialized policies");
    }

    State [] observableStatesList;
    foreach (s; model.S()) {
        observableStatesList ~= s;
    }

	if (algorithm == "DPMMEIRL") {

        //int max_iterations = 1; //7;
        double [][] foundWeights; 
        double val; 

        ////////////////////////////////////////////////////////////////////////////////////
        // Threshold on gradient descent of individual cluster decides the speed of convergence
        ////////////////////////////////////////////////////////////////////////////////////
        //double descent_error = 0.001; //0.0001; //0.00005; // 0.00001; 

        //(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=500, 
        //  double error=0.1, double solverError =0.1, int max_d)
        //double VI_threshold = 0.45;
        //VI_threshold = 0.275; // did not learn pcik-inspect-place and staystill
        //VI_threshold = 0.2; // could not solve MDPs
        //VI_threshold = 0.4; // 

        //(int max_iter, MDPSolver solver, State [] observableStates, int n_samples=200, 
        //double error=0.01, int descent_duration_thresh_secs=4*60, double gradient_descent_step_size=0.0001,
        //double solverError =0.1, int max_d=4, double Ephi_thresh=0.1)
        //int max_restarts = 3;

        NonParamMTIRL irl = new NonParamMTIRL(max_iterations,
        new TimedValueIteration(int.max,false,vi_duration_thresh_secs), 
        observableStatesList, 300, descent_error, 
        descent_duration_thresh_secs, gradient_descent_step_size, VI_threshold, 
        max_d, Ephi_thresh, max_restarts_descent);

        if (mapToUse == "sorting") { 

            double [] reward_weights =[0.15, 0.0, -0.1, 0.2, -0.1, 0.0, 0.3, -0.15];
            reward.setParams(reward_weights);
            //ValueIteration vi = new ValueIteration();
            TimedValueIteration vi = new TimedValueIteration(int.max,false,vi_duration_thresh_secs);
            //writeln("Testing VI threshold for sorting MDP ");
            Agent policy1 = vi.createPolicy(model,vi.solve(model, VI_threshold));
            //writeln("Does tuned VI threshold works fast enough? ");
            Agent policy2 = vi.createPolicy(model,vi.solve(model, VI_threshold));
            //writeln("sanity check - lba");
            irl.model = model;
            double lbatest = irl.computeLBA(cast(MapAgent)policy1, cast(MapAgent)policy2);
            //writeln(lbatest);

        }

        // convert from array of arrays to single array (one trajectory) of array of sar's
        size_t max_traj_length = 0;
        foreach (traj_ind, traj; SAR) {
            if (traj.length > max_traj_length) max_traj_length = traj.length;
        }

        //public void solve(Model model, double[State] initial, sar[][] true_samples,
        //size_t sample_length, double [][] init_weights_allclusters, 
        //out double [][] opt_weights);

        //writeln("input demo SAR.length ",SAR.length);
        double avd_EVD = irl.solve(mapToUse, model, initial, SAR, max_traj_length, lastWeights, 
            foundWeightsGlbl, reward.dim(), trueWeights, true_assignments, useAdaptive); 
        writeln("finished solve");

    }	
	

    //ValueIteration vi = new ValueIteration();
    TimedValueIteration vi = new TimedValueIteration(int.max,false,vi_duration_thresh_secs);
    double[State] V;
    double max_chance;
    Action action;
    
    foreach (q; 0 .. max_d) {
        //writeln("weight vector of cluster ",q);
        //writeln(foundWeightsGlbl[q]);
        // writeln("policy of cluster ",q);
        writeln("BEGPOLICY");
        reward.setParams(foundWeightsGlbl[q]);
        policy_ds[q] = vi.createPolicy(model, vi.solve(model, VI_threshold)); 

        foreach (State s; model.S()) {
            max_chance = 0.0;
            action = null;

            if (mapToUse == "sorting"){

                sortingState ss = cast(sortingState)s;
                foreach (Action act, double chance; policy_ds[q].actions(ss)) {
                    if (chance > max_chance) {
                        action = act;   
                    } 
                }
                //writeln("[",ss._onion_location,",", 
                //ss._prediction,",",ss._EE_location, 
                //",",ss._listIDs_status,"]", " = ", action); 

            } else {

                BoydState ps = cast(BoydState)s;
                foreach (Action act, double chance; policy_ds[q].actions(ps)) {
                    if (chance > max_chance) {
                        action = act;   
                    } 
                }
                writeln( ps.getLocation(), " = ", action);
            }

        }
    	
        writeln("ENDPOLICY");
        writeln("Sampled trajs of cluster ", q);
        sar [] traj; 
        for(int i = 0; i < 3; i++) {
            traj = simulate(model, policy_ds[q], initial, 40);
            foreach (sar pair ; traj) {
                //writeln(pair.s, " ", pair.a, " ", pair.r);
                ;
            }
            writeln(" ");
        }

    }

    auto endttime = Clock.currTime();
    auto duration = endttime - stattime;
    writeln("Runtime Duration ==> ", duration);

	return 0;
}
