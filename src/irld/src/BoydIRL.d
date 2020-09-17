import mdp;
import boydmdp;
import irl;
import std.stdio;
import std.format;
import std.string;
import std.math;
import std.random;
import std.algorithm;
import sortingMDP;
import core.stdc.stdlib : exit;
import std.datetime;
import std.numeric;

//import std.conv;
//import std.math;
//import derelict.sdl2.sdl;
//enum TONE_SAMPLE_RATE = 48000;
//enum WAVE_BUFFER_LENGTH = 512;

//// Accessed from SDL's audio thread
//__gshared uint toneLen, tonePhase;

//extern(C) void soundCallback(void *, ubyte *_stream, int _length) nothrow
//{
//    short *stream = cast(short*) _stream;
//    uint length = _length / 2;

//	if (toneLen == 0)
//		stream[0..length] = 0;
//	else
//		for (uint i=0; i<length; i++)
//		{
//			stream[i] = tonePhase % toneLen < toneLen/2 ? 0 : 0x2000;
//			tonePhase++;
//		}
//}

//void playTone(uint freq)
//{
//	toneLen = TONE_SAMPLE_RATE / freq;
//}

//void clearSound()
//{
//	toneLen = 0;
//}

//int openSound()
//{
//	DerelictSDL2.load();

//    SDL_AudioSpec desiredSpec;

//    desiredSpec.freq = TONE_SAMPLE_RATE;
//    desiredSpec.format = AUDIO_S16SYS;
//    desiredSpec.channels = 1;
//    desiredSpec.samples = WAVE_BUFFER_LENGTH;
//    desiredSpec.callback = &soundCallback;

//    SDL_AudioSpec obtainedSpec;

//    if (SDL_OpenAudio(&desiredSpec, &obtainedSpec) != 0)
//		throw new Exception("SDL_OpenAudio:" ~ SDL_GetError().to!string);

//    // start playing audio
//    SDL_PauseAudio(0);

//    return 0;
//}

//void closeSound()
//{
//    SDL_CloseAudio();
//}

int main() {
	
	// Read in stdin to get settings and trajectory
	bool addDelay = false;
	bool ne_known = true;
	sar [][][] SAR;
    sar [][][] SARfull;
//	double p_fail = 0.33;
//	double p_fail = 0;
    int num_Trajsofar;
    string [2] st;
    //writeln("featureExpecExpertfull");
	int interactionLength;
	int ne;
	double statesVisible;
	double[State][Action][State][] T;
	int useRegions;
	double last_val;
	string mapToUse;
	string buf;
	string algorithm;
	int length_subtrajectory;

	buf = readln();

	debug {
		writeln("starting "); 
	    auto stattime = Clock.currTime();
	} 
	//writeln("starting "); 

	formattedRead(buf, "%s", &mapToUse);
	mapToUse = strip(mapToUse);
	
	buf = readln();	
	formattedRead(buf, "%s", &addDelay);

	buf = readln();
	formattedRead(buf, "%s", &algorithm);
	algorithm = strip(algorithm);

	buf = readln();
	formattedRead(buf, "%s", &ne);

	buf = readln();
	formattedRead(buf, "%s", &ne_known);

	buf = readln();
	formattedRead(buf, "%s", &interactionLength);

	buf = readln();
	formattedRead(buf, "%s", &statesVisible);

	debug {
		writeln("statesVisible ",statesVisible); 
	} 

	int curT = 0;
	T.length = 1;	
	while ((buf = readln()) != null) {
    	buf = strip(buf);
    	
    	if (buf == "ENDT") {
    		curT ++;
    		T.length = T.length + 1;
    		if (T.length > 2)
    			break;
    		continue;	
    	}
    	
    	State s;
    	Action a;
    	State s_prime;
    	double p;
    	
    	if (mapToUse == "largeGridPatrol") {
            p = parse_transitions2(mapToUse, buf, s, a, s_prime);
    	} else {
            p = parse_transitions(mapToUse, buf, s, a, s_prime);
    	}

    	T[curT][s][a][s_prime] = p;
    }

	T.length = T.length - 1;

	buf = readln();
	formattedRead(buf, "%s", &useRegions);

	byte[][] map;
	LinearReward reward;
    Model [] models;

	if (mapToUse == "boyd2") {

		map = boyd2PatrollerMap();
        models ~= new BoydModel(null, map, T[0], 1, &simplefeatures);
        models ~= new BoydModel(null, map, T[1], 1, &simplefeatures);

	} else {

		if (mapToUse == "largeGridPatrol") {
			map = largeGridPatrollerMap();
            //models ~= new BoydModel(new BoydState([6, 8, 0]), map, T[0], 1, &simplefeatures);
            //models ~= new BoydModel(new BoydState([6, 8, 0]), map, T[1], 1, &simplefeatures);
            models ~= new BoydExtendedModel2(new BoydExtendedState2([-1,-1,-1],0), map, T[0], 1, &simplefeatures);
            models ~= new BoydExtendedModel2(new BoydExtendedState2([-1,-1,-1],0), map, T[1], 1, &simplefeatures);

		} else {
			if (mapToUse == "boydright2") {
				map = boydright2PatrollerMap();
				models ~= new BoydModel(null, map, T[0], 1, &simplefeatures);
				models ~= new BoydModel(null, map, T[1], 1, &simplefeatures);
			} else {
				if (mapToUse == "sorting") {
					//models ~= new sortingModel(0.05,null);
					//models ~= new sortingModel(0.05,null);
					//models ~= new sortingModel2(0.05,null);
					//models ~= new sortingModel2(0.05,null);
					//models ~= new sortingModelbyPSuresh(0.05,null);
					//models ~= new sortingModelbyPSuresh(0.05,null);
					models ~= new sortingModelbyPSuresh2(0.05,null);
					models ~= new sortingModelbyPSuresh2(0.05,null);
					//models ~= new sortingModelbyPSuresh3(0.05,null);
					//models ~= new sortingModelbyPSuresh3(0.05,null);
				} else {
					map = boydrightPatrollerMap();
					models ~= new BoydModel(null, map, T[0], 1, &simplefeatures);
					models ~= new BoydModel(null, map, T[1], 1, &simplefeatures);
				}
			}
		}
	}

	State [] observableStatesList;
	int[State] distances;
	double [] reward_weights;
	int dim;
	foreach(i;0..models.length) {
		if (mapToUse == "boyd2") {

			assignDistance(models[i], new BoydState([8,1,0]), distances);

	/*		if (statesVisible >= 1 && !addDelay) {
				reward = new Boyd2Reward(model, distances);
				reward_weights = new double[reward.dim()];
			} else { */
            reward = new Boyd2RewardGroupedFeatures(models[i]);
//				reward = new Boyd2RewardAlt(models[i], new BoydState([8,1,0]));
            reward_weights = new double[reward.dim()];
            reward_weights[] = 0;
	//		}
			reward.setParams(reward_weights);

			// build the observed states list

			foreach (s; models[i].S()) {
				if (boyd2isvisible(cast(BoydState)s, statesVisible))
					observableStatesList ~= s;
			}

		} else {

            if (mapToUse == "largeGridPatrol") {

                //reward = new largeGridRewardGroupedFeatures(models[i]);

                if (useRegions == 1) {
                    reward = new largeGridRewardGroupedFeatures3(models[i]);
                }
                else {
                    reward = new largeGridRewardGroupedFeatures2(models[i]);
                }

                reward_weights = new double[reward.dim()];
                reward_weights[] = 0;
                reward.setParams(reward_weights);

                // build the observed states list

                foreach (s; models[i].S()) {
                    if (largeGridisvisible(cast(BoydExtendedState2)s, statesVisible))
                        observableStatesList ~= s;
                }

            } else {

				if (mapToUse == "boydright2") {
					reward = new BoydRight2Reward(models[i]);
					reward_weights = new double[reward.dim()];
					reward_weights[] = 0;

					reward.setParams(reward_weights);

					foreach (s; models[i].S()) {
						if (boydright2isvisible(cast(BoydState)s, statesVisible))
							observableStatesList ~= s;
					}
				} else {

					if (mapToUse == "sorting") {
						// Which reward type is it? 
						//dim = 8;
						//reward = new sortingReward2(models[i],dim); 
						//dim = 10;
						//reward = new sortingReward3(models[i],dim); 
						//reward = new sortingReward4(models[i],dim); 
						//reward = new sortingReward5(models[i],dim); 
						dim = 11;
						reward = new sortingReward6(models[i],dim); 

						reward_weights = new double[reward.dim()];
						reward_weights[] = 0;
						reward.setParams(reward_weights);

						foreach (s; models[i].S()) {
							observableStatesList ~= s;
						}
					} else {
						reward = new BoydRightReward(models[i]);
						reward_weights = new double[reward.dim()];
						reward_weights[] = 0;

						reward.setParams(reward_weights);

						foreach (s; models[i].S()) {
							if (boydrightisvisible(cast(BoydState)s, statesVisible))
								observableStatesList ~= s;
						}
					}
				}
            }
		}

		models[i].setReward(reward);
		models[i].setGamma(0.99);
		models[i].setGamma(0.999);
	}


	int curPatroller = 0;
	SAR.length = 1;
	debug {
		writeln("started reading trajectories");
	}
    while ((buf = readln()) != null) {
    	buf = strip(buf);
	    debug {
	    	//writeln("buf ",buf);
	    }
    
    	if (buf == "ENDTRAJ") {
    		curPatroller ++;
    		SAR.length = SAR.length + 1;
		    debug {
		    	writeln("SAR.length ",SAR.length);
		    }
    		if (SAR.length > 2) {
			//writeln("finished reading traj, Break. ");
    			break;
			}
    		continue;

    	} else {
    		sar [] newtraj;
    		
    		while (buf.countUntil(";") >= 0) {
    			string percept = buf[0..buf.countUntil(";")];
    			buf = buf[buf.countUntil(";") + 1 .. buf.length];
    			
    			string state;
    			string action;
    			double p;
    			
   				formattedRead(percept, "%s:%s:%s", &state, &action, &p);
   				
   				if (mapToUse == "sorting") {

	                int ol;
	                int pr;
	                int el;
	                int ls;

					state = state[1..state.length];
	                formattedRead(state, " %s, %s, %s, %s]", &ol, &pr, &el, &ls);

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
	   				} else if (action == "PlaceInBinClaimNextInList") {
	   					a = new PlaceInBinClaimNextInList();
	                } else {
	                    a = new ClaimNextInList();
	   				}
	   				
	   				newtraj ~= sar(new sortingState([ol, pr, el, ls]),a,p);

				    debug {
				    	//writeln("finished reading ",[ol, pr, el, ls],action);
				    }

   				} else {

	   				int x;
	   				int y;
	   				int z;
	   				int cg;

					state = state[1..state.length];
					if (mapToUse == "largeGridPatrol") {

	                    formattedRead(state, "%s, %s, %s], %s", &x, &y, &z, &cg);
					} else {
	       				formattedRead(state, "%s, %s, %s]", &x, &y, &z);
					}

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
	   				
					if (mapToUse == "largeGridPatrol") {
	       				newtraj ~= sar(new BoydExtendedState2([x, y, z], cg), a, p);
					} else {
	       				newtraj ~= sar(new BoydState([x, y, z]), a, p);
					}

				}
    		}
    		
    		SAR[curPatroller] ~= newtraj;
    		
    	}
    	
    }
	SAR.length = SAR.length - 1;
    debug {
    	writeln("finished reading trajectories");
    }

    // change this acc to choice of reward model
    double [][] lastWeightsI2RL;
    double [][] featureExpecExpert;
    double [][] featureExpecExpertfull;
    double [][] foundWeightsGlbl;

    lastWeightsI2RL.length = 2;
    featureExpecExpert.length = 2;
    featureExpecExpertfull.length = 2;
    foundWeightsGlbl.length = 2;

    for (int i = 0; i < 2; i++)
    {
        lastWeightsI2RL[i].length = reward.dim();
        lastWeightsI2RL[i][] = 0.0;
        featureExpecExpertfull[i].length = reward.dim();
        featureExpecExpertfull[i][] = 0.0;
        featureExpecExpert[i].length = reward.dim();
        featureExpecExpert[i][] = 0.0;
        foundWeightsGlbl[i].length = reward.dim();

    }

    //    featureExpecExpertfull=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
    //    lastWeightsI2RL=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
    //    featureExpecExpert=[[0.0,0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0,0.0]];
    //
    //} else {
    //    featureExpecExpertfull=[[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]];
    //    lastWeightsI2RL=[[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]];
    //    featureExpecExpert=[[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]];
    //
    //}

    //writeln("finished initialize lastWeightsI2RL");
    if (mapToUse != "sorting") {
	    if (algorithm == "LME" || algorithm == "LME2" || algorithm == "MAXENTZAPPROX") {

	        buf = readln();
	        debug {
		        writeln("read buf for [[%s], [%s]] ");
		        writeln(buf);
	        }
	        formattedRead(buf, "[[%s], [%s]]", &st[0], &st[1]);
	        //change this acc to choice of reward model
	        for (int i = 0; i < 2; i++) {
	            for (int j = 0; j < reward.dim()-1; j++) {
	                formattedRead(st[i],"%s, ",&lastWeightsI2RL[i][j]);
	            }
	            formattedRead(st[i],"%s",&lastWeightsI2RL[i][reward.dim()-1]);
	            //if (mapToUse == "largeGrid") {
	            //    formattedRead(st[i], "%s, %s, %s, %s, %s, %s, %s", &lastWeightsI2RL[i][0], &lastWeightsI2RL[i][1], &lastWeightsI2RL[i][2], &lastWeightsI2RL[i][3], &lastWeightsI2RL[i][4], &lastWeightsI2RL[i][5], &lastWeightsI2RL[i][6]);
	            //} else {
	            //    formattedRead(st[i], "%s, %s, %s, %s, %s, %s", &lastWeightsI2RL[i][0], &lastWeightsI2RL[i][1], &lastWeightsI2RL[i][2], &lastWeightsI2RL[i][3], &lastWeightsI2RL[i][4], &lastWeightsI2RL[i][5]);
	            //}
	        }
	        //writeln("finished reading lastWeightsI2RL");

	        //writeln(lastWeightsI2RL);

	        buf = readln();
	        debug {
		        writeln("read buf for [[%s], [%s]] ");
		        writeln(buf);
	        }
	        formattedRead(buf, "[[%s], [%s]]", &st[0], &st[1]);
	        //change this acc to choice of reward model
	        for (int i = 0; i < 2; i++) {
	            for (int j = 0; j < reward.dim()-1; j++) {
	                formattedRead(st[i],"%s, ",&featureExpecExpert[i][j]);
	            }
	            formattedRead(st[i],"%s",&featureExpecExpert[i][reward.dim()-1]);
	            //if (mapToUse == "largeGrid") {
	            //    formattedRead(st[i], "%s, %s, %s, %s, %s, %s, %s", &featureExpecExpert[i][0], &featureExpecExpert[i][1], &featureExpecExpert[i][2], &featureExpecExpert[i][3], &featureExpecExpert[i][4], &featureExpecExpert[i][5], &featureExpecExpert[i][6]);
	            //} else {
	            //    formattedRead(st[i], "%s, %s, %s, %s, %s, %s", &featureExpecExpert[i][0], &featureExpecExpert[i][1], &featureExpecExpert[i][2], &featureExpecExpert[i][3], &featureExpecExpert[i][4], &featureExpecExpert[i][5]);
	            //}
	        }
	        //writeln(featureExpecExpert);
	        buf = readln();
	        formattedRead(buf, "%s", &num_Trajsofar);
		    debug {
		    	writeln("num_Trajsofar ");
		        writeln(num_Trajsofar);
		    }

	    } 
	}

    double[] trueWeights; 
    trueWeights.length = reward.dim(); 
    if (algorithm == "MAXENTZAPPROX" && mapToUse == "sorting") {

	    debug {
	    	writeln("reading trueWeights ");
	    }
        buf = readln();
        formattedRead(buf, "[%s]", &st[0]);
	    debug {
	    	writeln("st[0] ",st[0]);
	    }
        for (int j = 0; j < reward.dim()-1; j++) {
            formattedRead(st[0],"%s, ",&trueWeights[j]);
        }
        formattedRead(st[0],"%s",&trueWeights[reward.dim()-1]);
	    debug {
	    	writeln(trueWeights);
	    }

        buf = readln();
		formattedRead(buf, "%s", &length_subtrajectory);

    }


    if (mapToUse != "sorting") {

		curPatroller = 0;
		SARfull.length = 1;

	    while ((buf = readln()) != null) {
	    	buf = strip(buf);

	    	if (buf == "ENDTRAJ") {
	    		curPatroller ++;
	    		SARfull.length = SARfull.length + 1;
	    		if (SARfull.length > 2) {
				//writeln("finished reading traj, Break. ");
	    			break;
			}
	    		continue;

	    	} else {
	    		sar [] newtraj;

	    		while (buf.countUntil(";") >= 0) {
	    			string percept = buf[0..buf.countUntil(";")];
	    			buf = buf[buf.countUntil(";") + 1 .. buf.length];

	    			string state;
	    			string action;
	    			double p;

	   				formattedRead(percept, "%s:%s:%s", &state, &action, &p);

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

	    		}

	    		SARfull[curPatroller] ~= newtraj;

	    	}

	    }
		SARfull.length = SARfull.length - 1;

    }


	double[State][] initials;
	initials.length = SAR.length;
	
	// Use an initial state distribution that matches the observed trajectory
/*	foreach (int num, sar [][] temp; SAR) {
		bool initialized = false;
		foreach(sar [] SAR2; temp) {
			if (SAR2.length > 0) {
				initials[num][SAR2[0].s] = 1.0;
				initialized = true;
				break;
			}
		}
		if (! initialized) {
			foreach (s; models[num].S()) {
				initials[num][s] = 1.0;
			}
		}
		Distr!State.normalize(initials[num]);
		
	}*/
	
    // ALWAYS START FROm  0,2,0,2
    //sortingState iss = new sortingState([0,2,0,2]);
    // suresh's mdp
    sortingState iss = new sortingState([0,2,0,0]);
	foreach (int num, sar [][] temp; SAR) {
		if (mapToUse == "sorting") {

		    initials[num][iss] = 1.0;
			Distr!State.normalize(initials[num]); 

		} else {
			foreach (s; models[0].S()) {
				initials[num][s] = 1.0;
			}
			Distr!State.normalize(initials[num]);
		}
	}
	
	auto all_NEs = genEquilibria();
	double[Action][] selected_NE = all_NEs[ne];
    auto interaction_delegate = delegate bool(State s1, State s2) {
		return s1.samePlaceAs(s2);
	};

//	sar [][] samples1 = naiveInterpolateTraj(SAR[0], model, observedStatesList[0]);
//	sar [][] samples2 = naiveInterpolateTraj(SAR[1], model, observedStatesList[1]);

	sar [][] samples1 = SAR[0];
	sar [][] samples2 = SAR[1];
	
//	sar [][] samples1 = insertTurnArounds(SAR[0], model, distances, new TurnAroundAction());
//	sar [][] samples2 = insertTurnArounds(SAR[1], model, distances, new TurnAroundAction());
    
	Agent policy1 = new RandomAgent(models[0].A(null));
	Agent policy2 = new RandomAgent(models[1].A(null));

	int counter = 0;
	double [] featureExpectations1;
	double [] featureExpectations2;
	featureExpectations1.length = reward.dim();
	featureExpectations2.length = reward.dim();
	featureExpectations1[] = 0;
	featureExpectations2[] = 0;
	
	double [] lastWeights1 = new double[reward_weights.length];
	for (int i = 0; i < lastWeights1.length; i ++) {
		lastWeights1[i] = uniform(-0.99, 0.99);
		if (mapToUse == "sorting") lastWeights1[i] = uniform(0.01, 0.99);
	}

	debug {
        writeln("initialized lastWeights1 -- ",lastWeights1);
    }
	
    //writeln("lastWeights1:",lastWeights1);
	double [] lastWeights2 = new double[reward_weights.length];
	for (int i = 0; i < lastWeights2.length; i ++) {
		lastWeights2[i] = uniform(-0.99, 0.99);
		if (mapToUse == "sorting") lastWeights2[i] = uniform(0.01, 0.99);
	}
	
//	lastWeights1[] = 0;
//	lastWeights2[] = 0;
	
/*	lastWeights1[0] = 1;
	lastWeights1[1] = 0.55;
	lastWeights2[0] = 1;
	lastWeights2[1] = 0.55;*/

	
	int chosenEquilibrium = ne;

	//writeln("if reached");
	sar [][][] trajectoriesg;

	if (algorithm == "LMEBLOCKEDGIBBS2") {
        double [][] lastWeights;
		lastWeights = lastWeightsI2RL;
        int iterations = 5;
        //if (statesVisible >= 1.0)
        //    iterations = 1;

        double [][] foundWeights;
        double val;
		LatentMaxEntIrlZiebartApproxMultipleAgentsBlockedGibbs irl = new
		LatentMaxEntIrlZiebartApproxMultipleAgentsBlockedGibbs(iterations, new ValueIteration(),
		observableStatesList, 100, 0.0001, .1, interaction_delegate);

		// convert from array of arrays to single array (one trajectory) of array of sar's
		sar [][][] trajectories;
		trajectories.length = 2;
		size_t [] traj_lengths;
		foreach (agent_num, agent_traj; SAR) {
			trajectories[agent_num].length = 1;

			foreach (entry; agent_traj) {
				if (entry.length > 0)
					trajectories[agent_num][0] ~= entry[0];
				else
					trajectories[agent_num][0] ~= sar(null, null, 1.0);
			}
			traj_lengths ~= trajectories[agent_num][0].length;
		}

		// convert from array of arrays to single array (one trajectory) of array of sar's
		sar [][][] trajectoriesfull;
		trajectoriesfull.length = 2;
		size_t [] traj_lengthsfull;
		foreach (agent_num, agent_traj; SARfull) {
			trajectoriesfull[agent_num].length = 1;

			foreach (entry; agent_traj) {
				if (entry.length > 0)
					trajectoriesfull[agent_num][0] ~= entry[0];
				else
					trajectoriesfull[agent_num][0] ~= sar(null, null, 1.0);
			}
			traj_lengthsfull ~= trajectoriesfull[agent_num][0].length;
		}

		//writeln("read traj_lengthsfull");

		Agent [] policies = irl.solve2(cast(Model[])models, initials, trajectories,
		traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights,
		featureExpecExpert, num_Trajsofar, trajectoriesfull, featureExpecExpertfull);
		//writeln("finished solve2");

		policy1 = policies[0];
		policy2 = policies[1];

		chosenEquilibrium = ne;

		foundWeightsGlbl=foundWeights;
		last_val=val;

	}  else if (algorithm == "NG") {
/*		
		if (!addDelay) {
				
			double [] foundWeights;
			
			double val1;
	//		MaxEntIrlBothPatrollers irl = new MaxEntIrlBothPatrollers(20,new ValueIteration(), 50, .1, .001, .1);
			NgProjIrl irl = new NgProjIrl(50,new ValueIteration(), 100, .1, .1);
			
			policy1 = irl.solve(model, initials[0], samples1, lastWeights1, val1, foundWeights);
			
			NgProjIrl irl2 = new NgProjIrl(50,new ValueIteration(), 100, .1, .1);
			
			double [] foundWeights2;
			double val2;
			policy2 = irl2.solve(model, initials[1], samples2, lastWeights2, val2, foundWeights2);
			
			
		} else {
			
			
			
			double [][] foundWeights;
			Model [] models;
			double [][] lastWeights;
			
			
			models ~= model;
			models ~= model;
						
			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;
			
			
			double val;
	//		MaxEntIrlBothPatrollers irl = new MaxEntIrlBothPatrollers(20,new ValueIteration(), 50, .1, .001, .1);
			NgProjIrlDelayAgents irl = new NgProjIrlDelayAgents(50,new ValueIteration(), 100, .1, .1);
			
			sar [][][] trajectories;
			trajectories.length = 2;
			trajectories[0] = samples1;
			trajectories[1] = samples2;

			
			double[Action][][] equilibria = genEquilibria();
			Agent [] policies = irl.solve(models, initials, trajectories, lastWeights, equilibria, val, foundWeights, chosenEquilbrium, interactionLength);
			
			policy1 = policies[0];
			policy2 = policies[1];			
		}
		*/
	} else if (algorithm == "MAXENTZEXACT") {

			double [] foundWeights1;
			double val1;
			double [] foundWeights2;
			double val2;

			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			foreach (agent_num, agent_traj; SAR) {
				trajectories[agent_num].length = 1;

				foreach (entry; agent_traj) {
					if (entry.length > 0)
						trajectories[agent_num][0] ~= entry[0];
					else
						trajectories[agent_num][0] ~= sar(null, null, 1.0);
				}
				traj_lengths ~= trajectories[agent_num][0].length;
			}

			LatentMaxEntIrlZiebartExact irl1 = new LatentMaxEntIrlZiebartExact(50, new ValueIteration(), models[0].S(), 50, .0005, .1);
			//writeln("calling LatentMaxEntIrlZiebartExact.solve");
			policy1 = irl1.solve(models[0], initials[0], trajectories[0], lastWeights1, val1, foundWeights1);

			LatentMaxEntIrlZiebartExact irl2 = new LatentMaxEntIrlZiebartExact(50, new ValueIteration(), models[0].S(), 50, .0005, .1);
			policy2 = irl2.solve(models[1], initials[1], trajectories[1], lastWeights2, val2, foundWeights2);

			double [][] foundWeights;
			foundWeights ~= foundWeights1;
			foundWeights ~= foundWeights2;

			//writeln(foundWeights);
	} else if (algorithm == "MAXENTZAPPROX") {

			debug {
				writeln("Tune for sorting MDP behaviors");
			}

			double [] foundWeights1;
			double val1;
			double [] foundWeights2;
			double val2;

			
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			//int  length_subtrajectory = 40;
			foreach (agent_num, agent_traj; SAR) {
				
				if (mapToUse == "sorting") {
					// divide them into equal sized trajectories for each of two experts
					
					int t = 1;
					int ct = 0;
					trajectories[agent_num].length = 1;
					foreach (entry; agent_traj) {
						if (entry.length > 0)
							trajectories[agent_num][ct] ~= entry[0];
						else
							trajectories[agent_num][ct] ~= sar(null, null, 1.0);

						// if reached desired length,  make next trajectory
						if (t % length_subtrajectory == 0) {
							ct += 1;
							trajectories[agent_num].length += 1;
						}

						t = (t + 1) % length_subtrajectory;
						//writeln()
					}
					traj_lengths ~= length_subtrajectory;
				} else {
					// convert from array of trajectories to array of singleton arrays with one trajectory
					trajectories[agent_num].length = 1;
					foreach (entry; agent_traj) {
						if (entry.length > 0)
							trajectories[agent_num][0] ~= entry[0];
						else
							trajectories[agent_num][0] ~= sar(null, null, 1.0);
					}
					traj_lengths ~= trajectories[agent_num][0].length;
				}
			}

	        size_t max_sample_length = 0;
			foreach(sl; traj_lengths)
				if (sl > max_sample_length)
					max_sample_length = sl;

			//writeln("input to irl1:",trajectories[0]);

			double VI_threshold = 0.05; // 0.05 works 100% of times for forward RL in patrolling problem
			double grad_descent_threshold;
			double Ephi_thresh = 0.5;
	    	double gradient_descent_step_size = 0.25; // patrolling task
			int vi_duration_thresh_secs = 45; //30;
			int descent_duration_thresh_secs = 3*60;
			
			Ephi_thresh = 0.1;
			grad_descent_threshold = 0.0001; // patrolling problem
			//grad_descent_threshold = 0.00005;
	    	gradient_descent_step_size = 0.01; // reducing stepsize made it worse for patrolling domain
	    	gradient_descent_step_size = 0.02;
	    	gradient_descent_step_size = 0.1;

			if (mapToUse == "sorting") {
				grad_descent_threshold = 0.001; // sorting 0.01 
				grad_descent_threshold = 0.0001; 
				grad_descent_threshold = 0.00001;
				// specific to suresh' mdp 
				grad_descent_threshold = 0.000005; 
				// specific to suresh' mdp 
				grad_descent_threshold = 0.0000001; 

				VI_threshold = 0.15; // sorting wiht reward3
				//VI_threshold = 0.25; 
				VI_threshold = 0.2; 

				Ephi_thresh = 0.2;
				Ephi_thresh = 0.1;
				//Ephi_thresh = 0.01; // Ephi thresh tighter than  0.1 didn't imporve results for pick-inspect-place

		    	gradient_descent_step_size = 0.1; //  descent not converging to 0.0001
		    	gradient_descent_step_size = 0.05; // diff jumping too much takes too long to converge
		    	gradient_descent_step_size = 0.01; 
		    	gradient_descent_step_size = 0.001; 
		    	gradient_descent_step_size = 0.0001; 
		    	// specific to suresh' mdp
		    	gradient_descent_step_size = 0.00001; 
			}
			// roll-pick-place 0.15 ; 
			int nSamplesTrajSpace = 100;
			int restart_attempts = 5;

			MaxEntIrlZiebartApprox irl1 = new MaxEntIrlZiebartApprox(restart_attempts, 
				new TimedValueIteration(int.max,false,vi_duration_thresh_secs), models[0].S(), 
				nSamplesTrajSpace, grad_descent_threshold, VI_threshold);
			writeln("calling MaxEntIrlZiebartApprox.solve");
			policy1 = irl1.solve(models[0], initials[0], trajectories[0], max_sample_length, 
				lastWeights1, val1, foundWeights1, featureExpecExpert[0], num_Trajsofar, 
				Ephi_thresh, gradient_descent_step_size, descent_duration_thresh_secs); 

			//reward_weights =[0.15, 0.0, -0.1, 0.2, -0.1, 0.0, 0.3, -0.15];
			//reward.setParams(reward_weights);
			TimedValueIteration vi = new TimedValueIteration(int.max,false,vi_duration_thresh_secs);
			policy1 = vi.createPolicy(models[0],vi.solve(models[0], 0.1));

			//writeln("convergence irl1 solve");
			MaxEntIrlZiebartApprox irl2 = new MaxEntIrlZiebartApprox(restart_attempts, 
				new TimedValueIteration(int.max,false,vi_duration_thresh_secs), models[0].S(), 
				nSamplesTrajSpace, grad_descent_threshold, VI_threshold);
			policy2 = irl2.solve(models[1], initials[1], trajectories[1], max_sample_length,
				lastWeights2, val2, foundWeights2, featureExpecExpert[1], num_Trajsofar, 
				Ephi_thresh, gradient_descent_step_size, descent_duration_thresh_secs); 

			//reward_weights =[ 0.10, 0.0, 0.0, 0.22, -0.12, 0.44, 0.0, -0.12];
			//reward.setParams(reward_weights);
			//ValueIteration vi = new ValueIteration();
			
			policy2 = vi.createPolicy(models[1],vi.solve(models[1], 0.1));

			double [][] foundWeights;
			foundWeights ~= foundWeights1;
			foundWeights ~= foundWeights2;

	        foundWeightsGlbl=foundWeights;
	        trajectoriesg = trajectories;
	        last_val=val1+val2; 

   } else if (algorithm == "LME" || algorithm == "LME2") { 

//       lastWeights1[] = 0;
//       lastWeights2[] = 0;

        double [][] lastWeights;

        if (algorithm == "LME") {
            for (int i = 0; i < lastWeights1.length; i ++)
                lastWeights1[i] = uniform(-0.99, .99);

            for (int i = 0; i < lastWeights2.length; i ++)
                lastWeights2[i] = uniform(-0.99, .99);

            lastWeights ~= lastWeights1;
            lastWeights ~= lastWeights2;
        } else {
            lastWeights = lastWeightsI2RL;
        }

        int iterations = 10;
        //if (statesVisible >= 1.0)
        //    iterations = 1;

        double [][] foundWeights;
        double val;
        double error = .0001; //.0001

        LatentMaxEntIrlZiebartApproxMultipleAgents irl = new LatentMaxEntIrlZiebartApproxMultipleAgents(iterations,
        new ValueIteration(), observableStatesList, 200, error, .1, interaction_delegate);

        // convert from array of arrays to single array (one trajectory) of array of sar's
        sar [][][] trajectories;
        trajectories.length = 2;
        size_t [] traj_lengths;
        foreach (agent_num, agent_traj; SAR) {
            trajectories[agent_num].length = 1;

            foreach (entry; agent_traj) {
                if (entry.length > 0)
                    trajectories[agent_num][0] ~= entry[0];
                else
                    trajectories[agent_num][0] ~= sar(null, null, 1.0);
            }
            traj_lengths ~= trajectories[agent_num][0].length;
        }

        // convert from array of arrays to single array (one trajectory) of array of sar's
        sar [][][] trajectoriesfull;
        trajectoriesfull.length = 2;
        size_t [] traj_lengthsfull;
        foreach (agent_num, agent_traj; SARfull) {
            trajectoriesfull[agent_num].length = 1;

            foreach (entry; agent_traj) {
                if (entry.length > 0)
                    trajectoriesfull[agent_num][0] ~= entry[0];
                else
                    trajectoriesfull[agent_num][0] ~= sar(null, null, 1.0);
            }
            traj_lengthsfull ~= trajectoriesfull[agent_num][0].length;
        }

        //writeln("read traj_lengthsfull");

        Agent [] policies = irl.solve2(cast(Model[])models, initials, trajectories,
        traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights,
        featureExpecExpert, num_Trajsofar, trajectoriesfull, featureExpecExpertfull);
        //writeln("finished solve2");

        policy1 = policies[0];
        policy2 = policies[1];

        chosenEquilibrium = ne;

        foundWeightsGlbl=foundWeights;
        last_val=val;

   } else if (algorithm == "LMEOLD") {
			
//			lastWeights1[] = 0;
//			lastWeights2[] = 0;
					
			for (int i = 0; i < lastWeights1.length; i ++)
				lastWeights1[i] = uniform(-0.99, .99);

			for (int i = 0; i < lastWeights2.length; i ++)
				lastWeights2[i] = uniform(-0.99, .99);

			int iterations = 7;
			if (statesVisible >= 1.0)
				iterations = 1;
				
			double [][] foundWeights;
			double [][] lastWeights;

			
			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;
			
			double val;
			
			LatentMaxEntIrlZiebartApproxMultipleAgents irl = new LatentMaxEntIrlZiebartApproxMultipleAgents(iterations, new ValueIteration(), observableStatesList, 100, .0001, .1, interaction_delegate);
			
			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			foreach (agent_num, agent_traj; SAR) {
				trajectories[agent_num].length = 1;
				
				foreach (entry; agent_traj) {
					if (entry.length > 0)
						trajectories[agent_num][0] ~= entry[0];
					else
						trajectories[agent_num][0] ~= sar(null, null, 1.0);
				}
				traj_lengths ~= trajectories[agent_num][0].length;
			}
			
			Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights);
			
			policy1 = policies[0];
			policy2 = policies[1];
			
			chosenEquilibrium = ne;

			foundWeightsGlbl=foundWeights;
			last_val=val;

			debug {
				writeln(val, " ", foundWeights);
			}
	} else if (algorithm == "LMEBLOCKEDGIBBS") {
			
//			lastWeights1[] = 0;
//			lastWeights2[] = 0;

			for (int i = 0; i < lastWeights1.length; i ++)
				lastWeights1[i] = uniform(-0.99, .99);

			for (int i = 0; i < lastWeights2.length; i ++)
				lastWeights2[i] = uniform(-0.99, .99);

			int iterations = 7;
			if (statesVisible >= 1.0)
				iterations = 1;

			double [][] foundWeights;
			double [][] lastWeights;


			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;

			double val;

			LatentMaxEntIrlZiebartApproxMultipleAgentsBlockedGibbs irl = new LatentMaxEntIrlZiebartApproxMultipleAgentsBlockedGibbs(iterations, new ValueIteration(), observableStatesList, 100, 0.0001, .1, interaction_delegate);

			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			foreach (agent_num, agent_traj; SAR) {
				trajectories[agent_num].length = 1;

				foreach (entry; agent_traj) {
					if (entry.length > 0)
						trajectories[agent_num][0] ~= entry[0];
					else
						trajectories[agent_num][0] ~= sar(null, null, 1.0);
				}
				traj_lengths ~= trajectories[agent_num][0].length;
			}

			Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights);

			policy1 = policies[0];
			policy2 = policies[1];

			chosenEquilibrium = ne;

		    foundWeightsGlbl=foundWeights;
		    last_val=val;

			debug {
				writeln(val, " ", foundWeights);
			}
	} else if (algorithm == "LMEBLOCKEDGIBBSTIMESTEP") {
			
			int iterations = 7;
			if (statesVisible >= 1.0)
				iterations = 1;
			
			
			double [][] foundWeights;
			double [][] lastWeights;

			
			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;
			
			double val;
			
			LatentMaxEntIrlZiebartApproxMultipleAgentsTimestepBlockedGibbs irl = new LatentMaxEntIrlZiebartApproxMultipleAgentsTimestepBlockedGibbs(iterations, new ValueIteration(), observableStatesList, 100, .0001, .1, interaction_delegate);
			
			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			foreach (agent_num, agent_traj; SAR) {
				trajectories[agent_num].length = 1;
				
				foreach (entry; agent_traj) {
					if (entry.length > 0)
						trajectories[agent_num][0] ~= entry[0];
					else
						trajectories[agent_num][0] ~= sar(null, null, 1.0);
				}
				traj_lengths ~= trajectories[agent_num][0].length;
			}
			
			Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights);
			
			policy1 = policies[0];
			policy2 = policies[1];
			
			chosenEquilibrium = ne;

			foundWeightsGlbl=foundWeights;
			last_val=val;


			debug {
				writeln(val, " ", foundWeights);
			}	
	} else if (algorithm == "LMEBLOCKEDGIBBSTIMESTEPMA") {
			
			int iterations = 3;
			if (statesVisible >= 1.0)
				iterations = 1;
				
			double [][] foundWeights;
			double [][] lastWeights;

			
			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;
			
			double val;
			
			LatentMaxEntIrlZiebartApproxMultipleAgentsMultiTimestepBlockedGibbs irl = new LatentMaxEntIrlZiebartApproxMultipleAgentsMultiTimestepBlockedGibbs(iterations, new ValueIteration(), observableStatesList, 100, .0001, .1, interaction_delegate);
			
			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			foreach (agent_num, agent_traj; SAR) {
				trajectories[agent_num].length = 1;
				
				foreach (entry; agent_traj) {
					if (entry.length > 0)
						trajectories[agent_num][0] ~= entry[0];
					else
						trajectories[agent_num][0] ~= sar(null, null, 1.0);
				}
				traj_lengths ~= trajectories[agent_num][0].length;
			}
			
			Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights);
			
			policy1 = policies[0];
			policy2 = policies[1];
			
			chosenEquilibrium = ne;


			debug {
				writeln(val, " ", foundWeights);
			}	
	} else if (algorithm == "LMEBLOCKEDGIBBSSATIMESTEP") {
			
			int iterations = 3;
			if (statesVisible >= 1.0)
				iterations = 1;
				
			double [][] foundWeights;
			double [][] lastWeights;

			
			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;
			
			double val;
			
			LatentMaxEntIrlZiebartApproxMultipleAgentsMultiTimestepSingleAgentBlockedGibbs irl = new LatentMaxEntIrlZiebartApproxMultipleAgentsMultiTimestepSingleAgentBlockedGibbs(iterations, new ValueIteration(), observableStatesList, 100, .0001, .1, interaction_delegate);
			
			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			foreach (agent_num, agent_traj; SAR) {
				trajectories[agent_num].length = 1;
				
				foreach (entry; agent_traj) {
					if (entry.length > 0)
						trajectories[agent_num][0] ~= entry[0];
					else
						trajectories[agent_num][0] ~= sar(null, null, 1.0);
				}
				traj_lengths ~= trajectories[agent_num][0].length;
			}
			
			Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights);
			
			policy1 = policies[0];
			policy2 = policies[1];
			
			chosenEquilibrium = ne;


			debug {
				writeln(val, " ", foundWeights);
			}	
	}  else if (algorithm == "LMEFORWBACK") {
			
			
			double [][] foundWeights;
			double [][] lastWeights;

			
			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;
			
			double val;
			
			LatentMaxEntIrlZiebartApproxMultipleAgentsForwardBackward irl = new LatentMaxEntIrlZiebartApproxMultipleAgentsForwardBackward(3, new ValueIteration(), observableStatesList, 100, .02, .001, interaction_delegate);
			
			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			foreach (agent_num, agent_traj; SAR) {
				trajectories[agent_num].length = 1;
				
				foreach (entry; agent_traj) {
					if (entry.length > 0)
						trajectories[agent_num][0] ~= entry[0];
					else
						trajectories[agent_num][0] ~= sar(null, null, 1.0);
				}
				traj_lengths ~= trajectories[agent_num][0].length;
			}
			
			Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights);
			
			policy1 = policies[0];
			policy2 = policies[1];
			
			chosenEquilibrium = ne;


			debug {
				writeln(val, " ", foundWeights);
			}	
	} else if (algorithm == "LMEEXACT") {
			
			double [][] foundWeights;
			double [][] lastWeights;

			
			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;
			
			double val;
			
			LatentMaxEntIrlZiebartExactMultipleAgents irl = new LatentMaxEntIrlZiebartExactMultipleAgents(3, new ValueIteration(), observableStatesList, 50, .01, .001, interaction_delegate);
			
			// convert from array of arrays to single array (one trajectory) of array of sar's
			sar [][][] trajectories;
			trajectories.length = 2;
			size_t [] traj_lengths;
			foreach (agent_num, agent_traj; SAR) {
				trajectories[agent_num].length = 1;
				
				foreach (entry; agent_traj) {
					if (entry.length > 0)
						trajectories[agent_num][0] ~= entry[0];
					else
						trajectories[agent_num][0] ~= sar(null, null, 1.0);
				}
				traj_lengths ~= trajectories[agent_num][0].length;
			}
			
			Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights);
			
			policy1 = policies[0];
			policy2 = policies[1];
			
			chosenEquilibrium = ne;


			foundWeightsGlbl=foundWeights;
			last_val=val;

			debug {
				writeln(val, " ", foundWeights);
			}	
		
	} else {
		
		if (! addDelay) {
			double [] foundWeights;
			
			double val1;
			
			MaxEntIrl irl;
			
/*			if (statesVisible >= 1) {
				irl = new MaxEntIrl(100,new ValueIteration(), 0, .1, .01, .009);
			} else {*/
				irl = new MaxEntIrlPartialVisibility(100,new ValueIteration(), 300, .1, .1, .09, observableStatesList, samples1.length);
//			}
			
			policy1 = irl.solve(models[0], initials[0], samples1, lastWeights1, val1, foundWeights);
			
			/*			if (statesVisible >= 1) {
	 
				irl = new MaxEntIrl(100,new ValueIteration(), 0, .1, .01, .009);
	 
			} else { */
	 
				irl = new MaxEntIrlPartialVisibility(100,new ValueIteration(), 300, .1, .1, .09, observableStatesList, samples2.length);
	 
//			}

			double [] foundWeights2;
			double val2;
			
			policy2 = irl.solve(models[1], initials[1], samples2, lastWeights2, val2, foundWeights2);


			debug {
				writeln("Found Weights: ", foundWeights, " @ ", val1);
				writeln("Found Weights 2: ", foundWeights2, " @ ", val2);
			}
						
		} else {
			
			
			double [][] foundWeights;
			double [][] lastWeights;

			
			lastWeights ~= lastWeights1;
			lastWeights ~= lastWeights2;
			
			double val;
			
			if (ne_known) {
				
				//writeln("MaxEntIrlPartialVisibilityMultipleAgents");
			
				
				MaxEntIrlPartialVisibilityMultipleAgents irl = new MaxEntIrlPartialVisibilityMultipleAgents(200, new ValueIteration(), 150, .1, .1, .09, observableStatesList);
				
				sar [][][] trajectories;
				trajectories.length = 2;
				trajectories[0] = samples1;
				trajectories[1] = samples2;
				
				size_t [] traj_lengths;
				traj_lengths ~= samples1.length;
				traj_lengths ~= samples2.length;
				
				
				Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, selected_NE, interactionLength, val, foundWeights);
				
				policy1 = policies[0];
				policy2 = policies[1];
				
				chosenEquilibrium = ne;
				foundWeightsGlbl=foundWeights;
				last_val=0.0;
			
			} else {
				
				MaxEntIrlPartialVisibilityMultipleAgentsUnknownNE irl = new MaxEntIrlPartialVisibilityMultipleAgentsUnknownNE(100, new ValueIteration(), 150, .1, .1, .09, observableStatesList);
				
				sar [][][] trajectories;
				trajectories.length = 2;
				trajectories[0] = samples1;
				trajectories[1] = samples2;
				
				size_t [] traj_lengths;
				traj_lengths ~= samples1.length;
				traj_lengths ~= samples2.length;
				
				
				Agent [] policies = irl.solve(cast(Model[])models, initials, trajectories, traj_lengths, lastWeights, all_NEs, interactionLength, val, foundWeights);
				
				policy1 = policies[0];
				policy2 = policies[1];
				
				chosenEquilibrium = irl.getEquilibrium();
				foundWeightsGlbl=foundWeights;
				last_val=0.0;
			}

			debug {
			writeln(val, " ", foundWeights);
			}
		}	
	}	
	
	writeln("BEGPARSING");
	foreach (State s; models[0].S()) {
		foreach (Action a, double chance; policy1.actions(s)) {

            if (mapToUse == "largeGridPatrol") {
                BoydExtendedState2 ps = cast(BoydExtendedState2)s;
                writeln( ps.getLocation(),",", ps.getCurrentGoal(), " = ", a);
            } else {

	            if (mapToUse == "sorting") {
	                sortingState ps = cast(sortingState)s;
	                writeln( ps.toString(), " = ", a);
		            string str_s="";
					if (ps._onion_location == 0) str_s=str_s~"Onconveyor,";
					if (ps._onion_location == 1) str_s=str_s~"Infront,";
					if (ps._onion_location == 2) str_s=str_s~"Inbin,";
					if (ps._onion_location == 3) str_s=str_s~"Picked/AtHomePose,";
					if (ps._onion_location == 4) str_s=str_s~"Placed,";

					if (ps._prediction == 0) str_s=str_s~"bad,";
					if (ps._prediction == 1) str_s=str_s~"good,";
					if (ps._prediction == 2) str_s=str_s~"unknown,";

					if (ps._EE_location == 0) str_s=str_s~"Onconveyor,";
					if (ps._EE_location == 1) str_s=str_s~"Infront,";
					if (ps._EE_location == 2) str_s=str_s~"Inbin,";
					if (ps._EE_location == 3) str_s=str_s~"Picked/AtHomePose,";

					if (ps._listIDs_status == 0) str_s=str_s~"Empty";
					if (ps._listIDs_status == 1) str_s=str_s~"NotEmpty"; 
					if (ps._listIDs_status == 2) str_s=str_s~"Unavailable";

		            //writeln(str_s," = ", a);
	            } else {
	                BoydState ps = cast(BoydState)s;
	                writeln( ps.getLocation(), " = ", a);
            	}
            }

		}
	}
	
	writeln("ENDPOLICY");

	foreach (State s; models[0].S()) {
		foreach (Action a, double chance; policy2.actions(s)) {
            if (mapToUse == "largeGridPatrol") {
                BoydExtendedState2 ps = cast(BoydExtendedState2)s;
                writeln( ps.getLocation(),",", ps.getCurrentGoal(), " = ", a);
            } else {

	            if (mapToUse == "sorting") {
	                sortingState ps = cast(sortingState)s;
	                writeln( ps.toString(), " = ", a);
		            string str_s="";
					if (ps._onion_location == 0) str_s=str_s~"Onconveyor,";
					if (ps._onion_location == 1) str_s=str_s~"Infront,";
					if (ps._onion_location == 2) str_s=str_s~"Inbin,";
					if (ps._onion_location == 3) str_s=str_s~"Picked/AtHomePose,";
					if (ps._onion_location == 4) str_s=str_s~"Placed,";

					if (ps._prediction == 0) str_s=str_s~"bad,";
					if (ps._prediction == 1) str_s=str_s~"good,";
					if (ps._prediction == 2) str_s=str_s~"unknown,";

					if (ps._EE_location == 0) str_s=str_s~"Onconveyor,";
					if (ps._EE_location == 1) str_s=str_s~"Infront,";
					if (ps._EE_location == 2) str_s=str_s~"Inbin,";
					if (ps._EE_location == 3) str_s=str_s~"Picked/AtHomePose,";

					if (ps._listIDs_status == 0) str_s=str_s~"Empty";
					if (ps._listIDs_status == 1) str_s=str_s~"NotEmpty"; 
					if (ps._listIDs_status == 2) str_s=str_s~"Unavailable"; 

		            // writeln(str_s," = ", a); 
	            } else {
	                BoydState ps = cast(BoydState)s;
	                writeln( ps.getLocation(), " = ", a);
            	}
            }
		}
	}	
	writeln("ENDPOLICY");

	if (mapToUse!="sorting") { 
		double[Action][] equilibria = genEquilibria()[chosenEquilibrium];

		foreach (key, value; equilibria[0]) {
			writeln(key, " = ", value);
		}
		writeln("ENDE");
		 
		foreach (key, value; equilibria[1]) {
			writeln(key, " = ", value); 
		}	 
		writeln("ENDE");
	} 

	if (algorithm == "MAXENTZAPPROX") {
		writeln(foundWeightsGlbl);
		writeln(featureExpecExpert);
		writeln(num_Trajsofar);
		writeln(last_val);	
		debug {
			if (mapToUse == "sorting") {
				writeln("\nSimulation for 1:");
				sar [] traj;
				for(int i = 0; i < 2; i++) {
					traj = simulate(models[0], policy1, initials[0], 50);
					foreach (sar pair ; traj) {
						writeln(pair.s, " ", pair.a, " ", pair.r);
					}
					writeln(" ");
				}
				writeln("\nSimulation for 2:");
				for(int i = 0; i < 2; i++) {
					traj = simulate(models[1], policy2, initials[1], 50);
					foreach (sar pair ; traj) {
						writeln(pair.s, " ", pair.a, " ", pair.r);
					}
					writeln(" ");
				}
                //Compute average EVD
                double avg_EVD1 = 0.0;
			    double trajval_trueweight, trajval_learnedweight;
			    double [][] fk_Xms_demonstration;

                MaxEntIrlZiebartExact irl = new MaxEntIrlZiebartExact(50, new ValueIteration(), models[0].S(), 50, .0005, .1);
                fk_Xms_demonstration.length = trajectoriesg[0].length;
		        fk_Xms_demonstration = irl.calc_feature_expectations_per_trajectory(models[0], trajectoriesg[0]);
                foreach (j; 0 .. fk_Xms_demonstration.length) {
                    trajval_learnedweight = dotProduct(foundWeightsGlbl[0],fk_Xms_demonstration[j]);
                    trajval_trueweight = dotProduct(trueWeights,fk_Xms_demonstration[j]);
                    avg_EVD1 += abs(trajval_trueweight - trajval_learnedweight)/(trajval_trueweight*cast(double)fk_Xms_demonstration.length);
                }
                double avg_EVD2 = 0.0;
                fk_Xms_demonstration.length = trajectoriesg[1].length;
		        fk_Xms_demonstration = irl.calc_feature_expectations_per_trajectory(models[1], trajectoriesg[1]);
                foreach (j; 0 .. fk_Xms_demonstration.length) {
                    trajval_learnedweight = dotProduct(foundWeightsGlbl[1],fk_Xms_demonstration[j]);
                    trajval_trueweight = dotProduct(trueWeights,fk_Xms_demonstration[j]);
                    avg_EVD2 += abs(trajval_trueweight - trajval_learnedweight)/(trajval_trueweight*cast(double)fk_Xms_demonstration.length);
                }
                writeln("\n EVD1:",avg_EVD1);
                writeln("\n EVD2:",avg_EVD2);
			}
		}

		//import core.thread;
		//openSound();
		//foreach (n; 1..20)
		//{
		//	import std.stdio;writeln(n);
		//	playTone(100*n);
		//	Thread.sleep(250.msecs);
		//}
		//clearSound();
		//closeSound();
	}

    if (algorithm == "LME" || algorithm == "LME2" || algorithm == "LMEBLOCKEDGIBBS2") {

       writeln(foundWeightsGlbl);
       writeln(featureExpecExpert);
       writeln(num_Trajsofar);
       writeln(last_val);
       writeln(featureExpecExpertfull);

    }


	if (algorithm == "LMEBLOCKEDGIBBS" || algorithm == "LMEEXACT" || algorithm == "LMEBLOCKEDGIBBSTIMESTEP" || algorithm == "MAXENT") {

		writeln(foundWeightsGlbl);
		writeln(last_val);
	}
	writeln("ENDPARSING");

	debug {
	    auto endttime = Clock.currTime();
	    auto duration = endttime - stattime;
	    writeln("Runtime Duration ==> ", duration);
	}
	
	return 0;
}
