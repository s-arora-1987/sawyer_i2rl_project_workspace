# ROS Workspace for Incremental IRL and Multi Task IRL

The code is built upon the version of IRL code developed in Thinc lab at UGA, by Kenneth Bogert before the original code got modified and got uploaded in github. [https://github.com/kbogert/libirl]

Python frontend and D-code backend have MDP models and MDP solvers for two domains

- ground navigation (patrol) domain in paper [http://www.ifaamas.org/Proceedings/aamas2019/pdfs/p1170.pdf], src/navigation_irl/patrol/* and src/irld/src/boydmdp.d

- vegetable sorting domain in paper [https://arxiv.org/abs/2004.12873], src/navigation_irl/sortingMDP/*, src/irld/src/solveSortingMDP.d, and src/irld/src/sortingMDP.d

Python frontend in src/navigation_irl/ros_ctrl.py calls the backend src/irld/src/boydirl.d in backend, which forwards call to chosen IRL solver. 

Note: There are many mdp's defined for each of the two domains. Therefore, while running the code, the choice of MDP model in frontend should match with backend. 

# Incremental IRL or I2RL

Some of the IRL solver classes in src/irld/src/irl.d are modified to work in online / incremental fashion in which learner updates the weights learned in preivous sessions and generate new weights and feature expectations as outputs. 

Bogert's version of UncontrainedAdaptiveGradientDescent has been modified in terms of stopping criterion. Current descent stops when the standard deviation in moving window of diff-feature-expectation is lower than a threshold. 

#  Multi Task IRL

Note: The choice of sortingMDP model in frontend python code (test_singleTaskIRL*.py, ros_ctrl.py) should match with backend D-code (solveSortingMDP.d, singleTaskIRL.d, multiTaskIRL.d). 

Python frontend in src/navigation_irl/test_singleTaskIRL*.py can be used to verify if single task IRL is working as expected both domains. 

src/navigation_irl/sortingMDP/simulating_behaviors_callingDcode.py generates simulated mix of demonstration from two types of sorting methods, and calls multi-task IRL solver src/irld/src/multitaskIRL.d in backend. multitaskIRL.d implements Me-MTIRL method in  [https://arxiv.org/abs/2004.12873]
