
# ROS Workspace for Incremental IRL (Inverse Reinforcement Learning) and Multi Task IRL

<!-- TABLE OF CONTENTS -->
## Table of Contents

* [Motivation](#about-the-project)
* [Libraries Used](#prerequisites)
* [Files](#files)
* [Summary of Results](#summary)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)

## Motivation 

### For Latent Maximum Entropy Incremental IRL

Most of IRL (Inverse Reinforcement Learning) methods in literature are not appliable to time series datastream in real-time applications because they are off-line. Same holds for IRL under missing data. My research team instantiated a method for online IRL under missing data  [PaperI2RL](http://www.ifaamas.org/Proceedings/aamas2019/pdfs/p1170.pdf).

### For Maximum Entropy Multi Task IRL

Existing multi-taask IRL methods like EM-MLIRL and DPM-BIRL suffer from either local minima or label bias issues [PaperMaxEntIRL](https://www.aaai.org/Papers/AAAI/2008/AAAI08-227.pdf). To address this problem, we introduced a non-parametric multi-task IRL method without label bias [PaperME-MTIRL](https://arxiv.org/abs/2004.12873). It solves a joint optimization problem of learning clusters corresponding to multiple tasks in training data, but keeping number of learned clsuters minimal.

## Libraries Used
We used following libraries / tools for this project

- ROS
- Python 2.7
- D-language compiler
- Cmake for D language [repo-Cmake-D](https://github.com/dcarp/cmake-d)
- PyBrain (git://github.com/pybrain/pybrain.git)

### Instructions for Setup
- Install ROS

- Clone and build cmake-D

git clone https://github.com/dcarp/cmake-d.git
cd cmake-d
mkdir build
cd build
cmake ../cmake-d
sudo make install

- Clone and build PyBrain
git clone git://github.com/pybrain/pybrain.git
cd pybrain
sudo python setup.py install

- Do rosdep update

## Files

The code is partially built upon the version of IRL code developed in Thinc lab at UGA, by Kenneth Bogert before the original code got modified and got uploaded in github. [librirl-by-kbogert](https://github.com/kbogert/libirl)

### Domains 
The Python frontend and D-code backend have MDP models and MDP solvers for two domains

- stage suimulation for ground navigation (patrol) domain in paper [PaperI2RL](http://www.ifaamas.org/Proceedings/aamas2019/pdfs/p1170.pdf), coded in files src/navigation_irl/patrol/*.py and src/irld/src/boydmdp.d. 
For running this simulation, 
-- git clone this project in your home directory
-- rename workspace to catkin_ws
-- move folder src/patrolstudy to your home directory
-- set parameters in callruniteratively_varyObvty_LBAILEAnalysis.sh and run it

- vegetable sorting domain in paper [PaperME-MTIRL](https://arxiv.org/abs/2004.12873), coded in files src/navigation_irl/sortingMDP/*, src/irld/src/solveSortingMDP.d, and src/irld/src/sortingMDP.d 

The instructions for launching a Gazebo simulation of second domain can be found in [here](https://github.com/s-arora-1987/sawyer_irl_project)

Python frontend in src/navigation_irl/ros_ctrl.py calls the backend src/irld/src/boydirl.d in backend, which forwards call to chosen IRL solver. Note: There are many mdp classes defined for each of the two domains. Therefore, while running the code, the choice of MDP model in frontend should match with backend. 

### Solvers
Some of the IRL solver classes in src/irld/src/irl.d are modified to work in online / incremental fashion in which learner updates the weights learned in preivous sessions and generate new weights and feature expectations as outputs. Bogert's version of UncontrainedAdaptiveGradientDescent has been modified in terms of stopping criterion. Current descent stops when the standard deviation in moving window of diff-feature-expectation is lower than a threshold. 

## Summary of Results

### Demo: Online Imitation (Inverse Reinforcement) Learning under Missing Training Data

![I2RL Navigation Task](https://github.com/s-arora-1987/sawyer_i2rl_project_workspace/blob/master/navigation_task.gif)

### Demo: Maximum Entropy Multi Task Imitation (Inverse Reinforcement) Learning for Two Sorting Methods

Learning of Pick-Inspect-Place Task

![Learning of Pick-Inspect-Place Task](https://github.com/s-arora-1987/sawyer_i2rl_project_workspace/blob/master/sorting_task_1.gif)

Learning of Roll-Pick-Place Task

![Learning of Roll-Pick-Place Task](https://github.com/s-arora-1987/sawyer_i2rl_project_workspace/blob/master/sorting_task_2.gif)

Full video [https://drive.google.com/file/d/1UpOqgZ8_5tVPlQXkEea2jRDkeKOBU0wB/view?usp=sharing]

Note: The choice of sortingMDP model in frontend python code (test_singleTaskIRL*.py, ros_ctrl.py) should match with backend D-code (solveSortingMDP.d, singleTaskIRL.d, multiTaskIRL.d). 

Python frontend in src/navigation_irl/test_singleTaskIRL*.py can be used to verify if single task IRL is working as expected both domains. 

src/navigation_irl/sortingMDP/simulating_behaviors_callingDcode.py generates simulated mix of demonstration from two types of sorting methods, and calls multi-task IRL solver src/irld/src/multitaskIRL.d in backend. multitaskIRL.d implements Me-MTIRL method in  [https://arxiv.org/abs/2004.12873]

## Contact
email: sa08751@uga.edu

## Acknowledgements
- [KBogert](https://scholar.google.com/citations?user=HYCHbSUAAAAJ&hl=en)
- [lab-mates](http://thinc.cs.uga.edu/#person) 
