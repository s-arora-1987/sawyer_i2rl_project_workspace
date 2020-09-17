# sawyer_i2rl_project_workspace

Install ros-kinetic-moveit

clone catkin_simple from https://github.com/catkin/catkin_simple

git clone --branch release-5.2.0 https://github.com/RethinkRobotics/intera_sdk.git

git clone --branch release-5.2.0 https://github.com/RethinkRobotics/intera_common.git

git clone https://github.com/s-arora-1987/two_scara_collaboration.git

cd into catkin_ws and do a catkin_make. 

git clone https://github.com/prasuchit/roboticsgroup_gazebo_plugins-1

git clone --branch release-5.2.0 https://github.com/thinclab/sawyer_robot.git

git clone https://github.com/prasuchit/gazebo_ros_link_attacher

git clone https://github.com/prasuchit/kinect_v2_udrf


git clone --branch release-5.2.0 https://github.com/s-arora-1987/sawyer_simulator.git

git clone --branch release-5.2.0 https://github.com/s-arora-1987/sawyer_moveit.git

git clone --branch kinetic-devel https://github.com/s-arora-1987/robotiq.git

sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

cd into catkin_ws and install all dependencies for these packages:

 (For kinetic: rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y -i --verbose)
 
 Try following in different tabs:
 
 roslaunch sawyer_irl_project i2rl_movingOnions_Sorting.launch
 
 roslaunch sawyer_irl_project spawn_move_claim_onion.launch
 
 roslaunch sawyer_irl_project test_pnp.launch
 
 If last one gives import error for pyassimp, then it requires pyassimp 3.3, while apt-get installed by default is 3.2. Try:
 
 sudo apt-get remove python-pyassimp
 
 pip install pyassimp=3.3
 
 There will be a warning but package should run fine. 
 
