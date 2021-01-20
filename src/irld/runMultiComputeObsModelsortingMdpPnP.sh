#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash
catkin_make

> /home/saurabharora/Downloads/resultsApproxObsModelMetric1.csv 
> /home/saurabharora/Downloads/resultsApproxObsModelMetric2.csv 
> /home/saurabharora/Downloads/resultsApproxObsModelMetric3.csv 

./devel/bin/computeObsModelsortingMDP
