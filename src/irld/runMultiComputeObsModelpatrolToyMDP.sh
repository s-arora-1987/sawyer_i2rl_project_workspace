#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash
catkin_make

> /home/saurabharora/Downloads/resultsApproxObsModelMetric1.csv 
> /home/saurabharora/Downloads/resultsApproxObsModelMetric2.csv 

TRIALS=50

for ((i=1; i<=TRIALS; i++)) 
do
	./devel/bin/computeObsModelpatrolToyMDP
done
