#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash
TRIALS=10
N=1
j=1
	
for ((i=1; i<=TRIALS; i++))
do
	python src/navigation_irl/test_singletaskIRL.py > ~/Downloads/singleTaskIRLoutput$i.txt & 
	if (( ++j > N )); then
		wait -n
		((j--))
	fi
done

BACK_PID=$!
wait $BACK_PID

python ~/send_notification.py

