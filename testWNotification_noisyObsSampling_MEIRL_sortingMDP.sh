#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash

N=3
multiple=4
TRIALS=$((multiple * N))
j=1

for ((i=1; i<=TRIALS; i++))
do
	python src/navigation_irl/test_singletaskIRLNoisyObs_sortingMDP.py > ~/Downloads/singleTaskIRL_sortingMDP_output$i.txt & 

	if (( ++j > N )); then
		wait -n
		((j--))
		echo 'waiting for one of the processes to finish'
	fi
done

BACK_PID=$!
wait $BACK_PID

python ~/send_notification.py

