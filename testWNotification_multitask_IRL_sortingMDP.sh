#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash

N=3
multiple=3
TRIALS=$((multiple * N))
j=1

for ((i=1; i<=TRIALS; i++))
do
	#cat src/navigation_irl/data_MEMTIRL.log | devel/bin/dpmMEIRL > ~/Downloads/multiTaskIRL_sortingMDP_output$i.txt &
	python src/navigation_irl/sortingMDP/simulating_behaviors_callingDcode.py > ~/Downloads/multiTaskIRL_sortingMDP_output$i.txt &
	#python src/navigation_irl/sortingMDP/simulating_behaviors_callingDcode.py &

	if (( ++j > N )); then
		wait -n
		((j--))
		echo 'waiting for one of the processes to finish'
	fi
done

BACK_PID=$!
wait $BACK_PID

python ~/send_notification.py

