#!/bin/bash

cd ~/catkin_ws
source devel/setup.bash

N=3
multiple=3
TRIALS=$((multiple * N))
j=1

for ((i=1; i<=TRIALS; i++))
do
	# cat src/navigation_irl/data_Dcode_BoydIRL_sorting.log | devel/bin/boydirl > ~/Downloads/singleTaskIRL_sortingMDP_output$i.txt &
	# python src/navigation_irl/test_singletaskIRL_sortingMDP.py > ~/Downloads/RL_sortingMDP_output$i.txt & 
	python src/navigation_irl/test_singletaskIRL_sortingMDP.py > ~/Downloads/singleTaskIRL_sortingMDP_output$i.txt & 
	if (( ++j > N )); then
		wait -n
		((j--))
		echo 'waiting for one of the processes to finish'
	fi
done

BACK_PID=$!
wait $BACK_PID

python ~/send_notification.py

