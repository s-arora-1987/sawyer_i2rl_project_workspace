#!/bin/bash

chmod -R 777 ~/catkin_ws
chmod -R 777 ~/patrolstudy
# cp /dev/null > ~/patrolstudy/output_tracking/experiment_data.txt
cd ~/catkin_ws
source devel/setup.bash
catkin_make

# attackeralgo lmeirl lmei2rl maxentirl lmeirlblockedgibbs lmeirlblockedgibbs2 random perfect 
ARG1=lmeirlblockedgibbs2
# state visibility, at time of recording 
ARG2=14 
# obs time, currently not used in learning because learning is based on number of s,a input
ARG3=100
# weights storage in case of troubleshooting 
ARG4=~/patrolstudy/i2rl_troubleshooting/LearningCurve_LMEIRLBLGIBBS/weights.txt 
# BatchIRLFlag True False
ARG5=True
# add delay to reduce collisions
ARG6=0
# min_BatchIRL_InputLen
ARG7=90
# min_IncIRL_InputLen lower per session demo gives best reached accuracy/ ile in n-1 th session of i2rl
ARG8=5
# desiredRecordingLen at time of recording new trajectories
ARG9=200
# useRecordedTraj, to choose between only learning directly or using recorded states for comparison of two methods
ARG10=0
if [ "$ARG1" = "random" ]
then
	ARG10=0
fi
# recordConvTraj recording traj or learning from them?
ARG11=0
# analyzeAttack analyze only learning (0) or full attack (1)
ARG12=1
# timeoutThresh learningtime (depend on input size) + policycomputetime + findGotime
# recording time for 70  states is approx. 150 s (That time is reduced significantly.)
# and waiting for GoTime 
# + compute policy + learning was 900s-1200 s for Ken's experiment
# Then, time limit for computing policy
# and go time must be approx (900 - 150 - 600 =) 150 s to (1200 - 150 - 600 =) 450 s
# for batch and incr-irl with 80 input size, we have to take Batch's time in account for timeout-threshold. 600 + just enough to compute go time, 650
# With 70-80 input states, learning has reached saturation point
# learning time is approx. 600 s for Batch and 160 s for Incr. 
ARG13=500
# maptouse
ARG14="boyd2" #"boyd2" "boydright" "boydright2" "largeGridPatrol" "reducedGridPatrol" is used for testing patrolling - so no need for data log
# launch stage navigation or not
ARG15=1 
# useRegions of states in largeGrid or not. Boydpatroller didn’t compute optimal policy using regions. 
ARG16=0 

OB14=2
FOB14=~/patrolstudy/toupload/recd_convertedstates_2_14_270states.log
OB30=4
FOB30=~/patrolstudy/toupload/recd_convertedstates_4_14_200states.log
OB43=6
FOB43=~/patrolstudy/toupload/recd_convertedstates_6_14_150states.log
OB73=10
FOB73=~/patrolstudy/toupload/recd_convertedstates_10_14_500states.log
OB100=14
FOB100=~/patrolstudy/toupload/recd_convertedstates_14_14_200states.log
OB100LG=14
FOB100LG=~/patrolstudy/toupload/recd_convertedstates_14_14_200states_LG2.log
OB73LG=10
FOB73LG=~/patrolstudy/toupload/recd_convertedstates_10_14_200states_LG2.log
OB30LG=4
FOB30LG=~/patrolstudy/toupload/recd_convertedstates_4_14_200states_LG2.log
OB100LGR=14
FOB100LGR=~/patrolstudy/toupload/recd_convertedstates_14_14_200states_LGR.log 
OB73=10
FOB73=~/patrolstudy/toupload/recd_convertedstates_10_14_500states_BR.log 
OB30BR=4
FOB30BR=~/patrolstudy/toupload/recd_convertedstates_4_14_400states_BR.log 
OB14BR=2
FOB14BR=~/patrolstudy/toupload/recd_convertedstates_2_14_200states_BR.log 
OB73BR=10
FOB73BR=~/patrolstudy/toupload/recd_convertedstates_10_14_150states_BR.log 
OB100BR=14
FOB100BR=~/patrolstudy/toupload/recd_convertedstates_14_14_150states_BR.log 
OB73BR2=10
FOB73BR2=~/patrolstudy/toupload/recd_convertedstates_10_14_200states_boyright2.log

#perception for two rounds of patrolling/ 1000 ish states 3 min or 360 s
#boyd threshold for testing real time I2RL 360 + learning time + 100
#perception time is:222.0
TM10090S=522
TM73100S=522 # 300 without perception + 222. 
TM30100S=1600 # 1650 without perception, + 222. 
TM30110S=1900 # average of 14 runs 750
TM30120S=3000 # 
TMOB14100S=720
TMOB14110S=780
TMDUMMY=500 #guess? 1600+70*2.02+(some extra time due large input)
#boydright2
TM73BR2100S=522 # 300 without perception + 222. 
TM73BR290S=400 # ~ 222+120+50

RF=~/patrolstudy/toupload/recd_convertedstates.log
RFF=~/patrolstudy/toupload/recd_convertedstates_full.log
POLF=~/patrolstudy/onlyLBA_patpolicy.log
POL_BYD=~/patrolstudy/boydpolicy_mdppatrolcontrol_bkup
cp $POL_BYD $POLF

comma=','
csv=""
nl=$'\n'
#ALG1=lmeirl
#ALG2=lmei2rl
#ALG3=lmeirlblockedgibbs2
#ALG4=lmei2rlblockedgibbs2
#ALG5=random

# needed for recording states once
#~/patrolstudy/rundemo_teststoreweights.sh "$ALG1" "$OB73" "$ARG3" "$ARG4" "$ARG5" "$ARG6" "$ARG7" "$ARG8" "$ARG9" "$ARG10" "$ARG11" "$ARG12" "$ARG13" "$ARG14" "$ARG15" "$ARG16" "$FOB73"
#exit 1

# first fix observability, datalog for recorded states (may not be used), threshold and algorithm
for i in $OB100BR,$FOB100BR,700,$ARG1 ; do # $OB30BR,$FOB30BR,$TM30BR70S $OB73BR,$FOB73BR,$TM73BR70S $OB100BR,$FOB100BR,$TM100BR70S

	IFS=","; 
	set $i; 
	ARG2=$1
	#datalog
	ARG17=$2
	#threshold
	ARG13=$3
	#attackeralgo
	ARG1=$4

	if [ "$ARG1" = "lmei2rl" ] || [ "$ARG1" = "lmeirlblockedgibbs2" ]
	then
		ARG5=False
	fi

	time=`date +"%T"`
	echo $time >> ~/patrolstudy/output_tracking/experiment_data.txt 
	echo $time >> ~/patrolstudy/output_tracking/output.csv
	echo "TimeoutThresh:"$ARG13 >> ~/patrolstudy/output_tracking/experiment_data.txt 
	echo "TimeoutThresh:"$ARG13 >> ~/patrolstudy/output_tracking/output.csv
	echo "Algorithm:"$ARG1 >> ~/patrolstudy/output_tracking/experiment_data.txt 
	echo "Algorithm:"$ARG1 >> ~/patrolstudy/output_tracking/output.csv
	echo "analyzeAttack:"$ARG12 >> ~/patrolstudy/output_tracking/experiment_data.txt 
	echo "analyzeAttack:"$ARG12 >> ~/patrolstudy/output_tracking/output.csv
	echo "Observability:"$ARG2" "$time >> ~/patrolstudy/output_tracking/experiment_data.txt
	echo "Observability:"$ARG2" "$time >> ~/patrolstudy/output_tracking/output.csv
	echo "useRecordedTraj:""$ARG10" >> ~/patrolstudy/output_tracking/output.csv
	echo "useRecordedTraj:""$ARG10" >> ~/patrolstudy/output_tracking/experiment_data.txt 

	cp $2 $RF
	time=`date +"%T"`

	# fix session input size
	for ARG8 in 5
	do

		echo "min_IncrIRL_InputLen:""$ARG8" >> ~/patrolstudy/output_tracking/experiment_data.txt 
		TRIALS=10

		echo $TRIALS" TRIALS" >> ~/patrolstudy/output_tracking/output.csv
		echo $TRIALS" TRIALS" >> ~/patrolstudy/output_tracking/experiment_data.txt 

		if [ $ARG12 = 0 ] #decide if target analysis is for LBA Only
		then

			echo "Obs/ |x| LBA SD  ILE SD  Duration SD  MCMCERR SD" >> ~/patrolstudy/output_tracking/output.csv
			
			for ARG7 in 110 
			do
				LBASTR=
				ILESTR=
				MCMCERRSTR=
				QSTR=
				CUMLEARNTIMESTR=
				csv=$csv$ARG7$comma
					
				echo "min_BatchIRL_InputLen:""$ARG7" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				for ((i=1; i<=TRIALS; i++))
				do
					e=0
					e=1 # for debugging why lba is 0.0 
					#~/patrolstudy/rundemo_teststoreweights.sh "$ARG1" "$ARG2" "$ARG3" "$ARG4" "$ARG5" "$ARG6" "$ARG7" "$ARG8" "$ARG9" "$ARG10" "$ARG11" "$ARG12" "$ARG13" "$ARG14" "$ARG15" "$ARG16" "$ARG17" 

					OUTPUT=`~/patrolstudy/rundemo_teststoreweights.sh "$ARG1" "$ARG2" "$ARG3" "$ARG4" "$ARG5" "$ARG6" "$ARG7" "$ARG8" "$ARG9" "$ARG10" "$ARG11" "$ARG12" "$ARG13" "$ARG14" "$ARG15" "$ARG16" "$ARG17"` # > /tmp/output
					echo "$OUTPUT" > ~/patrolstudy/output_tracking/output.txt

					LBA=0
					ILE=0

					while read line
					do
						l=0
				
						if [ "$line" = "LBA1:" ]
						then
							read line
							#echo "read LBA" >> ~/patrolstudy/output_tracking/output.csv
							LBA=$(echo "$LBA + $line" | bc)
						fi
						if [ "$line" = "LBA2:" ]
						then
							read line
							LBA=$(echo "$LBA + $line" | bc)
						fi
						if [ "$line" = "CUMLEARNTIME:" ]
						then
							read line
							CUMLEARNTIMESTR=$CUMLEARNTIMESTR$nl$line
				
						fi
						if [ "$line" = "MCMC_err:" ]
						then
							read line
							MCMCERRSTR=$MCMCERRSTR$nl$line
				
						fi

						if [ "$line" = "ILE:" ]
						then
							read line
							a1=$line 
							ILE=$(echo "$ILE + $line" | bc) 
							read line 
							a2=$line 
							ILE=$(echo "$ILE + $line" | bc)
							ILE=$(echo "$ILE/2" | bc)
							l=1
				
						fi
						if [ $l -eq 1 ]
						then
							l=0
						fi
					done <<< "$(cat /tmp/studyresults2)"

					LBASTR=$LBASTR$nl$LBA
					ILESTR=$ILESTR$nl$ILE
				done

				echo "LBASTR:" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				echo $LBASTR >> ~/patrolstudy/output_tracking/experiment_data.txt 

				echo "ILESTR:" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				echo $ILESTR >> ~/patrolstudy/output_tracking/experiment_data.txt 

				echo "MCMCERRSTR:" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				echo $MCMCERRSTR >> ~/patrolstudy/output_tracking/experiment_data.txt 

				echo "CUMLEARNTIMESTR:" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				echo $CUMLEARNTIMESTR >> ~/patrolstudy/output_tracking/experiment_data.txt 

				LBAAVG="$(echo -e "$LBASTR" | octave --silent --eval 'disp(mean(scanf("%f")))')"
				csv=$csv$LBAAVG$comma
				LBASD="$(echo -e "$LBASTR" | awk '{sum+=$1; sumsq+=$1*$1}END{print sqrt(sumsq/NR - (sum/NR)*(sum/NR))}')"
				#echo $LBASTR >> ~/patrolstudy/output_tracking/output.csv
				#echo $ILESTR >> ~/patrolstudy/output_tracking/output.csv
				#echo $CUMLEARNTIMESTR >> ~/patrolstudy/output_tracking/output.csv
				#echo $MCMCERRSTR >> ~/patrolstudy/output_tracking/output.csv
				echo "STD DEVs" >> ~/patrolstudy/output_tracking/output.csv
				echo $LBASD >> ~/patrolstudy/output_tracking/output.csv
				csv=$csv$LBASD$comma

				ILEAVG="$(echo -e "$ILESTR" | octave --silent --eval 'disp(mean(scanf("%f")))')"
				csv=$csv$ILEAVG$comma
				ILESD="$(echo -e "$ILESTR" | awk '{sum+=$1; sumsq+=$1*$1}END{print sqrt(sumsq/NR - (sum/NR)*(sum/NR))}')"
				echo $ILESD >> ~/patrolstudy/output_tracking/output.csv
				csv=$csv$ILESTD$comma

				CUMLEARNTIMESTRAVG="$(echo -e "$CUMLEARNTIMESTR" | octave --silent --eval 'disp(mean(scanf("%f")))')"
				csv=$csv$CUMLEARNTIMESTRAVG$comma
				CUMLEARNTIMESTRSD="$(echo -e "$CUMLEARNTIMESTR" | awk '{sum+=$1; sumsq+=$1*$1}END{print sqrt(sumsq/NR - (sum/NR)*(sum/NR))}')"
				echo $CUMLEARNTIMESTRSD >> ~/patrolstudy/output_tracking/output.csv
				csv=$csv$CUMLEARNTIMESTRSD$comma

				MCMCERRAVG="$(echo -e "$MCMCERRSTR" | octave --silent --eval 'disp(mean(scanf("%f")))')"
				csv=$csv$MCMCERRAVG$comma
				MCMCERRSTD="$(echo -e "$MCMCERRSTR" | awk '{sum+=$1; sumsq+=$1*$1}END{print sqrt(sumsq/NR - (sum/NR)*(sum/NR))}')"
				echo $MCMCERRSTD >> ~/patrolstudy/output_tracking/output.csv
				csv=$csv$MCMCERRSTD$comma

				echo $csv >> ~/patrolstudy/output_tracking/output.csv
				csv=""
			done
		else 			
		# Timeout Rate is dependent on time threshold, which relies on input size. 
		# Analyze only successrate for current setting of sessionsize, inputsize 
			
			# fix size of total input demonstration
			for ARG7 in 100 #150
			do
				SRSTR=
				ORSTR=
				#Time taken to reach starting of gotime computation
				TMRGSTR=
					
				echo "min_BatchIRL_InputLen, |x|:""$ARG7" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				echo "min_BatchIRL_InputLen, |x|:""$ARG7" >> ~/patrolstudy/output_tracking/output.csv 
				
				echo "SuccessIn10 SD  TimeoutsIn10 SD" >> ~/patrolstudy/output_tracking/output.csv
				for ((i=1; i<=TRIALS; i++))
				do
					BLKSIZE=10
					j=1
					SCNT=0
					OCNT=0
					# Using while to not count the trial with empty input, patr deaths, runtimeout
					while [ "$j" -le "$BLKSIZE" ] #"$I_STR" != "Bye" ]
					do
						
						~/patrolstudy/rundemo_teststoreweights.sh "$ARG1" "$ARG2" "$ARG3" "$ARG4" "$ARG5" "$ARG6" "$ARG7" "$ARG8" "$ARG9" "$ARG10" "$ARG11" "$ARG12" "$ARG13" "$ARG14" "$ARG15" "$ARG16" "$ARG17"
						#OUTPUT=`~/patrolstudy/rundemo_teststoreweights.sh "$ARG1" "$ARG2" "$ARG3" "$ARG4" "$ARG5" "$ARG6" "$ARG7" "$ARG8" "$ARG9" "$ARG10" "$ARG11" "$ARG12" "$ARG13" "$ARG14" "$ARG15" "$ARG16" "$ARG17"` # > /tmp/output
						#echo "$OUTPUT" #> ~/patrolstudy/output_tracking/output.txt

						f=`cat /tmp/studyresults` 
						IFS=' ' read -ra farr <<< "$f" 
						f2=`cat /tmp/detection_pos` 
						IFS=' ' read -ra farr2 <<< "${f2}"
						

						#f=( $f )

						#echo "${f[*]}" >> ~/patrolstudy/output_tracking/experiment_data.txt #report pat-death success failure timeout
						
						a="${farr[0]}" #${f[@]:0:3}
						#b="y" #("y" "-1" "-1")
						#c="n" #("n" "-1" "-1")
						#d="o" #("o" "-1" "-1")
						# 0 is true and 1 is false 
						#[ "$a" = "$b" ] || [ "$a" = "$c" ] || [ "$a" = "$d" ]
						#eval=$?

						#echo "eval" >> ~/patrolstudy/output_tracking/experiment_data.txt
						#echo "$eval" >> ~/patrolstudy/output_tracking/experiment_data.txt

						if [ "$a" = "y" ] || [ "$a" = "n" ] || [ "$a" = "o" ] #[ ${eval} -eq 0 ]
						then
							#echo "true" >> ~/patrolstudy/output_tracking/experiment_data.txt
							if [ "$a" = "y" ]
							then
								echo "${farr[@]}""${farr2[@]}" >> ~/patrolstudy/output_tracking/experiment_data.txt #report pat-death success failure timeout
							fi
							# only yes, no, or timeout. no empty input, patr deaths, runtimeout. 
							j=$((j+1))

							#echo "j" >> ~/patrolstudy/output_tracking/experiment_data.txt
							#echo $j >> ~/patrolstudy/output_tracking/experiment_data.txt

							if [ "$a" = "y" ]
							then
								SCNT=$((SCNT+1))
							#	echo "SCNT" >> ~/patrolstudy/output_tracking/experiment_data.txt
							#	echo $SCNT >> ~/patrolstudy/output_tracking/experiment_data.txt
							fi
							if [ "$a" = "o" ]
							then
								OCNT=$((OCNT+1))
							fi

							while read line
							do

								if [ "$line" = "TMRG:" ]
								then
									read line
									TMRGSTR=$TMRGSTR$nl$line
									echo "TMRG""$line" >> ~/patrolstudy/output_tracking/experiment_data.txt #report time to compute gotime
				
								fi
							done <<< "$(cat /tmp/studyresults2)"

						fi

						#echo $j >> ~/patrolstudy/output_tracking/experiment_data.txt

					done

					#echo $SCNT >> ~/patrolstudy/output_tracking/experiment_data.txt
					#echo "OCNT"$OCNT >> ~/patrolstudy/output_tracking/experiment_data.txt
					SRSTR=$SRSTR$nl$SCNT
					ORSTR=$ORSTR$nl$OCNT

				done

				echo "SRSTR:" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				echo $SRSTR >> ~/patrolstudy/output_tracking/experiment_data.txt 

				echo "ORSTR:" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				echo $ORSTR >> ~/patrolstudy/output_tracking/experiment_data.txt 

				echo "TMRGSTR:" >> ~/patrolstudy/output_tracking/experiment_data.txt 
				echo $TMRGSTR >> ~/patrolstudy/output_tracking/experiment_data.txt 

				SRAVG="$(echo -e "$SRSTR" | octave --silent --eval 'disp(mean(scanf("%f")))')"
				csv=$csv$SRAVG$comma
				SRSD="$(echo -e "$SRSTR" | awk '{sum+=$1; sumsq+=$1*$1}END{print sqrt(sumsq/NR - (sum/NR)*(sum/NR))}')"
				csv=$csv$SRSD$comma

				ORAVG="$(echo -e "$ORSTR" | octave --silent --eval 'disp(mean(scanf("%f")))')"
				csv=$csv$ORAVG$comma
				ORSD="$(echo -e "$ORSTR" | awk '{sum+=$1; sumsq+=$1*$1}END{print sqrt(sumsq/NR - (sum/NR)*(sum/NR))}')"
				csv=$csv$ORSD$comma

				TMRGSTRAVG="$(echo -e "$TMRGSTR" | octave --silent --eval 'disp(mean(scanf("%f")))')"
				csv=$csv$TMRGSTRAVG$comma
				TMRGSTRSD="$(echo -e "$TMRGSTR" | awk '{sum+=$1; sumsq+=$1*$1}END{print sqrt(sumsq/NR - (sum/NR)*(sum/NR))}')"
				echo $TMRGSTRSD >> ~/patrolstudy/output_tracking/output.csv
				csv=$csv$TMRGSTRSD$comma

				echo "STD DEVs" >> ~/patrolstudy/output_tracking/output.csv
				echo $SRSD$comma$ORSD >> ~/patrolstudy/output_tracking/output.csv

				echo $csv >> ~/patrolstudy/output_tracking/output.csv
				csv=""

			done	
		fi
	done
done

time=`date +"%T"`
echo $time >> ~/patrolstudy/output_tracking/experiment_data.txt 
echo $time >> ~/patrolstudy/output_tracking/output.csv

python ~/catkin_ws/src/send_notification.py

if [ "$ARG14" = "largeGridPatrol" ] 
then 
	RFFLG=~/patrolstudy/toupload/recd_convertedstates_14_14_200states_LG2.log
	cp $RFFLG $RFF
	# echo "error cp here"
	POLLG=~/patrolstudy/largeGridpolicy_mdppatrol
	cp $POLLG $POLF
	# echo "error cp here"
	ARG15=0
#	echo "useRegions:"$ARG16 >> ~/patrolstudy/output_tracking/experiment_data.txt 
#	echo "useRegions:"$ARG16 >> ~/patrolstudy/output_tracking/output.csv
fi 



