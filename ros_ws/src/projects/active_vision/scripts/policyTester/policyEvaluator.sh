#!/bin/bash

# ARGUMENTS for policyEvaluator.sh
# [Path] [csv1] [csv2] ...
# [Path] : Path (relative to active_vision pkg) to folder with state vector (Eg : /misc/State_Vector/)
# [csv1] : Optional - csv file to be used, If not mentioned all files are used
# [csv2] : Optional - csv file to be used,

#e.g.
#rosrun active_vision policyEvaluator.sh /misc/State_Vector/ obj_2_3_g5.csv


extract_feature_type() {
    text=$1
    
    # Use grep to find uppercase words, then use awk to get the last one
    feature_type=$(echo $text | grep -o '[A-Z]\+' | tail -1)
    
    echo $feature_type
}

# extract_feature_type() {
#     text=$1
    
#     # Use grep to find uppercase words. Since the feature type is typically in the middle, we use awk to print it.
#     # This assumes the feature acronym is the second uppercase word.
#     feature_type=$(echo $text | grep -o '[A-Z]\{2,\}' | awk 'NR==2 {print $1}')
    
#     echo $feature_type
# }

# Function to check if a screen is still running
checkScreen () {
  local result=false
	shopt -s nullglob
	local screens=(/var/run/screen/S-*/*)
	shopt -u nullglob
	for s in ${screens[*]}; do
			if [[ "$s" == *"$1"* ]]; then
			  result=true
			fi
	done
  echo "$result"
}

#Source: https://stackoverflow.com/questions/14702148/how-to-fire-a-command-when-a-shell-script-is-interrupted
#Kill all screens on interrupt to stop weird persistence issues
exitfn () {
    trap SIGINT              # Restore signal handling for SIGINT
    echo; killall screen     # kill any open screens
		echo "Shut down completed"
    exit                     #   then exit script.
}

trap "exitfn" INT

# Script to test the provided list of policies on the provided data
cur=$(pwd)
pkgPath=$(rospack find active_vision)
src=$pkgPath"/dataCollected/testData/"

csvDataRec='dataRec.csv'
csvParams='parameters.csv'
csvStorageSummary='storageSummary.csv'

simulationMode="FRANKASIMULATION" # SIMULATION / FRANKASIMULATION / FRANKA
# simulationMode="FRANKA" # SIMULATION / FRANKASIMULATION / FRANKA
# objectID=(11 12 13 14 15 16 18 19 20)
# nDataPoints=(10 10 10 10 10 10 10 10 10)
# objectID=(2 8 9 28 5 19 12 41 51 4)  
# objectID=(2 4 5 8 9 12 19 28 41 51)  
# objectID=(5 12 28 41)  
# objectID=(2 9 41 58)  
objectID=(41 19)  

nDataPoints= 10


# "RANDOM" "PCA_LR" "PCA_LDA" "PCA_LDA_LR" "HEURISTIC")
# List all the policies to be tested with the prefix "Policy"
# Policyxxx = ("Policy" "Unique description" "Param 1 name" "Param 1 value" ...)

# Policy1A=("HEURISTIC" "Heuristic")
# Policy1B=("BRICK" "Brick")
# Policy1C=("RANDOM" "Random")
# Policy1D=("3DHEURISTIC" "3D_Heuristic")
# Policy1E=("PCA_LDA" "PCALDA")
# Policy1E=("PCA_LDA_LR" "PCALDALR")
# Policy1E=("PCA_LR" "PCALR")
# # Policy2A=("PCA_LR" "PCA_LR_95"
# #           "/active_vision/policyTester/PCAcomponents" 0.95
# #           "/active_vision/policyTester/HAFstVecGridSize" 7)
# Policy2B=("PCA_LR" "PCA_LR_85"
#            "/active_vision/policyTester/PCAcomponents" 0.85
#            "/active_vision/policyTester/HAFstVecGridSize" 5)
# # Policy2C=("PCA_LR" "PCA_LR_75"
# #           "/active_vision/policyTester/PCAcomponents" 0.75
# #           "/active_vision/policyTester/HAFstVecGridSize" 7)
# # Policy3A=("PCA_LDA" "PCA_LDA_95"
# #           "/active_vision/policyTester/PCAcomponents" 0.95
# #           "/active_vision/policyTester/HAFstVecGridSize" 7)
# Policy3B=("PCA_LDA" "PCA_LDA_85"
#             "/active_vision/policyTester/PCAcomponents" 0.85
#             "/active_vision/policyTester/HAFstVecGridSize" 5)
# # Policy3C=("PCA_LDA" "PCA_LDA_75"
# #           "/active_vision/policyTester/PCAcomponents" 0.75
# #           "/active_vision/policyTester/HAFstVecGridSize" 7)
# Policy5=("QLEARN" "Q_Learning"
#             "/active_vision/policyTester/HAFstVecGridSize" 5)
# Policy6=("PROBABILISTIC" "Prob_Paper")
# Policy7A=("IL" "ImitationLearningMinusEasyMed"
#           "/active_vision/policyTester/ILPolicy" "MissingEasyandMedium_dagger")
# Policy7B=("IL" "ImitationLearningMinusEasy"
#           "/active_vision/policyTester/ILPolicy" "MissingEasy_dagger")
# Policy7C=("IL" "ImitationLearningMinusMed"
#           "/active_vision/policyTester/ILPolicy" "MissingMedium_dagger")              
Policy7D1=("IL_GASD" "ImitationLearningMinusHard"
          "/active_vision/policyTester/ILPolicy" "Train_set_large_GASD_cross3")      
Policy7D2=("IL_HAF" "ImitationLearningMinusHard"
          "/active_vision/policyTester/ILPolicy" "Train_set_large_HAF_cross3")     
Policy7D3=("IL_GRSD" "ImitationLearningMinusHard"
           "/active_vision/policyTester/ILPolicy" "Train_set_large_GRSD_cross3")  
Policy7D4=("IL_VFH" "ImitationLearningMinusHard"
            "/active_vision/policyTester/ILPolicy" "Train_set_large_VFH_cross3") 
Policy7D5=("IL_ESF" "ImitationLearningMinusHard"
            "/active_vision/policyTester/ILPolicy" "Train_set_large_ESF_cross3") 
Policy7D6=("IL_FPFH" "ImitationLearningMinusHard"
            "/active_vision/policyTester/ILPolicy" "Train_set_large_FPFH_cross3") 
Policy7D7=("IL_CVFH" "ImitationLearningMinusHard"
            "/active_vision/policyTester/ILPolicy" "Train_set_large_CVFH_cross3")
# Policy7D8=("IL_NOFEATURE" "ImitationLearningMinusHard"
#             "/active_vision/policyTester/ILPolicy" "Train_set_large_NOFEATURE")   
# Policy7E=("IL_normal" "ImitationLearningTrained"
#           "/active_vision/policyTester/ILPolicy" "Train_set_normal_redo")
# Policy7F=("IL_small" "ImitationLearningTrained"
#           "/active_vision/policyTester/ILPolicy" "Train_set_small_redo")
# Policy8=("BFS" "Breadth_first_search")
# Policy8=("C_OPT" "ContinuousEpsilonOptimal")

files=()

stVecSrc=$pkgPath"$1"
# If csv file name is specified then use the ones specified else use all
argc=$#
argv=("$@")

if [[ "$argc" -eq "0" ]]; then
  echo "ERROR : No arguments"
  exit 1
fi

if [[ "$argc" -gt "1" ]]; then
  for (( idx=1; idx < argc; idx++)); do
    echo "Using state vector : ${argv[idx]}."
  	files+=("${argv[idx]}")
  done
else
  for file in $(ls $stVecSrc | grep .csv); do
  	echo "Using state vector : $file."
  	files+=("$file")
  done
fi

now="$(date +'%Y/%m/%d %I:%M:%S')"
printf "Started at yyyy/mm/dd hh:mm:ss format %s\n" "$now"

# Creating a screen to run ROS & gazebo
printf "Starting gazebo ...\n"
gnome-terminal -- bash -c 'screen -d -R -S session-environment' & sleep 5
# Starting the gazebo and loading the parameters

sleep 10
screen -S session-environment -X stuff $'roslaunch active_vision workspace.launch visual:="ON" simulationMode:="'$simulationMode'"\n'
# Looping over the objects and testing each policy for each.
nDataPtsCtr=0
for objID in ${objectID[@]}; do
  rosparam set /active_vision/policyTester/objID $objID
  rosparam set /active_vision/policyTester/nDataPoints $nDataPoints
	for vars in ${!Policy*}; do
    # Setting the policy and its parameters
    declare -n policy=$vars
		rosparam set /active_vision/policyTester/policy ${policy[0]}
    for (( idx=2; idx<${#policy[@]}; idx+=2)); do
      rosparam set ${policy[idx]} ${policy[idx+1]}
    done

    dataset_name=${policy[-1]}
    feature_type=$(extract_feature_type "$dataset_name")
    rosparam set /active_vision/policyTester/feature_type ${feature_type}
    echo "Using feature type: $feature_type"
    echo "Using Model: $dataset_name"
    echo "Using policy: $policy[0]"
    nHeuristic=0 # To ensure heuristic is run only once
    n3DHeuristic=0 # To ensure heuristic is run only once
    nProbabilistic=0 # To ensure probabilistic is run only once

    for stVec in ${files[@]}; do
      # Setting the csv state vector to be used
      rosparam set /active_vision/policyTester/csvStVecDir $1
      rosparam set /active_vision/policyTester/csvStVec $stVec

      #Save the results to policy:stVec:dataRec.csv (triming the extra .csv)
  		# rosparam set /active_vision/policyTester/csvName "${stVec: 0: -4}:dataRec.csv"
  		rosparam set /active_vision/policyTester/csvName "default.csv"

      printf "Evaluating ${policy[1]} with ${feature_type} on object ${objID}...\n"

      # Saving the parameters
  		rosrun active_vision saveParams.sh ${src}${csvParams}

  		# Open a terminal for the service and the tester
      # gnome-terminal -- bash -c 'screen -d -R -L -S session-kinectService'
  		gnome-terminal -- bash -c 'screen -d -R -L -S session-policyService'
  		gnome-terminal -- bash -c 'screen -d -R -S session-policyTester'
  		sleep 5

  		# Start the service and the tester
      # Heurisic doesnot depend on state vector
      if [[ "$policy" == "HEURISTIC" ]]; then
        if [[ "$nHeuristic" -eq "0" ]]; then
          screen -S session-policyService -X stuff $'source ~/RL/RLZoo/bin/activate\nrosrun active_vision heuristicPolicyService 0\n'
          screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 1\n1\n exit\n'
          # screen -S session-policyTester -X stuff $'sleep 7\n' #Debug line
          nHeuristic=1
        fi
      elif [[ "$policy" == "3DHEURISTIC" ]]; then
        if [[ "$n3DHeuristic" -eq "0" ]]; then
          screen -S session-policyService -X stuff $'source ~/RL/RLZoo/bin/activate\nrosrun active_vision 3DheuristicPolicyService\n'
          screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 1\n1\n exit\n'
          # screen -S session-policyTester -X stuff $'sleep 7\n' #Debug line
          n3DHeuristic=1
        fi
      elif [[ "$policy" == "PROBABILISTIC" ]]; then
        if [[ "$nProbabilistic" -eq "0" ]]; then
          screen -S session-policyTester -X stuff $'rosrun active_vision probabilisticPaper\n1\n exit\n'
          # screen -S session-policyTester -X stuff $'sleep 7\n' #Debug line
          nProbabilistic=1
        fi
      elif [[ "$policy" == "BRICK" ]]; then
        # screen -S session-kinectService -X stuff $'source rosrun active_vision kinectService\n'
        screen -S session-policyService -X stuff $'source ~/Envs/AVRL/bin/activate\nrosrun active_vision trainedPolicyServiceBrick.py\n'
  			screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 2\n1\n exit\n'
      elif [[ "$policy" == "RANDOM" ]]; then
        # screen -S session-policyService -X colon "logfile flush 0^M"
        screen -S session-policyService -X stuff $'source ~/RL/RLZoo/bin/activate\nrosrun active_vision trainedPolicyServiceRandom.py\n'
  			screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 2\n1\n exit\n'
      elif [[ "$policy" == IL* ]]; then
        screen -S session-policyService -X stuff $'source ~/Envs/AVRL/bin/activate\nrosrun active_vision trainedPolicyServiceIL.py '${feature_type}' '${dataset_name}'\n'
  			screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 2 '${feature_type}'\n1\n exit\n'
      elif [[ "$policy" == "C_OPT" ]]; then
        # screen -S session-policyService -X colon "logfile flush 0^M"
        screen -S session-policyService -X stuff $'source ~/RL/RLZoo/bin/activate\nrosrun active_vision trainedPolicyServiceEOpt.py\n'
  			screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 2\n1\n exit\n'
  			# screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 2\n0\n'
        # tail -Fn 0 screenlog.0
      elif [[ "$policy" == "BFS" ]]; then
        rosparam set /active_vision/dataCollector/objID $objID
        rosparam set /active_vision/dataCollector/nData $nDataPoints
        rosparam set /active_vision/dataCollector/csvName $csvDataRec
  			screen -S session-policyTester -X stuff $'rosrun active_vision dataCollector 1\nsleep 1\nexit\n'
      else
        screen -S session-policyService -X stuff $'source ~/RL/RLZoo/bin/activate\nrosrun active_vision trainedPolicyService.py\n'
  			screen -S session-policyTester -X stuff $'rosrun active_vision policyTester 2\n1\n exit\n'
        # screen -S session-policyTester -X stuff $'sleep 7\n' #Debug line
  		fi
  		sleep 5
  		screen -S session-policyTester -X stuff $'sleep 1\nexit\n'

  		# Waiting till testing is over
  		screenOK="$(checkScreen session-policyTester)"
  		while [[ "$screenOK" == "true" ]]; do
  			printf ".";	sleep 10
  			screenOK="$(checkScreen session-policyTester)"
  		done
  		printf "\n"
  		screen -S session-policyService -X stuff "^C"
  		screen -S session-policyService -X stuff $'sleep 1\nexit\n'
  		sleep 2
      
      
    done
    
      
	done
  nDataPtsCtr=$[$nDataPtsCtr+1]
done



# Closing gazebo
      printf "Closing gazebo ...\n"
      screen -S session-environment -X stuff "^C"
      screen -S session-environment -X stuff $'sleep 1\nexit\n'
      sleep 2

printf "***********\n"

# Creating the summary
printf "Generating Summary...\n"
rosrun active_vision summarizerResults.py $src

dst=$pkgPath"/dataCollected/storage/"

# Create a new directory
dataNo="1"
ok=false
while [ $ok = false ]; do
	dstToCheck="$dst""Test_$dataNo""/"
	if [ ! -d $dstToCheck ]; then
		mkdir $dstToCheck
		ok=true
		printf $(basename $dstToCheck)" folder created.\n"
	fi
	dataNo=$[$dataNo+1]
done

# Copying the parameters to summary folder
csvToCheck="${dst}""${csvStorageSummary}"
lineNo="1"
while IFS= read line; do
	if [ $lineNo == "1" ]; then
		if [ ! -f $csvToCheck ]; then
			echo -n "Folder Name, Description,,,,""$line" >> $csvToCheck
		fi
	else
		echo -n $(basename $dstToCheck)", Collection,,,,""$line" >> $csvToCheck
	fi
	echo "" >> $csvToCheck
	lineNo=$[$lineNo+1]
done <"${src}${csvParams}"

printf "Folder and parameter details added to "${csvStorageSummary}".\n"

# Copying the files to the created folder
cd $src
shopt -s extglob
mv !(ReadMe.txt) $dstToCheck
shopt -u extglob
cd $cur
printf "Files Moved.\n"

printf "***********\n"

now="$(date +'%Y/%m/%d %I:%M:%S')"
printf "Ended at yyyy/mm/dd hh:mm:ss format %s\n "$now
