#!/bin/bash

FILE=/home/rbe07/mer_lab/ros_ws/env_test_log.json
OUTPUT=/home/rbe07/mer_lab/ros_ws/env_log.json
rm $FILE
rm $OUTPUT
while [ true ]
do

	if [ -f "$FILE" ]
	then
		cat $FILE >> $OUTPUT
		echo "Wrote to env_log.json"
		rm $FILE
	#else
	#	echo "No log file to read from"
	fi
done
