#!/bin/bash

fullRun(){
    # angles=(30)
    angles=(80)
    for j in ${angles[@]}; do
        str="("
        for i in {0..99}; do
            # echo "rosrun active_vision discreteSearch "$1" "$i" "$j
            str+="rosrun active_vision discreteSearch "$1" "$i" "$j" && "
        done
        str+=" echo done)"
        eval "time "$str
        # eval "time rosrun active_vision continuousSearch "$1" 6000 "$j
    done
}

contRun(){
    # for i in {60 180 360 720 1000 2000 3000 4000 5000 6000 7000 8000 9000 10000}; do
    for i in {60 200 600}; do
            eval "rosrun active_vision continuousSearch "$1" "$i" 30"
    done
}

cur=$(pwd)
pkgPath=$(rospack find active_vision)
src=$pkgPath"/models/"

#I thought of doing this in a loop, I think it would be buggier than just
# manually writing it all out :(. Here's the template:
# rosrun active_vision continuousSearch 0 0 {z} {scale} .02 0 {file}
# fullRun "rosrun active_vision continuousSearch 0 0 -0.05 0.99 .02 0 cinderBlockAV"
# fullRun "rosrun active_vision continuousSearch 0 0 -0.23 0.999 .02 0 drillAV"
# fullRun "rosrun active_vision continuousSearch 0 0 -0.07 0.97 .02 0 handleAV"
# fullRun "rosrun active_vision continuousSearch 0 0 -0.02 0.999 .04 0 gasketAV"
# fullRun "0 0 0 0 .02 0 cube"
#fullRun "0 0 -.03 -.9 .02 0 003_cracker_box"
#fullRun "0 -.15 -.03 -.9 .02 0 005_tomato_soup_can"
#fullRun "0 .03 -.03 -.9 .02 0 006_mustard_bottle"
#fullRun "0 .03 -.03 -.9 .02 0 008_pudding_box"
#fullRun "0 .03 -.03 -.9 .02 0 010_potted_meat_can"
#fullRun "0 .03 -.03 -.9 .02 0 013_apple"
#fullRun "0 0 -.03 -.9 .02 0 021_bleach_cleanser"
    # fullRun "0 0 -.03 -.9 .02 0 024_bowl"
fullRun "0 0 -.03 -.9 .02 0 025_mug"
#fullRun "-.05 0 -.03 -.9 .02 0 035_power_drill"
fullRun "0 .1 -.03 -.9 .02 0 055_baseball"
fullRun "-.05 -.1 -.2 -.9 .02 0 072-a_toy_airplane"
# fullRun "rosrun active_vision discreteSearch 0 0 -.03 -.9 0.02 0 003_cracker_box"

# contRun "0 0 0 0 .02 0 003_cracker_box"

#Objects WITH output.pcd FILES
objects=("gasketAV" "drillAV" "cinderBlockAV" "handleAV")

#z offsets of corresponding objects
zs=(-0.02 -0.23 -0.05 -0.07)

# #https://stackoverflow.com/questions/41268570/how-can-i-assign-a-range-using-array-length
# #Who thought this was natural? Who looked at this and thought it was a good idea?
# #Iterate over objects + settings
# for i in `seq 0 $(( ${#objects[@]} - 1 ))`
# for object in ${objects[@]}; do
#     for ((i=0;i<360;++i)); do
#         echo $src$object"/output.pcd "$i" 45"
#     done
# done
