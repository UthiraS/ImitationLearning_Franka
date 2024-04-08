# Instructions to setup environment to run IMitation learning training and testing

# mer_lab
Joint repository for the Manipulation and Environmental Robotics Lab

## Build Status
Deps & catkin\_make: [![Build Status](https://travis-ci.com/berkcalli/mer_lab.svg?token=BAvmoQF6psbcxv8Fyp8p&branch=master)](https://travis-ci.com/berkcalli/mer_lab)

##brach avnish
## ROS Workspace
The 'ros_ws' folder contains all the ROS packages and projects.  For a description of what each folder and directory does, please refer to their individual 'README.md' files.

In brief, everything in the 'src' folder outside of the 'projects' folder are general packages that can be used by anyone in workspace, and are a general list of packages that can be used by others.

Anything in the 'projects' folder is project-specific, and are usually not meant to be used by others.  If you are looking to start up a new project/package, please create it here.  If possible, please organize all your project's packages into a single folder containing them.  This helps prevent clutter and keeps things organized.

Avoid using generic names for items in the 'projects' folder, like a node called 'robot\_simulator'.  If a package is really general, move it outside the projects folder and modify it to be flexible and modular.  Otherwise, be more specific with names to avoid collisions when running 'catkin\_make'.

### Setup
## included steps for running relaxed ik package
To setup the ros workspace locally:
1. Go to the workspace location you want
2. Run 'git clone https://www.github.com/berkcalli/mer_lab.git'
3. 'cd' into the mer_lab repository
4. Run 'rosdep install --from-paths --ignore-src ros_ws -y' to install all dependencies
5. 'cd' into the 'ros_ws' folder
6. clone https://github.com/uwgraphics/relaxed_ik_ros1/ into ros_ws/src/
7. clone https://github.com/uwgraphics/relaxed_ik_core/tree/collision-ik into ros_ws/src/relaxed_ik_ros1/relaxed_ik_core 
8. follow instructions to create appropriate franka_ yaml files or (download from this link for existing setup Franka file)
9. follow instructions to install prerequisites and cargo build as given in https://github.com/uwgraphics/relaxed_ik_core/tree/collision-ik
10. git checkout avnish
11. Run catkin_make -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_WHITELIST_PACKAGES="active_vision;active_vision_moveit_config;moveit_planner;franka_pos_grasping_gazebo;relaxed_ik_ros1;motion_planning"

##add this in the bash profile
#ROS NOETIC AND MERLAB SETUP
source /opt/ros/noetic/setup.bash
source /home/uthira/mer_lab/ros_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/uthira/mer_lab/ros_ws/src/franka_ros/panda_moveit_config:$ROS_PACKAGE_PATH
#export ROS_PACKAGE_PATH=/home/uthira/mer_lab/ros_ws/src/relaxed_ik:$ROS_PACKAGE_PATH

Note that the above instructions are no substitute for actually install ros and setting up rosdep, please first follow the instructions at 'http://wiki.ros.org/melodic/Installation/Ubuntu' (or whatever your ros distribution and linux distro combination happen to be).


## YCB dataset models 
1. ensure all ycb objects pcd files and sdf files are loaded in ros_ws/src/projects/active_vision/models/ycbAV
else
download from link

please make sure to change the path inside sdf file.  

## Imitation Learning 
simulation training for Imitation learning
install the necessary python venv version
1. setup python3 virtual environment python python3 -m venv ~/Envs/AVRL
2. activate the environment source ~/Envs/AVRL/bin/activate
3.  pip install -r requirements.txt
4.  install Cuda driver and Cuda toolkit
      ###add LD_LIBRARY PATH and CUDA_HOME in bash profile
      export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-11.8/lib64
      export CUDA_HOME=/usr/local/cuda-11.8
      export PATH=${CUDA_HOME}/bin:${PATH}
5.  install torch, TensorFlow, tensorrt
6.  check the compatibility and version matches
7.  cd ~/ros_ws/src/projects/active_vision/scripts/RL/
8.  python3 Imitation.py
9.  in other terminal run simualtion: roslaunch active_vision workspace.launch visual:="ON" simulationMode:="SIMULATION" 
10. train data collected in ros_ws/src/projects/active_vision/scripts/RL/features_var_log.txt
11. run python3 ros_ws/src/projects/active_vision/scripts/RL/ILSummarizer.py to visualize graph 
for multiple feature testing change feature_type in  ros_ws/src/projects/active_vision/src/kinectService.cpp and observation space in gazeboenv.py and model name in imitation.py


###
simulation testing for Imitation learning
choose either franka_simulation or Franka mode
1. in one terminal rosrun active_vision policyEvaluator.sh /misc/State_Vector/ obj_2_3_g5.csv
in policyevaulator
a) choose objects to test
b) simulation mode
c) choose the trained model file

in another terminal, 
2. roslaunch relaxed_ik_ros1 demo.launch
data collected in ros_ws/src/projects/active_vision/dataCollected/storage/test..


for running in Franka mode
1. switch on the robot
2. run vnc server in robot pc
3. sudo nano etc/hosts in robot pc add  your system name in
   in your remote pc
4. install vnc viewer in your laptop 
5. connect to ip address in vnc viewer
6. in your remote system, edit systems in sudo nano /etc/hosts
7. add these liens to bash profile
   ####FRANKA EMIKA PANDA Real Robot URI, IP Setup Commands
      export ROS_MASTER_URI=http://pandarobot:11311
      export ROS_IP=172.16.1.11
      export ROS_HOSTNAME="uthira-IdeaPad-L340-15IRH-Gaming"

8. run rosrun franka_control in robot pc
9. in remote terminal 1 run rosrun active_vision policyEvaluator.sh /misc/State_Vector/ obj_2_3_g5.csv
10. in remote terminal 2 roslaunch relaxed_ik_ros1 demo.launch


