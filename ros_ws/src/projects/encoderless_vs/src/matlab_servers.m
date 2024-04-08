%% This node sets up a host of MATLAB service servers that can be accesed by clients from the manager node
rosshutdown
clear
clc

%% Initialize ROS node and register to ROS MASTER
rosinit('http://cs35:11311/') % ROS_MASTER_URI, change if on a different machine

%% Skeletonization server
imgdata = rosmessage('sensor_msgs/Image')
skelServer = rossvcserver('/skel', 'imgdata', @skelCallback)