clear;
clc;
rosshutdown
%% Initialize ROS node and register to ROS MASTER
rosinit('http://cs35:11311/') % ROS_MASTER_URI, change if on a different machine
r = rosrate(1);
while(true)
    %% Declare Subscriber
    curimg = rossubscriber('/rosout', @binarizeCallback)'
    waitfor(r);
    
    %% Publish binary img
    
    %% Declare binary img sub
    
    %% Publish Skeleton
    
    
end

%% Shutdown ROS
rosshutdown