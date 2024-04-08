#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>

#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

void dsCallback(const std_msgs::Float64MultiArray &msg){
    // Write ds to excel
    std::ofstream ds_plotdata("ds.csv",std::ios::app);
    ds_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<"\n";
    ds_plotdata.close();
}
void drCallback(const std_msgs::Float64MultiArray &msg){
    // Write dr to excel
    std::ofstream dr_plotdata("dr.csv", std::ios::app);
    dr_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<"\n";
    dr_plotdata.close();
}
void JCallback(const std_msgs::Float32 &msg){
    // Plot J using rqtplot or matplotlib
    // std::cout<<"writing data to file"<<std::endl;
    std::ofstream J_plotdata("modelerror.csv",std::ios::app);
    J_plotdata<<msg.data<<"\n";
    J_plotdata.close();
}

void j1velCallback(const std_msgs::Float64 &msg){
    std::ofstream j1vel_plotdata("j1vel.csv",std::ios::app);
    j1vel_plotdata<<msg.data<<"\n";
    j1vel_plotdata.close();
}

void j2velCallback(const std_msgs::Float64 &msg){
    std::ofstream j2vel_plotdata("j2vel.csv",std::ios::app);
    j2vel_plotdata<<msg.data<<"\n";
    j2vel_plotdata.close();
}

void errCallback(const std_msgs::Float64MultiArray &msg){
    std::ofstream err_plotdata("err.csv", std::ios::app);
    err_plotdata<<msg.data.at(0)<<","<<msg.data.at(1)<<"\n";
    err_plotdata.close();
}

int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle n;

    // std::cout<<"Creating files"<<std::endl;
    // Create files to write data to
    std::ofstream J_plot("modelerror.csv");
    std::ofstream ds_plot("ds.csv");
    std::ofstream dr_plot("dr.csv");
    std::ofstream j1vel_plot("j1vel.csv");
    std::ofstream j2vel_plot("j2vel.csv");
    std::ofstream err_plot("err.csv");

    // Add column names to files
    J_plot <<"Model Error"<<"\n";
    J_plot.close();
    ds_plot <<"ds_x"<<","<<"ds_y"<<"\n";
    ds_plot.close();
    dr_plot <<"dr_x"<<","<<"dr_y"<<"\n";
    dr_plot.close();
    j1vel_plot <<"Joint 1" <<"\n";
    j1vel_plot.close();
    j2vel_plot <<"Joint 2" <<"\n";
    j2vel_plot.close();
    err_plot <<"Err_x"<<","<<"Err_y"<<"\n";
    err_plot.close();

    // Initialize subscribers
    ros::Subscriber ds_sub = n.subscribe("ds_record",1,dsCallback);
    ros::Subscriber dr_sub = n.subscribe("dr_record",1,drCallback);
    ros::Subscriber J_sub = n.subscribe("J_modelerror",1,JCallback);
    ros::Subscriber j1vel_sub = n.subscribe("/vsbot/joint1_velocity_controller/command",1,j1velCallback);
    ros::Subscriber j2vel_sub = n.subscribe("/vsbot/joint2_velocity_controller/command",1,j2velCallback);
    ros::Subscriber error_sub = n.subscribe("servoing_error",1,errCallback);

    // Also add subs for Jacobian and Jacobian update

    ros::spin();
    return 0;
}