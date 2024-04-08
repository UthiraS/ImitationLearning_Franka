#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>
#include <vector>
#include "encoderless_vs/energyFuncMsg.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"

float eemarkerX; // End effector x-pixel co-ordinate
float eemarkerY; // End effector y-pixel co-ordinate

void eeMarkerCallback(const std_msgs::Float64MultiArray &msg){
    eemarkerX = msg.data.at(0);
    eemarkerY = msg.data.at(1);
}

int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle n;
    
    // waiting for services and Gazebo
    ros::Duration(5).sleep(); 

    // Initializing ROS publishers
    ros::Publisher j1_pub = n.advertise<std_msgs::Float64>("/vsbot/joint1_velocity_controller/command",1);
    ros::Publisher j2_pub = n.advertise<std_msgs::Float64>("/vsbot/joint2_velocity_controller/command",1);
    ros::Publisher ds_pub = n.advertise<std_msgs::Float64MultiArray>("ds_record", 1);
    ros::Publisher dr_pub = n.advertise<std_msgs::Float64MultiArray>("dr_record", 1);
    ros::Publisher J_pub = n.advertise<std_msgs::Float32>("J_modelerror",1);
    ros::Publisher err_pub = n.advertise<std_msgs::Float64MultiArray>("servoing_error", 1);
    ros::Publisher status_pub = n.advertise<std_msgs::Int32>("vsbot/status", 1);
    // std::cout << "Initialized Publishers" <<std::endl;

    // Initializing ROS subscribers
    ros::Subscriber ee_marker = n.subscribe("marker_segmentation",1,eeMarkerCallback); 
    // std::cout << "Initialized Subscribers" <<std::endl;
    
    // Initializing service clients
    ros::service::waitForService("computeEnergyFunc",1000);
    ros::ServiceClient energyClient = n.serviceClient<encoderless_vs::energyFuncMsg>("computeEnergyFunc");
    // std::cout << "Initialized Service Clients"<< std::endl;

    // Initializing status msg
    std_msgs::Int32 status;
    status.data = 0;
    status_pub.publish(status);

    // Servoing variables 
    float gamma; // learning rate
    n.getParam("vsbot/estimation/gamma", gamma);
    float gamma2; // learning rate
    n.getParam("vsbot/estimation/gamma2", gamma2);
    int window; // Estimation window size
    n.getParam("vsbot/estimation/window", window);
    int it = 0; // iterator
    std::vector<float> error (2,0); //error vector
    float err = 0.0; // error norm
    float rate; // control loop rate
    n.getParam("vsbot/estimation/rate",rate);
    float thresh;
    n.getParam("vsbot/control/thresh",thresh);
    std::vector<float> goal (2,0);
    n.getParam("vsbot/control/goal_pos_x", goal[0]);
    n.getParam("vsbot/control/goal_pos_y", goal[1]);
    float lam;
    n.getParam("vsbot/control/lam",lam);
    std_msgs::Float64MultiArray err_msg;
    // std::cout << "Initialized Servoing Variables" << std::endl;

    // Initial estimation variables
    std::vector<float> ds; // change in ee pose
    std::vector<float> dr; // change in joint angles
    std::vector<float> qhat {7,13,-21,17}; // initial Jacobian matrix as vector
    std::vector<float> dSinitial; // Vector list of shape change vectors
    std::vector<float> dRinitial; // Vector list of velocity change vectors
    std_msgs::Float64 j1_vel; // joint1 velocity
    std_msgs::Float64 j2_vel; // joint2 velocity
    std_msgs::Float64MultiArray ds_msg;
    std_msgs::Float64MultiArray dr_msg;
    float old_angle_j1 = 0.0;
    float old_angle_j2 = 0.0;
    float markerX_cur;
    float markerY_cur;
    float t = 1/rate; // time in seconds, used for integrating angular velocity
    // std::cout <<"Initialized estimation variables" << std::endl;

// --------------------------- Initial Estimation -----------------------------
    // Obtain initial robot state
    float markerX_old = eemarkerX;
    float markerY_old = eemarkerY;
    // std::cout << "initial ee position:" << markerX_old <<"," << markerY_old <<std::endl;
    
    // command small displacements around initial position
    ros::Rate r{rate};  // Rate for control loop
    std::cout << "Ready to command small displacements" <<std::endl;
    
    status.data = 1;
    float param = 0.3;
    while (it < window)
    {
        // Publish sin vel w/ random noise to both joints
        j1_vel.data = 2*sin(param*3.14 + 1.7);
        j2_vel.data = 1.5*cos(param*3.14 - 1.7);
        param = param + 0.02;
        // j1_vel.data = (2.0*((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 0.5));
        // j2_vel.data = (2.0*((static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) - 0.5));
        j1_pub.publish(j1_vel);
        j2_pub.publish(j2_vel);
        
        // Obtain current robot state
        markerX_cur = eemarkerX;
        markerY_cur = eemarkerY;
        // std::cout << "current ee position "<< "X:" << markerX_cur <<",Y:"<<markerY_cur<<std::endl;

        // Compute change in state
        // End-effector position
        ds.clear();
        ds.push_back((markerX_cur - markerX_old));
        ds.push_back((markerY_cur - markerY_old));
        // std::cout<<"ds:"<<std::endl;
        // for(std::vector<float>::iterator itr = ds.begin(); itr!=ds.end(); ++itr)
        // {
        //     std::cout <<' '<<*itr;
        // }
        // std::cout<<'\n';

        // Joint angle
        dr.clear();
        dr.push_back((j1_vel.data*t));
        dr.push_back((j2_vel.data*t));
        // std::cout<<"dr:"<<std::endl;
        // for(std::vector<float>::iterator itr = dr.begin(); itr!=dr.end(); ++itr)
        // {
        //     std::cout <<' '<<*itr;
        // }
        // std::cout<<'\n';

        // std::cout << "computed change in state" <<std::endl;

        // Update dSinitial and dRinitial
        dSinitial.push_back(ds[0]);
        dSinitial.push_back(ds[1]);
        dRinitial.push_back(dr[0]);
        dRinitial.push_back(dr[1]);
        // std::cout<<"dSinitial:"<<std::endl;
        // for(std::vector<float>::iterator itr = dSinitial.begin(); itr!=dSinitial.end(); ++itr)
        // {
        //     std::cout <<' '<<*itr;
        // }
        // std::cout<<'\n';

        // std::cout<<"dRinitial:"<<std::endl;
        // for(std::vector<float>::iterator itr = dRinitial.begin(); itr!=dRinitial.end(); ++itr)
        // {
        //     std::cout <<' '<<*itr;
        // }
        // std::cout<<'\n';

        // Update state variables
        markerX_old = markerX_cur;
        markerY_old = markerY_cur;
        // old_angle_j1 = j1_vel.data*t;
        // old_angle_j2 = j2_vel.data*t;
        // std::cout << "v1:" << j1_vel_old << " v2:" << j2_vel_old << std::endl;
        // std::cout << "updated variables" << std::endl;
        
        // Publish ds, dr vectors to store
            // Convert to Float64multiarray
            ds_msg.data.clear();
            ds_msg.data.push_back(ds[0]);
            ds_msg.data.push_back(ds[1]);
            dr_msg.data.clear();
            dr_msg.data.push_back(dr[0]);
            dr_msg.data.push_back(dr[1]);
            // publish
            ds_pub.publish(ds_msg);
            dr_pub.publish(dr_msg);
            
        // publish status msg
        status_pub.publish(status);
        
        //Increase iterator 
        // std::cout <<"iterator:" << it <<std::endl;
        it++;
        ros::spinOnce();
        r.sleep();
    }

    // Commanding 0 velocity to robot 
    j1_vel.data = 0.0;
    j2_vel.data = 0.0;
    j1_pub.publish(j1_vel);
    j2_pub.publish(j2_vel);

    std::cout<<"Initial Movements Complete"<<std::endl;

    // Declare ROS Msg Arrays
    std_msgs::Float32MultiArray dSmsg;
    dSmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dSmsg.layout.dim[0].label = "dS_elements";
    dSmsg.layout.dim[0].size = dSinitial.size();
    dSmsg.layout.dim[0].stride = 1;
    dSmsg.data.clear();
    
    std_msgs::Float32MultiArray dRmsg;
    dRmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    dRmsg.layout.dim[0].label = "dR_elements";
    dRmsg.layout.dim[0].size = dRinitial.size();
    dRmsg.layout.dim[0].stride = 1;
    dRmsg.data.clear();

    std_msgs::Float32MultiArray qhatmsg;
    qhatmsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    qhatmsg.layout.dim[0].label = "qhat_elements";
    qhatmsg.layout.dim[0].size = qhat.size();
    qhatmsg.layout.dim[0].stride = 1;
    qhatmsg.data.clear();

    // std::cout << "Declared ROS msg arrays" <<std::endl;

    // Push data to ROS Msg
    for(std::vector<float>::iterator itr = dSinitial.begin(); itr != dSinitial.end(); ++itr){
        // std::cout <<*itr<<std::endl;
        dSmsg.data.push_back(*itr);
    }
    
    for(std::vector<float>::iterator itr = dRinitial.begin(); itr != dRinitial.end(); ++itr){
        // std::cout <<*itr<<std::endl;
        dRmsg.data.push_back(*itr);
    }

    for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
        // std::cout <<*itr<<std::endl;
        qhatmsg.data.push_back(*itr);
    }
    std::cout <<"Pushed initial data to ROS msgs"<<std::endl;

    // Compute Jacobian
    it = 0;
    encoderless_vs::energyFuncMsg msg;
    while(it < window){
        // Service request data
        msg.request.gamma = gamma;
        msg.request.it = it;
        msg.request.dS = dSmsg;
        msg.request.dR = dRmsg;
        msg.request.qhat = qhatmsg;

        // call compute energy functional
        energyClient.call(msg);

        // Populating service response
        std::vector<float> qhatdot = msg.response.qhat_dot.data;
        // std::cout<<"qhatdot:";
        // for(std::vector<float>::iterator itr = qhatdot.begin(); itr!=qhatdot.end(); ++itr){
        //     std::cout<<*itr<<",";
        // }
        // std::cout<<std::endl;

        //  Jacobian update
        // std::cout<<"size of qhat:"<<qhat.size()<<std::endl;
        for(int i = 0; i<qhat.size(); i++){
            qhat[i] = qhat[i] + qhatdot[i]; // Updating each element of Jacobian
        }
        // std::cout<<"Updated Jacobian vector:";

        // Push updated Jacobian vector to ROS Msg
        qhatmsg.data.clear();
        for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
            // std::cout <<*itr<<",";
            qhatmsg.data.push_back(*itr);
        }
        // std::cout<< std::endl;

        // Publish J value to store
        std_msgs::Float32 J;
        J.data = msg.response.J;
        J_pub.publish(J);
        
        it++;
    }
    std::cout <<"Initial Estimation Completed" << std::endl;

// ----------------------------- Start Servoing ---------------------------------- 
    // Compute error and error norm at start of servoing
    error[0] = eemarkerX - goal[0];
    error[1] = eemarkerY - goal[1];
    std::complex<float> temp (error[0],error[1]);
    err = sqrt(std::norm(temp));
    // std::cout<<"error:"<<error[0]<<","<<error[1];
    // std::cout<<" norm:"<<err<<std::endl;
    
    std::cout<<"Entering control loop"<<std::endl;
    
    status.data = 2;
    status_pub.publish(status);
    
    while(err >= thresh){    // convergence condition
        // error norm "err" is always positive
        // compute current error & norm
        error[0] = eemarkerX - goal[0];
        error[1] = eemarkerY - goal[1];

        // error[0] = 0;
        // error[1] = 10;
        std::complex<float> err_temp (error[0],error[1]);
        err = sqrt(std::norm(err_temp));

        // Publish current error
        err_msg.data.clear();
        err_msg.data.push_back(error[0]);
        err_msg.data.push_back(error[1]);
        err_pub.publish(err_msg);

        // Generate velocity
        // Convert qhat vector into matrix format
        Eigen::MatrixXf Qhat(2,2);
        
        // Frame rotation
        Eigen::MatrixXf rot(2,2);
        rot.row(0) << -1, 0;
        rot.row(1) << 0, 1;

        int row_count = 0;
        int itr = 0;
        while(row_count<2){
            Qhat.row(row_count) << qhat[itr], qhat[itr+1];
            row_count = row_count + 1;
            itr = itr + 2;
        }
        Eigen::Vector2f error_vec(error[0], error[1]);
        Eigen::Vector2f j_vel;
        j_vel = lam*(Qhat.inverse())*(error_vec);

        // Publish velocity to robot
        j1_vel.data = j_vel[0];
        j2_vel.data = j_vel[1];
        // std::cout<<"Joint Vel 1: "<<j_vel[0]<<std::endl;
        // std::cout<<"Joint vel 2: "<<j_vel[1]<<std::endl;
        j1_pub.publish(j1_vel);
        j2_pub.publish(j2_vel);
        
        // Compute change in state
        ds.clear();
        ds.push_back((markerX_cur - markerX_old));
        ds.push_back((markerY_cur - markerY_old));
        
        dr.clear();
        dr.push_back((j1_vel.data*t));
        dr.push_back((j2_vel.data*t));
        
        // Publish ds, dr vectors to store
            // Convert to Float64multiarray
            ds_msg.data.clear();
            ds_msg.data.push_back(ds[0]);
            ds_msg.data.push_back(ds[1]);
            dr_msg.data.clear();
            dr_msg.data.push_back(dr[0]);
            dr_msg.data.push_back(dr[1]);
            // publish
            ds_pub.publish(ds_msg);
            dr_pub.publish(dr_msg);

        // Update dSinitial and dRinitial
        dSinitial[0] = ds[0];
        dSinitial[1] = ds[1];
        // std::cout<<"ds:"<<std::endl;
        // for(std::vector<float>::iterator itr = dSinitial.begin(); itr!=dSinitial.end(); ++itr){
            // std::cout<<*itr<<", ";
        // }
        // std::cout<<std::endl;
        std::rotate(dSinitial.begin(), dSinitial.begin()+2, dSinitial.end());
        // std::cout<<"ds rotated:"<<std::endl;
        // for(std::vector<float>::iterator itr = dSinitial.begin(); itr!=dSinitial.end(); ++itr){
            // std::cout<<*itr<<", ";
        // }
        // std::cout<<std::endl;
        // std::cout<<"Size of dRinitial: "<<dRinitial.size()<<std::endl;
        dRinitial[0] = dr[0];
        dRinitial[1] = dr[1];
        // std::cout<<"Size of dRinitial after update: "<<dRinitial.size()<<std::endl;
        // std::cout<<"dr:"<<std::endl;
        // for(std::vector<float>::iterator itr = dRinitial.begin(); itr!=dRinitial.end(); ++itr){
            // std::cout<<*itr<<" ,";
        // }
        // std::cout<<std::endl;
        // std::rotate(dRinitial.begin(), dRinitial.begin()+2, dRinitial.end());
        // std::cout<<"dr rotated:"<<std::endl;
        // for(std::vector<float>::iterator itr = dRinitial.begin(); itr!=dRinitial.end(); ++itr){
            // std::cout<<*itr<<" ,";
        // }
        // std::cout<<std::endl;
        // std::cout<<"Size of dRinitial after rotate: "<<dRinitial.size()<<std::endl;
        
        // Update state variables
        // j1_vel_old = j1_vel.data;
        // j2_vel_old = j2_vel.data;
        markerX_old = markerX_cur;
        markerY_old = markerY_cur;

        // Compute Jacobian update
        // converting vectors to ros msg for service
        dSmsg.data.clear();
        for(std::vector<float>::iterator itr = dSinitial.begin(); itr != dSinitial.end(); ++itr){
            // std::cout <<*itr<<std::endl;
            dSmsg.data.push_back(*itr);
        }
        
        dRmsg.data.clear();
        for(std::vector<float>::iterator itr = dRinitial.begin(); itr != dRinitial.end(); ++itr){
            // std::cout <<*itr<<std::endl;
            dRmsg.data.push_back(*itr);
        }

        qhatmsg.data.clear();
        // std::cout<<"size of qhat: "<<qhat.size()<<std::endl;
        for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
            // std::cout <<*itr<<std::endl;
            qhatmsg.data.push_back(*itr);
        }

        // populating request data
        msg.request.gamma = gamma2;
        msg.request.it = window-1;
        msg.request.dS = dSmsg;
        msg.request.dR = dRmsg;
        msg.request.qhat = qhatmsg;

        // Call energy functional service
        energyClient.call(msg);
        
        // Populate service response
        std::vector<float> qhatdot = msg.response.qhat_dot.data;

        // Update Jacobian
        for(int i = 0; i<qhat.size(); i++){
            qhat[i] = qhat[i] + qhatdot[i]; // Updating each element of Jacobian
        }
        // std::cout<<"Updated Jacobian vector:";

        // Push updated Jacobian vector to ROS Msg
        qhatmsg.data.clear();
        for(std::vector<float>::iterator itr = qhat.begin(); itr != qhat.end(); ++itr){
            // std::cout <<*itr<<",";
            qhatmsg.data.push_back(*itr);
        }
        // std::cout<< std::endl;

        // Publish J
        std_msgs::Float32 J;
        J.data = msg.response.J;
        J_pub.publish(J);
        
        // Publish status msg
        status_pub.publish(status);

        // Refresh control loop variables, sleep to maintain control rate
        ros::spinOnce();
        r.sleep();
    }
    
    // Commanding 0 velocity to robot 
    j1_vel.data = 0.0;
    j2_vel.data = 0.0;
    j1_pub.publish(j1_vel);
    j2_pub.publish(j2_vel);

    std::cout<<"Servoing Complete"<<std::endl;
    status.data = -1;
    status_pub.publish(status);

    // Shutdown


    ros::spin();
    return 0;
}