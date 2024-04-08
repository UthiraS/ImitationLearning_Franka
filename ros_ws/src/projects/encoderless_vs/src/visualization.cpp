#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/types.hpp>

#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

float eeMarkerX;
float eeMarkerY;

sensor_msgs::Image curImg;

void markerCallback(const std_msgs::Float64MultiArray &msg ){
    eeMarkerX = msg.data.at(0);
    eeMarkerY = msg.data.at(1);
}

void camCallback(const sensor_msgs::Image& img){
    curImg = img;
    std::cout<<"received image"<<std::endl;
}

int main(int argc, char **argv){
    std::cout<<"Entering vis"<<std::endl;

    // ROS initialization
    ros::init(argc, argv, "visualizer_node");
    ros::NodeHandle n;
    ros::Duration(3).sleep(); // waiting for services and Gazebo

    // Get goal position
    float goal_x;
    float goal_y;
    n.getParam("vsbot/control/goal_pos_x",goal_x);
    n.getParam("vsbot/control/goal_pos_y",goal_y);

    // Initialize subscribers
    ros::Subscriber marker_sub = n.subscribe("segmentation_marker",1,markerCallback);
    ros::Subscriber cam_sub = n.subscribe("vsbot/camera1/image_raw",1,camCallback);

    // Initialize publishers
    ros::Publisher goal_img_pub = n.advertise<sensor_msgs::Image>("/vsbot/robot_vis",1);
    std::cout<<"Published vis"<<std::endl;
    
    ros::Rate r{30};
    while(ros::ok()){
        // ------------Add goal marker to curImg------------    
        //convert msg to cv image matrix
        cv_bridge::CvImagePtr cv_ptr;
        std::cout<<"Inside vis loop"<<std::endl;
        try{
            cv_ptr = cv_bridge::toCvCopy(curImg, sensor_msgs::image_encodings::MONO8);
            // cv_ptr->encoding ="bgr8";
        }
        catch(cv_bridge::Exception& e){
            // std::cout<<e.what()<<std::endl;
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return 1;
        }
        std::cout<<"Converted ROS image"<<std::endl;
        // Declare colors
        const cv::Scalar red(0,0,255);
        const cv::Scalar green(0,255,0);
        const cv::Scalar blue(255,0,0);

        // Declare cv point for goal
        cv::Point goal_pose(goal_x,goal_y);

        // Draw goal point on image
        int rad = 4;
        int thickness = 2;
        cv::circle(cv_ptr->image, goal_pose, rad, red, thickness);

        // --------Convert back to ROS msg and publish to robot_vis----
        goal_img_pub.publish(cv_ptr->toImageMsg());
        ros::spinOnce();
        r.sleep();
    }

    ros::spin();
    return 0;
}