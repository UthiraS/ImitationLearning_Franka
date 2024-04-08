#pragma once

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <active_vision/graspSRV.h>
#include <active_vision/graspVisSRV.h>
#include <moveit_planner/GetPose.h>
#include <moveit_planner/MoveCart.h>
#include <moveit_planner/MovePose.h>
#include <moveit_planner/MoveJoint.h>
#include <moveit_planner/Inv.h>
#include <moveit_planner/SetVelocity.h>
#include <moveit_planner/AddCollision.h>
#include <moveit_planner/SetConstraints.h>
#include <moveit_planner/MoveNamedState.h>
#include <moveit_planner/SetJointWithTime.h>
#include <relaxed_ik_ros1/IKPose.h>
#include <std_srvs/Empty.h>
#include <franka_pos_grasping_gazebo/GripPos.h>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <active_vision_tools/commonAV.h>

using namespace std;

/// @brief Check if a ROS NODE/TOPIC/SERVICE exists
/// @param type What type of ROS object is being examined: ["NODE", "TOPIC", "SERVICE"]
/// @param name The name of the ROS object
/// @return Whether the ROS object exists
bool ROSCheck(string type, string name);

class EnvSource{
public:
  int cameraDepth;
  string cameraName;
  EnvSource();
  ~EnvSource();
  virtual void setup(ros::NodeHandle *nh);
  Eigen::MatrixXf getProjectionMat() { return projectionMat; };
  ros::ServiceClient graspClient;
  ros::ServiceClient graspVisClient;
  ros::ServiceClient getPoseClient;
  ros::ServiceClient poseClient;
  ros::ServiceClient jsClient;
  ros::ServiceClient IKClient;
  ros::ServiceClient cartMoveClient;
  ros::ServiceClient velScalingClient;
  ros::ServiceClient gripperPosClient;
  ros::ServiceClient collisionClient;
  ros::ServiceClient setConstClient;
  ros::ServiceClient clearConstClient;
  ros::ServiceClient namedStateClient;
  ros::ServiceClient oneJointWithTimeClient;
  ros::Publisher pubObjPose;            // Publisher : Camera/Objects pose
  
  ros::ServiceClient gazeboSpawnModel;  // Service : Spawn Model
  ros::ServiceClient gazeboCheckModel;  // Service : Check Model
  ros::ServiceClient gazeboDeleteModel; // Service : Delete Model
  tf::TransformListener listener;
protected:
  Eigen::MatrixXf projectionMat; // Camera projection matrix
  virtual void performChecks();
  virtual void setupRosComms(ros::NodeHandle *nh);
  virtual void setupProjectionMat();
};

class SimSource : public EnvSource{
public:
  SimSource() {};
  ~SimSource() {};
protected:
  virtual void performChecks();
  virtual void setupRosComms(ros::NodeHandle *nh);
  virtual void setupProjectionMat();
};

class FrankaSimSource : public SimSource{
public:
  FrankaSimSource() {};
  ~FrankaSimSource() {};
protected:
  virtual void performChecks();
  virtual void setupRosComms(ros::NodeHandle *nh);
};

class FrankaSource : public EnvSource{
public:
  FrankaSource() {};
  ~FrankaSource() {};
protected:
  virtual void performChecks();
  virtual void setupRosComms(ros::NodeHandle *nh);
  virtual void setupProjectionMat();
};