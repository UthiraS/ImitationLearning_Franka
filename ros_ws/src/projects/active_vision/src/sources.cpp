#include <active_vision/sources.h>

bool ROSCheck(string type, string name)
{
  bool status = true;
  if (type == "NODE")
  {
    // Get the list of available nodes and search in them
    vector<string> nodeList;
    ros::master::getNodes(nodeList);
    status = find(nodeList.begin(), nodeList.end(), name) != nodeList.end();
  }
  else if (type == "TOPIC")
  {
    // Get the list of available topics and search in them
    ros::master::V_TopicInfo topicList;
    ros::master::getTopics(topicList);
    status = false;
    for (int i = 0; i < topicList.size(); i++)
    {
      if (topicList[i].name == name)
      {
        status = true;
        break;
      }
    }
  }
  else if (type == "SERVICE")
  {
    // Check if the service is running
    status = ros::service::exists(name, false);
  }
  if (!status)
    ROS_INFO_STREAM("Waiting for " << type << " : " << name << "...");
  return status;
}

EnvSource::EnvSource() {}
EnvSource::~EnvSource() {}

void EnvSource::setup(ros::NodeHandle *nh)
{
  signal(SIGINT, dieWithGrace);
  performChecks();
  setupRosComms(nh);
  setupProjectionMat();
}

void EnvSource::performChecks()
{
  bool allOK = false;
  while(!allOK)
  {
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    allOK  = true;
    allOK *= ROSCheck("SERVICE","/graspSynthesis/graspCalculate"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","/graspSynthesis/graspVisualize"); if(!allOK) continue;
    // allOK *= ROSCheck("TOPIC","/gazebo/model_states"); if(!allOK) continue;
  }
}

void EnvSource::setupRosComms(ros::NodeHandle *nh)
{
  graspClient = nh->serviceClient<active_vision::graspSRV>("/graspSynthesis/graspCalculate");
  graspVisClient = nh->serviceClient<active_vision::graspVisSRV>("/graspSynthesis/graspVisualize");
  cameraDepth = 2;
  cameraName = "/camera/depth/points";
}

void EnvSource::setupProjectionMat()
{
  projectionMat.resize(3,4);
}

void SimSource::performChecks()
{
  EnvSource::performChecks();
  bool allOK = false;
  while(!allOK)
  {
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    allOK  = true;
    allOK *= ROSCheck("NODE","/gazebo"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","/gazebo/set_model_state"); if(!allOK) continue;
    allOK *= ROSCheck("TOPIC","/camera/depth/points"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","/gazebo/get_model_state"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","/gazebo/spawn_sdf_model"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","/gazebo/delete_model"); if(!allOK) continue;
  }
}

void SimSource::setupRosComms(ros::NodeHandle *nh)
{
  EnvSource::setupRosComms(nh);
  pubObjPose = nh->advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 1);
  gazeboSpawnModel = nh->serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
  gazeboCheckModel = nh->serviceClient<gazebo_msgs::GetModelState> ("/gazebo/get_model_state");
  gazeboDeleteModel = nh->serviceClient<gazebo_msgs::DeleteModel> ("/gazebo/delete_model");
}

void SimSource::setupProjectionMat()
{
  EnvSource::setupProjectionMat();
  projectionMat << 554.254691191187, 0.0, 320.5, 0.0,
                     0.0, 554.254691191187, 240.5, 0.0,
                     0.0, 0.0, 1.0, 0.0;
}

void FrankaSimSource::performChecks()
{
  SimSource::performChecks();
  bool allOK = false;
  while(!allOK)
  {
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    allOK  = true;
    allOK *= ROSCheck("SERVICE","get_pose"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","move_to_pose"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","move_to_joint_space"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","cartesian_move"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","inverse_kinematics_collision_check"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","relaxed_ik/solve_pose"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","set_velocity_scaling"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","add_collision_object"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","gripPosServer"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","set_constraints"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","clear_constraints"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","move_to_named_state"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","one_joint_with_time"); if(!allOK) continue;
  }
}

void FrankaSimSource::setupRosComms(ros::NodeHandle *nh)
{
  SimSource::setupRosComms(nh);
  getPoseClient = nh->serviceClient<moveit_planner::GetPose>("get_pose");
  poseClient = nh->serviceClient<moveit_planner::MovePose>("move_to_pose");
  jsClient = nh->serviceClient<moveit_planner::MoveJoint>("move_to_joint_space");
  cartMoveClient = nh->serviceClient<moveit_planner::MoveCart>("cartesian_move");
  // IKClient =  nh->serviceClient<moveit_planner::Inv>("inverse_kinematics_collision_check");
  IKClient = nh->serviceClient<relaxed_ik_ros1::IKPose>("relaxed_ik/solve_pose");
  velScalingClient = nh->serviceClient<moveit_planner::SetVelocity>("set_velocity_scaling");
  collisionClient = nh->serviceClient<moveit_planner::AddCollision>("add_collision_object");
  gripperPosClient = nh->serviceClient<franka_pos_grasping_gazebo::GripPos>("gripPosServer");
  setConstClient = nh->serviceClient<moveit_planner::SetConstraints>("set_constraints");
  clearConstClient = nh->serviceClient<std_srvs::Empty>("clear_constraints");
  namedStateClient = nh->serviceClient<moveit_planner::MoveNamedState>("move_to_named_state");
  oneJointWithTimeClient = nh->serviceClient<moveit_planner::SetJointWithTime>("one_joint_with_time");
  if (!namedStateClient.exists())
    {
        ROS_ERROR("The service 'move_to_named_state' is not running!");
    }
    else
    {
        ROS_INFO("The service 'move_to_named_state' is running!");
    }
  listener.waitForTransform("panda_link0","camera_optical_link",ros::Time(0),ros::Duration(1.0));
  listener.waitForTransform("camera_optical_link","panda_link0",ros::Time(0),ros::Duration(1.0));
}

void FrankaSource::performChecks()
{
  EnvSource::performChecks();
  bool allOK = false;
  while(!allOK)
  {
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    allOK  = true;
    allOK *= ROSCheck("TOPIC","/camera/depth/color/points"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","get_pose"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","move_to_pose"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","move_to_joint_space"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","cartesian_move"); if(!allOK) continue;
    // allOK *= ROSCheck("SERVICE","inverse_kinematics_collision_check"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","relaxed_ik/solve_pose"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","add_collision_object"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","set_constraints"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","clear_constraints"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","move_to_named_state"); if(!allOK) continue;
    allOK *= ROSCheck("SERVICE","one_joint_with_time"); if(!allOK) continue;
  }
}

void FrankaSource::setupRosComms(ros::NodeHandle *nh)
{
  EnvSource::setupRosComms(nh);
  cameraDepth = 2;
  cameraName = "/camera/depth/color/points";
  // getPoseClient = nh->serviceClient<moveit_planner::GetPose>("get_pose");
  poseClient = nh->serviceClient<moveit_planner::MovePose>("move_to_pose");
  jsClient = nh->serviceClient<moveit_planner::MoveJoint>("move_to_joint_space");
  cartMoveClient = nh->serviceClient<moveit_planner::MoveCart>("cartesian_move");
  // IKClient =  nh->serviceClient<moveit_planner::Inv>("inverse_kinematics_collision_check");
  IKClient = nh->serviceClient<relaxed_ik_ros1::IKPose>("relaxed_ik/solve_pose");
  velScalingClient = nh->serviceClient<moveit_planner::SetVelocity>("set_velocity_scaling");
  collisionClient = nh->serviceClient<moveit_planner::AddCollision>("add_collision_object");
  gripperPosClient = nh->serviceClient<franka_pos_grasping_gazebo::GripPos>("gripPosServer");
  setConstClient = nh->serviceClient<moveit_planner::SetConstraints>("set_constraints");
  clearConstClient = nh->serviceClient<std_srvs::Empty>("clear_constraints");
  namedStateClient = nh->serviceClient<moveit_planner::MoveNamedState>("move_to_named_state");
  oneJointWithTimeClient = nh->serviceClient<moveit_planner::SetJointWithTime>("one_joint_with_time");
  if (!namedStateClient.exists())
    {
        ROS_ERROR("The service 'move_to_named_state' is not running!");
    }
  else
    {
        ROS_INFO("The service 'move_to_named_state' is running!");
    }
}

void FrankaSource::setupProjectionMat()
{
  EnvSource::setupProjectionMat();
  projectionMat << 383.28009033203125, 0.0, 323.4447021484375, 0.0,
                     0.0, 383.28009033203125, 237.4062042236328, 0.0,
                     0.0, 0.0, 1.0, 0.0;
}