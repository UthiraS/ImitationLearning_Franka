#include <active_vision/environment.h>
#include <active_vision/toolViewPointCalc.h>
#include <active_vision/toolVisualization.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <active_vision/heuristicPolicySRV.h>

// Typedef for convinience
typedef pcl::PointCloud<pcl::PointXYZRGB> ptCldColor;

int visualize = 0;
pcl::PointXYZ table;
ros::ServiceClient IKClient;
std::string simulationMode;
Eigen::MatrixXf projectionMat;

Eigen::Affine3f tfCamera(std::vector<double> &pose){
  std::vector<double> cartesian = {0,0,0,0,0,0};
  cartesian[0] = ::table.x+pose[0]*sin(pose[2])*cos(pose[1]);
  cartesian[1] = ::table.y+pose[0]*sin(pose[2])*sin(pose[1]);
  cartesian[2] = ::table.z+pose[0]*cos(pose[2]);
  cartesian[3] = 0;
  cartesian[4] = M_PI/2-pose[2];
  cartesian[5] = M_PI+pose[1];

  Eigen::Affine3f tfKinOptGaz,tfGazWorld;
  tfKinOptGaz = pcl::getTransformation(0,0,0,-M_PI/2,0,-M_PI/2);
  tfGazWorld = pcl::getTransformation(cartesian[0],cartesian[1],cartesian[2],\
                                      cartesian[3],cartesian[4],cartesian[5]);
  return(tfGazWorld * tfKinOptGaz);
}

geometry_msgs::Pose viewsphereToFranka(std::vector<double> &pose){

  Eigen::Matrix3f rotMat;
  rotMat = Eigen::AngleAxisf(M_PI+pose[1], Eigen::Vector3f::UnitZ()) * Eigen:: AngleAxisf(M_PI/2-pose[2], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX());

  // Incorporating the camera translation offset
  Eigen::Matrix4f tfMat; tfMat.setIdentity();
  tfMat.block<3,3>(0,0) = rotMat;
  tfMat(0,3) = ::table.x+pose[0]*sin(pose[2])*cos(pose[1]);
  tfMat(1,3) = ::table.y+pose[0]*sin(pose[2])*sin(pose[1]);
  tfMat(2,3) = ::table.z+pose[0]*cos(pose[2]);

  tfMat *= pcl::getTransformation(0,0,0,0,-M_PI/2,M_PI).matrix();
  Eigen::Matrix4f cameraOffset; cameraOffset.setIdentity();
  cameraOffset(0,3) = -0.037796115635;
  cameraOffset(1,3) = +0.0298131982299;
  cameraOffset(2,3) = -0.059671236405;
  if(tfMat(0,0) < 0) tfMat *= pcl::getTransformation(0,0,0,0,0,M_PI).matrix();
  tfMat *= cameraOffset;

  if(::simulationMode == "FRANKA"){
    tfMat *= pcl::getTransformation(0,0,0,0,0,M_PI/4).matrix();
  }

  Eigen::Quaternionf quat(tfMat.block<3,3>(0,0));

  geometry_msgs::Pose p;
  p.position.x = tfMat(0,3);  p.position.y = tfMat(1,3);  p.position.z = tfMat(2,3);
  p.orientation.x = quat.x(); p.orientation.y = quat.y(); p.orientation.z = quat.z(); p.orientation.w = quat.w();

  return p;
}

bool checkCameraOK(std::vector<double> &pose){
  bool check1 = checkValidPose(pose);
  bool check2 = true;
  if(::simulationMode != "SIMULATION"){
    geometry_msgs::Pose p = viewsphereToFranka(pose);
    check2 = checkFrankReach(::IKClient,p) || checkFrankReach(::IKClient,p);
  }
  return (check1 && check2);
}

void cleanUnexp(ptCldColor::Ptr obj, ptCldColor::Ptr unexp){
  pcl::PointXYZRGB minPtObj, maxPtObj;
  pcl::getMinMax3D(*obj, minPtObj, maxPtObj);

  ptCldColor::ConstPtr cPtrUnexp{unexp};
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cPtrUnexp);
  pass.setFilterFieldName("z"); pass.setFilterLimits(minPtObj.z,maxPtObj.z + 0.05);
  pass.filter(*unexp);
}

double findVisibleUnexp(ptCldColor::Ptr obj, ptCldColor::Ptr unexp, std::vector<double> &pose, cv::Mat &res){
  Eigen::Affine3f tf = tfCamera(pose);
  ptCldColor::Ptr tempObj{new ptCldColor};
  ptCldColor::Ptr tempUnexp{new ptCldColor};
  pcl::transformPointCloud(*obj, *tempObj, homoMatTranspose(tf));
  pcl::transformPointCloud(*unexp, *tempUnexp, homoMatTranspose(tf));

  cv::Mat projObj = cv::Mat::zeros(480,640,CV_8UC1);
  cv::Mat projUnexp = cv::Mat::zeros(480,640,CV_8UC1);

  Eigen::Vector4f v4fTemp;
  Eigen::Vector3f proj;

  for(int i = 0; i < tempUnexp->width; i++){
    v4fTemp = tempUnexp->points[i].getVector4fMap();
    proj = ::projectionMat*v4fTemp;
    proj = proj/proj[2];
    proj[0] = (round(proj[0])-1);
    proj[1] = (round(proj[1])-1);
    if(proj[0] >= 0 && proj[0] < 640 && proj[1] >= 0 && proj[1] < 480){
      uchar &intensity = projUnexp.at<uchar>(proj[1],proj[0]);
      intensity = 255;
    }
  }
  for(int i = 0; i < tempObj->width; i++){
    v4fTemp = tempObj->points[i].getVector4fMap();
    proj = ::projectionMat*v4fTemp;
    proj = proj/proj[2];
    proj[0] = (round(proj[0])-1);
    proj[1] = (round(proj[1])-1);
    if(proj[0] >= 0 && proj[0] < 640 && proj[1] >= 0 && proj[1] < 480){
      uchar &intensity = projObj.at<uchar>(proj[1],proj[0]);
      intensity = 255;
    }
  }

  int sA1 = 9, sA2 = 9;
  cv::Mat element1 = cv::getStructuringElement(0, cv::Size(2*sA1+1,2*sA1+1), cv::Point(sA1,sA1));
  cv::Mat element2 = cv::getStructuringElement(0, cv::Size(2*sA2+1,2*sA2+1), cv::Point(sA2,sA2));
  cv::morphologyEx(projObj,projObj,3,element1);
  cv::morphologyEx(projUnexp,projUnexp,3,element2);

  cv::Mat sub;
  cv::subtract(projUnexp, projObj, sub);

  double s = cv::sum(sub)[0]/255;

	cv::Rect roi;
  roi.x = 640/6;
  roi.y = 480/6;
  roi.width = 640 - (640/6*2);
  roi.height = 480 - (480/6*2);

  cv::hconcat(projObj(roi), projUnexp(roi), res);
  cv::hconcat(res, sub(roi), res);
  cv::resize(res, res, cv::Size(), 0.35, 0.35);
  cv::Scalar value(255);
  copyMakeBorder(res, res, 2, 2, 2, 2, cv::BORDER_CONSTANT, value);

  return(s);
}

std::vector<double> findVisibleUnexp8Dir(ptCldColor::Ptr obj, ptCldColor::Ptr unexp, std::vector<std::vector<double>> &pose, int &mode){
  std::vector<double> res;
  double temp;
  std::vector<double> newPose;
  cv::Mat single, final;
  for(int i = 1; i <= 8; i++){
    newPose = calcExplorationPose(pose.back(),i,mode);
    if(checkCameraOK(newPose)){
      temp = findVisibleUnexp(obj,unexp,newPose,single);
      cv::Size s = single.size();
      // std::cout << s.height << "," << s.width << std::endl;
      cv::putText(single, //target image
            dirLookup[i]+":"+std::to_string(temp), //text
            cv::Point(5, 20), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            0.5,
            cv::Scalar(255), //font color
            1);
    }else{
      temp = 0;
      single = cv::Mat::zeros(116,453,CV_8UC1);
    }
    if(i > 1) cv::vconcat(final, single, final);
    else final = single;

    res.push_back(temp);
  }
  if(::visualize == 1){
    cv::imshow("Projection",final);
  	cv::moveWindow("Projection",20,20);
  	cv::waitKey(200);
  }
  return(res);
}

bool heuristicPolicy(active_vision::heuristicPolicySRV::Request  &req,
                     active_vision::heuristicPolicySRV::Response &res){

  static ptCldColor::Ptr obj{new ptCldColor};
  static ptCldColor::Ptr unexp{new ptCldColor};
  static std::vector<std::vector<double>> path;
  static std::vector<double> temp;
  static int mode;
  static std::vector<double> nUnexp;

  pcl::fromROSMsg(req.object, *obj);
  pcl::fromROSMsg(req.unexplored, *unexp);
  temp = req.path.data; path.clear();
  for(int i = 0; i < temp.size(); i=i+3){
    path.push_back({temp[i],temp[i+1],temp[i+2]});
  }
  mode = req.mode;

  cleanUnexp(obj,unexp);
  nUnexp = findVisibleUnexp8Dir(obj,unexp,path,mode);
  res.direction = std::max_element(nUnexp.begin(),nUnexp.end()) - nUnexp.begin();
  res.vecVisiblePointsAllDirection.data = nUnexp;

  ROS_INFO_STREAM("Heuristic Policy Service Called. Direction -> "<< dirLookup[res.direction+1]);

  return true;
}

void help(){
  std::cout << "******* Heuristic Policy Service Help *******" << std::endl;
  std::cout << "Arguments : [Visualize]" << std::endl;
  std::cout << "Visualize : 0->No, 1->Yes" << std::endl;
  std::cout << "*******" << std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "HeuristicPolicyServer");

	if(argc != 2){
    ROS_ERROR("Incorrect number of arguments.");
    help(); return(-1);
  }

  ::visualize = std::atoi(argv[1]);
	if(::visualize != 0 && ::visualize != 1) ::visualize = 0;

 	ros::NodeHandle nh;
  nh.getParam("/active_vision/simulationMode", ::simulationMode);
  std::vector<double> tableCentre;
  nh.getParam("/active_vision/environment/tableCentre", tableCentre);
  ::IKClient =  nh.serviceClient<moveit_planner::Inv>("inverse_kinematics_collision_check");
  ::table.x = tableCentre[0];
  ::table.y = tableCentre[1];
  ::table.z = tableCentre[2];

  ::projectionMat.resize(3,4);
  if(::simulationMode == "FRANKA"){
    ::projectionMat << 383.28009033203125, 0.0, 323.4447021484375, 0.0,
                       0.0, 383.28009033203125, 237.4062042236328, 0.0,
                       0.0, 0.0, 1.0, 0.0;
  }else{
    ::projectionMat << 554.254691191187, 0.0, 320.5, 0.0,
                       0.0, 554.254691191187, 240.5, 0.0,
                       0.0, 0.0, 1.0, 0.0;
  }

  ros::ServiceServer service = nh.advertiseService("/active_vision/heuristic_policy", heuristicPolicy);
  ROS_INFO("Heuristic policy service ready.");
  ros::spin();
  return(0);
 }
