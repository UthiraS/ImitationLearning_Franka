#include <active_vision/environment.h>
#include <active_vision/toolVisualization.h>
// Function to fill holes in a ordered point cloud
void holeFilling(ptCldColor::ConstPtr input, ptCldColor::Ptr output){
  ptCldColor::Ptr tempOutput{new ptCldColor};
  *output = *input;
  *tempOutput = *output;
  for(int p = 0; p < 5; p++){
    // Hole Filling
    for(int i = 0; i < output->width ; i++){
      for(int j = 0; j < output->height ; j++){
        if(i < output->width / 6 * 1)  continue;
        if(i > output->width / 6 * 5)  continue;
        if(j < output->height / 6 * 1) continue;
        if(j > output->height / 6 * 5) continue;
        int index = j*(output->width)+i;
        if(output->points[index].z == 0){
          double x,y,z,wtSum;
          x = 0;y = 0;z = 0; wtSum = 0;
          int ctr = 0;

          int var;
          int k = 20;
          // Left
          var = i - 1;
          while(i - var < k){
            if(output->points[j*(output->width)+var].z != 0){
              double wt = 1 - (0.8*(i-var))/(1.414*k);
              wtSum+=wt;
              z += wt*output->points[j*(output->width)+var].z;
              ctr++;
              break;
            }
            var--;
          }
          // Right
          var = i + 1;
          while(var - i < k){
            if(output->points[j*(output->width)+var].z != 0){
              double wt = 1 - (0.8*(var-i))/(1.414*k);
              wtSum+=wt;
              z += wt*output->points[j*(output->width)+var].z;
              ctr++;
              break;
            }
            var++;
          }
          // Top
          var = j - 1;
          while(j - var < k){
            if(output->points[var*(output->width)+i].z != 0){
              double wt = 1 - (0.8*(j-var))/(1.414*k);
              wtSum+=wt;
              z += wt*output->points[var*(output->width)+i].z;
              ctr++;
              break;
            }
            var--;
          }
          // Bottom
          var = j + 1;
          while(var - j < k){
            if(output->points[var*(output->width)+i].z != 0){
              double wt = 1 - (0.8*(var-j))/(1.414*k);
              wtSum+=wt;
              z += wt*output->points[var*(output->width)+i].z;
              ctr++;
              break;
            }
            var++;
          }

          if(ctr > 2 && wtSum > 1){
            float calcX = x / wtSum;
            float calcY = y / wtSum;
            float calcZ = z / wtSum;
            tempOutput->points[index].z = calcZ;
            tempOutput->points[index].x = (i - 323.4447021484375)*calcZ/383.28009033203125;
            tempOutput->points[index].y = (j - 237.4062042236328)*calcZ/383.28009033203125;
            tempOutput->points[index].r = 0;
            tempOutput->points[index].g = 0;
            tempOutput->points[index].b = 255;
            // ctrrr ++;
          }
        }
      }
    }
    *output = *tempOutput;
  }
}

// ******************** ENVIRONMENT CLASS FUNCTIONS START ********************
// Environment class constructor
environment::environment(ros::NodeHandle *nh){

  nh->getParam("/active_vision/simulationMode", simulationMode);
  if("SIMULATION" == simulationMode){
    _world = new SimSource();
  } else if ("FRANKASIMULATION" == simulationMode)
  {
    _world = new FrankaSimSource();
  } else if ("FRANKA" == simulationMode){
    _world = new FrankaSource();
  } else {
    _world = new EnvSource();
  }
  
  _world->setup(nh);

  // subObjectNames = nh->subscribe("/gazebo/model_states", 1, &environment::updateNames, this);
  subCameraPtCld = nh->subscribe<pcl::PointCloud<pcl::PointXYZRGB>> (_world->cameraName, _world->cameraDepth, &environment::pointCloudProcessing, this);

  // NOT USED (JUST FOR REFERENCE)
  /*subCameraRGB = nh->subscribe ("/camera/color/image_raw", 1, &environment::cbImgRgb, this);
  subCameraDepth = nh->subscribe ("/camera/depth/image_raw", 1, &environment::cbImgDepth, this);*/

  readFlag[3] = {};           // Flag used to read data from camera only when needed

  nh->getParam("/active_vision/graspSynthesis/fingerZOffset", fingerZOffset);   // Z axis offset between gripper hand and finger
  
  // Transform : Camera Optical Frame to Camera Gazebo frame
  tfCamOptGaz = pcl::getTransformation(0,0,0,-M_PI/2,0,-M_PI/2);
  // tfCamOptGaz = pcl::getTransformation(0,0,0,0,0,-M_PI/2);

  path = ros::package::getPath("active_vision");  // Path to the active_vision package folder

  readObjectsList(*nh,"/active_vision/objectsInfo/",objectDict);


  nh->getParam("/active_vision/environment/voxelGridSizeUnexp", voxelGridSizeUnexp); // Voxel Grid size for unexplored point cloud
  nh->getParam("/active_vision/environment/voxelGridSize", voxelGridSize); // Voxel Grid size for environment

  nh->getParam("/active_vision/environment/viewsphereRad", viewsphereRad);
  nh->getParam("/active_vision/optPath/kNeighbors", knn);
  nh->getParam("/active_vision/environment/ICP1", icp_dist_1);
  nh->getParam("/active_vision/environment/ICP2", icp_dist_2);
  nh->getParam("/active_vision/environment/tableCentre", tableCentre); // Co-ordinates of table centre
  nh->getParam("/active_vision/environment/internalVoxelFactor", internalVoxelFactor); // Co-ordinates of table centre
  if(simulationMode == "FRANKA") tableCentre[2] = 0.125;

  minUnexp = {0,0,0};
  maxUnexp = {0,0,0};
  nh->getParam("/active_vision/environment/scale", scale); // Scale value for unexplored point cloud generation

  nh->getParam("/active_vision/environment/addNoise", addNoise);
  nh->getParam("/active_vision/environment/depthNoise", depthNoise);
  
  octCompress = boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>>(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, false, .001, (voxelGridSize)/internalVoxelFactor, true, 30U, true, 6U));

  octCompressExt = boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>>(new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>(pcl::io::MANUAL_CONFIGURATION, false, .001, voxelGridSize, true, 30U, true, 6U));

  ros::Rate r(60);

  if(simulationMode != "SIMULATION"){
    moveit_planner::SetVelocity velscale;
    velscale.request.velScaling = 0.5;
    _world->velScalingClient.call(velscale);
    AVLOG("Constructor in Environment", logging, 1);
    moveFrankaHome();
    AVLOG("Starting reset", logging, 1);
    editMoveItCollisions("TABLE","ADD");
  }
}

// Function to reset the environment
void environment::reset(){
  // cout << "Reseting env!" << endl;
  internalEnv->clear();
  internalObject->clear();
  internalNormal->clear();
  ptrPtCldObject->clear();
  ptrPtCldUnexp->clear();
  ptrObjNormal->clear();
  configurations.clear();
  graspID = -1;
  if(simulationMode != "SIMULATION") moveFrankaHome();
}

// Store the configuration
int environment::saveConfiguration(std::string name){
  stateConfig configTemp;
  configTemp.env = *internalEnv;
  configTemp.unexp = *ptrPtCldUnexp;
  configTemp.cameraPose = lastCameraPoseViewsphere;
  configTemp.description = name;
  configTemp.unexpMin = minUnexp;
  configTemp.unexpMax = maxUnexp;
  configurations.push_back(configTemp);
  //std::cout << "State saved : " << name << std::endl;
  return configurations.size()-1;
}

// Rollback to a configuration
void environment::rollbackConfiguration(int index){
  *internalEnv = configurations[index].env;
  *ptrPtCldUnexp = configurations[index].unexp;
  lastCameraPoseViewsphere = configurations[index].cameraPose;
  minUnexp = configurations[index].unexpMin;
  maxUnexp = configurations[index].unexpMax;
  //std::cout << "Rolled back to state : " << configurations[index].description << std::endl;
}

// 1A: Callback function to point cloud subscriber
void environment::pointCloudProcessing(const ptCldColor::ConstPtr& msg){
  if(firstData){
    firstData = false;
    AVLOG("Camera ready!", logging, 1);
  }
  if(readFlag[0]==1){
    *ptrPtCldLast = *msg;
    AVLOG("PointCloudProcessing!",logging,1);
    if(addNoise == true){
      // Looping through all the points
      for(int i = 0; i < ptrPtCldLast->points.size(); i++){
        if(!isnan(ptrPtCldLast->points[i].z)){
          // if z value is not nan then add noise
          float stdDev = (ptrPtCldLast->points[i].z)*depthNoise/100;
          std::normal_distribution<float> normDistb{0,stdDev};
          float noise = normDistb(generator);
          // Truncating to 1 sigma limit
          noise = std::min(noise,stdDev); noise = std::max(-stdDev,noise);
          ptrPtCldLast->points[i].z += noise;
        }
      }
    }
    //If you get an empty pointcloud, discard it and retry.
    if(0 != ptrPtCldLast->points.size()){
      readFlag[0] = 0;
      if(simulationMode == "FRANKA") 
      {
        holeFilling(cPtrPtCldLast,ptrPtCldLastFill); 
        AVLOG("Hole Filling Done!!",logging,1);
      }
      else *ptrPtCldLastFill = *cPtrPtCldLast;
    } else {
      AVLOG("Oops, no points!", logging, 1);
    }
  }
}

// 1B: Callback function to check world state by examining # of object names
void environment::updateNames(const gazebo_msgs::ModelStates::ConstPtr& msg){
  // for(int i=0; i<(msg->name).size(); i++){
  //   std::cout << msg->name[i] << " ";
  // }
  // std::cout << (msg->name).size() << std::endl;
  nObjects = (msg->name).size();
}

// Function to set noise variables
void environment::setPtCldNoise(float num){

  depthNoise = abs(num);
  if(depthNoise != 0) addNoise = true;
  else addNoise = false;
}

// NOT USED (JUST FOR REFERENCE)
/*// 1B: Callback function to RGB image subscriber
void cbImgRgb (const sensor_msgs::ImageConstPtr& msg){
  if (readFlag[1]==1) {
    ptrRgbLast = cv_bridge::toCvShare(msg);
    readFlag[1] = 0;
  }
}
// 1C: Callback function to RGB image subscriber
void cbImgDepth (const sensor_msgs::ImageConstPtr& msg){
  if (readFlag[2]==1) {
    ptrDepthLast = cv_bridge::toCvShare(msg);
    readFlag[2] = 0;
  }
}*/

// 2A: Spawning objects in gazebo on the table centre for a given pose option and yaw
void environment::spawnObject(int objectID, int choice, float yaw)
{

  if (simulationMode == "FRANKA")
    return;

  gazebo_msgs::SpawnModel spawnObj;
  gazebo_msgs::GetModelState checkObj;
  geometry_msgs::Pose pose;

  cObject = objectID;
  cYaw = yaw;

  if (tableCentre[2] > 0)
  {
    checkObj.request.model_name = "boxOnTable";
    _world->gazeboCheckModel.call(checkObj);
    if (!checkObj.response.success)
    {
      printf("************SPAWNING BOX************\n");
      spawnBoxOnTable();
    }
  }
  checkObj.request.model_name = objectDict[objectID].description;
  _world->gazeboCheckModel.call(checkObj);
  if(checkObj.response.success){
    printf("Object already exists! Skipping to move\n");
    moveObject(objectID, choice, yaw);
    return;
  }

  pose.position.x = tableCentre[0];
  pose.position.y = tableCentre[1];
  pose.position.z = tableCentre[2]+objectDict[objectID].poses[0][0];
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  spawnObj.request.model_name = objectDict[objectID].description;

  if (objectDict[objectID].fileName.substr(0, 3) == "YCB")
  {
    std::ifstream ifs(path + "/models/ycbAV/sdf/" + objectDict[objectID].fileName + ".sdf");
    std::string sdfFile((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));
    spawnObj.request.model_xml = sdfFile;
    ifs.close();
  }
  else if(objectDict[objectID].fileName.substr(0, 3) == "GDS")
  {
    std::ifstream ifs(path + "/models/google/" + objectDict[objectID].description + "/model.sdf");
    std::string sdfFile((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));
    spawnObj.request.model_xml = sdfFile;
    ifs.close();
  }
  else
  {
    std::ifstream ifs(path + "/models/" + objectDict[objectID].fileName + "/model.sdf");
    std::string sdfFile((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));
    spawnObj.request.model_xml = sdfFile;
    ifs.close();
  }

  spawnObj.request.reference_frame = "world";
  spawnObj.request.initial_pose = pose;

  while (not spawnObj.response.success)
  {
    _world->gazeboSpawnModel.call(spawnObj);
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  }

  moveObject(objectID, choice, yaw);
}

void environment::spawnBoxOnTable(){
  gazebo_msgs::SpawnModel spawnObj;
  gazebo_msgs::GetModelState checkObj;
  geometry_msgs::Pose pose;

  pose.position.x = tableCentre[0];
  pose.position.y = tableCentre[1];
  pose.position.z = tableCentre[2];
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  spawnObj.request.model_name = "boxOnTable";

  std::ifstream ifs(path+"/models/boxOnTable/model.sdf");
  std::string sdfFile( (std::istreambuf_iterator<char>(ifs)),
                       (std::istreambuf_iterator<char>()));
  spawnObj.request.model_xml = sdfFile;

  spawnObj.request.reference_frame = "world";
  spawnObj.request.initial_pose = pose;

  checkObj.request.model_name = "boxOnTable";

  //Box should not already exist- wait for it to get deleted.
  _world->gazeboCheckModel.call(checkObj);
  while(checkObj.response.success){
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    ros::spinOnce();
    _world->gazeboCheckModel.call(checkObj);
  }
  
  _world->gazeboSpawnModel.call(spawnObj);
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  while(not checkObj.response.success){
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    ros::spinOnce();
    _world->gazeboCheckModel.call(checkObj);
  }
}

// 2B: Function to move the object. Same args as spawnObject
void environment::moveObject(int objectID, int choice, float yaw){

  if(simulationMode == "FRANKA") return;

  gazebo_msgs::GetModelState checkObj;
  checkObj.request.model_name = objectDict[objectID].description;

  if(choice >= objectDict[objectID].nPoses){
    choice = 0;
    printf("WARNING moveObject: Pose choice invalid. Setting choice to 0.\n");
  }
  //Make sure the object exists- otherwise create it
  _world->gazeboCheckModel.call(checkObj);
  ros::spinOnce();
  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  
  if(!checkObj.response.success){
    printf("Object to be moved does not exist?\n");
    spawnObject(objectID, choice, yaw);
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }
  
  for(int i=1; i<501; i++){
    if(checkObj.response.success) break;
    if(i % 100 == 0){ 
      printf("Retrying object spawn...\n");
      spawnObject(objectID, choice, yaw);
    }
    _world->gazeboCheckModel.call(checkObj);
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
  }

  //Create Matrix3x3 from Euler Angles
  tf::Matrix3x3 m_rot;
  m_rot.setEulerYPR(yaw, objectDict[objectID].poses[choice][2], objectDict[objectID].poses[choice][1]);

  // Convert into quaternion
  tf::Quaternion quat;
  m_rot.getRotation(quat);

  // Converting it to the required gazebo format
  gazebo_msgs::ModelState ModelState;
  ModelState.model_name = objectDict[objectID].description;
  ModelState.reference_frame = "world";
  ModelState.pose.position.x = tableCentre[0];
  ModelState.pose.position.y = tableCentre[1];
  ModelState.pose.position.z = tableCentre[2]+objectDict[objectID].poses[choice][0];
  ModelState.pose.orientation.x = quat.x();
  ModelState.pose.orientation.y = quat.y();
  ModelState.pose.orientation.z = quat.z();
  ModelState.pose.orientation.w = quat.w();

  // Publishing it to gazebo
  _world->pubObjPose.publish(ModelState);
  _world->gazeboCheckModel.call(checkObj);
  ros::spinOnce();
  boost::this_thread::sleep(boost::posix_time::milliseconds(500));
  //Will infinite loop if there's something wrong, but it did that
  // anyways
  if((not checkObj.response.success) or (abs(checkObj.response.pose.position.z - (tableCentre[2]+objectDict[objectID].poses[choice][0]) > 0.2))){
    printf("Object not positioned yet, pose z = %1.4f, not %1.4f, diff of %1.4f\n", checkObj.response.pose.position.z, tableCentre[2]+objectDict[objectID].poses[choice][0], abs(checkObj.response.pose.position.z - (tableCentre[2]+objectDict[objectID].poses[choice][0])));
    moveObject(objectID, choice, yaw);
  }
  tfObject.setIdentity();
  tfObject.translate(Eigen::Vector3f(-ModelState.pose.position.x,-ModelState.pose.position.y,-ModelState.pose.position.z));
  tfObject.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()));
  tfObject.translate(Eigen::Vector3f(ModelState.pose.position.x,ModelState.pose.position.y,ModelState.pose.position.z));
}

// 3: Deleting objects in gazebo
void environment::deleteObject(int objectID, int triesLeft)
{
  
  if(0 >= triesLeft){
    std::cout << "Took too long to delete object, giving up :(" << std::endl;
    throw std::invalid_argument("Out of tries to delete object!");
  }
  gazebo_msgs::GetModelState checkObj;
  gazebo_msgs::DeleteModel delObj;

  if (simulationMode == "FRANKA")
    return;

  int triesSoFar = 0;
  while(4 >= nObjects, triesSoFar <= triesLeft){
    std::cout << "Begining delete object, " << triesSoFar << " tries so far." << std::endl;
    checkObj.request.model_name = objectDict[objectID].description;
    _world->gazeboCheckModel.call(checkObj);

    // check if the object is deleted
    if (not checkObj.response.success)
    {
      std::cout << "Object does not exist for deleting" << std::endl;
      return;
    }

    // delete the object
    delObj.request.model_name = objectDict[objectID].description;
    _world->gazeboDeleteModel.call(delObj);
    
    int i = 0;
    while (not delObj.response.success && i < 100)
    {
      i = i + 1;
      std::cout << "deleting object " << delObj.request.model_name << " Status " << delObj.response.success << std::endl;
      boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }

    // wait for object to get deleted
    i = 0;
    _world->gazeboCheckModel.call(checkObj);
    while (checkObj.response.success && i < 100)
    {
      i = i + 1;
      boost::this_thread::sleep(boost::posix_time::milliseconds(50));
      _world->gazeboCheckModel.call(checkObj);
    }
    triesSoFar++;
  }
  return;
}

// 5: Update gripper (Only for visualization)
void environment::updateGripper(int index ,int choice){
  if(choice == 0){
    graspVisMsg.request.graspID = index;
    _world->graspVisClient.call(graspVisMsg);
    pcl::fromROSMsg(graspVisMsg.response.graspPtCld, *ptrPtCldGripper);
  }
}

// 6A: Function to move the camera. Args: Array of X,Y,Z,Roll,Pitch,Yaw
bool environment::moveCameraCartesian(std::vector<double> pose, bool execute){
  if(simulationMode == "SIMULATION"){
    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 rotMat;
    rotMat.setEulerYPR(pose[5], pose[4], pose[3]);

    // Convert into quaternion
    tf::Quaternion quat;
    rotMat.getRotation(quat);

    // Converting it to the required gazebo format
    gazebo_msgs::ModelState ModelState;
    ModelState.model_name = "Kinect";           // This should be the name of camera in gazebo
    ModelState.reference_frame = "world";
    ModelState.pose.position.x = pose[0];
    ModelState.pose.position.y = pose[1];
    ModelState.pose.position.z = pose[2];
    ModelState.pose.orientation.x = quat.x();
    ModelState.pose.orientation.y = quat.y();
    ModelState.pose.orientation.z = quat.z();
    ModelState.pose.orientation.w = quat.w();

    // Publishing it to gazebo
    _world->pubObjPose.publish(ModelState);
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(250));

    // Storing the camera pose
    lastCameraPoseCartesian = pose;
  }
  else if(simulationMode == "FRANKASIMULATION" || simulationMode == "FRANKA"){
    // Create Matrix3x3 from Euler Angles
    // Additional rotation of PI about Z and -PI/2 about Y so that camera frame orientation aligns with the gripper
    Eigen::Matrix3f rotMat;
    rotMat = Eigen::AngleAxisf(pose[5], Eigen::Vector3f::UnitZ()) * Eigen:: AngleAxisf(pose[4], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(pose[3], Eigen::Vector3f::UnitX());

    // Incorporating the camera translation offset
    Eigen::Matrix4f tfMat; tfMat.setIdentity();
    tfMat.block<3,3>(0,0) = rotMat;
    tfMat(0,3) = pose[0];
    tfMat(1,3) = pose[1];
    tfMat(2,3) = pose[2];

    geometry_msgs::Pose p;
    bool res = moveFranka(tfMat,"JOINT",true,execute,p);
    if(!res) return false;

    if(execute){
      // Storing the camera pose
      if(simulationMode != "FRANKA")
        boost::this_thread::sleep(boost::posix_time::milliseconds(250));
      else
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      lastCameraPoseCartesian = pose;
    }
  }
  return true;
}

// 6B: Funtion to move the Camera in a viewsphere which has the table cente as its centre
// R (Radius)
// Phi (Azhimuthal angle) -> 0 to 2*PI
// Theta (Polar Angle)) -> 0 to PI/2
bool environment::moveCameraViewsphere(std::vector<double> pose, bool execute){

  if(simulationMode == "SIMULATION"){
    //Create Matrix3x3 from Euler Angles
    tf::Matrix3x3 rotMat;
    //rotMat.setEulerYPR(M_PI+pose[1], M_PI/2-pose[2], 0);
    rotMat.setEulerYPR(M_PI+pose[1], M_PI/2-pose[2], 0);

    // Convert into quaternion
    tf::Quaternion quat;
    rotMat.getRotation(quat);

    // Converting it to the required gazebo format
    gazebo_msgs::ModelState ModelState;
    ModelState.model_name = "Kinect";           // This should be the name of camera in gazebo
    ModelState.reference_frame = "world";
    ModelState.pose.position.x = tableCentre[0]+pose[0]*sin(pose[2])*cos(pose[1]);
    ModelState.pose.position.y = tableCentre[1]+pose[0]*sin(pose[2])*sin(pose[1]);
    ModelState.pose.position.z = tableCentre[2]+pose[0]*cos(pose[2]);
    ModelState.pose.orientation.x = quat.x();
    ModelState.pose.orientation.y = quat.y();
    ModelState.pose.orientation.z = quat.z();
    ModelState.pose.orientation.w = quat.w();

    // Publishing it to gazebo
    _world->pubObjPose.publish(ModelState);
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(250));

    // Storing the camera pose
    lastCameraPoseViewsphere = pose;
    lastCameraPoseCartesian = {ModelState.pose.position.x,
                               ModelState.pose.position.y,
                               ModelState.pose.position.z,
                               0,M_PI/2-pose[2],M_PI+pose[1]};
  }
  else if(simulationMode == "FRANKASIMULATION" || simulationMode == "FRANKA"){

    // Create Matrix3x3 from Euler Angles
    // Additional rotation of PI about Z and -PI/2 about Y so that camera frame orientation aligns with the gripper
    Eigen::Matrix3f rotMat;
    rotMat = Eigen::AngleAxisf(M_PI+pose[1], Eigen::Vector3f::UnitZ()) * Eigen:: AngleAxisf(M_PI/2-pose[2], Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX());

    // Incorporating the camera translation offset
    Eigen::Matrix4f tfMat; tfMat.setIdentity();
    tfMat.block<3,3>(0,0) = rotMat;
    tfMat(0,3) = tableCentre[0]+pose[0]*sin(pose[2])*cos(pose[1]);
    tfMat(1,3) = tableCentre[1]+pose[0]*sin(pose[2])*sin(pose[1]);
    tfMat(2,3) = tableCentre[2]+pose[0]*cos(pose[2]);

    geometry_msgs::Pose p;
    bool res = moveFranka(tfMat,"JOINT",true,execute,p);
    AVLOG(to_string(res), 1, 1);
    if(!res) 
    {
    return false;
    AVLOG("MoveFranka Returned error!",logging,1);
    }
    AVLOG("MoveFranka succeeded", logging, 1);

    if(execute){
      if(simulationMode != "FRANKA")
        boost::this_thread::sleep(boost::posix_time::milliseconds(250));
      else
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      // Storing the camera pose
      // PLOG;
      lastCameraPoseViewsphere = pose;
      // PLOG;
      lastCameraPoseCartesian = {tfMat(0,3),
                                 tfMat(1,3),
                                 tfMat(2,3),
                                 0,M_PI/2-pose[2],M_PI+pose[1]};
      // PLOG;
    }
    // PLOG;
  }
  // PLOG;
  return true;
}

bool environment::moveFranka(Eigen::Matrix4f tfMat, std::string mode ,bool isCamera ,bool execute, geometry_msgs::Pose &p){

  tfMat *= pcl::getTransformation(0,0,0,0,-M_PI/2,M_PI).matrix();
  if(isCamera){
    Eigen::Matrix4f cameraOffset; cameraOffset.setIdentity();
    cameraOffset(0,3) = -0.037796115635;
    cameraOffset(1,3) = +0.0298131982299;
    cameraOffset(2,3) = -0.059671236405;
    if(tfMat(0,0) < 0) tfMat *= pcl::getTransformation(0,0,0,0,0,M_PI).matrix();
    tfMat *= cameraOffset;
  }

  if(simulationMode == "FRANKA"){
    tfMat *= pcl::getTransformation(0,0,0,0,0,M_PI/4).matrix();
    AVLOG("Attempting franka move", logging, 1);
  }

  Eigen::Quaternionf quat(tfMat.block<3,3>(0,0));


  p.position.x = tfMat(0,3);  p.position.y = tfMat(1,3);  p.position.z = tfMat(2,3);
  p.orientation.x = quat.x(); p.orientation.y = quat.y(); p.orientation.z = quat.z(); p.orientation.w = quat.w();

  if(execute){
    string display_string = to_string(p.position.x) + ", " +
      to_string(p.position.y) + ", " +
      to_string(p.position.z) + ", " +
      to_string(p.orientation.x) + ", " +
      to_string(p.orientation.y) + ", " +
      to_string(p.orientation.z) + ", " +
      to_string(p.orientation.w) + ".";
    AVLOG(display_string, logging, 1);
    // std::cout << p.position.x << "," <<
    //              p.position.y << "," <<
    //              p.position.z << "," <<
    //              p.orientation.x << "," <<
    //              p.orientation.y << "," <<
    //              p.orientation.z << "," <<
    //              p.orientation.w << std::endl;
  }

  // if(!checkFrankReach(_world->IKClient,p)){
  //   AVLOG("Grasp outside of reach", logging, 1);
  //   return false;
  // } 


  relaxed_ik_ros1::IKPose srv;
  // Assuming `p` is a geometry_msgs::Pose representing the desired pose
  srv.request.ee_poses.push_back(p); // Adapt based on your specific requirements
  std::vector<double> joint_states;
  _world->IKClient.call(srv);

      // If the service call was successful, process the response
  joint_states = srv.response.joint_state;
  string joint_string;
  for (auto joint_value : joint_states)
      {
          // For example, print each joint value
        // ROS_INFO_STREAM("Joint value: " << joint_value);
        joint_string += to_string(joint_value) ;
        
      }
  AVLOG("RELAXED IK", logging, 1);
  AVLOG(joint_string, logging, 1);   
// }
// else
// {
//    ROS_ERROR("Failed to call IK service");
//    AVLOG("IK solution not found",logging,1);
// }

  // Moveit move command.
  if(execute){
    bool res = true;
    AVLOG("Exceute is True",logging,1);
    if(isCamera){
      editMoveItCollisions("OBJECT","ADD");
      addVisibilityConstraint();
    }
    if(mode == "JOINT"){
      // AVLOG("Moving to pose", logging, 2);
      // moveit_planner::MovePose movePoseMsg;
      // movePoseMsg.request.val = p;
      // movePoseMsg.request.execute = true;
      // res = _world->poseClient.call(movePoseMsg);
      
      // moveit_planner::MoveNamedState namedState;
      // namedState.request.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3","panda_joint4","panda_joint5","panda_joint7"};
      // namedState.request.joint_positions = joint_states;
      // res = _world->namedStateClient.call(namedState);
      AVLOG("Moving to joint", logging, 2);
      moveit_planner::MoveJoint moveJointMsg;
      moveJointMsg.request.val = joint_states;
      moveJointMsg.request.execute = true;
      res = _world->jsClient.call(moveJointMsg);


      AVLOG(to_string(res), logging, 1);
    }else if(mode == "CARTESIAN"){
      AVLOG("Moving by offset", logging, 2);
      moveit_planner::MoveCart moveCartMsg;
      moveCartMsg.request.val.push_back(p);
      moveCartMsg.request.time = 0;
      moveCartMsg.request.execute = true;
      res = _world->cartMoveClient.call(moveCartMsg);

      AVLOG(to_string(res), logging, 1);
    }

    if(isCamera){
      moveGripper(0.08);
      editMoveItCollisions("OBJECT","REMOVE");
      clearAllConstraints();
    }

    ros::Duration(1).sleep();

    float frankaOffset[3] = {0.0,0.0,0.0};
    float frankaX = tfMat(0,3)-frankaOffset[0];
    float frankaY = tfMat(1,3)-frankaOffset[1];
    float frankaZ = tfMat(2,3)-frankaOffset[2];

    // std::cout << frankaX << "," <<
    //              frankaY << "," <<
    //              frankaZ << "," <<
    //              quat.x() << "," <<
    //              quat.y() << "," <<
    //              quat.z() << "," <<
    //              quat.w() << std::endl;
    AVLOG("Move finished", logging, 2);
    if((simulationMode == "FRANKA")||(simulationMode == "FRANKASIMULATION"))
    {
      AVLOG(to_string(res), logging, 1);
      return res;
    }

    moveit_planner::GetPose curPose;
    _world->getPoseClient.call(curPose);

    // std::cout << curPose.response.pose.position.x << "," <<
    //              curPose.response.pose.position.y << "," <<
    //              curPose.response.pose.position.z << "," <<
    //              curPose.response.pose.orientation.x << "," <<
    //              curPose.response.pose.orientation.y << "," <<
    //              curPose.response.pose.orientation.z << "," <<
    //              curPose.response.pose.orientation.w << std::endl;

    float positionError = sqrt(pow(curPose.response.pose.position.x-frankaX,2) +
                               pow(curPose.response.pose.position.y-frankaY,2) +
                               pow(curPose.response.pose.position.z-frankaZ,2));
    float orientationError = curPose.response.pose.orientation.x * p.orientation.x +
                             curPose.response.pose.orientation.y * p.orientation.y +
                             curPose.response.pose.orientation.z * p.orientation.z +
                             curPose.response.pose.orientation.w * p.orientation.w;

    if(positionError > 5e-03 || 1-abs(orientationError) > 1e-03) return false;
  }

  return true;
}

void environment::moveGripper(double Grasp_Width, bool grasp){
  if(simulationMode == "FRANKA"){
    if(grasp){
      actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);
      ac.waitForServer();

      franka_gripper::GraspGoal goal;
      goal.width = Grasp_Width;   // Distance between fingers [m]
      goal.speed = 0.1;           // Closing speed. [m/s]
      goal.force = 40;            // Grasping (continuous) force [N]
      goal.epsilon.inner = 0.05;  // Maximum tolerated deviation when the actual grasped width is
                                  // smaller than the commanded grasp width.
      goal.epsilon.outer = 0.05;  // Maximum tolerated deviation when the actual grasped width is
                                  // larger than the commanded grasp width.
      ac.sendGoal(goal);          // Sending the Grasp command to gripper

      bool finished_before_timeout = ac.waitForResult(ros::Duration(5));

      // if (finished_before_timeout){
      // ROS_INFO("Gripper action finished.");
      // }
      // else {
      // ROS_INFO("Gripper action did not finish before the time out.");
      // }
    }else{
      actionlib::SimpleActionClient<franka_gripper::MoveAction> ac("franka_gripper/move", true);
      ac.waitForServer();

      franka_gripper::MoveGoal goal;
      goal.width = Grasp_Width;   // Distance between fingers [m]
      goal.speed = 0.1;           // Closing speed. [m/s]
      ac.sendGoal(goal);
      bool finished_before_timeout = ac.waitForResult(ros::Duration(5));
    }
  }else if(simulationMode == "FRANKASIMULATION"){
    franka_pos_grasping_gazebo::GripPos grasp;
    grasp.request.finger_pos = Grasp_Width/2.0;
    _world->gripperPosClient.call(grasp);
    ros::Duration(1).sleep();
  }

}

void environment::moveFrankaHome(bool gripper){
  // std::cout << "MoveFrankaHome : Moving to home pose..." << std::endl;
  static moveit_planner::MoveNamedState namedState; namedState.request.name = "ready";
  _world->namedStateClient.call(namedState);
  if(gripper) moveGripper(0.08);
}

void environment::addVisibilityConstraint(){
  moveit_msgs::VisibilityConstraint visConstraint;
  visConstraint.target_radius = 0.125;

  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  geometry_msgs::PoseStamped tgtPose;
  tgtPose.header.frame_id = "panda_link0";
  pose.position.x = tableCentre[0];
  pose.position.y = tableCentre[1];
  pose.position.z = tableCentre[2];
  tgtPose.pose = pose;
  visConstraint.target_pose = tgtPose;

  visConstraint.cone_sides = 4;

  geometry_msgs::PoseStamped sensorPose;
  sensorPose.header.frame_id = "panda_hand";
  pose.position.x =  0.037796115635;
  pose.position.y = -0.0298131982299;
  pose.position.z =  0.059671236405;
  sensorPose.pose = pose;
  visConstraint.sensor_pose = sensorPose;
  visConstraint.max_view_angle = 85*M_PI/180;
  visConstraint.sensor_view_direction = visConstraint.SENSOR_Z;
  visConstraint.weight = 1.0;

  moveit_planner::SetConstraints constraintsMsg;
  constraintsMsg.request.constraints.name = "VisConstraint";
  constraintsMsg.request.constraints.joint_constraints = {};
  constraintsMsg.request.constraints.orientation_constraints = {};
  constraintsMsg.request.constraints.position_constraints = {};
  constraintsMsg.request.constraints.visibility_constraints = {visConstraint};

  // setConstClient.call(constraintsMsg);
}

void environment::addOrientationConstraint(Eigen::Affine3f tf){
  tf = tf*pcl::getTransformation(0,0,0,0,-M_PI/2,M_PI);
  Eigen::Quaternionf quat(tf.matrix().block<3,3>(0,0));

  moveit_planner::SetConstraints constraintsMsg;
  moveit_msgs::OrientationConstraint orientConstraint;
  orientConstraint.header.frame_id = "/world";
  orientConstraint.link_name = "panda_hand";
  orientConstraint.orientation.x = quat.x();
  orientConstraint.orientation.y = quat.y();
  orientConstraint.orientation.z = quat.z();
  orientConstraint.orientation.w = quat.w();
  orientConstraint.absolute_x_axis_tolerance = 0.01;
  orientConstraint.absolute_y_axis_tolerance = 0.01;
  orientConstraint.absolute_z_axis_tolerance = 0.01;
  orientConstraint.weight = 1.0;

  constraintsMsg.request.constraints.orientation_constraints = {orientConstraint};
  _world->setConstClient.call(constraintsMsg);
}

void environment::clearAllConstraints(){
  std_srvs::Empty emptyMsg;
  _world->clearConstClient.call(emptyMsg);
}

// 7: Function to read the camera data.
void environment::readCamera(){
  readFlag[0] = 1;
  while (readFlag[0]==1) {
    while(1 > subCameraPtCld.getNumPublishers()){
      AVLOG("No connected publishers?", logging, 1);
      // r.sleep();
      ros::spinOnce();
    }
    // r.sleep();
    ros::spinOnce();
    // boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    // 
    // std::cout<<"IN"<<endl;
  }
}

// 8: Function to Fuse last data with existing data
void environment::fuseLastData(){
  ptrPtCldTemp->clear();
  ICP_Tf = Eigen::Matrix4f::Identity();

  if(simulationMode == "SIMULATION"){
    // Transform : Camera Gazebo Frame to Gazebo World frame
    tfGazWorld = pcl::getTransformation(lastCameraPoseCartesian[0],lastCameraPoseCartesian[1],lastCameraPoseCartesian[2],\
                                        lastCameraPoseCartesian[3],lastCameraPoseCartesian[4],lastCameraPoseCartesian[5]);
    rotate = getTransformation(0, 0, 0, 0, 0, 0);
    // rotate = getTransformation(0, 0, 0, -cYaw, 0, 0);
    // Apply transformation
    Eigen::Affine3f tf = tfGazWorld * tfCamOptGaz * rotate;
    pcl::transformPointCloud(*ptrPtCldLast, *ptrPtCldTemp, tf);
    AVLOG("Initial transform", logging, 1);
  }else if(simulationMode == "FRANKASIMULATION"){
    tf::StampedTransform transform;
    _world->listener.lookupTransform("panda_link0", "camera_optical_link", ros::Time(0), transform);
    pcl_ros::transformPointCloud(*ptrPtCldLast, *ptrPtCldTemp, transform);
  }else{
    AVLOG("FRANKA transfrom lookup!",logging,1);
    tf::StampedTransform transform;
    _world->listener.lookupTransform("panda_link0", "camera_depth_optical_frame", ros::Time(0), transform);
    pcl_ros::transformPointCloud(*ptrPtCldLast, *ptrPtCldTemp, transform);
    ICPRegistration();
  }

  // Fuse the two pointclouds (except for the first time) and downsample again
  if(internalEnv->width == 0) *internalEnv = *ptrPtCldTemp;
  else                        *internalEnv += *ptrPtCldTemp;

  // Downsample using voxel grid
  // AVLOG("octree", logging, 1);
  // octCompress->defineBoundingBox(2);
  // octCompress->encodePointCloud(cPtrPtCldEnv, compressedData);
  // octCompress->decodePointCloud(compressedData, internalEnv);
  // octCompress->deleteTree();
  // AVLOG("octree done", logging, 1);
  // vector<PointXYZRGB,Eigen::aligned_allocator<PointXYZRGB> > r{};
  // oct.getOccupiedVoxelCenters(r);
  voxelGrid.setInputCloud(cPtrPtCldEnv);
  voxelGrid.setLeafSize(voxelGridSize/internalVoxelFactor, voxelGridSize/internalVoxelFactor, voxelGridSize/internalVoxelFactor);
  // constantFilter(internalEnv, internalEnv, &voxelGrid);
  voxelGrid.filter(*internalEnv);

  // Using pass through filter to focus on the required region
  pass.setInputCloud(cPtrPtCldEnv);
  pass.setFilterFieldName("x"); pass.setFilterLimits(0.20,0.70); pass.filter(*internalEnv);
  pass.setFilterFieldName("y"); pass.setFilterLimits(-0.25,0.25); pass.filter(*internalEnv);
  pass.setFilterFieldName("z"); pass.setFilterLimits(-0.60,0.60); pass.filter(*internalEnv);
  AVLOG("Pass filter", logging, 1);
  
  // octCompressExt->defineBoundingBox(2);
  // octCompressExt->encodePointCloud(cPtrPtCldEnv, compressedData);
  // octCompressExt->decodePointCloud(compressedData, ptrPtCldEnv);
  // octCompressExt->deleteTree();

  // AVLOG("octree 2", logging, 1);
  
  voxelGrid.setInputCloud(cPtrPtCldEnv);
  voxelGrid.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
  // constantFilter(internalEnv, ptrPtCldEnv, &voxelGrid);
  voxelGrid.filter(*ptrPtCldEnv);
  ptrPtCldCView->clear();
  *ptrPtCldCView += *ptrPtCldTemp;
  ptrPtCldTemp->clear();
  AVLOG("Finished", logging, 1);
}

void environment::ICPRegistration(){

  if(internalEnv->width == 0) return;

  ptCldColor::Ptr ptrPtCldSrc{new ptCldColor}; 
  ptCldColor::ConstPtr cPtrPtCldSrc{ptrPtCldSrc};
  *ptrPtCldSrc = *ptrPtCldTemp;
  // octCompress->encodePointCloud(cPtrPtCldSrc, compressedData);
  // octCompress->decodePointCloud(compressedData, ptrPtCldSrc);
  voxelGrid.setInputCloud(cPtrPtCldSrc);
  voxelGrid.setLeafSize(voxelGridSize/internalVoxelFactor, voxelGridSize/internalVoxelFactor, voxelGridSize/internalVoxelFactor);
  // constantFilter(ptrPtCldSrc, ptrPtCldSrc, &voxelGrid);
  voxelGrid.filter(*ptrPtCldSrc);

  ptCldColor::Ptr ptrPtCldTgt{new ptCldColor}; ptCldColor::ConstPtr cPtrPtCldTgt{ptrPtCldTgt};
  *ptrPtCldTgt = *internalEnv;




  ptCldColor::Ptr ptrPtCldSrcTemp{new ptCldColor}; 
  ptCldColor::ConstPtr cPtrPtCldSrcTemp{ptrPtCldSrcTemp};
  ptCldColor::Ptr ptrPtCldTgtTemp{new ptCldColor}; 
  ptCldColor::ConstPtr cPtrPtCldTgtTemp{ptrPtCldTgtTemp};

  // Aligning with table
  *ptrPtCldSrcTemp = *ptrPtCldSrc;
  *ptrPtCldTgtTemp = *ptrPtCldTgt;


 
  // Extracting table
  pass.setInputCloud(cPtrPtCldSrcTemp);
  pass.setFilterFieldName("z"); pass.setFilterLimits(tableCentre[2]-0.01,tableCentre[2]+0.01);  pass.filter(*ptrPtCldSrcTemp);

  pass.setInputCloud(cPtrPtCldTgtTemp);
  pass.setFilterFieldName("z"); pass.setFilterLimits(tableCentre[2]-0.01,tableCentre[2]+0.01);  pass.filter(*ptrPtCldTgtTemp);
  // std::cout << "Size of ptrPtCldSrcTemp: " << ptrPtCldSrcTemp->points.size() << std::endl;
  // std::cout << "Size of ptrPtCldTgtTemp: " << ptrPtCldTgtTemp->points.size() << std::endl;

  // Check type and size
  if (typeid(*ptrPtCldSrcTemp) == typeid(*ptrPtCldTgtTemp) && ptrPtCldSrcTemp->points.size() == ptrPtCldTgtTemp->points.size()) {
      // std::cout << "The point clouds are of the same type and have the same size." << std::endl;
  } else {
      // std::cout << "The point clouds are either of different types or different sizes." << std::endl;
  }

  // ICP to align the table
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp1;
  icp1.setInputSource(ptrPtCldSrcTemp);
  icp1.setInputTarget(ptrPtCldTgtTemp);
  icp1.setMaxCorrespondenceDistance(icp_dist_1);

  Eigen::Matrix4f Ti1 = Eigen::Matrix4f::Identity();
  icp1.setMaximumIterations(30);
  icp1.align(*ptrPtCldSrcTemp);
  Ti1 = icp1.getFinalTransformation();
  pcl::transformPointCloud(*ptrPtCldSrc, *ptrPtCldSrc, Ti1);
  pcl::transformPointCloud(*ptrPtCldTemp, *ptrPtCldTemp, Ti1);

  // Align the object
  *ptrPtCldSrcTemp = *ptrPtCldSrc;
  *ptrPtCldTgtTemp = *ptrPtCldTgt;

  pass.setInputCloud(cPtrPtCldSrcTemp);
  pass.setFilterFieldName("x"); pass.setFilterLimits(minPtObj.x-0.05,maxPtObj.x+0.05);  pass.filter(*ptrPtCldSrcTemp);
  pass.setFilterFieldName("y"); pass.setFilterLimits(minPtObj.y-0.05,maxPtObj.y+0.05);  pass.filter(*ptrPtCldSrcTemp);
  pass.setFilterFieldName("z"); pass.setFilterLimits(tableCentre[2]+0.01,1);   pass.filter(*ptrPtCldSrcTemp);

  pass.setInputCloud(cPtrPtCldTgtTemp);
  pass.setFilterFieldName("x"); pass.setFilterLimits(minPtObj.x-0.05,maxPtObj.x+0.05);  pass.filter(*ptrPtCldTgtTemp);
  pass.setFilterFieldName("y"); pass.setFilterLimits(minPtObj.y-0.05,maxPtObj.y+0.05);  pass.filter(*ptrPtCldTgtTemp);
  pass.setFilterFieldName("z"); pass.setFilterLimits(tableCentre[2]+0.01,1);   pass.filter(*ptrPtCldTgtTemp);

  pcl::registration::WarpPointXY<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr warp_fcn
    (new pcl::registration::WarpPointXY<pcl::PointXYZRGB, pcl::PointXYZRGB>);

  // Create a TransformationEstimationLM object, and set the warp to it
  pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr te
    (new pcl::registration::TransformationEstimationLM<pcl::PointXYZRGB, pcl::PointXYZRGB>);

  te->setWarpFunction(warp_fcn);

  // ICP to align the object
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp2;
  icp2.setTransformationEstimation(te);
  icp2.setInputSource(ptrPtCldSrcTemp);
  icp2.setInputTarget(ptrPtCldTgtTemp);
  icp2.setMaxCorrespondenceDistance(icp_dist_2);

  Eigen::Matrix4f Ti2 = Eigen::Matrix4f::Identity(), prev;
  icp2.setMaximumIterations(30);
  icp2.align(*ptrPtCldSrcTemp);
  Ti2 = icp2.getFinalTransformation();

  pcl::transformPointCloud(*ptrPtCldTemp, *ptrPtCldTemp, Ti2);

  ICP_Tf = Ti1 * Ti2;
}

// 9: Extracting the major plane (Table) and object
void environment::dataExtract(){
  dataExtractPlaneSeg();
  if(simulationMode != "FRANKA") dataColorCorrection();
}

void environment::dataExtractPlaneSeg(){
  AVLOG("segmenting", logging, 1);
  // Find the major plane and get its coefficients and indices
  seg.setInputCloud(cPtrPtCldEnv);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.005+viewsphereRad*depthNoise/100);
  Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); //y axis
  seg.setAxis(axis);
  seg.setEpsAngle(10.0f*(M_PI/180.0f) );
  seg.segment(*tableIndices,*tableCoeff);

  if (tableIndices->indices.size () == 0){
    std::cerr << "No table found in the environment" << std::endl;
    return;
  }

  // Seperating the table and storing its point
  extract.setInputCloud(cPtrPtCldEnv);
  extract.setIndices(tableIndices);
  extract.setNegative(false); extract.filter(*ptrPtCldTable);
  extract.setNegative(true);  extract.filter(*internalObject);

  pcl::compute3DCentroid(*ptrPtCldTable, cenTable);
  pcl::getMinMax3D(*ptrPtCldTable, minTable, maxTable);
  //std::cout << tableCoeff->values[0] << " " << tableCoeff->values[1] << " " << tableCoeff->values[2] << " " << tableCoeff->values[3] << std::endl;
  //std::cout << minTable.z << " " << cenTable[2] << " " << maxTable.z << std::endl;

  // Using convex hull to get the table boundary which would be like a rectangle
  cvHull.setInputCloud(cPtrPtCldTable);
  cvHull.setDimension(2);
  cvHull.reconstruct(*ptrPtCldHull);

  // Double checking the hull dimensions
  if(cvHull.getDimension() != 2){
    std::cerr << "Convex hull dimension != 2" << std::endl;
    return;
  }

  // Using polygonal prism and hull the extract object above the table
  prism.setInputCloud(cPtrPtCldObject);
  prism.setInputPlanarHull(cPtrPtCldHull);
  prism.setViewPoint(tableCentre[0],tableCentre[1],tableCentre[2]+1);        // Ensuring normals point above the table
  prism.setHeightLimits(0.01,1.5f);                                 // Z height (min, max) in m
  prism.segment(*objectIndices);

  // Using extract to get the point cloud
  extract.setInputCloud(cPtrPtCldObject);
  extract.setNegative(false);
  extract.setIndices(objectIndices);
  extract.filter(*internalObject);

  pass.setInputCloud(cPtrPtCldObject);
  pass.setFilterFieldName("x"); pass.setFilterLimits(tableCentre[0]-0.15,tableCentre[0]+0.15); pass.filter(*internalObject);
  pass.setFilterFieldName("y"); pass.setFilterLimits(tableCentre[1]-0.15,tableCentre[1]+0.15); pass.filter(*internalObject);
  pass.setFilterFieldName("z"); pass.setFilterLimits(tableCentre[2]+0.01,tableCentre[2]+0.30); pass.filter(*internalObject);

  // Getting the min and max co-ordinates of the object
  pcl::compute3DCentroid(*internalObject, cenObject);
  pcl::getMinMax3D(*internalObject, minPtObj, maxPtObj);

  findNormals(internalObject, internalNormal, knn);
  for(int i=0; i<internalNormal->size(); i++){
    internalNormal->points[i].x = internalObject->points[i].x;
    internalNormal->points[i].y = internalObject->points[i].y;
    internalNormal->points[i].z = internalObject->points[i].z;
  }
  
  // octCompressExt->defineBoundingBox(2);
  // octCompressExt->encodePointCloud(cPtrPtCldObject, compressedData);
  // octCompressExt->decodePointCloud(compressedData, ptrPtCldObject);
  // octCompressExt->deleteTree();

  
  voxelGrid.setInputCloud(cPtrPtCldObject);
  voxelGrid.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
  // constantFilter(ptrPtCldObject, ptrPtCldObject, &voxelGrid);
  voxelGrid.filter(*ptrPtCldObject);
  // ptrPtCldObject->clear();
  // extract.setInputCloud(cPtrPtCldObject);
  // extract.setNegative(true);
  // extract.setIndices(voxelGrid.getRemovedIndices());
  // extract.filter(*ptrPtCldObject);
  // findNormals(ptrPtCldObject, ptrObjNormal, knn);

  voxelGridNormal.setInputCloud(cInternalNormal);
  voxelGridNormal.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
  voxelGridNormal.filter(*ptrObjNormal);
  // AVLOG(to_string(ptrPtCldObject->size())+"=?"+to_string(ptrObjNormal->size()), 0, 0);
  // AVLOG(to_string(ptrPtCldObject->points[0].x)+"=?"+to_string(ptrObjNormal->points[0].x), 0, 0);
  // AVLOG(to_string(ptrPtCldObject->points[0].y)+"=?"+to_string(ptrObjNormal->points[0].y), 0, 0);
  // AVLOG(to_string(ptrPtCldObject->points[0].z)+"=?"+to_string(ptrObjNormal->points[0].z), 0, 0);
  
  // pcl::ExtractIndices<pcl::PointNormal> extractNormal;
  // extractNormal.setInputCloud(cInternalNormal);
  // extractNormal.setNegative(true);
  // extractNormal.setIndices(voxelGrid.getRemovedIndices());
  // extractNormal.filter(*ptrObjNormal);
}

void environment::dataColorCorrection(){
  // Green Color for table (Hue range 81 to 140)
  // Removing all non-green points from table
  // for(int i = 0; i < ptrPtCldTable->points.size(); i++){
  //   float r,g,b,h,s,v;
  //   r = static_cast<float>(ptrPtCldTable->points[i].r);
  //   g = static_cast<float>(ptrPtCldTable->points[i].g);
  //   b = static_cast<float>(ptrPtCldTable->points[i].b);
  //   RGBtoHSV(r/255.0,g/255.0,b/255.0,h,s,v);
  //   if(!(h >= 81 && h <=140)){
  //     ptrPtCldTable->points.erase(ptrPtCldTable->points.begin()+i);
  //     i--;
  //   }
  // }
  // ptrPtCldTable->width = ptrPtCldTable->points.size();
  // ptrPtCldTable->height = 1;
  // pcl::compute3DCentroid(*ptrPtCldTable, cenTable);
  // pcl::getMinMax3D(*ptrPtCldTable, minTable, maxTable);

  // Removing all green points from object and manipulator points
  for(int i = 0; i < internalObject->points.size(); i++){
    float r,g,b,h,s,v;
    r = static_cast<float>(internalObject->points[i].r);
    g = static_cast<float>(internalObject->points[i].g);
    b = static_cast<float>(internalObject->points[i].b);
    RGBtoHSV(r/255.0,g/255.0,b/255.0,h,s,v);
    // if(internalObject->points[i].x <= 0.1){
    if(h >= 81 && h <=140 || internalObject->points[i].x <= 0.1){
      internalObject->points.erase(internalObject->points.begin()+i);
      i--;
    } else if(h >= 81 && h <=140){
      internalObject->points[i].r = 0;
      internalObject->points[i].g = 255;
      internalObject->points[i].b = 0;
    }
  }
  internalObject->width = internalObject->points.size();
  internalObject->height = 1;

  // Getting the min and max co-ordinates of the object
  pcl::compute3DCentroid(*internalObject, cenObject);
  pcl::getMinMax3D(*internalObject, minPtObj, maxPtObj);
}

// 10: Generating unexplored point cloud
void environment::genUnexploredPtCld(){
  // std::cout << "width == " << ptrPtCldUnexp->width << endl;
  if(ptrPtCldUnexp->width != 0){
    // std::cout << "Unexplored point cloud already created. Not creating new one." << std::endl;
    return;
  }
  // std::cout << "Generating new unexplored point cloud." << std::endl;
  // Setting the min and max limits based on the object dimension and scale.
  // Min of 0.40m on each side
  // Note: Z scale is only used on +z axis
  minUnexp[0] = (minPtObj.x-std::max((scale-1)*(maxPtObj.x-minPtObj.x)/2,0.30f));
  minUnexp[1] = (minPtObj.y-std::max((scale-1)*(maxPtObj.y-minPtObj.y)/2,0.30f));
  minUnexp[2] = tableCentre[2]-voxelGridSizeUnexp;
  maxUnexp[0] = (maxPtObj.x+std::max((scale-1)*(maxPtObj.x-minPtObj.x)/2,0.30f));
  maxUnexp[1] = (maxPtObj.y+std::max((scale-1)*(maxPtObj.y-minPtObj.y)/2,0.30f));
  maxUnexp[2] = (maxPtObj.z+std::max((scale-1)*(maxPtObj.z-minPtObj.z)/2,0.25f));

  pcl::PointXYZRGB ptTemp;
  for(float x = minUnexp[0]; x < maxUnexp[0]; x+=voxelGridSizeUnexp){
    for(float y = minUnexp[1]; y < maxUnexp[1]; y+=voxelGridSizeUnexp){
      for(float z = minUnexp[2]; z < maxUnexp[2]; z+=voxelGridSizeUnexp){
        ptTemp.x = x; ptTemp.y = y; ptTemp.z = z;
        ptrPtCldUnexp->points.push_back(ptTemp);
      }
    }
  }
  ptrPtCldUnexp->width = ptrPtCldUnexp->points.size();
  ptrPtCldUnexp->height = 1;
}

// 11: Updating the unexplored point cloud
void environment::updateUnexploredPtCld(){

  ptCldColor::Ptr lastFilled{new ptCldColor};
  if(simulationMode == "SIMULATION"){
    // Transforming the point cloud to Camera frame from world frame
    Eigen::Affine3f tf = tfGazWorld*tfCamOptGaz*rotate;
    Eigen::Affine3f tfTranspose = homoMatTranspose(tf);
    pcl::transformPointCloud(*ptrPtCldUnexp, *ptrPtCldTemp, tfTranspose);
    
  }else if(simulationMode == "FRANKASIMULATION"){
    tf::StampedTransform transform;
    _world->listener.lookupTransform("camera_optical_link", "panda_link0", ros::Time(0), transform);
    pcl_ros::transformPointCloud(*ptrPtCldUnexp, *ptrPtCldTemp, transform);
  }else{
    tf::StampedTransform transform;
    _world->listener.lookupTransform("camera_depth_optical_frame", "panda_link0", ros::Time(0), transform);
    pcl::transformPointCloud(*ptrPtCldUnexp, *ptrPtCldTemp, ICP_Tf.inverse());
    pcl_ros::transformPointCloud(*ptrPtCldTemp, *ptrPtCldTemp, transform);
  }

  Eigen::Vector4f ptTemp;
  Eigen::Vector3f proj;
  static pcl::PointIndices::Ptr occludedIndices(new pcl::PointIndices());
  occludedIndices->indices.clear();
  int projIndex;

  // Looping through all the points and finding occluded ones.
  // Using the camera projection matrix to project 3D point to camera plane
  for(int i = 0; i < ptrPtCldTemp->width; i++){
    ptTemp = ptrPtCldTemp->points[i].getVector4fMap();
    proj = _world->getProjectionMat()*ptTemp;
    proj = proj/proj[2];
    proj[0] = round(proj[0]);
    proj[1] = round(proj[1]);

    // Leave Points below the table as it is
    if(ptrPtCldUnexp->points[i].z <= maxTable.z){
      occludedIndices->indices.push_back(i);
    }else{
      if(proj[0] >=0 && proj[0] <= 640-1 && proj[1] >=0 && proj[1] <= 480-1){
        projIndex = proj[1]*(ptrPtCldLastFill->width)+proj[0];
        // If the z value of unexplored pt is greater than the corresponding
        // projected point in Camera Raw data then that point is occluded.
        if(ptrPtCldLastFill->points[projIndex].z <= ptTemp[2]*(1.0-voxelGridSizeUnexp)){
          occludedIndices->indices.push_back(i);
        }
      //Include the unexplored points that are slightly above the object, to handle
      // corner case where full object is not visible
      } else if(proj[1] <0 && ptrPtCldUnexp->points[i].z <= maxPtObj.z *1.1){
        occludedIndices->indices.push_back(i);
      }
    }
  }

  // Only keeping the occluded points
  extract.setInputCloud(cPtrPtCldUnexp);
  extract.setIndices(occludedIndices);
  extract.setNegative(false);
  extract.filter(*ptrPtCldUnexp);

  if(minUnexp[0] > minPtObj.x || minUnexp[1] > minPtObj.y ||
     maxUnexp[0] < maxPtObj.x || maxUnexp[1] < maxPtObj.y || maxUnexp[2] < maxPtObj.z){
    ROS_WARN("Unexplored point cloud initially generated smaller than the object.");
    AVLOG("("+ to_string(minUnexp[0])+ "," + to_string(minUnexp[1])+ "," + to_string(minUnexp[2])+ ")", logging, 1);
    AVLOG("("+ to_string(maxUnexp[0])+ "," + to_string(maxUnexp[1])+ "," + to_string(maxUnexp[2])+ ")", logging, 1);
    AVLOG("("+ to_string(minPtObj.x)+ "," + to_string(minPtObj.y)+ "," + to_string(minPtObj.z)+ ")", logging, 1);
    AVLOG("("+ to_string(maxPtObj.x)+ "," + to_string(maxPtObj.y)+ "," + to_string(maxPtObj.z)+ ")", logging, 1);
  }

  ptrPtCldTemp->clear();

  // Recalculating centroid based on unexplored point cloud
  *ptrPtCldTemp = *ptrPtCldUnexp + *internalObject;
  pass.setInputCloud(cPtrPtCldTemp);
  pass.setFilterFieldName("x"); pass.setFilterLimits(minPtObj.x,maxPtObj.x); pass.filter(*ptrPtCldTemp);
  pass.setFilterFieldName("y"); pass.setFilterLimits(minPtObj.y,maxPtObj.y); pass.filter(*ptrPtCldTemp);
  pass.setFilterFieldName("z"); pass.setFilterLimits(minPtObj.z,maxPtObj.z); pass.filter(*ptrPtCldTemp);
  pcl::compute3DCentroid(*ptrPtCldTemp, cenObject);

  ptrPtCldTemp->clear();
}

// 12: Finding normals and pairs of grasp points from object point cloud
void environment::graspsynthesis(){
  // printf("Grasp synthesis");
  pcl::toROSMsg(*ptrPtCldObject,graspMsg.request.object);
  pcl::toROSMsg(*ptrPtCldUnexp,graspMsg.request.unexplored);
  pcl::toROSMsg(*ptrObjNormal,graspMsg.request.normal);
  graspMsg.request.voxelGridSize = voxelGridSize;
  graspMsg.request.thinObject = true;
  // Eigen::Affine3f tf = tfGazWorld*tfCamOptGaz;
  // graspMsg.request.x = tf.data()[0];
  // graspMsg.request.y = tf.data()[1];
  // graspMsg.request.z = tf.data()[2];
  
  _world->graspClient.call(graspMsg);

  nGrasps = graspMsg.response.totalGrasps;
  graspID = graspMsg.response.selectedID;
  if(graspID != -1){
    AVLOG("Grasp found "+to_string(graspID), logging, 1);
    graspData.quality = graspMsg.response.graspQuality;
    graspData.gripperWidth = graspMsg.response.graspQuality;
    
    graspData.p1.x = graspMsg.response.contactA[0];
    graspData.p1.y = graspMsg.response.contactA[1];
    graspData.p1.z = graspMsg.response.contactA[2];
    
    graspData.p2.x = graspMsg.response.contactB[0];
    graspData.p2.y = graspMsg.response.contactB[1];
    graspData.p2.z = graspMsg.response.contactB[2];

    graspData.pose = graspMsg.response.pose;
    graspData.addnlPitch = graspMsg.response.addnlPitch;
    AVLOG("graspMsg :",logging,1);
    cout << graspMsg.response.pose[0] << "," << graspMsg.response.pose[1] << "," << graspMsg.response.pose[2] << "\n"
       << graspMsg.response.pose[3] << "," << graspMsg.response.pose[4] << "," << graspMsg.response.pose[5] << std::endl;
    AVLOG("grasp Data :",logging,1);
    cout << graspData.pose[0] << "," << graspData.pose[1] << "," << graspData.pose[2] << "\n"
       << graspData.pose[3] << "," << graspData.pose[4] << "," << graspData.pose[5] << std::endl;
  }
}

// 16: Modify moveit collision elements
void environment::editMoveItCollisions(std::string object, std::string mode){

  if(object == "OBJECT" && internalObject->points.size() == 0) return;

  moveit_planner::AddCollision collisionObjMsg;

  collisionObjMsg.request.collObject.header.frame_id = "/world";
  collisionObjMsg.request.collObject.id = object;

  if(mode == "ADD"){
    shape_msgs::SolidPrimitive primitive;

    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if(object == "TABLE"){
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.7;
      primitive.dimensions[1] = 0.7;
      primitive.dimensions[2] = 0.01;

      pose.position.x = tableCentre[0];
      pose.position.y = tableCentre[1];
      pose.position.z = tableCentre[2];
      if(simulationMode == "FRANKA") pose.position.z -= 0.03;
    }else if(object == "OBJECT"){
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[0] = std::min((maxPtObj.z-tableCentre[2])*1.1,0.35);
      primitive.dimensions[1] = std::min(double(std::max(maxPtObj.x - minPtObj.x,maxPtObj.y - minPtObj.y)),0.25)/2.0;
      if(simulationMode == "FRANKA"){
        primitive.dimensions[0] -= 0.05;
        primitive.dimensions[1] -= 0.03;
      }
      if(primitive.dimensions[0] < 0) primitive.dimensions[0] = 0.01;
      if(primitive.dimensions[1] < 0) primitive.dimensions[1] = 0.01;

      pose.position.x = cenObject[0];
      pose.position.y = cenObject[1];
      pose.position.z = tableCentre[2]+primitive.dimensions[0]/2;
    } 
    collisionObjMsg.request.collObject.primitives.push_back(primitive);
    collisionObjMsg.request.collObject.primitive_poses.push_back(pose);
    collisionObjMsg.request.collObject.operation = collisionObjMsg.request.collObject.ADD;
  }else if(mode == "REMOVE"){
    collisionObjMsg.request.collObject.operation = collisionObjMsg.request.collObject.REMOVE;
  }

  _world->collisionClient.call(collisionObjMsg);
}

// 17: Object grasping pipeline
void environment::graspObject(graspPoint &graspData){

  if(simulationMode == "SIMULATION") return;

  int temp;
  std::cout << "Do you want to run the grasp test? (1/0)"; std::cin >> temp;
  if(temp == 0) return;

  clearAllConstraints();
  editMoveItCollisions("OBJECT","ADD");

  if(simulationMode == "FRANKA"){
    graspData.pose[0] += 0.010;
    graspData.pose[1] -= 0.010;
  }

  cout << graspData.pose[0] << "," << graspData.pose[1] << "," << graspData.pose[2] << "\n"
       << graspData.pose[3] << "," << graspData.pose[4] << "," << graspData.pose[5] << std::endl;

  cout << "-----------1st tf------------" << endl;

  Eigen::Affine3f tfGrasp = pcl::getTransformation(graspData.pose[0],graspData.pose[1],
                                                   graspData.pose[2],graspData.pose[3],
                                                   graspData.pose[4],graspData.pose[5])*
                            pcl::getTransformation(0,0,0,0,graspData.addnlPitch,0);
  
  // if(tfGrasp(0,0) < 0) tfGrasp = tfGrasp*pcl::getTransformation(0,0,0,0,0,M_PI);

  std::cout << tfGrasp(0,0) << "," << tfGrasp(0,1) << "," << tfGrasp(0,2) << "," << tfGrasp(0,3) << "\n"
            << tfGrasp(1,0) << "," << tfGrasp(1,1) << "," << tfGrasp(1,2) << "," << tfGrasp(1,3) << "\n"
            << tfGrasp(2,0) << "," << tfGrasp(2,1) << "," << tfGrasp(2,2) << "," << tfGrasp(2,3) << std::endl;

  cout << "-----------2nd tf------------" << endl;

  tfGrasp = tfGrasp * pcl::getTransformation(0.0,0,-0.0447-fingerZOffset,0,0,0)*
                      pcl::getTransformation(0,0,0,0,-M_PI/2,-M_PI);


  std::cout << tfGrasp(0,0) << "," << tfGrasp(0,1) << "," << tfGrasp(0,2) << "," << tfGrasp(0,3) << "\n"
            << tfGrasp(1,0) << "," << tfGrasp(1,1) << "," << tfGrasp(1,2) << "," << tfGrasp(1,3) << "\n"
            << tfGrasp(2,0) << "," << tfGrasp(2,1) << "," << tfGrasp(2,2) << "," << tfGrasp(2,3) << std::endl;

  Eigen::Affine3f tfGraspMoveUp = tfGrasp; tfGraspMoveUp(2,3)+=0.1;
  Eigen::Affine3f tfPreGrasp;
  geometry_msgs::Pose p;

  float clearance = 0;
  float rad = std::min(double(std::max(maxPtObj.x - minPtObj.x,maxPtObj.y - minPtObj.y)),0.25)/2.0;
  do{
    tfPreGrasp = tfGrasp*pcl::getTransformation(-clearance+0.0447+fingerZOffset,0,0,0,0,0);
    clearance += 0.01;
  }while((sqrt(pow(tfPreGrasp(0,3)-cenObject[0],2) + pow(tfPreGrasp(1,3)-cenObject[1],2)) <= rad &&
          tfPreGrasp(2,3)>0 && tfPreGrasp(2,3)<0.25));

  tfPreGrasp = tfGrasp*pcl::getTransformation(-clearance*1.1,0,0,0,0,0);

  bool res = true;

  // std::cout << moveFranka(tfGrasp.matrix(),"JOINT",false,false,p) << std::endl;
  // std::cout << moveFranka(tfPreGrasp.matrix(),"JOINT",false,false,p) << std::endl;

  moveFrankaHome();

  std::cout << tfPreGrasp(0,0) << "," << tfPreGrasp(0,1) << "," << tfPreGrasp(0,2) << "," << tfPreGrasp(0,3) << "\n"
            << tfPreGrasp(1,0) << "," << tfPreGrasp(1,1) << "," << tfPreGrasp(1,2) << "," << tfPreGrasp(1,3) << "\n"
            << tfPreGrasp(2,0) << "," << tfPreGrasp(2,1) << "," << tfPreGrasp(2,2) << "," << tfPreGrasp(2,3) << std::endl;

  std::cout << "Moving to Pre Grasp..." << std::endl;
  res = moveFranka(tfPreGrasp.matrix(),"JOINT",false,true,p);
  if(!res){
    std::cout << "Retrying I..." << std::endl;
    tfPreGrasp = tfGrasp*pcl::getTransformation(-clearance-0.07,0,0,0,0,0);
    res = moveFranka(tfPreGrasp.matrix(),"JOINT",false,true,p);
  }
  if(!res){
    std::cout << "Retrying II..." << std::endl;
    tfPreGrasp = tfGrasp*pcl::getTransformation(-clearance-0.15,0,0,0,0,0);
    res = moveFranka(tfPreGrasp.matrix(),"JOINT",false,true,p);
  }
  editMoveItCollisions("OBJECT","REMOVE");
  moveGripper(graspData.gripperWidth+0.04);

  // Move to Grasp and grasp
  if(res){
    std::cout << "Enter any key to start grasping..."; std::cin >> temp;
    res = moveFranka(tfGrasp.matrix(),"CARTESIAN",false,true,p);
    if(simulationMode == "FRANKA") moveGripper(graspData.gripperWidth-voxelGridSize-0.02,true);
    else                           moveGripper(graspData.gripperWidth-1.5*voxelGridSize,true);
  }else std::cout << "Skipping Grasp" << std::endl;

  // Lift test
  if(res){
    std::cout << "Do you want to run the Lift test? (1/0) : "; std::cin >> temp;
    if(temp == 1){
      std::cout << "Lift test running" << std::endl;
      res = moveFranka(tfGraspMoveUp.matrix(),"CARTESIAN",false,true,p);
      std::cout << "Lift test ended" << std::endl;
    }
  }else std::cout << "Skipping Lift Test" << std::endl;

  // Rotation test
  if(res){
    std::cout << "Do you want to run the Rotation test? (1/0) : "; std::cin >> temp;
    if(temp == 1){
      moveFrankaHome(false);
      std::cout << "Rotation test running" << std::endl;
      moveit_planner::SetJointWithTime setJointWithTimeMsg;
      setJointWithTimeMsg.request.joint_name = "panda_joint7";
      for(int i = 0; i < 2; i++){
        setJointWithTimeMsg.request.joint_angle = 0.000; _world->oneJointWithTimeClient.call(setJointWithTimeMsg);
        setJointWithTimeMsg.request.joint_angle = 1.570; _world->oneJointWithTimeClient.call(setJointWithTimeMsg);
      }
      setJointWithTimeMsg.request.joint_angle = 0.785; _world->oneJointWithTimeClient.call(setJointWithTimeMsg);
      std::cout << "Rotation test ended" << std::endl;
    }
  }else std::cout << "Skipping Rotation Test" << std::endl;

  // Shaking test
  if(res){
    std::cout << "Do you want to run the Shaking test? (1/0) : "; std::cin >> temp;
    if(temp == 1){
      moveFrankaHome(false);
      std::cout << "Shaking test running" << std::endl;
      moveit_planner::SetJointWithTime setJointWithTimeMsg;
      setJointWithTimeMsg.request.joint_name = "panda_joint5";
      for(int i = 0; i < 4; i++){
        setJointWithTimeMsg.request.joint_angle = -0.35; _world->oneJointWithTimeClient.call(setJointWithTimeMsg);
        setJointWithTimeMsg.request.joint_angle =  0.35; _world->oneJointWithTimeClient.call(setJointWithTimeMsg);
      }
      setJointWithTimeMsg.request.joint_angle =  0.00; _world->oneJointWithTimeClient.call(setJointWithTimeMsg);
      std::cout << "Shaking test ended" << std::endl;
    }
  }else std::cout << "Skipping Shaking test" << std::endl;

  res = moveFranka(tfGraspMoveUp.matrix(),"JOINT",false,true,p);
  // Go to same location and release
  if(res){
    std::cout << "Enter any key to place the object back..."; std::cin >> temp;
    res = moveFranka(tfGrasp.matrix(),"CARTESIAN",false,true,p);
    moveGripper(graspData.gripperWidth+0.04);
  }else std::cout << "Skipping placing object back" << std::endl;

  // Move to Post-grasp
  if(res){
    std::cout << "Moving to Retreat...";
    res = moveFranka(tfPreGrasp.matrix(),"CARTESIAN",false,true,p);
    std::cout << res << std::endl;
    editMoveItCollisions("OBJECT","ADD");
  }else std::cout << "Skipping Post Grasp" << std::endl;

  moveFrankaHome();
  editMoveItCollisions("OBJECT","REMOVE");

}

// ******************** ENVIRONMENT CLASS FUNCTIONS END ********************


// Function to do a single pass
std::vector<double> singlePass(environment &av, std::vector<double> cameraPose, bool firstTime, bool findGrasp, int mode, int logging)
{
  std::vector<double> timer;
  std::chrono::high_resolution_clock::time_point start, end;
  start = std::chrono::high_resolution_clock::now();

  AVLOG("Moving Camera", logging, 1);
  bool move_success = av.moveCameraViewsphere(cameraPose);

  if(!move_success){
    AVLOG("Move failed!", logging, 0);
    end = std::chrono::high_resolution_clock::now();
    timer.push_back((std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count());
    return timer;
  }

  AVLOG("Reading data", logging, 1);
  av.readCamera();

  AVLOG("Fusing data", logging, 1);
  av.fuseLastData();


  AVLOG("Extracting table & object", logging, 1);
  av.dataExtract();

  // std::cout << " 5" << std::endl;

  AVLOG("Updating unexplored regions", logging, 1);
  av.genUnexploredPtCld();

  av.updateUnexploredPtCld();
  AVLOG("Grasp synthesis", logging, 1);

  end = std::chrono::high_resolution_clock::now();
  timer.push_back((std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count());

  start = std::chrono::high_resolution_clock::now();
  if (findGrasp)
    AVLOG("We want to find grasp", logging, 1);
    av.graspsynthesis();
    

  // std::cout << " 7" << std::endl;

  end = std::chrono::high_resolution_clock::now();

  timer.push_back((std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count());

  AVLOG("Ending single pass", logging, 1);
  return timer;
}
