#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>
#include <active_vision/optimalDistSRV.h>
#include <active_vision/toolDataHandling.h>
#include <map>
#include "optimality/persist_path_finder.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <active_vision/pointCloudTools.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

using namespace std;
// string file = "prismAV10x8x4";
// string file = "072-a_toy_airplane";
string file = "009_gelatin_box";
int cObjID = 11;
double rotation = 0;
map<int, objectInfo> objectDict;
int pointCloudSize = 0;
int angleTarget = 0;
float raytracingGridSize;
float scale = 0.95;
float searchRadius;
float minViewSize;
int logging = 0;

/* *************Instructions for use*************
(terminal A): roscore
(terminal B): rosrun active_vision optPathService
(terminal C): rosservice call /active_vision/optimalDistance "points:                    
- x: 1.0
  y: 0.0
  z: 0.5
- x: 0.9
  y: 0.1
  z: 0.5
- x: 0.8
  y: 0.2
  z: 0.5
curr_direction: 0"

Note: The points vector cannot contain any (0,0,0) points- it should be fine if they're not
  normalized, but they have to have nonzero magnitude
Note: The last point [here it's (0.8,0.2,0.0)] is the one the viewpoint will be calculated 
  from. For now curr_direction is just discarded. Passing a vector with only one point
  will work fine- it searches from that point with no history.

*/

map<string, PersistentPathFinder*> objects;
map<string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> trueClouds;
map<string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objectClouds;
map<string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_spheres;
map<string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_starts;
map<string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_unexploreds;
map<string, pcl::PointCloud<pcl::PointNormal>::Ptr> normals;
map<string, PointXYZRGB> centroids;
map<string, unordered_map<int, vector<int>>* > vis_pairs;
map<string, unordered_map<int, vector<int>>* > o_vis_pairs;

vector<vector<int>> loadVec(string source){
  string cLine;
  ifstream loadFile(source);
  vector<vector<int>> target;
  while(getline(loadFile, cLine)){
    vector<int> cList;
    string cDigit;
    for(int i=0; i<cLine.length(); i++){
      if(','==cLine.at(i)){
        cList.push_back(atoi(cDigit.c_str()));
        cDigit = "";
      } else {
        cDigit += cLine.at(i);
      }
    }
    if(0 != cLine.length()){
      cList.push_back(atoi(cDigit.c_str()));
      // printf("%s", cDigit.c_str());
    }
    target.push_back(cList);
  }
  loadFile.close();
  return target;
}

//https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
PointXYZRGB loadPoint(string source){
  string cLine;
  ifstream loadFile(source);
  getline(loadFile, cLine);
  PointXYZRGB p{};
  p.x = atof(cLine.substr(0, cLine.find(",")).c_str());
  cLine = cLine.substr(cLine.find(",")+1);
  p.y = atof(cLine.substr(0, cLine.find(",")).c_str());
  cLine = cLine.substr(cLine.find(",")+1);
  p.z = atof(cLine.c_str());
  loadFile.close();
  return p;
}

bool optimalDistance(active_vision::optimalDistSRV::Request &req,
                     active_vision::optimalDistSRV::Response &res)
{

  int direction = req.curr_direction;
  int len = req.points.size();
  vector<double> finalPoint = {req.points[len-1].x, req.points[len-1].y-rotation, req.points[len-1].z};
  vector<double> viz = req.viz.data;
  // printf("oPS: %d points being sent\n", viz.size());
  // ROS_INFO("Running search %s", file.c_str());
  double ret = objects[file]->searchFrom(finalPoint[1], finalPoint[2], viz, rotation);
  res.optimal_distance = ret;
  res.next_direction = objects[file]->nextDirection;
  // ROS_INFO("Finished search %s", file.c_str());
  return true;
}

void setup(char const* file, float roll=0, float yaw=0, float pitch=0)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadObj(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadUnexp(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointNormal>::Ptr loadNormal(new pcl::PointCloud<pcl::PointNormal>);
  string directory;
  //Kludge, assumes objects 1->53 are ycb and all later are google
  if(53 >= cObjID){
    directory = "ycbAV/";
  } else {
    directory = "google/";
  }
  string target = AV_PATH + "models/"+directory+string(file)+"/generated/";
  io::loadPCDFile<pcl::PointXYZRGB>(target+"obj.pcd", *loadObj);
  io::loadPCDFile<pcl::PointXYZRGB>(target+"unexp.pcd", *loadUnexp);
  io::loadPCDFile<pcl::PointNormal>(target+"normals.pcd", *loadNormal);
  centroids[file] = loadPoint(target+"point.txt");

  //https://en.wikipedia.org/wiki/Rotation_matrix
  Eigen::Matrix4d translation;
  translation << 
      (cos(yaw)*cos(pitch)), (cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll)), (cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)), 0,
      (sin(yaw)*cos(pitch)), (sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll)), (sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)), 0,
      (-sin(pitch)), (cos(pitch)*sin(roll)), (cos(pitch)*cos(roll)), 0,
      0, 0, 0, 1;
  pcl::transformPointCloud(*loadObj, *loadObj, translation);
  pcl::transformPointCloud(*loadUnexp, *loadUnexp, translation);

  for(int i = 0; i < loadObj->size(); i++){
    pcl::PointXYZRGB cObjectPoint = loadObj->points.data()[i];
    pcl::PointNormal n = loadNormal->points.data()[i];
    trueClouds[file]->push_back(cObjectPoint);
    objectClouds[file]->push_back(cObjectPoint);
    normals[file]->push_back(n);
    vis_pairs[file]->insert(make_pair(i, vector<int>({})));
    o_vis_pairs[file]->insert(make_pair(i, vector<int>({})));
  }
  pcl::PointXYZRGB minObj, maxObj;
  pcl::getMinMax3D(*loadObj, minObj, maxObj);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr unexpTemp(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int i=0; i < loadUnexp->size(); i++){
    pcl::PointXYZRGB cUnexploredPoint = loadUnexp->points.data()[i];
    cloud_unexploreds[file]->push_back(cUnexploredPoint);
    if(cUnexploredPoint.z >= minObj.z){
      pcl::PointXYZRGB scaledUnexploredPoint = loadUnexp->points.data()[i];
      scaledUnexploredPoint.x *= scale;
      scaledUnexploredPoint.y *= scale;
      scaledUnexploredPoint.z *= scale;
      trueClouds[file]->push_back(scaledUnexploredPoint);
    }
  }

  int maxCollisionIndex = trueClouds[file]->size();

  vector<vector<int>> visibilities = loadVec(target+"vismap.txt");
  //Iterate over each viewpoint
  // printf("Max collision index = %d\n", maxCollisionIndex);
  for(int i=0; i < visibilities.size(); i++){
    //For each point that is visible from i, load the vis_pairs that already exist.
    // Add the viewpoint to its list
    for(int j=0; j < visibilities[i].size(); j++){
      vector<int> c = vis_pairs[file]->at(visibilities[i][j]);
      c.push_back(i+maxCollisionIndex);
      vis_pairs[file]->erase(visibilities[i][j]);
      vis_pairs[file]->insert(make_pair(visibilities[i][j], c));
      o_vis_pairs[file]->erase(visibilities[i][j]);
      o_vis_pairs[file]->insert(make_pair(visibilities[i][j], c));
    }
  }
}

void setupCloudPointers()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr trueCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  trueClouds[file] = trueCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  objectClouds[file] = objectCloud;
  pcl::PointCloud<pcl::PointNormal>::Ptr trueNormals(new pcl::PointCloud<pcl::PointNormal>);
  normals[file] = trueNormals;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sphere(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_spheres[file] = cloud_sphere;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_start(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_starts[file] = cloud_start;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_unexplored(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_unexploreds[file] = cloud_unexplored;
  centroids[file] = PointXYZRGB{};
  unordered_map<int, vector<int>> *vis_pair(new unordered_map<int, vector<int>>);
  vis_pairs[file] = vis_pair;
  unordered_map<int, vector<int>> *o_vis_pair(new unordered_map<int, vector<int>>);
  o_vis_pairs[file] = o_vis_pair;
}

void setupF()
{
  //Check if we've already created the object
  if(objects.find(file) == objects.end())
  {
    setupCloudPointers();
    float z = objectDict[cObjID].poses[0][0];
    //    (x, y, z, scale, name, 0, rotation, 0)
    // setup(file.c_str(), objectDict[cObjID].poses[0][1], 0, objectDict[cObjID].poses[0][2]);
    setup(file.c_str(), 0, 0, 0);
    addSphere(cloud_spheres[file], 1, pointCloudSize);
    // addCircle(cloud_spheres[file], 1);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(raytracingGridSize));
    *cloud_starts[file] = *trueClouds[file] + *cloud_spheres[file];
    PersistentPathFinder *x = new PersistentPathFinder(logging, 2<=logging);
    x->minVisSize = int(minViewSize * pointCloudSize);
    x->visPairs = vis_pairs[file];
    x->origVisPairs = o_vis_pairs[file];
    x->centroid = centroids[file];
    x->initialize(trueClouds[file], objectClouds[file], cloud_spheres[file], cloud_starts[file], normals[file], cloud_unexploreds[file], octree, angleTarget);
    objects[file] = x;
    // printf("Path service setup completed\n");
  } 
}

//Change the object loaded- call as rarely as possible
void updateFile(const std_msgs::String::ConstPtr& msg)
{
  // ROS_INFO("Got new file name, restarting...");
  cObjID = stoi(msg->data);
  file = objectDict[cObjID].description;
  // ROS_INFO("Restarting with file = %s", file.c_str());
  setupF();
}

//Change the camera offset to simulate rotation
void updateRotation(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("Got new rotation, clearing finder");
  rotation = msg->data * M_PI / 180.0;
  objects[file]->resetViewsphere();
}

void debugCallback(const std_msgs::String::ConstPtr& msg)
{
  pair<string, int> details = fromDebugMessage(msg->data.c_str());
  if("OptimalPathService" != details.first){
    return;
  }
  logging = details.second;
  for(pair<string, PersistentPathFinder*> p : objects){
    objects[p.first]->updateLogging(logging);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OptimalPathService");
  ros::NodeHandle nh;
  readObjectsList(nh, "/active_vision/objectsInfo/", objectDict);
  nh.getParam("/active_vision/optPath/optPathDebugLevel", logging);
  nh.getParam("/active_vision/optPath/pointCloudSize", pointCloudSize);
  nh.getParam("/active_vision/optPath/angleTarget", angleTarget);
  nh.getParam("/active_vision/optPath/infillScale", scale);
  nh.getParam("/active_vision/optPath/minViewSize", minViewSize);
  nh.getParam("/active_vision/environment/raytracingGridSize", raytracingGridSize);
  nh.getParam("/active_vision/graspSynthesis/searchRadius", searchRadius);
  //You need to call this before anything else, but you should setup automatically, so...
  // setupF();
  ros::ServiceServer service = nh.advertiseService("/active_vision/optimalDistance", optimalDistance);
  ros::Subscriber sub1 = nh.subscribe("/active_vision/cFileName", 1, updateFile);
  ros::Subscriber sub2 = nh.subscribe("/active_vision/cRotation", 1, updateRotation);
  ros::Subscriber dSub = nh.subscribe("/debugDelta", 1, debugCallback);
  // ROS_INFO("Optimal distance service ready.");
  ros::spin();
  // while (!objects[file]->viewer->wasStopped())
  // {
  //   objects[file]->stepViewer();
  //   // ros::spin();
  // }
  return (0);
}