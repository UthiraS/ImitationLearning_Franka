#include <iostream>
#include <fstream>
#include <csignal>
#include <active_vision/environment.h>
#include <active_vision/toolViewPointCalc.h>
#include <active_vision/pointCloudTools.h>
#include <pcl/features/integral_image_normal.h>
#include <active_vision/toolGraspSynthesis.h>

using namespace std;
using namespace pcl;

float voxelGridSize;
graspSynthesis *g;

// rosrun active_vision buildModel Weisshai_Great_White_Shark 65

class ModelVis : public AVVis{
public:
  void stepVis(bool hardStop, environment &env)
  {
    if (hardStop)
    {
      _keypress = WAIT;
    }
    setRGB(env.ptrPtCldObject, "object", 3, 0);
    setRGB(env.ptrPtCldUnexp, "unexplored", 2, 0);
    viewer->addPointCloudNormals<PointXYZRGB, PointNormal>(env.ptrPtCldObject, env.ptrObjNormal, 1, .01, "normal", 0);
    AVVis::stepVis();
  }
};

void saveVec(vector<vector<int>> &source, string target)
{
  ofstream saveFile(target + "/vismap.txt");
  for (int i = 0; i < source.size(); i++)
  {
    string data = "";
    for (int j = 0; j < source.at(i).size(); j++)
    {
      if (j != 0)
      {
        data += ",";
      }
      data += to_string(source.at(i).at(j));
    }
    data += "\n";
    saveFile << data;
    // printf("%s", data.c_str());
  }
  saveFile.close();
}

vector<vector<int>> loadVec(string source)
{
  string cLine;
  ifstream loadFile(source);
  vector<vector<int>> target;
  while (getline(loadFile, cLine))
  {
    vector<int> cList;
    string cDigit;
    for (int i = 0; i < cLine.length(); i++)
    {
      if (',' == cLine.at(i))
      {
        cList.push_back(atoi(cDigit.c_str()));
        cDigit = "";
      }
      else
      {
        cDigit += cLine.at(i);
      }
    }
    if (0 != cLine.length())
    {
      cList.push_back(atoi(cDigit.c_str()));
      // printf("%s", cDigit.c_str());
    }
    target.push_back(cList);
  }
  loadFile.close();
  return target;
}

void savePoint(PointXYZRGB p, string target){
  ofstream saveFile(target + "/point.txt");
  string coords = "";
  coords += to_string(p.x) + ",";
  coords += to_string(p.y) + ",";
  coords += to_string(p.z) + "\n";
  saveFile << coords;
  saveFile.close();
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

void checkVisibility(environment &env, PointXYZRGB centroid, vector<double> cameraPose, octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree, vector<int> *visiblePoints, vector<int> *invalidPoints){
  singlePass(env, cameraPose, true, false, 0);
  PointCloud<PointXYZRGB>::Ptr current(new PointCloud<PointXYZRGB>);
  *current += *(env.ptrPtCldObject);
  applyCentering(current, centroid);
  // printf("About to grasp\n");
  g->setNormals(env.ptrObjNormal);
  // printf("Preprocessing\n");
  g->preprocessing(env.ptrPtCldObject, env.ptrPtCldUnexp);
  // printf("Calcing...\n");
  g->calcGraspPairs();
  // PLOG;
  for (int j = 0; j < current->points.size(); j++)
  {
    vector<int> center;
    octree.voxelSearch(current->points[j], center);
    if(center.empty()) continue;
    // PLOG;
    if (!g->validForGrasping(j)){
      // PLOG;
      if (!vectorContainsElement<int>(*invalidPoints, center[0]))
      {
        invalidPoints->push_back(center[0]);
      }
      // PLOG;
      continue;
    }
    // PLOG;
    // env.ptrPtCldObject->points[j].r = 0;
    // env.ptrPtCldObject->points[j].g = 255;
    // env.ptrPtCldObject->points[j].b = 255;
    // cout << center.size() << " " << center[0] << endl;
    if (!vectorContainsElement<int>(*visiblePoints, center[0]))
    {
      visiblePoints->push_back(center[0]);
    }
  }
  env.reset();
}

void buildModel(environment &env, int objectID, PointCloud<PointXYZRGB>::Ptr cloud_sphere, bool vis, string target)
{
  // Spawn the model
  env.spawnObject(objectID, 0, 0);
  env.moveObject(objectID, 0, 0);
  ModelVis* visualizer = new ModelVis();

  vector<double> cameraPose;
  PointXYZ *cPoint;
  int targetSize = cloud_sphere->points.size();
  // targetSize = 4; //For testing

  for (int i = 0; i < targetSize; i++)
  {
    ros::spinOnce();
    cPoint = new PointXYZ(cloud_sphere->points[i].x, cloud_sphere->points[i].y, cloud_sphere->points[i].z);
    cameraPose = cartesianToSpherical(*cPoint);
    singlePass(env, cameraPose, 0 == i, false, 0);

    if (vis && (0 == i % (targetSize/4))){
      visualizer->setup();
      visualizer->stepVis(false, env);
    }
  }
  
  // Generate the normals
  ptCldColor::Ptr pObject{new ptCldColor};
  ptCldColor::Ptr pTrash{new ptCldColor};
  ptCldColor::Ptr pUnexp{new ptCldColor};
  ptCldNormal::Ptr ptrObjNormal{new ptCldNormal};
  *pObject = *(env.ptrPtCldObject);
  *pUnexp = *(env.ptrPtCldUnexp);
  *ptrObjNormal = *(env.ptrObjNormal);

  vector<boost::shared_ptr<PointCloud<PointXYZRGB>>> v;
  v.push_back(pObject);
  v.push_back(pUnexp);
  PointXYZRGB centroid = centerClouds<PointXYZRGB>(v);

  // PointCloud<PointXYZRGBNormal>::Ptr ptrPtCldObject{new PointCloud<PointXYZRGBNormal>};
  // //I hate the authors of PCL
  // for(int i=0; i<ptrObjNormal->points.size(); i++){
  //   PointXYZRGBNormal pt;
  //   pt.x = env.ptrPtCldObject->points[i].x;
  //   pt.y = env.ptrPtCldObject->points[i].y;
  //   pt.z = env.ptrPtCldObject->points[i].z;
  //   pt.r = 255;
  //   pt.normal_x = ptrObjNormal->points[i].normal_x;
  //   pt.normal_y = ptrObjNormal->points[i].normal_y;
  //   pt.normal_z = ptrObjNormal->points[i].normal_z;
  //   pt.curvature = ptrObjNormal->points[i].curvature;
  //   ptrPtCldObject->push_back(pt);
  // }
  octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree{voxelGridSize};
  double x1, y1, z1, x2, y2, z2;
  octree.defineBoundingBox(2);
  octree.setInputCloud(pObject);
  octree.addPointsFromInputCloud();
  octree.getBoundingBox(x1, y1, z1, x2, y2, z2);
  printf("(%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)\n", x1, y1, z1, x2, y2, z2);
  // VoxelGrid<PointXYZRGB> voxelGrid;
  // printf("Initial size = %ld\n", pObject->points.size());
  // voxelGrid.setInputCloud(pObject);
  // voxelGrid.setLeafSize(voxelGridSize * 1.0, voxelGridSize * 1.0, voxelGridSize * 1.0);
  // voxelGrid.setSaveLeafLayout(true);
  // constantFilter(pObject, pTrash, &voxelGrid);
  // voxelGrid.filter(*(pTrash));
  // https://stackoverflow.com/questions/44921987/removing-points-from-a-pclpointcloudpclpointxyzrgb
  // PointIndices::Ptr removed(new PointIndices());
  // voxelGrid.getRemovedIndices(*removed);
  // printf("New size = %ld\n", pObject->points.size());

  vector<int> pointCentroidMap;

  // for (int i = 0; i < pObject->points.size(); i++)
  // {
  //   Eigen::Vector3i target = voxelGrid.getGridCoordinates(pObject->points[i].x, pObject->points[i].y, pObject->points[i].z);
  //   pointCentroidMap.push_back(voxelGrid.getCentroidIndexAt(target));
  // }
  PointNormal centroidNormal;
  centroidNormal.x = centroid.x;
  centroidNormal.y = centroid.y;
  centroidNormal.z = centroid.z;
  applyCentering(ptrObjNormal, centroidNormal);

  printf("%ld object, %ld normal\n", pObject->points.size(), ptrObjNormal->points.size());

  // Save the merged point clouds
  io::savePCDFileASCII(target + "/unexp.pcd", *pUnexp);
  io::savePCDFileASCII(target + "/obj.pcd", *pObject);
  io::savePCDFileASCII(target + "/normals.pcd", *ptrObjNormal);
  savePoint(centroid, target);
  PointXYZRGB load = loadPoint(target+"/point.txt");
  printf("Saved centroid = (%.4f, %.4f, %.4f)\n", centroid.x, centroid.y, centroid.z);
  printf("Loaded centroid = (%.4f, %.4f, %.4f)\n", load.x, load.y, load.z);

  // Accumulate the single views and save them.
  // Note- doing it like this takes twice as long, but doesn't require keeping all the views in memory at once.
  // That adds up when there are 2k views.
  env.reset();
  std::vector<std::vector<int>> viewpointIndices;

  for (int i = 0; i < targetSize; i++)
  {
    ros::spinOnce();
    cPoint = new PointXYZ(cloud_sphere->points[i].x, cloud_sphere->points[i].y, cloud_sphere->points[i].z);
    cameraPose = cartesianToSpherical(*cPoint);
    std::vector<int> visiblePoints = {};
    std::vector<int> invalidPoints = {};
    checkVisibility(env, centroid, cameraPose, octree, &visiblePoints, &invalidPoints);
    sort(visiblePoints.begin(), visiblePoints.end());
    sort(invalidPoints.begin(), invalidPoints.end());
    // PLOG;
    for(int dir = 1; dir <= 8; dir++){
      std::vector<int> visiblePointsE = {};
      std::vector<int> invalidPointsE = {};
      //Spiral outwards in half degree increments
      std::vector<double> perturbedPose = calcExplorationPoseB(cameraPose, dir, 0.5*(1+(dir % 4))*(M_PI/180.0));
      // PLOG;
      checkVisibility(env, centroid, perturbedPose, octree, &visiblePointsE, &invalidPointsE);
      // PLOG;
      vector<int> stillVisiblePoints = {};
      for(int index : visiblePoints){
        if(vectorContainsElement<int>(visiblePointsE, index)){
          stillVisiblePoints.push_back(index);
        }
      }
      visiblePoints.swap(stillVisiblePoints);
      stillVisiblePoints.clear();
    }
    // visualize(viewer, vp, keyPress, env);
    if(vis){
      visualizer->stepVis(false, env);
    }
    viewpointIndices.push_back(visiblePoints);
  }
  if(vis){
    free(visualizer);
  }
  // Save that map.
  saveVec(viewpointIndices, target);

  vector<vector<int>> loadTest = loadVec(target + "/vismap.txt");
  for (int i = 0; i < loadTest.size(); i++)
  {
    printf("--------%d---------\n", i);
    string c = "";
    for (int j = loadTest.at(i).size() - 20; j < loadTest.at(i).size(); j++)
    {
      if (0 != j)
      {
        c += ",";
      }
      c += to_string(loadTest.at(i).at(j));
    }
    printf("%s", c.c_str());
    printf("--------%d---------\n", i);
  }

  env.deleteObject(objectID);
  env.reset();
}

void mainLoop(environment &env, string objectName, int objectID, PointCloud<PointXYZRGB>::Ptr cloud_sphere, bool vis)
{
  // Check if records exist
  // Kludge- only objects 1->53 are in ycb, so everything else is google. Adjust according to new object yaml.
  string target;
  if (objectID <= 53)
  {
    target = AV_PATH + "models/ycbAV/" + objectName + "/generated";
  }
  else
  {
    target = AV_PATH + "models/google/" + objectName + "/generated";
  }
  ifstream dir(target);
  ifstream file(target + "/obj.pcd");
  if (!dir.is_open() || !file.is_open())
  {
    // https://pubs.opengroup.org/onlinepubs/009695399/functions/mkdir.html
    if (!dir.is_open())
    {
      mkdir(target.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    buildModel(env, objectID, cloud_sphere, vis, target);
  }
  else
  {
    // load the model into the environment
    return;
    io::loadPCDFile<PointXYZRGB>(target + "/unexp.pcd", *env.ptrPtCldUnexp);
    io::loadPCDFile<PointXYZRGB>(target + "/obj.pcd", *env.ptrPtCldObject);
    io::loadPCDFile<PointNormal>(target + "/normals.pcd", *env.ptrObjNormal);
    ModelVis* visualizer = new ModelVis();
    visualizer->setup();
    visualizer->stepVis(false, env);
    // visualize(viewer, vp, keyPress, env);
    env.reset();
  }
  cout << "Finished building " << objectName << endl;
}

void interuptHandler(int signalNumber)
{
  cerr << "Got interrupt signal! exiting" << endl;
  exit(-1);
}

int main(int argc, char **argv)
{
  if (3 > argc)
  {
    cerr << "Too few arguments! Expected at least 2 arguments- [objectName] [pobjectID]" << endl;
    return -1;
  }
  int pointCloudSize;
  PointCloud<PointXYZRGB>::Ptr cloud_sphere(new PointCloud<PointXYZRGB>);

  string objectName = argv[1];
  int objectID = atoi(argv[2]);

  ros::init(argc, argv, "buildModel");
  ros::NodeHandle nh;
  environment env(&nh);
  int debugLevel;
  nh.getParam("/active_vision/optPath/pointCloudSize", pointCloudSize);
  nh.getParam("/active_vision/environment/voxelGridSize", voxelGridSize);
  nh.getParam("/active_vision/policyTester/PolicyVis", debugLevel);

  addSphere(cloud_sphere, 0.5, pointCloudSize, false);

  signal(SIGINT, interuptHandler);
  g = new graspSynthesis(&nh);
  // g->debugCollisionCheck = true;
  if (0 == strcmp("ALL", objectName.c_str()))
  {
    std::map<int, objectInfo> objectDict;
    readObjectsList(nh, "/active_vision/objectsInfo/", objectDict);

    // string objects[objs] = {"009_gelatin_box", "055_baseball", "072-a_toy_airplane",
    //                         "010_potted_meat_can", "003_cracker_box", "035_power_drill", "005_tomato_soup_can",
    //                         "006_mustard_bottle", "021_bleach_cleanser", "025_mug", "013_apple"};
    // int startIndex = 11;
    // https://en.cppreference.com/w/cpp/container/map
    for (const auto &[key, value] : objectDict)
    {
      if(key < 52) continue;
      cout << "Building object " << value.description << endl;
      mainLoop(env, value.description, key, cloud_sphere, (0 != debugLevel));
    }
  }
  else
  {
    mainLoop(env, objectName, objectID, cloud_sphere, (0 != debugLevel));
  }
}