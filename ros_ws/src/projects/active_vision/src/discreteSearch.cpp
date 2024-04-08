#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <fstream>
#include <numeric>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <active_vision/toolVisualization.h>
#include <active_vision/toolDataHandling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <CGAL/Exact_spherical_kernel_3.h>
#include "optimality/point_relationships.h"
#include "optimality/opt_path_finder.h"
#include "optimality/persist_path_finder.h"
#include <time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <active_vision/pointCloudTools.h>


typedef CGAL::Exact_spherical_kernel_3 SK;
typedef CGAL::Sphere_3<SK> Sphere_3;
typedef CGAL::Plane_3<SK> Plane_3;
typedef CGAL::Point_3<SK> Point_3;
typedef CGAL::Circle_3<SK> Circle_3;
// typedef CGAL::Cartesian<float> K;
SK::Sphere_3 s1(SK::Point_3(0, 0, 0), 1.0);
bool firstPoint = true;
bool display = false;
Point_3 p1;

int trueSize;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr trueCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr trueNormals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr displayCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr displayNormals(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sphere(new pcl::PointCloud<pcl::PointXYZRGB>);

std::vector<Point_3> normalIntersections;
std::vector<Circle_3> regionsOfVisibility;
PointRelationship *pr(new PointRelationship());

// Select point on object, not the normal
int correctIndex(int ind)
{
  if (ind >= trueSize)
  {
    ind -= trueSize;
  }
  return ind;
}

void addNormals()
{
  int difference = trueNormals->points.size();
  displayCloud->resize(displayCloud->points.size() + difference);
  for (int i = 0; i < difference; i++)
  {
    pcl::Normal n = trueNormals->points[i];
    displayCloud->points.data()[i + difference].x = n.normal_x;
    displayCloud->points.data()[i + difference].y = n.normal_y;
    displayCloud->points.data()[i + difference].z = n.normal_z;
    displayCloud->points.data()[i + difference].r = 0;
    displayCloud->points.data()[i + difference].g = 255;
    displayCloud->points.data()[i + difference].b = 0;
  }
  std::cout << displayCloud->points[2 * difference - 1].x << std::endl;
  std::cout << displayCloud->points[2 * difference - 1].r << std::endl;
}

void setup(float a, float b, float c, float d, char* file)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr load(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr load(new pcl::PointCloud<pcl::PointXYZRGB>);
  // if(0 == strcmp(file, "cube")) addCube(load, .1, .025);
  // else if (0 == strcmp(file, "rect")) addRect(load, .2, .2, .05, .025);
  // else 
  if(true)
  {
    //ycb objects on the lab machine
    std::string target = AV_PATH + "/models/ycbAV/"+std::string(file)+"/google_16k/nontextured.ply";
    // pcl::io::loadPCDFile("/home/diyogon/Downloads/lightbulb_1/rgbd-dataset/lightbulb/lightbulb_1/lightbulb_1_1_1.pcd", *load);
    // pcl::io::loadPCDFile("/home/diyogon/Documents/WPI/Berk_Lab/mer_lab/ros_ws/src/projects/active_vision/models/ycbAV/072-a_toy_airplane/google_16k/output.pcd", *load);
    pcl::io::loadPLYFile(target, *load);
  }
  // 
  // addCube(load, .25, .025);
  //Blocks moving through the main object
  // addCube(load, .045, .003);
  std::vector<double> pose = {0, 0, 0};
  pcl::PointXYZ sphereCentre;
  pcl::PointXYZ table, a1, a2;
  table.x = 0.55;
  table.y = 0;
  table.z = 0;
  Eigen::Matrix4d translation;
  double sc = 1.0 - d;
  translation << sc, 0, 0, a,
      0, sc, 0, b,
      0, 0, sc, c,
      0, 0, 0, sc;
  pcl::transformPointCloud(*load, *load, translation);
  // pcl::transformPointCloud(*trueNormals, *trueNormals, translation);

  pcl::VoxelGrid<pcl::PointXYZRGBNormal> filter;
  filter.setInputCloud(load);
  // Heavy Downsample
  //  filter.setLeafSize(0.02f, 0.02f, 0.02f);
  // Medium Downsample
  filter.setLeafSize(0.01f, 0.01f, 0.01f);
  // Light/No Downsample
  // filter.setLeafSize(0.001f, 0.001f, 0.001f);
  filter.filter(*load);
  //PCL's normal estimation is much worse than the ycbs, so
  // only use it to estimate curvature
  pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;
  ne.setInputCloud(load);
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.1);
  // ne.setViewPoint(0.0, 0.0, 0.0);
  ne.compute(*displayNormals);
  for(int i = 0; i < load->size(); i++){
    pcl::PointXYZRGBNormal c = load->points.data()[i];
    pcl::Normal n = displayNormals->points.data()[i];
    pcl::PointXYZRGB cPt;
    cPt.x = c.x;
    cPt.y = c.y;
    cPt.z = c.z;
    cPt.r = c.r;
    cPt.g = c.g;
    cPt.b = c.b;
    trueCloud->push_back(cPt);
    pcl::Normal cN;
    cN.normal_x = -c.normal_x;
    cN.normal_y = -c.normal_y;
    cN.normal_z = -c.normal_z;
    cN.curvature = n.curvature;
    trueNormals->push_back(cN);
  }
}

//Adds (phi, theta) to cloud at radius scale
void addSphericalPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<double> angles, double scale = 1.0, int color = 50)
{
  double phi = angles[1];
  double theta = angles[0];
  pcl::PointXYZRGB cPt;
  cPt.x = cos(theta) * sin(phi) * scale;
  cPt.y = sin(theta) * sin(phi) * scale;
  cPt.z = cos(phi) * scale;
  cPt.r = color;
  cPt.g = color;
  // cPt.g = 255;
  cloud->push_back(cPt);
}

// https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
void addSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double scale = 1.0, int nPoints = 720)
{
  double r, theta, phi;
  r = scale;
  int cPoint = 0;
  while(cPoint < nPoints){
    double cI = ((double(cPoint)+.5)/double(nPoints));
    phi = acos(1-(2*cI));
    theta = M_PI * (1 + pow(5.0, .5)) * (double(cPoint)+.5);
    pcl::PointXYZRGB cPt;
    cPt.x = cos(theta) * sin(phi) * scale;
    cPt.y = sin(theta) * sin(phi) * scale;
    cPt.z = cos(phi) * scale;
    // cPt.g = 255;
    cloud->push_back(cPt);
    cPoint++;
  }
}

void addContPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<double> startPose, int steps=0, double scale = 1.0)
{
  std::vector<std::vector<double>> currentPoses, nextPoses;
  //Begin by calculating all the initial explorations
  addSphericalPoint(cloud, {startPose[1], startPose[2]}, scale);
  // for(int dir=0; dir < 8; dir++)
  // {
  //   std::vector<double> next = calcExplorationPoseB(startPose, dir+1);
  //   currentPoses.push_back(next);
  //   addSphericalPoint(cloud, {next[1], next[2]}, scale, 255);
  // }
  currentPoses.push_back(startPose);
  //Only 5 steps are ever taken
  for(int step=0; step < steps; step++)
  {
    // std::cout << currentPoses.size() << std::endl;
		// std::cout << "****************" << std::endl;
    for(int i = 0; i < currentPoses.size(); i++)
    {
      for(int dir=0; dir < 8; dir++)
      {
        std::vector<double> next = calcExplorationPoseB(currentPoses[i], dir+1);
        nextPoses.push_back(next);
        addSphericalPoint(cloud, {next[1], next[2]}, scale, (step+1)*50);
      }
    }
    currentPoses.swap(nextPoses);
    nextPoses.clear();
  }
}

// void distanceTesting(PathFinder* f)
// {
//   //symmetry
//   double a1 = f->sphericalDistance(-1,-1,-1, -1,-1,1);
//   double a2 = f->sphericalDistance(-1,-1,1, -1,-1,-1);
//   // std::cout << a1 << " " << a2 << std::endl;
//   a1 = f->sphericalDistance(-1,-1,-1, -1,1,1);
//   a2 = f->sphericalDistance(-1,-1,1, -1,1,-1);
//   std::cout << a1 << " " << a2 << std::endl;
//   a1 = f->sphericalDistance(-1,0,0, 0,0,1);
//   a2 = f->sphericalDistance(1,0,0, 0,0,1);
//   std::cout << a1 << " " << a2 << std::endl;
//   a1 = f->sphericalDistance(-1,-1,-1, 1,1,1);
//   a2 = f->sphericalDistance(1,1,1, -1,-1,-1);
//   std::cout << a1 << " " << a2 << std::endl;
// }

std::map<std::string, std::vector<int>> buildAngleList()
{
  // Reading the yawValues csv file
  std::string yawAnglesCSVDir = AV_PATH + "misc/yawValues/";
  std::string yawAnglesCSV = "Seed1.csv";
  std::vector<std::vector<std::string>> yawAngle = readCSV(yawAnglesCSVDir + yawAnglesCSV);

  // Converting it to a dictionary format (First column is the key)
  std::map<std::string, std::vector<int>> yawAngleDict;
  for (int row = 0; row < yawAngle.size(); row++)
  {
    yawAngleDict.insert({yawAngle[row][0], {}});
    for (int col = 1; col < yawAngle[row].size(); col++)
    {
      yawAngleDict[yawAngle[row][0]].push_back(std::stoi(yawAngle[row][col]));
    }
  }
  return yawAngleDict;
}

/* Args:
    1- x offset
    2- y offset
    3- z offset
    4- scale factor- 0=100%, 0.5=50%, 0.99=1%
    5- octree grid size
    6- logging level: -2=past run visualization, -1=test run, 0=no logging, 1=logging, 2=verbose logging, graphics
    7- file name to load, can't be blank
    8- increment to start with

*/
int main(int argc, char **argv)
{
  setup(atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), argv[7]);
  std::map<std::string, std::vector<int>> yawAngles = buildAngleList();
  // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  // ne.setInputCloud(trueCloud);
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  // ne.setSearchMethod(tree);
  // ne.setRadiusSearch(0.1);
  // ne.compute(*trueNormals);
  // ne.compute(*displayNormals);
  // trueSize = displayCloud->points.size();
  // addNormals();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_start(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_visible(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_occluded(new pcl::PointCloud<pcl::PointXYZRGB>());
  double theta, phi;
  int cPoint = atoi(argv[8]);
  theta = yawAngles["0-359"][cPoint];
  phi = M_PI/7;
  std::vector<double> source({1.0, theta, phi});
	int depth = 0;
  addContPoints(cloud_sphere, source, depth, 1);
  *cloud_start = *trueCloud + *cloud_sphere;
  // searchVis(cloud_start, cloud_visible, cloud_occluded, atoi(argv[5]), atof(argv[6]), 0, 255, 0);
  // searchVis(cloud_start, cloud_visible, cloud_occluded, atoi(argv[7]), atof(argv[6]), 255, 0, 0);
  // addRGB(viewer, cloud_start, "Object", 1);

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>(atof(argv[5])));
  // addRGB(viewer, trueCloud, "object", 1);
  // addRGB(viewer, cloud_sphere, "sphere", 1);
  std::string file = std::string(argv[7]) + "_discrete_" + std::string(argv[9]) + ".csv";
  std::ofstream file_out;
  file_out.open(file, std::ios_base::app);
  std::cout << "starting" << std::endl;
	PathFinder* f = (new PathFinder(atoi(argv[6]), atoi(argv[6])>1));
  f->initialize(trueCloud, cloud_sphere, cloud_start, trueNormals, octree, atof(argv[9]), true, true);
	bool done = f->getNSteps(depth);
  std::cout << "starting 2" << std::endl;
	while(!done && depth < 4){
		cloud_start->clear();
		cloud_sphere->clear();
		depth += 1;
		// std::cout << cloud_sphere->points.size() << std::endl;
		addContPoints(cloud_sphere, source, depth, 1);
		// std::cout << cloud_sphere->points.size() << std::endl;
		*cloud_start = *trueCloud + *cloud_sphere;
		// f->cleanup();
		PathFinder* x = new PathFinder(atoi(argv[6]), atoi(argv[6])>1);
		f = x;
  	f->initialize(trueCloud, cloud_sphere, cloud_start, trueNormals, octree, atof(argv[9]));
		done = f->getNSteps(depth);
    std::cout << "done w/ depth " << depth << std::endl;
    sleep(1);
	}
  //Assume at least one more step if you're not done yet
  if(!done)
    depth += 1;
  double ret = depth * minAngle * 180 / M_PI;
	// double ret = f->searchFrom(theta, phi);
	std::cout << cPoint << ": " << yawAngles["0-359"][cPoint] << " " << ret << std::endl;
	file_out << theta << "," << phi << "," << ret << std::endl;
  file_out.close();
  
  return 0;
}