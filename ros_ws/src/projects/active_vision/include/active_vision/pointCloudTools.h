#ifndef POINT_CLOUD_TOOLS
#define POINT_CLOUD_TOOLS

#include <active_vision/toolDataHandling.h>
#include <algorithm>
#include <pcl/common/centroid.h>
#include <pcl/search/search.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/linear_least_squares_normal.h>
#include <pcl/octree/octree_pointcloud.h>
#include <active_vision_tools/pointCloudConvenience.h>
using namespace std;
using namespace pcl;



bool roughlyEqual(double a, double b);

void addRect(PointCloud<PointXYZRGB>::Ptr cloud, double scaleX = 1.0, double scaleY = 1.0, double scaleZ = 1.0, double division = .1);

void addCube(PointCloud<PointXYZRGB>::Ptr cloud, double scale = 1.0, double division = .1);

// https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
void addSphere(PointCloud<PointXYZRGB>::Ptr cloud, double scale = 1.0, int nPoints = 720, bool fullSphere = false);

void addCircle(PointCloud<PointXYZRGB>::Ptr cloud, double scale = 1.0, int nPoints = 360);

// template <typename PointT> void centerClouds(vector<boost::shared_ptr<PointCloud<PointT>>> clouds);
// template <typename PointT> void centerCloud(boost::shared_ptr<PointCloud<PointT>> cloud);

bool findNormals(ptCldColor::Ptr ptrPtCldObject, ptCldNormal::Ptr ptrObjNormal, int knn=9);

ptCldNormal::Ptr refineNormals(ptCldColor::Ptr ptrPtCldObject, ptCldNormal::Ptr ptrObjNormal, int k);

template <typename PointT> void applyCentering(boost::shared_ptr<PointCloud<PointT>> cloud, PointT center){
  for(int i=0; i < cloud->size(); i++){
    cloud->points[i].x -= center.x;
    cloud->points[i].y -= center.y;
    cloud->points[i].z -= center.z;
  }
}

//Centers any number of clouds to the centroid of the first in the vector
template <typename PointT> PointT centerClouds(vector<boost::shared_ptr<PointCloud<PointT>>> clouds)
{
  CentroidPoint<PointT> centroid;
  for(int i=0; i < clouds[0]->size(); i++){
    centroid.add(clouds[0]->points[i]);
  }
  PointT c;
  centroid.get(c);
  for(int i=0; i<clouds.size(); i++){
    applyCentering(clouds[i], c);
  }
  return c;
}

//Centers any number of clouds to the middle x/y point of the first in the vector
template <typename PointT> PointT centerAxisClouds(vector<boost::shared_ptr<PointCloud<PointT>>> clouds)
{
  CentroidPoint<PointT> centroid;
  PointT min;
  PointT max;
  getMinMax3D(*(clouds[0]), min, max);
  PointT c;
  c.x = (max.x+min.x)/2.0;
  c.y = (max.y+min.y)/2.0;
  c.z = (max.z+min.z)/2.0;
  for(int i=0; i<clouds.size(); i++){
    applyCentering(clouds[i], c);
  }
  return c;
}

//Center a single cloud
template <typename PointT> void centerCloud(boost::shared_ptr<PointCloud<PointT>> cloud)
{
  centerClouds<PointT>(vector<boost::shared_ptr<PointCloud<PointT>>>{cloud});
}


template <typename PointT> 
void downsampleCloud(boost::shared_ptr<PointCloud<PointT>> input, float resolution){
  octree::OctreePointCloud<PointT> tree(resolution);
  vector<PointT, Eigen::aligned_allocator<PointT>> ret;
  tree.setInputCloud(input);
  tree.addPointsFromInputCloud();
  tree.getOccupiedVoxelCenters(ret);
  input->clear();
  for(int i=0; i<ret.size(); i++){
    input->points.push_back(ret[i]);
  }
  // input->assign(ret.begin(), ret.end());
}
#endif