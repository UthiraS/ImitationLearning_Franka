#ifndef TOOLVISUALIZATION
#define TOOLVISUALIZATION

#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include <map>
#include <string>
#include <math.h>
#include <pcl/octree/octree_pointcloud.h>
#include <active_vision_tools/commonAV.h>


// Typedef for convinience
typedef pcl::visualization::PCLVisualizer ptCldVis;

extern std::map<int, std::string> dirLookup;

void setupViewer(ptCldVis::Ptr viewer, int nVps, std::vector<int> &vp);

// Fuction to add a rgb point cloud
void addRGB(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, std::string name,int vp);

// Fuction to add a rgb point cloud with normals
void addRGBN(ptCldVis::Ptr viewer, ptCldColor::ConstPtr cloud, ptCldNormal::ConstPtr normal, std::string name,int vp);

// Fuction to add the viewsphere
void addViewsphere(ptCldVis::Ptr viewer, int vp, pcl::PointXYZ centre, double &rad, bool all);

void addHelp(ptCldVis::Ptr viewer, int size, int vp);

void setCamView(ptCldVis::Ptr viewer, std::vector<double> pose, pcl::PointXYZ &centre);

void setCamView(ptCldVis::Ptr viewer, std::vector<double> pose, pcl::PointXYZ &centre, int vp);

void showVoxelBounds(ptCldVis::Ptr viewer, pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::Ptr oct, int vp);

class keyboardEvent{
public:
  int type;
  bool called;
  int counter;
  bool ok;
  bool skipTo;
  int dir;
  int mode;

  keyboardEvent();
  keyboardEvent(ptCldVis::Ptr viewer,int num);
  void setup(ptCldVis::Ptr viewer,int num);
  void help();
  void keyboardEventOccurredA(const pcl::visualization::KeyboardEvent &event, void*);
  void keyboardEventOccurredB(const pcl::visualization::KeyboardEvent &event, void*);
  void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void*);
};

#endif
