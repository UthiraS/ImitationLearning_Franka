#ifndef FEATURES_H
#define FEATURES_H

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/histogram_visualizer.h>

class Features
{
public:
  // Constructor
  Features(){};

  // Destructor
  virtual ~Features(){};

  void setInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  virtual void calculateFeatures() = 0;

  virtual void vizualize() = 0;

  virtual void convertDesctoVec(std::vector<float> &featVector) = 0;

  static void mergeFeaturesVectors();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloud;

  // Plotter object.
  pcl::visualization::PCLHistogramVisualizer viewer;

private:
};

#endif