#ifndef VFH_FEATURES_H
#define VFH_FEATURES_H

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/histogram_visualizer.h>

#include "features.h"

class VFH_Features : public Features
{
public:
    VFH_Features();

    ~VFH_Features(){}

    void calculateFeatures();

    void vizualize();

    void convertDesctoVec(std::vector<float> &featVector);

private:
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> vfh;
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors;
};

#endif