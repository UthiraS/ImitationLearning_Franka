#ifndef GRSD_FEATURES_H
#define GRSD_FEATURES_H

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/grsd.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/histogram_visualizer.h>

#include "features.h"

class GRSD_Features : public Features
{
public:
    GRSD_Features();

    ~GRSD_Features() {}

    void calculateFeatures();

    void vizualize();

    void convertDesctoVec(std::vector<float> &featVector);

private:
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::GRSDEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::GRSDSignature21> grsd;
    pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptors;
};

#endif