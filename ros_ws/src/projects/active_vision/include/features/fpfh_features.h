#ifndef FPFH_FEATURES_H
#define FPFH_FEATURES_H

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/histogram_visualizer.h>

#include "features.h"

class FPFH_Features : public Features
{
public:
    FPFH_Features();

    ~FPFH_Features(){}

    void calculateFeatures();

    void vizualize();

    void convertDesctoVec(std::vector<float> &featVector);

private:
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors;
};

#endif