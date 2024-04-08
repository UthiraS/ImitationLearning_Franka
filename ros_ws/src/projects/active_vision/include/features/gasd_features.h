#ifndef GASD_FEATURES_H
#define GASD_FEATURES_H

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/gasd.h>
#include <pcl/visualization/histogram_visualizer.h>

#include "features.h"

class GASD_Features : public Features
{
public:
    GASD_Features();

    ~GASD_Features(){}

    void calculateFeatures();

    void vizualize();

    void convertDesctoVec(std::vector<float> &featVector);

private:
    pcl::GASDEstimation<pcl::PointXYZRGB, pcl::GASDSignature512> gasd;
    pcl::PointCloud<pcl::GASDSignature512>::Ptr descriptors;
};

#endif