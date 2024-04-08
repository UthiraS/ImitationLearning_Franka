#ifndef ESF_FEATURES_H
#define ESF_FEATURES_H

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/esf.h>
#include <pcl/visualization/histogram_visualizer.h>

#include "features.h"

class ESF_Features : public Features
{
public:
    ESF_Features();

    ~ESF_Features() {}

    void calculateFeatures();

    void vizualize();

    void convertDesctoVec(std::vector<float> &featVector);

private:
    pcl::ESFEstimation<pcl::PointXYZRGB, pcl::ESFSignature640> esf;
    pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptors;
};

#endif