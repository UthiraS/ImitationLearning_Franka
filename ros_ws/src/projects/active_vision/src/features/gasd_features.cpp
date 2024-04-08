#include "../include/features/gasd_features.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

GASD_Features::GASD_Features()
{
    descriptors = pcl::PointCloud<pcl::GASDSignature512>::Ptr(new pcl::PointCloud<pcl::GASDSignature512>());
    objCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void GASD_Features::calculateFeatures()
{
    gasd.setInputCloud(objCloud);

    // Compute the features
    try
    {
        gasd.compute(*descriptors);
    } catch (...) {
        std::cerr << "computation error" << std::endl;
    }
}

void GASD_Features::convertDesctoVec(std::vector<float> &featVector)
{
    for (float d: (*descriptors)[0].histogram)
    {
        featVector.push_back(d);
    }
}

void GASD_Features::vizualize()
{
    // We need to set the size of the descriptor beforehand.
    viewer.addFeatureHistogram(*descriptors, 512);
    viewer.spin();
}
