#include "../include/features/esf_features.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

ESF_Features::ESF_Features()
{
    descriptors = pcl::PointCloud<pcl::ESFSignature640>::Ptr(new pcl::PointCloud<pcl::ESFSignature640>());
    objCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void ESF_Features::calculateFeatures()
{
    esf.setInputCloud(objCloud);
    // Compute the features
    esf.compute(*descriptors);
}

void ESF_Features::convertDesctoVec(std::vector<float> &featVector)
{
    for (float d : (*descriptors)[0].histogram)
    {
        featVector.push_back(d);
    }
}

void ESF_Features::vizualize()
{
    // We need to set the size of the descriptor beforehand.
    viewer.addFeatureHistogram(*descriptors, 640);
    viewer.spin();
}