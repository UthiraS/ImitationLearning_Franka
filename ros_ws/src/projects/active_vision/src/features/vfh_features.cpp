#include "../include/features/vfh_features.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

VFH_Features::VFH_Features()
{
    tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    descriptors = pcl::PointCloud<pcl::VFHSignature308>::Ptr(new pcl::PointCloud<pcl::VFHSignature308>());
    objCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void VFH_Features::calculateFeatures()
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(objCloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    vfh.setInputCloud(objCloud);
    vfh.setInputNormals(normals);
    vfh.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    vfh.setRadiusSearch(0.05);
    
    // Compute the features
    vfh.compute(*descriptors);
}

void VFH_Features::convertDesctoVec(std::vector<float> &featVector)
{
    //pcl::VFHSignature308 descriptor = (*descriptors)[0];
    for (float d: (*descriptors)[0].histogram)
    {
        featVector.push_back(d);
    }
}

void VFH_Features::vizualize()
{
    // We need to set the size of the descriptor beforehand.
    viewer.addFeatureHistogram(*descriptors, 308);
    viewer.spin();
}
