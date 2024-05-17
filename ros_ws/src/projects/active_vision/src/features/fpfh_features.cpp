#include "../include/features/fpfh_features.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

FPFH_Features::FPFH_Features()
{
    tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    descriptors = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>());
    objCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void FPFH_Features::calculateFeatures()
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(objCloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    fpfh.setInputCloud(objCloud);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch(0.05);

    fpfh.compute(*descriptors);
}

void FPFH_Features::convertDesctoVec(std::vector<float> &featVector)
{
    for (float d: (*descriptors)[0].histogram)
    {
        featVector.push_back(d);
    }    
    
}

void FPFH_Features::vizualize()
{
    // We need to set the size of the descriptor beforehand.
    viewer.addFeatureHistogram(*descriptors, 33);
    viewer.spin();
}
