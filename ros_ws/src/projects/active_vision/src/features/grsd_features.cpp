#include "../include/features/grsd_features.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

GRSD_Features::GRSD_Features()
{
    tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    descriptors = pcl::PointCloud<pcl::GRSDSignature21>::Ptr(new pcl::PointCloud<pcl::GRSDSignature21>());
    objCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void GRSD_Features::calculateFeatures()
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(objCloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    grsd.setInputCloud(objCloud);
    grsd.setInputNormals(normals);
    grsd.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    grsd.setRadiusSearch(0.05);

    // Compute the features
    grsd.compute(*descriptors);
}

void GRSD_Features::convertDesctoVec(std::vector<float> &featVector)
{
    for (float d: (*descriptors)[0].histogram)
    {
        featVector.push_back(d);
    }
}

void GRSD_Features::vizualize()
{
    // We need to set the size of the descriptor beforehand.
    viewer.addFeatureHistogram(*descriptors, 21);
    viewer.spin();
}
