#include "../include/features/cvfh_features.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

CVFH_Features::CVFH_Features()
{
    tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    descriptors = pcl::PointCloud<pcl::VFHSignature308>::Ptr(new pcl::PointCloud<pcl::VFHSignature308>());
    objCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void CVFH_Features::calculateFeatures()
{
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(objCloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);

    cvfh.setInputCloud(objCloud);
    cvfh.setInputNormals(normals);
    cvfh.setSearchMethod(tree);

    // Set the maximum allowable deviation of the normals,
    // for the region segmentation step.
    cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.

    // Set the curvature threshold (maximum disparity between curvatures),
    // for the region segmentation step.
    cvfh.setCurvatureThreshold(1.0);

    // Set to true to normalize the bins of the resulting histogram,
    // using the total number of points. Note: enabling it will make CVFH
    // invariant to scale just like VFH, but the authors encourage the opposite.
    cvfh.setNormalizeBins(true);

    // Compute the features
    cvfh.compute(*descriptors);
}

void CVFH_Features::convertDesctoVec(std::vector<float> &featVector)
{
    for (float d: (*descriptors)[0].histogram)
    {
        featVector.push_back(d);
    }
}

void CVFH_Features::vizualize()
{
    // We need to set the size of the descriptor beforehand.
    viewer.addFeatureHistogram(*descriptors, 308);
    viewer.spin();
}
