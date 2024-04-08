#ifndef HAF_FEATURES_H
#define HAF_FEATURES_H

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include "features.h"

class HAF_Features : public Features
{
public:
    HAF_Features();

    ~HAF_Features(){};

    void calculateFeatures();

    void vizualize();

    void convertDesctoVec(std::vector<float> &featVector);

    void setInputUnexp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloudunexpPtr);

private:
    int gridDim;
    bool dataSet = false;
    bool maintainScale = false;

    void setGridDim(int dim);
    void setMaintainScale(bool val);
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPtCld, pcl::PointXYZRGB min, pcl::PointXYZRGB max);
    std::vector<float> descriptors;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloudunexp;
};

#endif