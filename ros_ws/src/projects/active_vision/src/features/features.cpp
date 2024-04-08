#include "../include/features/features.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>

void Features::mergeFeaturesVectors()
{

};

void Features::setInput(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    objCloud = cloud;
}
