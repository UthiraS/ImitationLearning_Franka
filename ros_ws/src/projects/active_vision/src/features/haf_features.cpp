#include "../include/features/haf_features.h"

#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <memory>

HAF_Features::HAF_Features()
{
    objCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    objCloudunexp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    this->setGridDim(5);
    this->setMaintainScale(true);
}

void HAF_Features::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrPtCld, pcl::PointXYZRGB min, pcl::PointXYZRGB max)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cPtrPtCld{ptrPtCld};
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cPtrPtCld);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(min.x, max.x);
    pass.filter(*ptrPtCld);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(min.y, max.y);
    pass.filter(*ptrPtCld);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(min.z, max.z);
    pass.filter(*ptrPtCld);
}

void HAF_Features::setGridDim(int dim)
{
    gridDim = dim;
}

void HAF_Features::setMaintainScale(bool val)
{
    maintainScale = val;
}

void HAF_Features::calculateFeatures()
{
    Eigen::Vector3f pt;
    float gridSize[2] = {};
    int gridLoc[2] = {};

    pcl::PointXYZRGB minPtObj, maxPtObj;
    pcl::getMinMax3D(*objCloud, minPtObj, maxPtObj);

    if (maintainScale == true)
    {
        float midPointX, midPointY;
        midPointX = (maxPtObj.x + minPtObj.x) / 2;
        midPointY = (maxPtObj.y + minPtObj.y) / 2;
        if ((maxPtObj.x - minPtObj.x) > (maxPtObj.y - minPtObj.y))
        {
            minPtObj.y = midPointY - (maxPtObj.x - minPtObj.x) / 2;
            maxPtObj.y = midPointY + (maxPtObj.x - minPtObj.x) / 2;
        }
        else
        {
            minPtObj.x = midPointX - (maxPtObj.y - minPtObj.y) / 2;
            maxPtObj.x = midPointX + (maxPtObj.y - minPtObj.y) / 2;
        }
    }

    float stateObj[gridDim][gridDim] = {};
    gridSize[0] = (maxPtObj.x - minPtObj.x) / gridDim;
    gridSize[1] = (maxPtObj.y - minPtObj.y) / gridDim;

    for (int i = 0; i < objCloud->size(); i++)
    {
        pt = objCloud->points[i].getVector3fMap();
        pt[0] = pt[0] - minPtObj.x;
        pt[1] = pt[1] - minPtObj.y;
        pt[2] = pt[2] - minPtObj.z;
        if (pt[0] < 0 || pt[1] < 0 || pt[2] < 0)
            printf("ERROR HAFStVec1.calculate : Object point outside\n");

        gridLoc[0] = (int)(pt[0] / gridSize[0]);
        gridLoc[0] = std::min(gridLoc[0], gridDim - 1); // Ensuring that the grid loc for the extreme point is inside the grid
        gridLoc[0] = (gridDim - 1) - gridLoc[0];        // Reversing so that the array and grid indices match
        gridLoc[1] = (int)(pt[1] / gridSize[1]);
        gridLoc[1] = std::min(gridLoc[1], gridDim - 1); // Ensuring that the grid loc for the extreme point is inside the grid

        stateObj[gridLoc[0]][gridLoc[1]] = std::max(stateObj[gridLoc[0]][gridLoc[1]], pt[2]);
    }

    float scale = 3;
    pcl::PointXYZRGB minPtUnexp, maxPtUnexp;
    minPtUnexp.x = minPtObj.x - (scale - 1) * (maxPtObj.x - minPtObj.x) / 2;
    minPtUnexp.y = minPtObj.y - (scale - 1) * (maxPtObj.y - minPtObj.y) / 2;
    minPtUnexp.z = minPtObj.z;
    maxPtUnexp.x = maxPtObj.x + (scale - 1) * (maxPtObj.x - minPtObj.x) / 2;
    maxPtUnexp.y = maxPtObj.y + (scale - 1) * (maxPtObj.y - minPtObj.y) / 2;
    maxPtUnexp.z = maxPtObj.z + 0.05;

    passThroughFilter(objCloudunexp, minPtUnexp, maxPtUnexp);

    float stateUnexp[gridDim][gridDim] = {};
    gridSize[0] = (maxPtUnexp.x - minPtUnexp.x) / gridDim;
    gridSize[1] = (maxPtUnexp.y - minPtUnexp.y) / gridDim;
    for (int i = 0; i < objCloudunexp->size(); i++)
    {
        pt = objCloudunexp->points[i].getVector3fMap();
        pt[0] = pt[0] - minPtUnexp.x;
        pt[1] = pt[1] - minPtUnexp.y;
        pt[2] = pt[2] - minPtUnexp.z;
        if (pt[0] < 0 || pt[1] < 0 || pt[2] < 0)
            printf("ERROR HAFStVec1.calculate : Unexp point outside\n");

        gridLoc[0] = (int)(pt[0] / gridSize[0]);
        gridLoc[0] = std::min(gridLoc[0], gridDim - 1); // Ensuring that the grid loc for the extreme point is inside the grid
        gridLoc[0] = (gridDim - 1) - gridLoc[0];        // Reversing so that the array and grid indices match
        gridLoc[1] = (int)(pt[1] / gridSize[1]);
        gridLoc[1] = std::min(gridLoc[1], gridDim - 1); // Ensuring that the grid loc for the extreme point is inside the grid
        stateUnexp[gridLoc[0]][gridLoc[1]] = std::max(stateUnexp[gridLoc[0]][gridLoc[1]], pt[2]);
    }

    descriptors.clear();

    // Storing both in the state vector
    for (int i = 0; i < gridDim; i++)
    {
        for (int j = 0; j < gridDim; j++)
        {
            descriptors.push_back(stateObj[i][j] * 100);
        }
    }
    for (int i = 0; i < gridDim; i++)
    {
        for (int j = 0; j < gridDim; j++)
        {
            descriptors.push_back(stateUnexp[i][j] * 100);
        }
    }
}

void HAF_Features::setInputUnexp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr objCloudunexpPtr)
{
    objCloudunexp = objCloudunexpPtr;
}

void HAF_Features::convertDesctoVec(std::vector<float> &featVector)
{
    //pcl::HAFSignature308 descriptor = (*descriptors)[0];
    for (float d : descriptors)
    {
        featVector.push_back(d);
    }
}

void HAF_Features::vizualize()
{
    // We need to set the size of the descriptor beforehand.
}
