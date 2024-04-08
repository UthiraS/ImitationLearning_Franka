#include <active_vision_tools/pointCloudConvenience.h>
#include <active_vision_tools/mathConvenience.h>

void constantFilter(ptCldColor::Ptr input, ptCldColor::Ptr output, Filter<PointXYZRGB>* filter, float limit)
{
  PointXYZRGB minPoint;
  PointXYZRGB maxPoint;
  minPoint.x = -limit;
  minPoint.y = -limit;
  minPoint.z = -limit;
  maxPoint.x = limit;
  maxPoint.y = limit;
  maxPoint.z = limit;
  input->points.push_back(minPoint);
  input->points.push_back(maxPoint);
  filter->setInputCloud(input);
  filter->filter(*output);
  input->points.pop_back();
  input->points.pop_back();
  if(input == output){
    return;
  }
  output->points.pop_back();
  output->points.pop_back();
}

