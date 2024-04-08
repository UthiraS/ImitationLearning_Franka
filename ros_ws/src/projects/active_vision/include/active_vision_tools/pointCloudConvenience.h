#pragma once

#include <active_vision_tools/commonAV.h>
#include <boost/make_shared.hpp>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/// @brief Stabilizes the given filter so it's voxelgrid boundaries are always in the same locations
/// @param input The cloud to be filtered
/// @param output The location to store the filtered cloud
/// @param filter The filter to apply with a stabilized voxel grid
/// @param limit The boundaries in both directions of the voxel grid. MUST BE LARGER than the input object.
void constantFilter(ptCldColor::Ptr input, ptCldColor::Ptr output, Filter<PointXYZRGB>* filter, float limit=0.5);