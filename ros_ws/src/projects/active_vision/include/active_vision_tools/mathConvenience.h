#pragma once


#include <iostream>
#include <Eigen/Dense>
#include <active_vision_tools/commonAV.h>

//--------------Conversion functions------------------

/// @brief Converts radians to degrees
/// @param radians Value to be converted to degrees
/// @return radians in degrees
float radiansToDegrees(float radians);

/// @brief Converts decimalNumber to octal, padded to depth digits
/// @param decimalNumber Decimal value to conver
/// @param depth Size to pad the octal value to
/// @return The octal value, in reverse of the desired order
/// @ref https://www.programiz.com/cpp-programming/examples/octal-decimal-convert
vector<int> decimalToOctal(int decimalNumber, int depth);

/// @brief Convert RGB to HSV color space
/// @param fR Red value in range [0, 1]
/// @param fG Green value in range [0, 1]
/// @param fB Blue value in range [0, 1]
/// @param fH Storage for Hue. Final value will be in range [0, 360]
/// @param fS Storage for Saturation. Final value will be in range [0, 1]
/// @param fV Storage for Value. Final value will be in range [0, 1]
void RGBtoHSV(float fR, float fG, float fB, float &fH, float &fS, float &fV);

/// @brief Converts a spherical pose to cartesian coordinates
/// @tparam PointT pcl point type
/// @param pose (r, phi, theta) to be converted
/// @param center Point the sphere is centered on
/// @return Cartesian coordinates of the point
template <typename PointT = pcl::PointXYZ>
PointT sphericalToCartesian(vector<double> pose, PointT center = PointT(0, 0, 0))
{
  PointT res;
  res.x = center.x + pose[0] * sin(pose[2]) * cos(pose[1]);
  res.y = center.y + pose[0] * sin(pose[2]) * sin(pose[1]);
  res.z = center.z + pose[0] * cos(pose[2]);
  return (res);
}

/// @brief Converts a cartesian coordinate to a spherical pose
/// @tparam PointT pcl point type
/// @param point (r, phi, theta) to be converted
/// @param center Point the sphere is centered on
/// @return (r, phi, theta) spherical representation of the point
template <typename PointT = pcl::PointXYZ>
vector<double> cartesianToSpherical(PointT point, PointT center = PointT(0, 0, 0))
{
  vector<double> res = {0, 0, 0};
  res[0] = sqrt(pow(point.x - center.x, 2) + pow(point.y - center.y, 2) + pow(point.z - center.z, 2));
  res[1] = atan2(point.y - center.y, point.x - center.x);
  res[2] = atan2(sqrt(pow(point.x - center.x, 2) + pow(point.y - center.y, 2)), point.z - center.z);
  return (res);
}

//--------------Spherical distance functions------------------
// Based on https://en.wikipedia.org/wiki/Great-circle_distance

/// @brief Calculate the distance along the viewsphere from (x1,y1,z1) to (x2,y2,z2)
/// @param x1 X coordinate of the first point
/// @param y1 Y coordinate of the first point
/// @param z1 Z coordinate of the first point
/// @param x2 X coordinate of the second point
/// @param y2 Y coordinate of the second point
/// @param z2 Z coordinate of the second point
/// @return Positive distance in the range [0, pi]
double sphericalDistance(double x1, double y1, double z1, double x2, double y2, double z2);

/// @brief Calculate the distance along the viewsphere from v1 to v2
/// @param v1 (x, y, z) representation of the first point
/// @param v2 (x, y, z) representation of the second point
/// @return Positive distance in the range [0, pi]
double sphericalDistance(Eigen::Vector3f v1, Eigen::Vector3f v2);

/// @brief Calculate the distance along the viewsphere from v1 to v2
/// @param v1 (x, y, z) representation of the first point
/// @param v2 (x, y, z) representation of the second point
/// @return Positive distance in the range [0, pi]
double sphericalDistance(Eigen::Vector3d v1, Eigen::Vector3d v2);

/// @brief Calculates the euclidean distance between two spherical coordinates
/// @param poseA (r, phi, theta) representation of point A
/// @param poseB (r, phi, theta) representation of point B
/// @return The euclidean distance between A and B
double euclideanDistanceSpherical(std::vector<double> poseA, std::vector<double> poseB);

//--------------Structure handling functions------------------

/// @brief Helper to copy the spatial
/// @param target Location to store the coordinates
/// @param source Point to extract the spatial coordinates from
template <typename PointT>
void populateArray(double target[], PointT source)
{
  target[0] = source.x;
  target[1] = source.y;
  target[2] = source.z;
}

/// @brief Helper to check vector for element
/// @tparam T Type of the vector
/// @param vec Vector to inspect
/// @param element Element to search for
/// @return Whether the vector contained the element
/// @ref https://stackoverflow.com/questions/5998576/how-do-i-check-if-a-value-is-contained-in-a-vector-c
template <typename T>
bool vectorContainsElement(vector<T> vec, T element){
  return find(vec.begin(), vec.end(), element) != vec.end();
}

/// @brief Checks if two doubles are within .1% of eachother
/// @param a First double to compare
/// @param b Second double to compare
/// @return If the absolute difference between a and b is < .1% of the larger of the two
bool roughlyEqual(double a, double b);