#include "active_vision_tools/mathConvenience.h"

float radiansToDegrees(float radians)
{
    return radians * 180 / M_PI;
}

vector<int> decimalToOctal(int decimalNumber, int depth)
{
    int rem;
    vector<int> ret;
    if (decimalNumber == 0)
    {
        ret.push_back(1);
    }
    while (decimalNumber != 0)
    {
        rem = decimalNumber % 8;
        decimalNumber /= 8;
        ret.push_back(rem + 1);
    }
    while (ret.size() < depth)
    {
        ret.push_back(1);
    }
    return ret;
}

/*! \brief Convert RGB to HSV color space.
  Converts a given set of RGB values `r', `g', `b' into HSV
  coordinates. The input RGB values are in the range [0, 1], and the
  output HSV values are in the ranges h = [0, 360], and s, v = [0,
  1], respectively.
*/
void RGBtoHSV(float fR, float fG, float fB, float &fH, float &fS, float &fV)
{
    float fCMax = max(max(fR, fG), fB);
    float fCMin = min(min(fR, fG), fB);
    float fDelta = fCMax - fCMin;

    if (fDelta > 0)
    {
        if (fCMax == fR)
            fH = 60 * (fmod(((fG - fB) / fDelta), 6));
        else if (fCMax == fG)
            fH = 60 * (((fB - fR) / fDelta) + 2);
        else if (fCMax == fB)
            fH = 60 * (((fR - fG) / fDelta) + 4);

        if (fCMax > 0)
            fS = fDelta / fCMax;
        else
            fS = 0;

        fV = fCMax;
    }
    else
    {
        fH = 0;
        fS = 0;
        fV = fCMax;
    }

    if (fH < 0)
        fH = 360 + fH;
}

double sphericalDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
    Eigen::Vector3d v1(x1, y1, z1);
    Eigen::Vector3d v2(x2, y2, z2);
    return sphericalDistance(v1, v2);
}

double sphericalDistance(Eigen::Vector3f v1, Eigen::Vector3f v2)
{
    Eigen::Vector3d v1_ = v1.cast<double>();
    Eigen::Vector3d v2_ = v2.cast<double>();
    return sphericalDistance(v1_, v2_);
}

double sphericalDistance(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    if (0 == v1.norm() || 0 == v2.norm())
    {
        cerr << "Neither input can be (0, 0, 0)" << endl;
        return -1;
    }
    v1.normalize();
    v2.normalize();
    double dotP = v1.dot(v2);
    // Vectors with oposite directions will have dotP of 0, but be on the oposite
    //  sides of the sphere. Check for that.
    double magCross = v1.cross(v2).norm();
    double ret = atan(magCross / dotP);
    if (dotP < 0)
    {
        ret += M_PI;
    }
    return ret;
}

double euclideanDistanceSpherical(std::vector<double> poseA, std::vector<double> poseB)
{
    pcl::PointXYZ poseAcart = sphericalToCartesian(poseA);
    pcl::PointXYZ poseBcart = sphericalToCartesian(poseB);
    return (sqrt(pow(poseAcart.x - poseBcart.x, 2) + pow(poseAcart.y - poseBcart.y, 2) + pow(poseAcart.z - poseBcart.z, 2)));
}

bool roughlyEqual(double a, double b)
{
  return (abs(a-b) <= .001*max(abs(a),abs(b)));
}