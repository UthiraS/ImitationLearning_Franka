#ifndef TOOLVIEWPOINTCALC
#define TOOLVIEWPOINTCALC

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <map>
#include <string>
#include <math.h>
#include <active_vision_tools/mathConvenience.h>

static double minAngle = 10*(M_PI/180.0);
static pcl::PointXYZ defautCentre(0,0,0);

bool checkValidPose(std::vector<double> pose);

bool checkIfNewPose(std::vector<std::vector<double>> &oldPoses, std::vector<double> &pose, int type);

std::vector<double> calcExplorationPoseA(std::vector<double> &startPose, int dir, double step=4.0*minAngle);

std::vector<double> calcExplorationPoseB(std::vector<double> &startPose, int dir, double step=4.0*minAngle);

std::vector<double> calcExplorationPoseC(std::vector<double> &startPose, int dir, double step=minAngle);

std::vector<double> calcExplorationPose(std::vector<double> &startPose, int dir, int mode, double step=minAngle);

#endif
