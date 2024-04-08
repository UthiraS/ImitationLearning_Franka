#pragma once

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <stdio.h>
#include <string>
#include "std_msgs/String.h"
#include <signal.h>

using namespace std;
using namespace pcl;

typedef PointXYZRGB ptColor;
typedef PointCloud<ptColor> ptCldColor;
typedef PointCloud<PointNormal> ptCldNormal;

// string AV_PATH = "/home/rbe07/mer_lab/ros_ws/src/projects/active_vision/";
const static string AV_PATH = "/home/uthira/mer_lab/ros_ws/src/projects/active_vision/";

enum policyModes{
	MANUAL,
	HEURISTIC,
	TRAINED
};

/// @brief Handles user interupts
/// @param s Interupt signal
/// @ref https://stackoverflow.com/questions/1641182/how-can-i-catch-a-ctrl-c-event
void dieWithGrace(sig_atomic_t s);

/// @brief Converts a node name and a debug level to a single string
/// @param node Node name
/// @param debugLevel Desired debug level
/// @return Combined string, formatted for debug message
string toDebugMessage(string node, int debugLevel);

/// @brief Converts a formatted string to a node name and debug level
/// @param message Formatted debug message
/// @return Split string
pair<string, int> fromDebugMessage(string message);

void log(string message, string file, string func, int line, int logLevel, int targetSeverity);

//https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define AVLOG(message, logLevel, targetSeverity) log(message, __FILENAME__, __func__, __LINE__, logLevel, targetSeverity)
#define PLOG AVLOG("", 0, 0)