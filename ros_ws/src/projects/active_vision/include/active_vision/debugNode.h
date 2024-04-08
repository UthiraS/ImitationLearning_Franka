#pragma once
#include "active_vision_tools/commonAV.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/master.h>

#include <unordered_map>


/// @brief List of nodes that can take debug messages
/// @details Note: 0 cannot be the key for any node, or it will break the error checking in getUserInput
const unordered_map<int, string> AVAILABLE_NODES{
  {2, "Camera_Service"},
  {1, "OptimalPathService"},
};

/// @brief Displays the available nodes
void displayNodes();

/// @brief Lets the user select node & debug values
/// @param debugPub Publisher to broadcast the message from
void getUserInput(ros::Publisher debugPub);