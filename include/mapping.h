#ifndef _DRONE_EXPLORER_H_
#define _DRONE_EXPLORER_H_

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/OcTree.h>

#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <algorithm>
#include <queue>


#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

typedef geometry_msgs::PoseStamped geo_pose;

double distance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b);
double distance(octomap::point3d a, geometry_msgs::PoseStamped b);
void print(octomap::point3d, std::string title = "3Dpoint");

#endif