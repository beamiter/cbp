#ifndef _OPTIMIZATION_PROGRAM_H
#define _OPTIMIZATION_PROGRAM_H

#pragma once

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include "stateheaders.h"

using namespace std;

class Optimizer 
{
public:
    Optimizer();

    virtual ~Optimizer();

    void GetStarted();
    bool Solve(std::vector<sFreeCircle> bubbles);

public:
    std::vector<geometry_msgs::Point32> path;

};

#endif