#ifndef _ELASTIC_DWA_PLANNER_H
#define _ELASTIC_DWA_PLANNER_H

//c++ lib
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <algorithm>
#include <vector>
#include <deque>
#include <queue>
#include <iterator>
#include <cmath>
#include <iostream>
#include <ctime>
#include <cstdio>
#include <cassert>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <algorithm>

#include "ros/callback_queue.h"
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/heap/binomial_heap.hpp>
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

//other lib

#include <bitset>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <deque>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "visualize.h"
#include "reeds_shepp.h"
#include "dubins.h"
#include "functions.h"

#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgobj.h"
#include "ivpathplanner/path.h"

#include "ivpredict/ivmsgpredict.h"

#include "hermite_curve.h"
#include "commonheaders.h"

#include "../../../../../avoslib/geotool.h"
#include "../../../../../avoslib/iabasemaptool.h"

using namespace std;
using namespace cv;

#ifndef MACRO
#define MACRO
#define USE_FREE_SPACE 0
#define STOP_AND_WAIT 1
#endif

class dynamicapproach
{
public:
    dynamicapproach(ros::NodeHandle nh);
    dynamicapproach();
    ~dynamicapproach();

    int collisiondetection(ivpathplanner::path ego, sorted_tree &obspts, sVehicleElem vehiclemodel);
    int collisiondetection(std::vector<geometry_msgs::Point32> ego, sorted_tree &obspts, sVehicleElem vehiclemodel);
    int collisiondetection(std::vector<sVehicleStatus> ego, sorted_tree& obspts,
                           sVehicleStatus goalvs, bool& reachedGoal, sVehicleElem vehiclemodel);

    float hValUpdate(sVehicleStatus cur, sVehicleStatus goal);
    float gValUpdate(sVehicleStatus cur, sVehicleStatus next);

    sVehicleStatus dynamicWindowApproach(sVehicleStatus start, sVehicleStatus goal, unordered_t<sVehicleStatus> &came_from,
    unordered_t<float> &cost_so_far, unordered_t<std::vector<sVehicleStatus>> &pathtable, sorted_tree sortedobjs);

    /**
    * @brief Return the end point of the generated path
    */
    std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>>
    GenerateTrajectory(sVehicleStatus currentvs, sKinematic sk);

    void obstaclemodify(ivpredict::ivmsgpredict objs, sorted_tree &nearestobs,
    sorted_tree &rawobs, sorted_tree &dilatedobs, float carspeed);

    int reachgoal(sVehicleStatus currentvs, sVehicleStatus goalvs);

    std::vector<sVehicleStatus> reconstructPath(sVehicleStatus start, sVehicleStatus goal, 
    unordered_t<sVehicleStatus> &came_from, sVehicleStatus last);

    ivpathplanner::path pathinterface(ivmap::ivmapmsglocpos ivlocpos, ivpathplanner::path current, 
    ivpathplanner::path rawpath, sorted_tree sortedobjs);

    void testModule();

private:

    float stepsize;
    float steplength;
    float controlCycle;

    // degrees
    float yawtolerance;
    // m
    float disttolerance;
    // m/s
    float speedtolerance;

    bool REVERSE;
    bool reachedGoal;

    std::unordered_map<int, int> samplenum;
    std::vector<std::vector<sVehicleStatus>> trajectorys;
    std::vector<sVehicleStatus> trajectory;

    /**Computance index**/
    sVehicleStatus initialstatus, goalstatus;
    sKinematic dyn;

    std::unique_ptr<visualize_marker> visdwa_marker;

    /*iavasemaptool class loader*/
    iabasemaptool mapTool;
    ivsensorgps::ivsensorgps iabasemapTL;
    ivsensorgps::ivsensorgps iabasemapBR;
    std::string mapPath;
    double gicsCellSize;

    sPointOfGCCS egoPos;

};

#endif
