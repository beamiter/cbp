#ifndef _SPACE_EXPLORATION_HEURISTIC_SEARCH_H
#define _SPACE_EXPLORATION_HEURISTIC_SEARCH_H

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
#include <stack>
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

#include "functions.h"
#include "visualize.h"

#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgobj.h"
#include "ivpathplanner/path.h"
#include "ivpredict/ivmsgpredict.h"

#include "../../../../../avoslib/geotool.h"
#include "../../../../../avoslib/iabasemaptool.h"

#include "Optimizer.h"
#include "stateheaders.h"
#include "spline.h"

using namespace std;
using namespace cv;

#ifndef MACRO
#define MACRO
#endif

class sehs
{
public:

    sehs();

    ~sehs();

    bool FreeSpaceEstimate(sFreeCircle current);

    bool GenerateCircle(sKinodynamic current, sFreeCircle& cfree);
    bool SampleCircle(sFreeCircle current, sFreeCircle& cfree);

    /**
    * @brief Return the end point of the generated path
    */
    std::vector<std::tuple<sKinodynamic, std::vector<sKinodynamic>, float>>
    PrimitiveActions(sKinodynamic currentvs, float step_length);

    std::tuple<int, float> HeuristicIndex(int current_index, sKinodynamic current_status, 
    const std::vector<std::pair<sFreeCircle, float> >& circle_array);
    
    bool HeuristicSearch(sKinodynamic start, sKinodynamic goal, std::vector<sKinodynamic>& trajectory);

    bool SnakeCrawl(sKinodynamic start, sKinodynamic goal, std::vector<sKinodynamic>& trajectory);

    std::vector<sFreeCircle> LookAheadDistance(sKinodynamic current);

    float ForwardDrive(sKinodynamic current, std::vector<sFreeCircle> bubbles);

    std::vector<std::tuple<sKinodynamic, std::vector<sKinodynamic>, float>>
    ClosedLoopPrediction(sKinodynamic current);

    bool WavefrontExpansion(sKinodynamic start, sKinodynamic goal, int sample_number);

    bool SpaceExploration(sKinodynamic start, sKinodynamic goal);

    bool OverLap(sFreeCircle current, sFreeCircle comparison, float margin);

    bool NotExist(sFreeCircle current, std::vector<sFreeCircle> closed_vector);

    bool ReachedGoal(sKinodynamic currentvs, sKinodynamic goalvs);

    void testModule();

    ivpathplanner::path planning(ivmap::ivmapmsglocpos ivlocpos, ivpathplanner::path current, 
    ivpathplanner::path rawpath, sortedTree sortedobjs);

private:

    sControlLimits dynamic_constraints;
    sVehicleModel omega;

    // Global value makes the algorithm work better and faster
    sortedTree cObs;

    std::unique_ptr<visualize_marker> visdwa_marker;

    std::vector<std::pair<sFreeCircle, float> > circle_path;
    std::vector<geometry_msgs::Point32> path_points;
    std::vector<sKinodynamic> waypoint;

    std::multimap<sKinodynamic, sKinodynamic> virtualgoal;

    sKinodynamic start;
    sKinodynamic goal;

    float disttolerance;
    float yawtolerance;

    sPointOfGCCS egoPos;

    /*iavasemaptool class loader*/
    iabasemaptool mapTool;
    ivsensorgps::ivsensorgps iabasemapTL;
    ivsensorgps::ivsensorgps iabasemapBR;
    std::string mapPath;
    double gicsCellSize;

};


#endif
