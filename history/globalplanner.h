#ifndef _GLOBAL_PLANNER_H
#define _GLOBAL_PLANNER_H

//c++ lib
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <vector>
#include <string>
#include <deque>
#include <queue>
#include <iterator>
#include <cmath>
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

//other lib

#include <bitset>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <deque>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "functions.h"

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace cv;


class globalplanner
{
public:
    globalplanner();
    ~globalplanner();

    int globalpath(int start, int goal, 
    std::unordered_map<int, std::vector<int> > edges,
    std::unordered_map<int, int>& came_from,
    std::unordered_map<int, float>& cost_so_far);

    bool globalpath(int start, int goal, 
    std::unordered_map<int, int>& came_from,
    std::unordered_map<int, float>& cost_so_far);

    std::vector<int> reconstruct_path(
    int start, int goal, std::unordered_map<int, int> came_from);

    void initParam();

    void run();


private:

    int start;
    int goal;
    
    std::unordered_map<int, std::vector<int> > edges;

};

#endif