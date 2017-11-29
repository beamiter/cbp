#ifndef _CURVEMODEL_H
#define _CURVEMODEL_H

#include <iostream>
#include <ctime>
#include <cstdio>
#include <cmath>
#include <cassert>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

//#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>
#include <boost/heap/binomial_heap.hpp>

#include "dubins.h"
#include "reeds_shepp.h"
#include "visualize.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//using namespace Eigen;
using namespace std;
using namespace cv;

using std::unordered_map;
using std::unordered_set;
using std::array;
using std::vector;
using std::queue;
using std::priority_queue;
using std::pair;
using std::tuple;
using std::tie;
using std::string;

template <typename T, typename P>
/**
* @brief Class indexnode
* \brief
* @param
* @param
* @param
* @return
*/
class indexnode
{
  public:
    //   int index;
    //   float cost;
    std::pair<T, P> node;

    indexnode() {}

    indexnode(std::pair<T, P> ref)
    {
        //Store the path index
        node.first = ref.first;
        //Store the path cost
        node.second = ref.second;
    }

    indexnode(T a, P c)
    {
        //Store the path index
        node.first = a;
        //Store the path cost
        node.second = c;
    }

    ~indexnode() {}

    /**
    * @brief The overload of the operator <
    * \brief
    * @param
    * @param
    * @param
    * @return
    */
    friend bool operator<(const indexnode nd1, const indexnode nd2)
    {
        return (nd1.node.second > nd2.node.second);
    }

    /**
* @brief This matters the sequence of the boost::heap::binomial_heap
* \brief
* @param
* @param
* @param
* @return
*/
    struct comparenode
    {
        bool operator()(const indexnode lhs, const indexnode rhs) const
        {
            return lhs.node.second > rhs.node.second;
        }
    };

    /************************/
    typedef boost::heap::binomial_heap<indexnode, boost::heap::compare<comparenode>> boostqueue;
};

class curvemodel
{
  public:
    curvemodel();

    ~curvemodel();

    void evaluatecurve(std::vector<ReedsSheppStateSpace::ReedsSheppPath> path, indexnode<int, float>::boostqueue &bq,
                       double goal[3], std::vector<geometry_msgs::Point32> goalarray);

    void mypolygon(Mat &img, geometry_msgs::Polygon poly);

    void samplegoal(geometry_msgs::Point32 goal, std::vector<geometry_msgs::Point32> &rtn);

    void run();

  private:
    std::unique_ptr<visualize_pose> visdubins_pose;
    std::unique_ptr<visualize_pose> visreeds_pose;
    std::unique_ptr<visualize_path> visdubins_path;
    std::unique_ptr<visualize_path> visreeds_path;
    std::unique_ptr<visualize_marker> visdubins_marker;
    std::unique_ptr<visualize_marker> visreeds_marker;
    std::unique_ptr<visualize_polygon> vislines;
    int count;
    double radius;
    double q0[3];
    double q1[3];
    std::string dirname;
};

#endif
