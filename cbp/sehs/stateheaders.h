#ifndef _STATE_HEADERS_H
#define _STATE_HEADERS_H

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

#include "functions.h"
#include "visualize.h"

#include "ivmap/ivmapmsglocpos.h"
#include "ivmap/ivmapmsgobj.h"
#include "ivpathplanner/path.h"
#include "ivpredict/ivmsgpredict.h"

#include "../../../../../avoslib/geotool.h"
#include "../../../../../avoslib/iabasemaptool.h"

using namespace std;
using namespace cv;

#define BDEBUG false

struct sVehicleModel
{
    float halfWidth;
    float halfLength;
    float wheelBase;
    float frontAxle;
    float tailAxle;
    // l fw is the distance of the forward anchor point from the rear axle
    float forwardAnchor;
    // l rv is the rearward distance of the reverse anchor point from the rear axle
    float rearwardAnchor;
};

struct sControlLimits
{
    // m/s
    float maxSpeed;
    // m/s
    float minSpeed;
    // rad/s
    float maxSteeringSpeed;
    // m/s^2
    float maxAcceleration;
    // rad
    float maxSteeringAngle;
    // m/s^2
    float maxDecceleration;
    // m
    float minTurningRadius;
    // m/s^2
    float centrifugalAcceleration;
};


class sKinodynamic
{
public:
    sKinodynamic(){}
    sKinodynamic(float x0, float y0, float theta0, float speed0, float steeringAngle0)
    : x(x0), y(y0), theta(theta0), speed(speed0), steeringAngle(steeringAngle0)
    {
    }
    ~sKinodynamic(){}

    /**
     * \brief The overload of the operator "=="
     *
     */
    bool operator== (const sKinodynamic &rhs)
    {
        return (fabs(theta - rhs.theta) <= 1e-3 && fabs(x - rhs.x) <= 1e-3 && fabs(y - rhs.y) <= 1e-3 &&
            fabs(speed - rhs.speed) <= 1e-3 && fabs(steeringAngle - rhs.steeringAngle) <= 1e-3);
    }

    sKinodynamic& operator= (const sKinodynamic &rhs)
    {
        this->steeringAngle = rhs.steeringAngle;
        this->speed = rhs.speed;
        this->x = rhs.x;
        this->y = rhs.y;
        this->theta = rhs.theta;
        return *this;
    }

    /**
     * \brief The overload of the operator "<"
     *
     */
    friend bool operator<(const sKinodynamic lhs, const sKinodynamic rhs)
    {
        return (lhs.x < rhs.x);
    }


    // m
    float x;
    float y;
    // rad
    float theta;
    // m/s
    float speed;
    // rad
    float steeringAngle;
    // time, s
    float secs;
};

class sFreeCircle
{
public:
    sFreeCircle(){}
    sFreeCircle(const sFreeCircle& ref)
    {
        center = ref.center;
        radius = ref.radius;
    }
    ~sFreeCircle(){}

    /*Overload of operator*/
    sFreeCircle& operator= (const sFreeCircle &rhs)
    {
        this->center = rhs.center;
        this->radius = rhs.radius;
        return *this;
    }
    bool operator== (const sFreeCircle &rhs)
    {
        return (center == rhs.center && fabs(radius - rhs.radius) <= 1e-3);
    }

    sKinodynamic center;
    float radius;
    bool forward;
};


/**
* @brief 函数类
 * @brief 作为hash_map的比较函数
 * @brief 查找的时候不同的key往往可能会用到相同的hash值
*/
template <typename T>
struct hashCompare
{
    bool operator()(const T &lhs, const T &rhs) const
    {
        return (fabs(lhs.theta - rhs.theta) < 1e-3 && fabs(lhs.x - rhs.x) < 1e-3 && fabs(lhs.y - rhs.y) < 1e-3);
    }
};

/**
 * @brief 模板的特例化
 * @brief 函数类
 * @brief 作为hash_map的hash函数
 * @brief class没有默认的hash函数
 * @brief overload of the operation "()"
 */
namespace std
{
    template <>
    struct hash<sKinodynamic>
    {
        inline size_t operator()(const sKinodynamic &vs) const
        {
            std::hash<float> hash_fn;
            return size_t(hash_fn(vs.x + vs.y + vs.theta));
        }
    };
}

/**
 * @brief 函数类
 * @brief 作为hash_map的hash函数
 * @brief class没有默认的hash函数
 * @brief overload of the operation "()"
 */
template <typename T>
class hashKey
{
public:
    size_t operator()(const T &str) const
    {
        std::hash<float> hash_fn;
        return size_t(hash_fn(str.x + str.y + str.theta));
    }
};


/*The usage of unordered_map is marvelous*/

template <typename Val>
using KinodynamicMap = std::unordered_map<sKinodynamic, Val, hashKey<sKinodynamic>, hashCompare<sKinodynamic>>;


/**
* @brief 函数类
 * @brief 作为hash_map的比较函数
 * @brief 查找的时候不同的key往往可能会用到相同的hash值
*/
template <typename T>
struct hashCompare_
{
    bool operator()(const T &lhs, const T &rhs) const
    {
        return (fabs(lhs.center.theta - rhs.center.theta) < 1e-3 && 
                fabs(lhs.center.x - rhs.center.x) < 1e-3 && 
                fabs(lhs.center.y - rhs.center.y) < 1e-3 &&
                fabs(lhs.radius - rhs.radius) < 1e-3);
    }
};


/**
 * @brief 函数类
 * @brief 作为hash_map的hash函数
 * @brief class没有默认的hash函数
 * @brief overload of the operation "()"
 */
template <typename T>
class hashKey_
{
public:
    size_t operator()(const T &str) const
    {
        std::hash<float> hash_fn;
        return size_t(hash_fn(str.center.x + str.center.y + str.center.theta + str.radius));
    }
};

template <typename Val>
using FreeCircleMap = std::unordered_map<sFreeCircle, Val, hashKey_<sFreeCircle>, hashCompare_<sFreeCircle>>;

using sortedTree = std::map<float, functions::PtsTree<geometry_msgs::Point32>::ytree>;


/**
 * @brief The multiset template class
 */
 namespace stparam
 {
     static sFreeCircle target;
 };

template <typename T>
class OpenMultiset
{
  public:
    OpenMultiset(){}
    ~OpenMultiset() {}

    struct CompareX
    {
        bool operator()(const T& lhs, const T& rhs) const
        {
            return lhs.x < rhs.x;
        }
    };
    struct CompareY
    {
        bool operator()(const T& lhs, const T& rhs) const
        {
            return lhs.y < rhs.y;
        }
    };
    struct CompareCenterX
    {
        bool operator()(const T& lhs, const T& rhs) const
        {
            return lhs.center.x < rhs.center.x;
        }
    };
    struct CompareCenterY
    {
        bool operator()(const T& lhs, const T& rhs) const
        {
            return lhs.center.y < rhs.center.y;
        }
    };
    struct CompareRadius
    {
        bool operator()(const T& lhs, const T& rhs) const
        {
            return lhs.radius > rhs.radius;
        }
    };
    struct CompareCenterDistance
    {
        bool operator()(const T& lhs, const T& rhs) const
        {
            float ldis = std::hypot(lhs.center.x - stparam::target.center.x, lhs.center.y - stparam::target.center.y) - lhs.radius;
            float rdis = std::hypot(rhs.center.x - stparam::target.center.x, rhs.center.y - stparam::target.center.y) - rhs.radius;
            // float ldis = std::hypot(lhs.center.x - stparam::target.center.x, lhs.center.y - stparam::target.center.y);
            // float rdis = std::hypot(rhs.center.x - stparam::target.center.x, rhs.center.y - stparam::target.center.y);
            return ldis < rdis;
        }
    };
    typedef std::multiset<T, CompareX> xTable;
    typedef std::multiset<T, CompareY> yTable;
    typedef std::multiset<T, CompareCenterX> xCenterTable;
    typedef std::multiset<T, CompareCenterY> yCenterTable;
    typedef std::multiset<T, CompareRadius> radiusTable;
    typedef std::multiset<T, CompareCenterDistance> distanceTable;
    radiusTable largest;
    distanceTable nearest;

private:


};


#endif
