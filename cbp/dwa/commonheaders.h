#ifndef _COMMON_HEADERS_H
#define _COMMON_HEADERS_H


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

typedef struct VEHICLESTATUS
{
    // dm
    float x;
    float y;
    // rad
    float heading;
    // m/s
    float linear_velocity;
    // rad/s
    float angular_velocity;

    /**
     * \brief The overload of the operator "=="
     *
     */
    bool operator== (const VEHICLESTATUS &rhs)
    {
        return (fabs(heading - rhs.heading) <= 1e-3 && fabs(x - rhs.x) <= 1e-3 && fabs(y - rhs.y) <= 1e-3 &&
            fabs(linear_velocity - rhs.linear_velocity) <= 1e-3 && fabs(angular_velocity - rhs.angular_velocity) <= 1e-3);
    }

    VEHICLESTATUS& operator= (const VEHICLESTATUS &rhs)
    {
        this->angular_velocity = rhs.angular_velocity;
        this->linear_velocity = rhs.linear_velocity;
        this->x = rhs.x;
        this->y = rhs.y;
        this->heading = rhs.heading;
        return *this;
    }
} sVehicleStatus;

/**
* @brief 函数类
 * @brief 作为hash_map的比较函数
 * @brief 查找的时候不同的key往往可能会用到相同的hash值
*/
template <typename T>
struct hash_compare
{
    bool operator()(const T &lhs, const T &rhs) const
    {
        return (lhs.heading == rhs.heading) && (lhs.x == rhs.x) && (lhs.y == rhs.y);
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
    struct hash<sVehicleStatus>
    {
        inline size_t operator()(const sVehicleStatus &vs) const
        {
            std::hash<float> hash_fn;
            return size_t(hash_fn(vs.x + vs.y + vs.heading));
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
class hash_key
{
public:
    size_t operator()(const T &str) const
    {
        std::hash<float> hash_fn;
        return size_t(hash_fn(str.x + str.y + str.heading));
    }
};

typedef struct KINEMATIC
{
    // m/s
    float max_linear_velocity;
    // ras/s
    float max_angular_velocity;
    // m/s^2
    float max_linear_acceleration;
    // m/s^3
    float maximum_acceleration;
    // m/s^3
    float maximum_decceleration;
    //rad/s^2
    float max_angular_acceleration;
    // m/s^2
    float min_turn_radius;
    // m/s^2
    float centrifugal_acceleration; // Multiply the gravitational acceleration
} sKinematic;

typedef struct VEHICLEELEMENT
{
    /*Unit: metre*/
    float length;
    float width;
    float halfwheeltrack;
    float frontoverhang;
    float backoverhang;
    float wheelbase;
    float headwheel;
    float tailwheel;

    /*turnangle, Unit: degree*/

    /*Left wheel steer angle*/
    float innerangle;
    /*Right wheel steer angle*/
    float outerangle;
    //>= 2.5 dm
    float outersafedis;
    float innersafedis;
} sVehicleElem;

typedef struct ROADELEMENT
{
    /*Unit: metre*/
    float roadwidthmin;
    float turnradiusmin;
    float turnradiusmax;
    float roadouterradius;
    float roadinnerradius;
} sRoadElem;

// The usage of unordered_map is marvelous
template <typename Val>
using unordered_t = std::unordered_map<sVehicleStatus, Val, hash_key<sVehicleStatus>, hash_compare<sVehicleStatus>>;

using sorted_tree = std::map<float, functions::PtsTree<geometry_msgs::Point32>::ytree>;

#endif