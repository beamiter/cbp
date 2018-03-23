#ifndef IVCBP_FUNCTIONS
#define IVCBP_FUNCTIONS

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
#include <limits>
#include <utility>

#include "ros/callback_queue.h"
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/heap/binomial_heap.hpp>
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
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


namespace functions
{

static const float radius = 2.0;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
static const float penaltyTurning = 1.0;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
static const float penaltyReversing = 2.0;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
static const float penaltyCOD = 2.0;

/**
 * @brief The boost::heap::binomial_heap implementaion of the priority_queue
 * @brief template pair and template priority_queue and template node
 */
template <typename T, typename priority_t>
struct PriorityNode
{
    typedef std::pair<T, priority_t> PQElement;

    PQElement node;

    PriorityNode() {}

    PriorityNode(PQElement ref)
        : node(ref) {}

    PriorityNode(T a, priority_t b)
    {
        node.first = a;
        node.second = b;
    }

    /**
    * \brief The overload of the operator "()"
    *
    */
    struct CompareNode
    {
        bool operator()(const PQElement lhs, const PQElement rhs) const
        {
            return (lhs.second > rhs.second);
        }
    };

    /**
     * \brief The overload of the operator "<"
     * \make the big top heap
     */
    friend bool operator<(const PriorityNode lhs, const PriorityNode rhs)
    {
        return (lhs.node.second < rhs.node.second);
    }

    //The default priority queue is big top elements heap!!!
    typedef boost::heap::priority_queue<PriorityNode> PriorityQueue;
    typedef boost::heap::binomial_heap<PQElement, boost::heap::compare<CompareNode>> boostPriority;
    typedef std::priority_queue<PQElement, std::vector<PQElement>, CompareNode> stdPriority;

    boostPriority elements;
    boostPriority big_elements;
};

/**
 * @brief The boost::heap::priority_queue implementaion of the priority_queue
 */
template <typename T, typename priority_t>
class CostNode
{
  public:
    typedef boost::heap::priority_queue<CostNode> boostPriorityQueue;

    typedef std::pair<T, priority_t> PQElement;
    PQElement node;

    CostNode() {}

    CostNode(PQElement ref)
        : node(ref) {}

    CostNode(T a, priority_t b)
    {
        node.first = a;
        node.second = b;
    }

    ~CostNode() {}

    /**
     * \brief The overload of the operator "<"
     *
     */
    friend bool operator<(const CostNode lhs, const CostNode rhs)
    {
        return (lhs.node.second < rhs.node.second);
    }

    struct CompareNode
    {
        bool operator()(const CostNode lhs, const CostNode rhs) const
        {
            return (lhs.node.second < rhs.node.second);
        }
    };
};

/**
 * @brief The multeset obstacles tree
 */
template <typename T>
class PtsTree
{
  public:
    PtsTree() {}
    PtsTree(T ref)
        : elem(ref) {}
    ~PtsTree() {}
    T elem;
    struct CompareLeafX
    {
        bool operator()(const T lhs, const T rhs) const
        {
            return lhs.x < rhs.x;
        }
    };
    struct CompareLeafY
    {
        bool operator()(const T lhs, const T rhs) const
        {
            return lhs.y < rhs.y;
        }
    };
    typedef std::multiset<T, CompareLeafX> xtree;
    typedef std::multiset<T, CompareLeafY> ytree;
};

static std::tuple<geometry_msgs::Point32, geometry_msgs::Point32> carmodel(geometry_msgs::Point32 ego,
       geometry_msgs::Polygon &polygon, float headCarLength, float tailCarLength, float halfWheelTrack)
{
    polygon.points.clear();
    geometry_msgs::Point32 pt, ld, ru;
    std::set<float> cmpx, cmpy;
    float vyaw = ego.z + M_PI / 2.0;
    pt.x = ego.x + headCarLength * cos(ego.z) - halfWheelTrack * cos(vyaw);
    pt.y = ego.y + headCarLength * sin(ego.z) - halfWheelTrack * sin(vyaw);
    cmpx.insert(pt.x);
    cmpy.insert(pt.y);
    polygon.points.push_back(pt);
    pt.x = ego.x + headCarLength * cos(ego.z) + halfWheelTrack * cos(vyaw);
    pt.y = ego.y + headCarLength * sin(ego.z) + halfWheelTrack * sin(vyaw);
    cmpx.insert(pt.x);
    cmpy.insert(pt.y);
    polygon.points.push_back(pt);
    pt.x = ego.x - tailCarLength * cos(ego.z) + halfWheelTrack * cos(vyaw);
    pt.y = ego.y - tailCarLength * sin(ego.z) + halfWheelTrack * sin(vyaw);
    cmpx.insert(pt.x);
    cmpy.insert(pt.y);
    polygon.points.push_back(pt);
    pt.x = ego.x - tailCarLength * cos(ego.z) - halfWheelTrack * cos(vyaw);
    pt.y = ego.y - tailCarLength * sin(ego.z) - halfWheelTrack * sin(vyaw);
    cmpx.insert(pt.x);
    cmpy.insert(pt.y);
    polygon.points.push_back(pt);
    ld.x = *(cmpx.begin());
    ld.y = *(cmpy.begin());
    ru.x = *(cmpx.rbegin());
    ru.y = *(cmpy.rbegin());
    auto rtn = std::forward_as_tuple(ld, ru);
    return rtn;
}

static void vArrayCreate(std::vector<std::vector<unsigned char>> &array, int m, int n)
{
    std::vector<unsigned char> temparr;
    temparr.assign(n, 0);
    array.assign(m, temparr);
    return;
}

static bool IsInsideFootprint(geometry_msgs::Point32 pt, geometry_msgs::Polygon &bounding_polygon)
{
    int counter = 0;
    int i;
    double xinters;
    geometry_msgs::Point32 p1;
    geometry_msgs::Point32 p2;
    int N = bounding_polygon.points.size();
    p1 = bounding_polygon.points.at(0);
    for (i = 1; i <= N; i++)
    {
        p2 = bounding_polygon.points.at(i % N);
        if (pt.y > std::min<float>(p1.y, p2.y))
        {
            if (pt.y <= std::max<float>(p1.y, p2.y))
            {
                if (pt.x <= std::max<float>(p1.x, p2.x))
                {
                    if (p1.y != p2.y)
                    {
                        xinters = (pt.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                        if (p1.x == p2.x || pt.x <= xinters)
                            counter++;
                    }
                }
            }
        }
        p1 = p2;
    }
    if (counter % 2 == 0)
        return false;
    else
        return true;
}

static std::string getcurrentdir()
{
    std::string dirname = __FILE__;
    boost::filesystem::path pathname(dirname);
    pathname.remove_filename();
    dirname = pathname.string();
    return dirname;
}

//This is a new distance heuristic method!!!
static float arcpath(geometry_msgs::Point32 startpt, geometry_msgs::Point32 goalpt, std::vector<geometry_msgs::Point32> &pts)
{
    pts.clear();
    geometry_msgs::Point32 crspt, temp, center, center1, center2;
    geometry_msgs::Vector3 connect, vec;
    float radius;
    float lenstep = 0.20;
    float thetastep;
    float minradius = functions::radius;
    float tempradian;
    float length;
    float symbol1, symbol2;
    float radiandiff = acos(cos(startpt.z) * cos(goalpt.z) + sin(startpt.z) * sin(goalpt.z));

    connect.x = goalpt.x - startpt.x;
    connect.y = goalpt.y - startpt.y;
    // Determins the goal to start position
    symbol1 = cos(startpt.z) * connect.y - sin(startpt.z) * connect.x;
    // Determins the goal to the linkline position
    symbol2 = cos(goalpt.z) * connect.y - sin(goalpt.z) * connect.x;
    // Normalize the symbol
    symbol1 = (symbol1 > 1e-3) ? 1.0 : ((symbol1 < -1e-3) ? -1.0 : 0.0);
    symbol2 = (symbol2 > 1e-3) ? 1.0 : ((symbol2 < -1e-3) ? -1.0 : 0.0);

    if (fabs(symbol1) < 1e-3)
    {
        float diff = cos(startpt.z) * cos(goalpt.z) + sin(startpt.z) * sin(goalpt.z) > 0;
        if (diff >= 0.0 && fabs(symbol2) < 1e-3)
        {
            float d = 0.0;
            vec.x = (goalpt.x - startpt.x) >= 0.0 ? 1.0 : -1.0;
            vec.y = (goalpt.y - startpt.y) >= 0.0 ? 1.0 : -1.0;
            float dis = sqrt(pow(goalpt.x - startpt.x, 2) + pow(goalpt.y - startpt.y, 2));
            while (d <= dis)
            {
                temp.x = startpt.x + vec.x * d * cos(startpt.z);
                temp.y = startpt.y + vec.y * d * sin(startpt.z);
                temp.z = startpt.z;
                pts.push_back(temp);
                d += lenstep;
            }
            length = dis;
        }
        else
        {
            std::cout<<"This case remains to be implemented!"<<std::endl;
        }
    }
    else
    {
        // This is the normal case
        if ((symbol2 > 1e-3 && symbol1 < -1e-3) || (symbol2 < -1e-3) && (symbol1 > 1e-3))
        {
            // Startline
            float c1 = sin(startpt.z) * startpt.x - cos(startpt.z) * startpt.y;
            // Goalline
            float c2 = sin(goalpt.z) * goalpt.x - cos(goalpt.z) * goalpt.y;
            // Get the crossing point
            float denominator = sin(startpt.z) * cos(goalpt.z) - cos(startpt.z) * sin(goalpt.z);
            if (fabs(denominator) < 1e-3)
            {
                geometry_msgs::Point32 transpt;
                float radiansum = M_PI;
                float radian = 0.0;
                // This dir determines the goal position to the start
                float dir = cos(startpt.z) * cos(connect.z) + sin(startpt.z) * sin(connect.z);
                // Get te radius
                radius = fabs(-sin(startpt.z) * goalpt.x + cos(startpt.z) * goalpt.y + c1) / 2.0;
                thetastep = lenstep / radius;
                if (dir >= 0.0)
                {
                    tempradian = goalpt.z  - symbol1 * M_PI / 2.0;
                    center.x = goalpt.x - cos(tempradian) * radius;
                    center.y = goalpt.y - sin(tempradian) * radius;
                    transpt.x = 2 * center.x - goalpt.x;
                    transpt.y = 2 * center.y - goalpt.y;
                    float dis = sqrt(pow(transpt.x - startpt.x, 2) + pow(transpt.y - startpt.y, 2));
                    length = radius * radiansum + dis;
                    while (radian <= radiansum)
                    {
                        temp.x = center.x + radius * cos(tempradian);
                        temp.y = center.y + radius * sin(tempradian);
                        temp.z = goalpt.z - symbol1 * radian;
                        radian += thetastep;
                        tempradian -= symbol1 * thetastep;
                        pts.push_back(temp);
                    }
                    std::reverse(pts.begin(), pts.end());
                    float d = 0.0;
                    std::vector<geometry_msgs::Point32> array;
                    while(d <= dis)
                    {
                        temp.x = startpt.x + d * cos(startpt.z);
                        temp.y = startpt.y + d * sin(startpt.z);
                        temp.z = startpt.z;
                        d += lenstep;
                        array.push_back(temp);
                    }
                    pts.insert(pts.begin(), array.begin(), array.end());
                }
                else
                {
                    tempradian = startpt.z  - symbol1 * M_PI / 2.0;
                    center.x = startpt.x - cos(tempradian) * radius;
                    center.y = startpt.y - sin(tempradian) * radius;
                    transpt.x = 2 * center.x - startpt.x;
                    transpt.y = 2 * center.y - startpt.y;
                    float dis = sqrt(pow(transpt.x - goalpt.x, 2) + pow(transpt.y - goalpt.y, 2));
                    length = radius * radiansum + dis;
                    while (radian <= radiansum)
                    {
                        temp.x = center.x + radius * cos(tempradian);
                        temp.y = center.y + radius * sin(tempradian);
                        temp.z = startpt.z - symbol1 * radian;
                        tempradian += symbol1 * thetastep;
                        radian += thetastep;
                        pts.push_back(temp);
                    }
                    float d = 0.0;
                    std::vector<geometry_msgs::Point32> array;
                    while(d <= dis)
                    {
                        temp.x = goalpt.x - d * cos(goalpt.z);
                        temp.y = goalpt.y - d * sin(goalpt.z);
                        temp.z = goalpt.z;
                        d += lenstep;
                        array.push_back(temp);
                    }
                    std::reverse(array.begin(), array.end());
                    pts.insert(pts.end(), array.begin(), array.end());
                }
            }
            else
            {
                crspt.x = (c1 * cos(goalpt.z) - c2 * cos(startpt.z)) / denominator;
                crspt.y = (c1 * sin(goalpt.z) - c2 * sin(startpt.z)) / denominator;
                float includedradian = (M_PI - radiandiff) / 2.0;
                float dis1 = sqrt(pow(startpt.x - crspt.x, 2) + pow(startpt.y - crspt.y, 2));
                float dis2 = sqrt(pow(goalpt.x - crspt.x, 2) + pow(goalpt.y - crspt.y, 2));
                float dis = fabs(dis1 - dis2);

                // The judgement is essential, determines if the crosspoint is in front of startpt or not
                float flag = (crspt.x - startpt.x) * cos(startpt.z) + (crspt.y - startpt.y) * sin(startpt.z);
                if (flag * (dis1 - dis2) > 1e-3)
                {
                    radius = tan(includedradian) * dis2;
                    thetastep = lenstep / radius;
                    tempradian = goalpt.z - symbol1 * M_PI / 2.0;
                    center.x = goalpt.x - cos(tempradian) * radius;
                    center.y = goalpt.y - sin(tempradian) * radius;
                    float radian = 0.0;
                    float radiansum = (flag > 0.0) ? radiandiff : (2 * M_PI - radiandiff);
                    length = dis + radiansum * radius;
                    while (radian <= radiansum)
                    {
                        temp.x = center.x + radius * cos(tempradian);
                        temp.y = center.y + radius * sin(tempradian);
                        temp.z = goalpt.z - symbol1 * radian;
                        radian += thetastep;
                        tempradian -= symbol1 * thetastep;
                        pts.push_back(temp);
                    }
                    std::reverse(pts.begin(), pts.end());
                    float d = 0.0;
                    std::vector<geometry_msgs::Point32> array;
                    while (d <= dis)
                    {
                        temp.x = startpt.x + d * cos(startpt.z);
                        temp.y = startpt.y + d * sin(startpt.z);
                        temp.z = startpt.z;
                        d += lenstep;
                        array.push_back(temp);
                    }
                    pts.insert(pts.begin(), array.begin(), array.end());
                }
                else if (flag * (dis2 - dis1) > 1e-3)
                {
                    radius = tan(includedradian) * dis1;
                    thetastep = lenstep / radius;
                    tempradian = startpt.z - symbol1 * M_PI / 2.0;
                    center.x = startpt.x - cos(tempradian) * radius;
                    center.y = startpt.y - sin(tempradian) * radius;
                    float radian = 0.0;
                    float radiansum = (flag > 0.0) ? radiandiff : (2 * M_PI - radiandiff);
                    length = dis + radiansum * radius;
                    while (radian <= radiansum)
                    {
                        temp.x = center.x + radius * cos(tempradian);
                        temp.y = center.y + radius * sin(tempradian);
                        temp.z = startpt.z + symbol1 * radian;
                        tempradian += symbol1 * thetastep;
                        radian += thetastep;
                        pts.push_back(temp);
                    }
                    float d = 0.0;
                    std::vector<geometry_msgs::Point32> array;
                    while (d <= dis)
                    {
                        temp.x = goalpt.x - d * cos(goalpt.z);
                        temp.y = goalpt.y - d * sin(goalpt.z);
                        temp.z = goalpt.z;
                        d += lenstep;
                        array.push_back(temp);
                    }
                    std::reverse(array.begin(), array.end());
                    pts.insert(pts.end(), array.begin(), array.end());
                }
                else
                {
                    radius = tan(includedradian) * dis1;
                    thetastep = lenstep / radius;
                    tempradian = startpt.z - symbol1 * M_PI / 2.0;
                    center.x = startpt.x - cos(tempradian) * radius;
                    center.y = startpt.y - sin(tempradian) * radius;
                    float radian = 0.0;
                    float radiansum = (flag > 0.0) ? radiandiff : (2 * M_PI - radiandiff);
                    length = dis + radiansum * radius;
                    while (radian <= radiansum)
                    {
                        temp.x = center.x + radius * cos(tempradian);
                        temp.y = center.y + radius * sin(tempradian);
                        temp.z = startpt.z + symbol1 * radian;
                        tempradian += symbol1 * thetastep;
                        radian += thetastep;
                        pts.push_back(temp);
                    }
                }
            }
        }
        else
        {
            // The radian of the start and goal circle center
            geometry_msgs::Vector3 vec1, vec2;
            float tempradian1, tempradian2;
            tempradian1 = startpt.z - symbol1 * M_PI / 2.0;
            tempradian2 = goalpt.z + symbol1 * M_PI / 2.0;
            float radiandiff1, radiandiff2;
            float c = connect.x * connect.x + connect.y * connect.y;
            float b = 2.0 * (connect.x * (cos(tempradian1) - cos(tempradian2)) + connect.y * (sin(tempradian1) - sin(tempradian2)));
            float a = -2.0 * cos(tempradian1 - tempradian2) - 2.0;
            if (fabs(a) < 1e-3)
            {
                radius = -c / b;
            }
            else
            {
                radius = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            }
            thetastep = lenstep / radius;
            center1.x = startpt.x - radius * cos(tempradian1);
            center1.y = startpt.y - radius * sin(tempradian1);
            center2.x = goalpt.x - radius * cos(tempradian2);
            center2.y = goalpt.y - radius * sin(tempradian2);
            vec1.x = center2.x - center1.x;
            vec1.y = center2.y - center1.y;
            vec2.x = center1.x - center2.x;
            vec2.y = center1.y - center2.y;
            // The judge flag is quite essential
            float flag1 = cos(startpt.z) * vec1.x + sin(startpt.z) * vec1.y;
            float flag2 = cos(goalpt.z) * vec2.x + sin(goalpt.z) * vec2.y;
            float d = sqrt(vec1.x * vec1.x + vec1.y * vec1.y);
            radiandiff1 = acos((vec1.x * cos(tempradian1) + vec1.y * sin(tempradian1)) / d);
            radiandiff1 = (flag1 >= 0.0) ? radiandiff1 : (2 * M_PI - radiandiff1);
            radiandiff2 = acos((vec2.x * cos(tempradian2) + vec2.y * sin(tempradian2)) / d);
            radiandiff2 = (flag2 <= 0.0) ? radiandiff2 : (2 * M_PI - radiandiff2);
            length = (radiandiff1 + radiandiff2) * radius;
            float radian1 = 0.0;
            while (radian1 <= radiandiff1)
            {
                temp.x = center1.x + radius * cos(tempradian1);
                temp.y = center1.y + radius * sin(tempradian1);
                temp.z = startpt.z + symbol1 * radian1;
                pts.push_back(temp);
                tempradian1 += symbol1 * thetastep;
                radian1 += thetastep;
            }
            std::vector<geometry_msgs::Point32> array;
            float radian2 = 0.0;
            while (radian2 <= radiandiff2)
            {
                temp.x = center2.x + radius * cos(tempradian2);
                temp.y = center2.y + radius * sin(tempradian2);
                temp.z = goalpt.z + symbol1 * radian2;
                array.push_back(temp);
                tempradian2 += symbol1 * thetastep;
                radian2 += thetastep;
            }
            std::reverse(array.begin(), array.end());
            pts.insert(pts.end(), array.begin(), array.end());
        }
    }
    return length;
}


static float rad2Deg(float radian)
{
    if (radian > 2 * M_PI)
    {
        radian -= 2 * M_PI;
    }
    else if (radian < 0.0)
    {
        radian += 2 * M_PI;
    }
    return radian * 180 / M_PI;
}

static float deg2Rad(float degree)
{
    if (degree > 360.0)
    {
        degree -= 360.0;
    }
    else if (degree < 0.0)
    {
        degree += 360.0;
    }
    return M_PI * degree / 180.0;
}

static float normalizeradian(float rad)
{
  if (rad < 0.0)
  {
    rad += 2 * M_PI;
  }
  if (rad > 2 * M_PI)
  {
    rad -= 2 * M_PI;
  }
  return rad;
}

static double generateGaussianNoise(double mu, double sigma)
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;

    static double z0, z1;
    static bool generate;
    generate = !generate;

    if (!generate)
       return z1 * sigma + mu;

    double u1, u2;
    do
     {
       u1 = rand() * (1.0 / RAND_MAX);
       u2 = rand() * (1.0 / RAND_MAX);
     }
    while ( u1 <= epsilon );

    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}

// generateGaussianNoise box_muller algorithm
static float boxmuller(float m, float sd)
{
    const double epsilon = std::numeric_limits<double>::min();
    //Get random number
    srand((unsigned)time(NULL));
    float u1 = rand() * (1.0 / RAND_MAX);
    float u2 = rand() * (1.0 / RAND_MAX);

    while(u2 <= epsilon)
    {
        u2 = rand() * (1.0 / RAND_MAX);
    }

    float r = sqrt(-2.0 * log(u2));
    float theta = 2 * M_PI * u1;
    float z = r * cos(theta);

    return (m + z * sd);
}

static std::vector<geometry_msgs::Point32> curvefitting(std::vector<geometry_msgs::Point32> pathpoint)
{
  int step = 20;
  float g0, g1, g2, g3, t = 0.0;
  std::deque<geometry_msgs::Point32> controlvec;
  std::vector<geometry_msgs::Point32> bspline;
  geometry_msgs::Point32 temp, temppt, prept;
  geometry_msgs::Point32 startpt, goalpt;
  if (pathpoint.size() < 3)
  {
    return pathpoint;
  }
  startpt = pathpoint.front();
  goalpt = pathpoint.back();
  for (auto a : pathpoint)
  {
    controlvec.push_back(a);
  }

  temp.x = 2 * controlvec.at(0).x - controlvec.at(1).x;
  temp.y = 2 * controlvec.at(0).y - controlvec.at(1).y;
  temp.z = 0;
  controlvec.push_front(temp);

  int num = controlvec.size();

  temp.x = 2 * controlvec.at(num - 1).x - controlvec.at(num - 2).x;
  temp.y = 2 * controlvec.at(num - 1).y - controlvec.at(num - 2).y;
  temp.z = 0;
  controlvec.push_back(temp);

  prept = startpt;
  step = 8;
  for (int a = 0; a < controlvec.size() - 3; ++a)
  {
    t = 0.0;
    step = std::ceil(std::hypot(controlvec.at(a + 1).x - controlvec.at(a + 2).x,
                     controlvec.at(a + 1).y - controlvec.at(a + 2).y) / 0.2);
    if (step < 1)
    {
        step = 1;
    }

    for (int b = 0; b <= step; ++b)
    {
      g0 = (-t * t * t + 3 * t * t - 3 * t + 1) / 6.0;
      g1 = (3 * t * t * t - 6 * t * t + 4) / 6.0;
      g2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0;
      g3 = (t * t * t) / 6.0;
      t += 1.0 / step;
      temppt.x = controlvec.at(a).x * g0 + controlvec.at(a + 1).x * g1 + controlvec.at(a + 2).x * g2 +
                 controlvec.at(a + 3).x * g3;
      temppt.y = controlvec.at(a).y * g0 + controlvec.at(a + 1).y * g1 + controlvec.at(a + 2).y * g2 +
                 controlvec.at(a + 3).y * g3;
      temppt.z = atan2(temppt.y - prept.y, temppt.x - prept.x);
      prept = temppt;

      // Avoiding the overlapped point and it is quite essential
      if (b == 0)
        continue;
      bspline.push_back(temppt);
    }
  }
  return bspline;
}

static std::vector<geometry_msgs::Point32> bsplinecurve(geometry_msgs::Point32 startpt, geometry_msgs::Point32 goalpt)
{
  int step = 20;
  geometry_msgs::Point32 temp, temppt, prept;
  std::deque<geometry_msgs::Point32> controlvec;
  std::vector<geometry_msgs::Point32> bspline;
  float dis = sqrt(pow(startpt.x - goalpt.x, 2) + pow(startpt.y - goalpt.y, 2));
  if (dis < 0.4)
    return bspline;
  step = std::ceil(dis / 0.3);
  float g0, g1, g2, g3, t = 0.0;
  float offset = 0.5;
  offset = std::max(fabs(startpt.x - goalpt.x), fabs(startpt.y - goalpt.y)) / 3.0;

  controlvec.push_back(startpt);
  temp.x = startpt.x + offset * cos(startpt.z);
  temp.y = startpt.y + offset * sin(startpt.z);
  controlvec.push_back(temp);
  temp.x = goalpt.x - offset * cos(goalpt.z);
  temp.y = goalpt.y - offset * sin(goalpt.z);
  controlvec.push_back(temp);
  controlvec.push_back(goalpt);

  temp.x = 2 * controlvec.at(0).x - controlvec.at(1).x;
  temp.y = 2 * controlvec.at(0).y - controlvec.at(1).y;
  temp.z = 0;
  controlvec.push_front(temp);

  int num = controlvec.size();

  temp.x = 2 * controlvec.at(num - 1).x - controlvec.at(num - 2).x;
  temp.y = 2 * controlvec.at(num - 1).y - controlvec.at(num - 2).y;
  temp.z = 0;
  controlvec.push_back(temp);

  prept = startpt;
  for (int a = 0; a < controlvec.size() - 3; ++a)
  {
    t = 0;
    for (int b = 0; b <= step; ++b)
    {
      // t += 1.0 / step;
      g0 = (-t * t * t + 3 * t * t - 3 * t + 1) / 6.0;
      g1 = (3 * t * t * t - 6 * t * t + 4) / 6.0;
      g2 = (-3 * t * t * t + 3 * t * t + 3 * t + 1) / 6.0;
      g3 = (t * t * t) / 6.0;
      t += 1.0 / step;
      temppt.x = controlvec.at(a).x * g0 + controlvec.at(a + 1).x * g1 + controlvec.at(a + 2).x * g2 +
                 controlvec.at(a + 3).x * g3;
      temppt.y = controlvec.at(a).y * g0 + controlvec.at(a + 1).y * g1 + controlvec.at(a + 2).y * g2 +
                 controlvec.at(a + 3).y * g3;
      temppt.z = atan2(temppt.y - prept.y, temppt.x - prept.x);
      prept.x = temppt.x;
      prept.y = temppt.y;
      prept.z = temppt.z;

      // Avoiding the overlapped point and it is quite essential
      if (b == 0)
        continue;
      bspline.push_back(temppt);
    }
  }
  return bspline;
}

}//The end of the namespace functions!!!

#endif
