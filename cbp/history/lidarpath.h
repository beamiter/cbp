#ifndef _LIDAR_PATH_H
#define _LIDAR_PATH_H

//c++ lib
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <algorithm>
#include <vector>
#include <deque>
#include <iterator>
#include <cmath>

#include "ros/callback_queue.h"
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>
// ROS lib
#include "ros/ros.h"
#include "ros/time.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <laser_geometry/laser_geometry.h>

//other lib

#include <bitset>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

#define random(x) (rand() % x)
#define CVDEBUG 0
#define LINEDEBUG 0

#ifndef uchar
#define uchar unsigned char;
#endif

typedef enum LANE {
    RIGHT_LANE = 0,
    LEFT_LANE
} emLane;

class lidarpath
{
  public:
    lidarpath(ros::NodeHandle nh);
    lidarpath();
    ~lidarpath();
    void run();
    void lidarcallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void point2map(const sensor_msgs::PointCloud &pl);
    void createmap(const char *str);
    int traceboundary(const std::vector<std::vector<unsigned char>> &tempmap);

    void maprhdpt(const geometry_msgs::Point &currentpt, std::vector<geometry_msgs::Point> &temppt);
    void maplhdpt(const geometry_msgs::Point &currentpt, std::vector<geometry_msgs::Point> &temppt);
    void vArrayCreate(std::vector<std::vector<unsigned char>> &array, int m, int n);
    void bresenham(int xc, int yc, int r);
    void circleplot(int xc, int yc, int x, int y);
    void quarterplot(int xc, int yc, int x, int y);
    void sortpt(const std::vector<geometry_msgs::Point> &temppt);
    void searchlgoal(const geometry_msgs::Point &currentpt, const std::vector<std::vector<unsigned char>> &tempmap);
    void searchrgoal(const geometry_msgs::Point &currentpt, const std::vector<std::vector<unsigned char>> &tempmap);
    int searchgoal(const geometry_msgs::Point &currentpt, const std::vector<std::vector<unsigned char>> &tempmap, emLane lane);
    int updatergoal(geometry_msgs::Point &goalref, const std::vector<std::vector<unsigned char>> &tempmap);
    int updatelgoal(geometry_msgs::Point &goalref, const std::vector<std::vector<unsigned char>> &tempmap);
    void setresolution(const float res);
    void setsideinfo(const int w, const int h);
    void setradius(const int r);
    void generateline(int x0, int y0, int x1, int y1, std::vector<geometry_msgs::Point> &lineseg);
    void linecasting(geometry_msgs::Point src, geometry_msgs::Point dst, std::vector<geometry_msgs::Point> &rtn);

  private:
    int count;
    int radius;
    int curindex;
    int lcurindex;
    int rcurindex;
    int endindex;
    int lendindex;
    int rendindex;
    int tracecount;
    int width;
    int height;
    float resolution;
    float llaneoff;
    float rlaneoff;
    std::string dirname;
    std::string filename;
    std::vector<std::vector<unsigned char>> lidarmap;
    ros::Subscriber lidarsub;
    sensor_msgs::PointCloud cloudmsg;
    sensor_msgs::LaserScan lasermsg;
    laser_geometry::LaserProjection laser2pl;
    std::vector<std::vector<geometry_msgs::Point>> validpath;
    std::vector<geometry_msgs::Point> lvalidpt, rvalidpt;
    geometry_msgs::Point startpt, goalpt;
    std::vector<geometry_msgs::Point> circlept;
    std::vector<geometry_msgs::Point> quarterpt;
    std::vector<geometry_msgs::Point> eighthpt;
    //Right-handed coordinate system
    std::vector<geometry_msgs::Point> rhd;
    std::vector<geometry_msgs::Point> lhd;
    std::vector<geometry_msgs::Point> rgridpt;
    std::vector<geometry_msgs::Point> lgridpt;
    std::vector<geometry_msgs::Point> m_pts;
    std::deque<geometry_msgs::Point> lfilterpt;
    std::deque<geometry_msgs::Point> rfilterpt;
    std::vector<double> coeff;
    Mat pt_trace;
};

#endif
