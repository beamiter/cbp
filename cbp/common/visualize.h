#ifndef _VISUALIZE_H
#define _VISUALIZE_H

#include <ros/ros.h>
#include "ros/callback_queue.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

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

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>

#include <bitset>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

class visualize_pose
{
public:
  visualize_pose(std::string name);
  ~visualize_pose();

  // void clear();

  int setstart(geometry_msgs::PoseStamped &start);
  int setgoal(geometry_msgs::PoseStamped &goal);

  bool updatestate();

  void startcallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void goalcallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
  bool gflag;
  bool sflag;
  ros::NodeHandle nh;
  ros::Subscriber substart;
  ros::Subscriber subgoal;

  ros::Publisher pubstart;
  ros::Publisher pubgoal;

  geometry_msgs::PoseStamped startPose;
  geometry_msgs::PoseStamped goalPose;

  tf::TransformBroadcaster pathbr;
  tf::Transform pathtr;
  tf::Quaternion pathquater;

protected:
};

class visualize_path
{
public:
  visualize_path(std::string name);
  ~visualize_path();
  int publishpath(std::vector<geometry_msgs::Point32> patharray);

private:
  ros::Publisher pubpath;
  nav_msgs::Path path;
  ros::NodeHandle nh;
  tf::TransformBroadcaster pathbr;
  tf::Transform pathtr;
  tf::Quaternion pathquater;
};

class visualize_marker
{
public:
  visualize_marker(std::string name);
  ~visualize_marker();
  int publishmultipath(std::vector<std::vector<geometry_msgs::Point32>> patharray, int index);
  int publishobstacle(std::vector<geometry_msgs::Point32> obs, int index, float size);
  int publishcircles(std::vector<geometry_msgs::Point32> obs);

private:
  ros::Publisher pubmultipath;
  ros::Publisher pubobstacle;
  ros::Publisher pubcirlce;
  visualization_msgs::Marker multipath;
  visualization_msgs::Marker obstacle;
  visualization_msgs::MarkerArray circlearray;
  ros::NodeHandle nh;
  tf::TransformBroadcaster pathbr;
  tf::Transform pathtr;
  tf::Quaternion pathquater;
};

class visualize_polygon
{
public:
  visualize_polygon(std::string name);
  ~visualize_polygon();
  int publishmultipolygon(std::vector<std::vector<geometry_msgs::Polygon>> bounding_polygon, int index);
  int publishpolygon(geometry_msgs::Polygon bounding_polygon);

private:
  ros::Publisher pubpolygon, pubmultipolygon;
  geometry_msgs::PolygonStamped poly;
  visualization_msgs::Marker multipoly;
  ros::NodeHandle nh;
  tf::TransformBroadcaster pathbr;
  tf::Transform pathtr;
  tf::Quaternion pathquater;
};
#endif