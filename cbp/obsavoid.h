#ifndef _DWA_OBSAVOID_H
#define _DWA_OBSAVOID_H

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
#include <bitset>

#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/heap/binomial_heap.hpp>

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

//other lib


#include "visualize.h"
#include "functions.h"

#include "hermite_curve.h"

using namespace std;

class obsavoid
{
public:
  
  obsavoid();
  ~obsavoid();
  
  geometry_msgs::Pose transformPose(geometry_msgs::Pose &pose, tf::Transform &tf);
  geometry_msgs::Pose relativePose(geometry_msgs::Pose source, geometry_msgs::Pose target);
  geometry_msgs::Point32 getGuidingPoint(geometry_msgs::Point32 ego_point, geometry_msgs::Point32 angular_point, int side);
  geometry_msgs::Point32 getAngularPoint(geometry_msgs::Point32 ego_point, std::vector<geometry_msgs::Point32> input);
  std::vector<geometry_msgs::Pose> getFinalPath();
  std::vector<geometry_msgs::Pose> getRawPath();
  void run();
  
  
private:
  
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  geometry_msgs::Pose m_start, m_end;
  geometry_msgs::Pose obs_pose;

  std::vector<geometry_msgs::Pose> raw_path, final_path;
  
};

#endif