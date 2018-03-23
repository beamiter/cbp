#include "obsavoid.h"

obsavoid::obsavoid()
{
  private_nh = ros::NodeHandle("~");
  obs_pose.position.x = 6.0;
  obs_pose.position.y = 0.0;
  obs_pose.orientation.w = 1.0;
  m_start.orientation.w = 1.0;
  m_start.position.x = 0.0;
  m_start.position.y = 0.0;
  m_start.position.z = 0.0;
}

obsavoid::~obsavoid()
{

}

geometry_msgs::Point32 obsavoid::getGuidingPoint(geometry_msgs::Point32 ego_point, geometry_msgs::Point32 angular_point, int side)
{
  /**
   * \side: 0-left, 1-right 
   *
   */
   float theta = 0.0;
   float car_width = 0.6;
   float safe_distance = 0.6;
   float distance_deviation = car_width + safe_distance;
   float theta_bias = 0.0;
   geometry_msgs::Point32 guiding_point;
  if (side == 0 || 1)
  {
    theta = atan2(angular_point.y - ego_point.y, angular_point.x - ego_point.x);
    theta_bias = theta + M_PI / 2.0;
    guiding_point.z = theta;
    guiding_point.x = angular_point.x + distance_deviation * cos(theta_bias);
    guiding_point.y = angular_point.y + distance_deviation * sin(theta_bias);
    return guiding_point;
  }
}


geometry_msgs::Point32 obsavoid::getAngularPoint(geometry_msgs::Point32 ego_point, std::vector<geometry_msgs::Point32> input)
{
  float maxcosine = 0.0;
  geometry_msgs::Point32 angular_point;
  geometry_msgs::Vector3 vb, va;
  va.x = cos(ego_point.z);
  va.y = sin(ego_point.z);
  for (auto a : input)
  {
    vb.x = a.x - ego_point.x;
    vb.y = a.y - ego_point.y;
    float temp = va.x * vb.x + va.y * vb.y;
  }
}

std::vector< geometry_msgs::Pose > obsavoid::getFinalPath()
{
  return final_path;
}

std::vector<geometry_msgs::Pose> obsavoid::getRawPath()
{
  return raw_path;
}

void obsavoid::run()
{
  geometry_msgs::Pose temp_pose;
  geometry_msgs::Point32 temp_pt, temp, start_pt;
  float interval = 6.0;
  if (std::fabs(m_start.position.x - obs_pose.position.x) > 1.5)
  {
    if (raw_path.size() > 3)
    {
      m_start.position = raw_path.at(2).position;
      m_start.orientation = raw_path.at(2).orientation;
      final_path.insert(final_path.end(), raw_path.begin(), raw_path.begin() + 2);
    }
    start_pt.x = m_start.position.x;
    start_pt.y = m_start.position.y;
    temp.x = obs_pose.position.x;
    temp.y = obs_pose.position.y;
    temp_pt = getGuidingPoint(start_pt, temp, 0);
    float dis = std::hypot(m_start.position.x - temp_pt.x, m_start.position.y - temp_pt.y);
    if (dis < interval)
    {
      m_end.position.x = temp_pt.x + (interval - dis) * cos(temp_pt.z);
      m_end.position.y = temp_pt.y + (interval - dis) * sin(temp_pt.z);
      m_end.orientation = tf::createQuaternionMsgFromYaw(temp_pt.z);  
    }
    else
    {
      m_end.position.x = temp_pt.x;
      m_end.position.y = temp_pt.y;
      m_end.orientation = tf::createQuaternionMsgFromYaw(temp_pt.z);      
    }

    std::vector<geometry_msgs::Pose>().swap(raw_path);
    raw_path = generatecurve::generateHermiteCurveForROS(m_start, m_end, 8.0);
  }
  else
  {
    float len = 0.0;
    float ptlen = 0.15;
    float target_dis = 8.0;
    float theta = tf::getYaw(raw_path.front().orientation);
    final_path.clear();
    final_path.shrink_to_fit();
    while(len <= target_dis)
    {
      temp_pose.orientation = raw_path.front().orientation;
      temp_pose.position.x = m_start.position.x + len * cos(theta);
      temp_pose.position.y = m_start.position.y + len * sin(theta);
      final_path.push_back(temp_pose);
      len += ptlen;
    }
  }
  return;
}

geometry_msgs::Pose obsavoid::relativePose(geometry_msgs::Pose source, geometry_msgs::Pose target)
{
  geometry_msgs::Pose relative_pose;
  tf::Pose tf_source, tf_target, temp_pose;
  tf::poseMsgToTF(source, tf_source);
  tf::poseMsgToTF(target, tf_target);
  tf::Transform tf_trans = tf::Transform(tf_source.getRotation(), tf_source.getOrigin());
  temp_pose = tf_trans.inverse() * tf_target;
  tf::poseTFToMsg(temp_pose, relative_pose);
  return relative_pose;
}


geometry_msgs::Pose obsavoid::transformPose(geometry_msgs::Pose &pose, tf::Transform &tf)
{
  // Convert ROS pose to TF pose
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);
  // Transform pose
  tf_pose = tf * tf_pose;
  // Convert TF pose to ROS pose
  geometry_msgs::Pose ros_pose;
  tf::poseTFToMsg(tf_pose, ros_pose);
  return ros_pose;
}


