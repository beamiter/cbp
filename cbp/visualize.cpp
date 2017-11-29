#include "visualize.h"
/************************visualize_pose************************/

visualize_pose::visualize_pose(std::string name)
    : gflag(false), sflag(false)
{
  pubstart = nh.advertise<geometry_msgs::PoseStamped>("/start_pose" + name, 5);
  pubgoal = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose" + name, 5);
  substart = nh.subscribe("/initialpose", 1, &visualize_pose::startcallback, this);
  subgoal = nh.subscribe("/move_base_simple/goal", 1, &visualize_pose::goalcallback, this);
  startPose.header.frame_id = "map";
  goalPose.header.frame_id = "map";
}

visualize_pose::~visualize_pose()
{
}

void visualize_pose::goalcallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  gflag = true;
  sflag = true;
  goalPose = *msg;
  pubgoal.publish(goalPose);
}

int visualize_pose::setgoal(geometry_msgs::PoseStamped &goal)
{
  goal = goalPose;
  gflag = false;
  sflag = false;
}

int visualize_pose::setstart(geometry_msgs::PoseStamped &start)
{
  start = startPose;
  sflag = false;
}

void visualize_pose::startcallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  sflag = true;
  startPose.header = (*msg).header;
  startPose.pose = (*msg).pose.pose;
  pubstart.publish(startPose);
}

bool visualize_pose::updatestate()
{
  return (gflag && sflag);
}

/************************/

/************************visualize_path************************/

visualize_path::visualize_path(std::string name)
{
  pubpath = nh.advertise<nav_msgs::Path>("/path" + name, 5);
  path.header.frame_id = "map";
}

visualize_path::~visualize_path()
{
}

int visualize_path::publishpath(std::vector<geometry_msgs::Point32> patharray)
{
  path.header.stamp = ros::Time::now();
  path.poses.clear();
  geometry_msgs::PoseStamped temp;
  temp.header.frame_id = "/map";
  temp.header.stamp = ros::Time::now();

  pathtr.setOrigin(tf::Vector3(0, 0, 0.0));
  pathquater.setRPY(0, 0, 0);
  pathtr.setRotation(pathquater);

  for (auto iter = patharray.begin(); iter != patharray.end(); ++iter)
  {
    temp.pose.orientation.w = 1.0;
    temp.pose.position.x = (*iter).x;
    temp.pose.position.y = (*iter).y;
    path.poses.push_back(temp);
  }
  pubpath.publish(path);
  return 1;
}

/************************/

/************************visualize_marker************************/

visualize_marker::visualize_marker(std::string name)
{
  pubmultipath = nh.advertise<visualization_msgs::Marker>("/multipath" + name, 5);
  pubobstacle = nh.advertise<visualization_msgs::Marker>("/obstacles" + name, 5);
  pubcirlce = nh.advertise<visualization_msgs::MarkerArray>("/freecircle" + name, 5);
  multipath.header.frame_id = "map";
  multipath.ns = "multipathsimulate";
  obstacle.header.frame_id = "map";
  obstacle.ns = "obstaclesimulate";
}

visualize_marker::~visualize_marker()
{
}

int visualize_marker::publishmultipath(std::vector<std::vector<geometry_msgs::Point32>> patharray, int index)
{
  multipath.header.stamp = ros::Time::now();
  multipath.action = visualization_msgs::Marker::ADD;
  multipath.pose.orientation.w = 1.0;
  multipath.lifetime = ros::Duration(0);
  multipath.id = 100;
  multipath.type = visualization_msgs::Marker::POINTS;
  multipath.scale.x = 0.03;
  multipath.scale.y = 0.03;
  multipath.color.r = 0.1;
  multipath.color.g = 1.0;
  multipath.color.b = 0.1;
  multipath.color.a = 1.0;
  multipath.points.clear();

  pathtr.setOrigin(tf::Vector3(0, 0, 0.0));
  pathquater.setRPY(0, 0, 0);
  pathtr.setRotation(pathquater);

  for (int i = 0; i < patharray.size(); ++i)
  {
    for (int j = 0; j < patharray.at(i).size(); ++j)
    {
      geometry_msgs::Point p;
      p.x = patharray[i][j].x;
      p.y = patharray[i][j].y;
      multipath.points.push_back(p);
    }
  }
  pubmultipath.publish(multipath);

  multipath.scale.x = 0.05;
  multipath.scale.y = 0.05;
  multipath.color.r = 0.1;
  multipath.color.g = 0.1;
  multipath.color.b = 1.0;
  multipath.color.a = 1.0;
  multipath.id = 1;
  multipath.points.clear();
  for (int j = 0; j < patharray.at(index).size(); ++j)
  {
    geometry_msgs::Point p;
    p.x = patharray[index][j].x;
    p.y = patharray[index][j].y;
    multipath.points.push_back(p);
  }
  pubmultipath.publish(multipath);
  return 1;
}

int visualize_marker::publishcircles(std::vector<geometry_msgs::Point32> obs)
{
  circlearray.markers.clear();
  for (int i = 0; i < obs.size(); ++i)
  {
    visualization_msgs::Marker tempMarker;
    tempMarker.header.stamp = ros::Time::now();
    tempMarker.header.frame_id = "map";
    tempMarker.lifetime =  ros::Duration(0);
    tempMarker.id = i;
    tempMarker.type = visualization_msgs::Marker::SPHERE;
    tempMarker.action =  visualization_msgs::Marker::ADD;

    tempMarker.color.r = 0.0;
    tempMarker.color.g = 1.0;
    tempMarker.color.b = 0.0;
    tempMarker.color.a = 0.2;

    tempMarker.scale.x = 2 * obs.at(i).z;
    tempMarker.scale.y = 2 * obs.at(i).z;
    tempMarker.scale.z = 2 * obs.at(i).z;

    tempMarker.pose.position.x = obs.at(i).x;
    tempMarker.pose.position.y = obs.at(i).y;
    tempMarker.pose.position.z = 0.0;

    circlearray.markers.push_back (tempMarker);
  }
  pubcirlce.publish(circlearray);
}

int visualize_marker::publishobstacle(std::vector< geometry_msgs::Point32 > obs, int index, float size)
{
  obstacle.header.stamp = ros::Time::now();
  obstacle.action = visualization_msgs::Marker::ADD;
  obstacle.pose.orientation.w = 1.0;
  obstacle.lifetime = ros::Duration(0);
  obstacle.id = index;
  obstacle.type = visualization_msgs::Marker::POINTS;
  // obstacle.type = visualization_msgs::Marker::SPHERE;
  obstacle.scale.x = size;
  obstacle.scale.y = size;
  obstacle.scale.z = size;
  if (index % 3 == 0)
  {
    obstacle.color.r = 1;
    obstacle.color.g = 0.0;
    obstacle.color.b = 0.0;
  }
  else if (index % 3 == 1)
  {
    obstacle.color.g = 1;
    obstacle.color.b = 0.0;
    obstacle.color.r = 0.0;
  }
  else
  {
    obstacle.color.b = 1.0;
    obstacle.color.g = 0.0;
    obstacle.color.r = 0.0; 
  }
  
  obstacle.color.a = 1;
  obstacle.points.clear();

  pathtr.setOrigin(tf::Vector3(0, 0, 0.0));
  pathquater.setRPY(0, 0, 0);
  pathtr.setRotation(pathquater);
  
  for (int i = 0; i < obs.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = obs.at(i).x;
    p.y = obs.at(i).y;
    p.z = -0.5;
    obstacle.points.push_back(p);
  }
  pubobstacle.publish(obstacle);
  return 1;
}


/************************/

/************************visualize_polygon************************/

visualize_polygon::visualize_polygon(std::string name)
{
  pubpolygon = nh.advertise<geometry_msgs::PolygonStamped>("/polygon" + name, 5);
  pubmultipolygon = nh.advertise<visualization_msgs::Marker>("/multipolygon" + name, 5);
  poly.header.frame_id = "map";
  multipoly.header.frame_id = "map";
}

visualize_polygon::~visualize_polygon()
{
}

int visualize_polygon::publishpolygon(geometry_msgs::Polygon bounding_polygon)
{
    poly.header.stamp = ros::Time::now();
    poly.polygon = bounding_polygon;
    pubpolygon.publish(poly);
}

int visualize_polygon::publishmultipolygon(std::vector<std::vector<geometry_msgs::Polygon>> bounding_polygon, int index)
{
  multipoly.header.stamp = ros::Time::now();
  multipoly.action = visualization_msgs::Marker::ADD;
  multipoly.pose.orientation.w = 1.0;
  multipoly.lifetime = ros::Duration(0);
  multipoly.id = 0;
  multipoly.type = visualization_msgs::Marker::LINE_LIST;
  multipoly.scale.x = 0.02;
  multipoly.scale.y = 0.02;
  multipoly.color.r = 1.0;
  multipoly.color.g = 0.1;
  multipoly.color.b = 0.1;
  multipoly.color.a = 1.0;
  multipoly.points.clear();

  pathtr.setOrigin(tf::Vector3(0, 0, 0.0));
  pathquater.setRPY(0, 0, 0);
  pathtr.setRotation(pathquater);
  // pathbr.sendTransform(tf::StampedTransform(pathtr, ros::Time::now(), "/map", "/path"));

  geometry_msgs::Point p;
  //   Mat rook_image = Mat::zeros(200, 800, CV_8UC3);
  for (int j = 0; j < bounding_polygon.at(index).size(); ++j)
  {
    for (int a = 0; a < bounding_polygon[index][j].points.size(); ++a)
    {
      int b = a + 1;
      if (b < bounding_polygon[index][j].points.size())
      {
        p.x = bounding_polygon[index][j].points.at(a).x;
        p.y = bounding_polygon[index][j].points.at(a).y;
        multipoly.points.push_back(p);
        p.x = bounding_polygon[index][j].points.at(b).x;
        p.y = bounding_polygon[index][j].points.at(b).y;
        multipoly.points.push_back(p);
      }
      else
      {
        p.x = bounding_polygon[index][j].points.at(a).x;
        p.y = bounding_polygon[index][j].points.at(a).y;
        multipoly.points.push_back(p);
        p.x = bounding_polygon[index][j].points.at(0).x;
        p.y = bounding_polygon[index][j].points.at(0).y;
        multipoly.points.push_back(p);
      }
    }
    //     rectangle(rook_image,
    // 	      Point(bounding_polygon[index][j].points.at(0).x * 10.0, bounding_polygon[index][j].points.at(0).y * 10.0),
    // 	      Point(bounding_polygon[index][j].points.at(2).x * 10.0, bounding_polygon[index][j].points.at(2).y * 10.0),
    // 	      Scalar(123, 231, 213),
    // 	      CV_FILLED,
    // 	      8);
  }
  //   //The C macro is marvelous
  //   std::string dirname = __FILE__;
  //   boost::filesystem::path pathname(dirname);
  //   pathname.remove_filename();
  //   dirname = pathname.string();
  //   imwrite(dirname.append("/scripts/wtf.jpg"), rook_image);
  // //   namedWindow("Polygon contours", CV_WINDOW_AUTOSIZE);
  // //   imshow( "Polygon contours", rook_image );
  pubmultipolygon.publish(multipoly);
  return 1;
}

/************************/