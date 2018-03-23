#include "lidarpath.h"
lidarpath::lidarpath(ros::NodeHandle nh)
    : count(0), radius(4), curindex(0), lcurindex(0), rcurindex(0), endindex(0), lendindex(0), rendindex(0), tracecount(0), resolution(0.1), llaneoff(0), rlaneoff(0)
{
  //Get the current file folder place string
  dirname = __FILE__;
  boost::filesystem::path pathname(dirname);
  pathname.remove_filename();
  dirname = pathname.string();
  filename = dirname + "/scripts/lidarmap.cfg";
  lidarsub = nh.subscribe("vrep/front_scan", 1000, &lidarpath::lidarcallback, this);
  width = static_cast<int>(80 / resolution);
  height = static_cast<int>(20 / resolution);
  vArrayCreate(lidarmap, height, width);
  startpt.x = 600;
  startpt.y = 100;
  startpt.z = M_PI - 0;
  coeff.push_back(0.1);
  coeff.push_back(0.2);
  coeff.push_back(0.3);
  coeff.push_back(0.4);
}

lidarpath::lidarpath()
    : count(0), radius(4), curindex(0), lcurindex(0), rcurindex(0), endindex(0), lendindex(0), rendindex(0), tracecount(0), resolution(0.1), llaneoff(0), rlaneoff(0)
{
  dirname = __FILE__;
  boost::filesystem::path pathname(dirname);
  pathname.remove_filename();
  dirname = pathname.string();
  filename = dirname + "/scripts/lidarmap.cfg";
  coeff.push_back(0.1);
  coeff.push_back(0.2);
  coeff.push_back(0.3);
  coeff.push_back(0.4);
}

lidarpath::~lidarpath()
{
}

void lidarpath::run()
{
  std::vector<geometry_msgs::Point> ppt;
  generateline(50, 50, 52, 52, ppt);
  // createmap(filename.c_str());
  // point2map(cloudmsg);
  // createmap(filename.c_str());
  // setresolution(0.4);
  // setsideinfo(80, 20);
  // traceboundary(lidarmap);
  // ROS_ERROR_STREAM("The llaneoff: "<<llaneoff<<"; rlaneoff: "<<rlaneoff);
  // updategoal(goalpt, lidarmap);
}

int lidarpath::updatelgoal(geometry_msgs::Point &goalref, const std::vector<std::vector<unsigned char>> &tempmap)
{
  //This lane offset value should be devided by the resolution of the grid map!!!
  float defaultlane = 3.0 / 0.4;
  float predeg, tempdeg, refdeg;
  // ROS_ERROR_STREAM("Count: "<<count);
  // ROS_ERROR_STREAM("The lendindex: "<<lendindex<<"; rendindex: "<<rendindex);
  std::vector<cv::Point> points;
  double a = 0;
  double b = 0;
  double rad = 0;
  int num = lvalidpt.size();

  if (num >= 2)
  {
#if BLOCK
    //The following algorithm needs to be revised in the future occasions
    //Currently it works pretty well!!!
    if (lvalidpt.at(num - 1).y == lvalidpt.at(num - 2).y)
    {
      ROS_ERROR_STREAM("T______N______T");
      for (int i = -2; i <= 2; ++i)
      {
        if (lvalidpt.at(num - 1).x < lvalidpt.at(num - 2).x)
        {
          for (int j = lvalidpt.at(num - 2).x; j >= lvalidpt.at(num - 1).x; --j)
          {
            if ((lvalidpt.at(num - 1).y + i) > 0 && (lvalidpt.at(num - 1).y + i) < tempmap.size())
            {
              if (tempmap[lvalidpt.at(num - 1).y + i][j] > 250)
              {
                cv::Point temp;
                temp.x = j;
                temp.y = lvalidpt.at(num - 1).y + i;
                points.push_back(temp);
              }
            }
          }
        }
        else
        {
          for (int j = lvalidpt.at(num - 2).x; j <= lvalidpt.at(num - 1).x; ++j)
          {
            if ((lvalidpt.at(num - 1).y + i) > 0 && (lvalidpt.at(num - 1).y + i) < tempmap.size())
            {
              if (tempmap[lvalidpt.at(num - 1).y + i][j] > 250)
              {
                cv::Point temp;
                temp.x = j;
                temp.y = lvalidpt.at(num - 1).y + i;
                points.push_back(temp);
              }
            }
          }
        }
      }
    }
    else if (lvalidpt.at(num - 1).x == lvalidpt.at(num - 2).x)
    {
      ROS_ERROR_STREAM("W______T______F");
      for (int i = -2; i <= 2; ++i)
      {
        if (lvalidpt.at(num - 1).y < lvalidpt.at(num - 2).y)
        {
          for (int j = lvalidpt.at(num - 2).y; j >= lvalidpt.at(num - 1).y; --j)
          {
            if ((lvalidpt.at(num - 1).x + i) > 0 && (lvalidpt.at(num - 1).x + i) < tempmap.front().size())
            {
              if (tempmap[j][lvalidpt.at(num - 1).x + i] > 250)
              {
                cv::Point temp;
                temp.x = lvalidpt.at(num - 1).x + i;
                temp.y = j;
                points.push_back(temp);
              }
            }
          }
        }
        else
        {
          for (int j = lvalidpt.at(num - 2).y; j <= lvalidpt.at(num - 1).y; ++j)
          {
            if ((lvalidpt.at(num - 1).x + i) > 0 && (lvalidpt.at(num - 1).x + i) < tempmap.front().size())
            {
              if (tempmap[j][lvalidpt.at(num - 1).x + i] > 250)
              {
                cv::Point temp;
                temp.x = lvalidpt.at(num - 1).x + i;
                temp.y = j;
                points.push_back(temp);
              }
            }
          }
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("H______H______H");
      double k = (lvalidpt.at(num - 2).y - lvalidpt.at(num - 1).y) / (lvalidpt.at(num - 2).x - lvalidpt.at(num - 1).x);
      double theta = atan(-1.0 / k);
      k = -k;
      for (int i = -2; i <= 2; ++i)
      {
        for (int j = lvalidpt.at(num - 2).x; j >= lvalidpt.at(num - 1).x; --j)
        {
          int dy = static_cast<int>((lvalidpt.at(num - 2).x - j) * k + 0.5);
          int yn = static_cast<int>(lvalidpt.at(num - 2).y + dy + i * sin(theta) + 0.5);
          int xn = static_cast<int>(j + i * cos(theta) + 0.5);
          if (yn > 0 && yn < tempmap.size() && xn > 0 && xn < tempmap.front().size())
          {
            if (tempmap[yn][xn] > 250)
            {
              cv::Point temp;
              temp.x = xn;
              temp.y = yn;
              points.push_back(temp);
            }
          }
        }
      }
    }
#endif

//     double dx = lvalidpt.at(num - 1).x - lvalidpt.at(num - 2).x;
//     double dy = lvalidpt.at(num - 1).y - lvalidpt.at(num - 2).y;
//     rad = atan2(dy, dx);
//     if (rad < 0)
//       rad = 2 * M_PI + rad;
//     goalref.z = M_PI - rad;
//     double l = defaultlane / sqrt(dx * dx + dy * dy);
//     goalref.x = lvalidpt.at(num - 1).x + l * dy;
//     goalref.y = lvalidpt.at(num - 1).y - l * dx;
//     if (goalref.y < 0 || goalref.y > height || goalref.x < 0 || goalref.x > width)
//     {
//       return 0;
//     }

#if BLOCK
    if (points.size() > 0)
    {
      cv::Vec4f line;
      cv::fitLine(
          points,
          line,
          CV_DIST_HUBER,
          0,
          0.01,
          0.01);
      //The direction vector
      float cos_theta = line[0];
      float sin_theta = line[1];
      //A single point on the line
      float x0 = line[2];
      float y0 = line[3];
      float phi = atan2(sin_theta, cos_theta);
      //The line heading
      phi = (phi >= 0.0) ? phi : (phi + 2 * M_PI);
      goalref.z = M_PI - phi;
      //The line function
      float rho = y0 * cos_theta - x0 * sin_theta;
      //The perpendicular line through the last point in the point cluster
      float perpendicular = lvalidpt.at(num - 1).y * sin_theta + lvalidpt.at(num - 1).x * cos_theta;

      float theta = phi - M_PI / 2.0;
      theta = (theta >= 0.0) ? theta : (phi + 2 * M_PI);
      goalref.x = lvalidpt.at(num - 1).x + defaultlane * cos(theta);
      goalref.y = lvalidpt.at(num - 1).y + defaultlane * sin(theta);
    }
#endif

    cv::Point temp;
    temp.x = lvalidpt.at(num - 2).x;
    temp.y = lvalidpt.at(num - 2).y;
    points.push_back(temp);
    temp.x = lvalidpt.at(num - 1).x;
    temp.y = lvalidpt.at(num - 1).y;
    points.push_back(temp);
    if (points.size() > 0)
    {
      cv::Vec4f line;
      cv::fitLine(
          points,
          line,
          CV_DIST_HUBER,
          0,
          0.01,
          0.01);
      //The direction vector
      float cos_theta = line[0];
      float sin_theta = line[1];
      //A single point on the line
      float x0 = line[2];
      float y0 = line[3];
      float phi = atan2(sin_theta, cos_theta);
      //The line heading
      phi = (phi >= 0.0) ? phi : (phi + 2 * M_PI);
      goalref.z = M_PI - phi;
      //The line function
      float rho = y0 * cos_theta - x0 * sin_theta;
      //The perpendicular line through the last point in the point cluster
      float perpendicular = lvalidpt.at(num - 1).y * sin_theta + lvalidpt.at(num - 1).x * cos_theta;

      float theta = phi - M_PI / 2.0;
      theta = (theta >= 0.0) ? theta : (phi + 2 * M_PI);
      goalref.x = lvalidpt.at(num - 1).x + defaultlane * cos(theta);
      goalref.y = lvalidpt.at(num - 1).y + defaultlane * sin(theta);
    }

    if (lfilterpt.size() < 4)
    {
      lfilterpt.push_back(goalref);
    }
    else
    {
      geometry_msgs::Point temp;
      for (int i = 0; i < lfilterpt.size(); ++i)
      {
        temp.y += lfilterpt.at(i).y * coeff.at(i);
        temp.z += lfilterpt.at(i).z * coeff.at(i);
      }
      lfilterpt.pop_front();
      goalref.y = 0.4 * goalref.y + 0.6 * temp.y;
      goalref.z = 0.4 * goalref.z + 0.6 * temp.z;
      lfilterpt.push_back(goalref);
    }
    return 1;
  }
  return 0;
}

int lidarpath::updatergoal(geometry_msgs::Point &goalref, const std::vector<std::vector<unsigned char>> &tempmap)
{
  //This lane offset value should be devided by the resolution of the grid map!!!
  float defaultlane = 3.0 / 0.4;
  float predeg, tempdeg, refdeg;
  // ROS_ERROR_STREAM("Count: "<<count);
  // ROS_ERROR_STREAM("The lendindex: "<<lendindex<<"; rendindex: "<<rendindex);
  std::vector<cv::Point> points;
  //   std::vector< geometry_msgs::Point > temppt;
  double a = 0;
  double b = 0;
  double rad = 0;
  int num = rvalidpt.size();
  if (num >= 2)
  {
//The following algorithm needs to be revised in the future occasions
//Currently it works pretty well!!!
#if BLOCK
    if (rvalidpt.at(num - 1).y == rvalidpt.at(num - 2).y)
    {
      ROS_ERROR_STREAM("T______N______T");
      for (int i = -2; i <= 2; ++i)
      {
        if (rvalidpt.at(num - 1).x < rvalidpt.at(num - 2).x)
        {
          for (int j = rvalidpt.at(num - 2).x; j >= rvalidpt.at(num - 1).x; --j)
          {
            if ((rvalidpt.at(num - 1).y + i) > 0 && (rvalidpt.at(num - 1).y + i) < tempmap.size())
            {
              if (tempmap[rvalidpt.at(num - 1).y + i][j] > 250)
              {
                //                 geometry_msgs::Point temp;
                //                 temp.x = j;
                //                 temp.y = rvalidpt.at(num - 1).y + i;
                //                 temppt.push_back(temp);
                cv::Point temp;
                temp.x = j;
                temp.y = rvalidpt.at(num - 1).y + i;
                points.push_back(temp);
              }
            }
          }
        }
        else
        {
          for (int j = rvalidpt.at(num - 2).x; j <= rvalidpt.at(num - 1).x; ++j)
          {
            if ((rvalidpt.at(num - 1).y + i) > 0 && (rvalidpt.at(num - 1).y + i) < tempmap.size())
            {
              if (tempmap[rvalidpt.at(num - 1).y + i][j] > 250)
              {
                //                 geometry_msgs::Point temp;
                //                 temp.x = j;
                //                 temp.y = rvalidpt.at(num - 1).y + i;
                //                 temppt.push_back(temp);
                cv::Point temp;
                temp.x = j;
                temp.y = rvalidpt.at(num - 1).y + i;
                points.push_back(temp);
              }
            }
          }
        }
      }
    }
    else if (rvalidpt.at(num - 1).x == rvalidpt.at(num - 2).x)
    {
      ROS_ERROR_STREAM("W______T______F");
      for (int i = -2; i <= 2; ++i)
      {
        if (rvalidpt.at(num - 1).y < rvalidpt.at(num - 2).y)
        {
          for (int j = rvalidpt.at(num - 2).y; j >= rvalidpt.at(num - 1).y; --j)
          {
            if ((rvalidpt.at(num - 1).x + i) > 0 && (rvalidpt.at(num - 1).x + i) < tempmap.front().size())
            {
              if (tempmap[j][rvalidpt.at(num - 1).x + i] > 250)
              {
                //                 geometry_msgs::Point temp;
                //                 temp.x = rvalidpt.at(num - 1).x + i;
                //                 temp.y = j;
                //                 temppt.push_back(temp);
                cv::Point temp;
                temp.x = rvalidpt.at(num - 1).x + i;
                temp.y = j;
                points.push_back(temp);
              }
            }
          }
        }
        else
        {
          for (int j = rvalidpt.at(num - 2).y; j <= rvalidpt.at(num - 1).y; ++j)
          {
            if ((rvalidpt.at(num - 1).x + i) > 0 && (rvalidpt.at(num - 1).x + i) < tempmap.front().size())
            {
              if (tempmap[j][rvalidpt.at(num - 1).x + i] > 250)
              {
                //                 geometry_msgs::Point temp;
                //                 temp.x = rvalidpt.at(num - 1).x + i;
                //                 temp.y = j;
                //                 temppt.push_back(temp);
                cv::Point temp;
                temp.x = rvalidpt.at(num - 1).x + i;
                temp.y = j;
                points.push_back(temp);
              }
            }
          }
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM("H______H______H");
      double k = (rvalidpt.at(num - 2).y - rvalidpt.at(num - 1).y) / (rvalidpt.at(num - 2).x - rvalidpt.at(num - 1).x);
      double theta = atan(-1.0 / k);
      k = -k;
      for (int i = -2; i <= 2; ++i)
      {
        for (int j = rvalidpt.at(num - 2).x; j >= rvalidpt.at(num - 1).x; --j)
        {
          int dy = static_cast<int>((rvalidpt.at(num - 2).x - j) * k + 0.5);
          int yn = static_cast<int>(rvalidpt.at(num - 2).y + dy + i * sin(theta) + 0.5);
          int xn = static_cast<int>(j + i * cos(theta) + 0.5);
          if (yn > 0 && yn < tempmap.size() && xn > 0 && xn < tempmap.front().size())
          {
            if (tempmap[yn][xn] > 250)
            {
              //               geometry_msgs::Point temp;
              //               temp.x = xn;
              //               temp.y = yn;
              //               temppt.push_back(temp);
              cv::Point temp;
              temp.x = xn;
              temp.y = yn;
              ROS_ERROR_STREAM(xn << "; " << yn);
              points.push_back(temp);
            }
          }
        }
      }
    }
#endif

#if BLOCK
    //This beautiful method is not used due to the lack of points
    if (points.size() > 0)
    {
      cv::Vec4f line;
      ROS_ERROR_STREAM(points.size());
      cv::fitLine(
          points,
          line,
          CV_DIST_HUBER,
          0,
          0.01,
          0.01);
      //The direction vector
      float cos_theta = line[0];
      float sin_theta = line[1];
      //A single point on the line
      float x0 = line[2];
      float y0 = line[3];
      float phi = atan2(sin_theta, cos_theta);
      //The line heading
      phi = (phi >= 0.0) ? phi : (phi + 2 * M_PI);
      goalref.z = M_PI - phi;
      //The line function
      float rho = y0 * cos_theta - x0 * sin_theta;
      //The perpendicular line through the last point in the point cluster
      float perpendicular = rvalidpt.at(num - 1).y * sin_theta + rvalidpt.at(num - 1).x * cos_theta;

      float theta = phi + M_PI / 2.0;
      theta = (theta <= 2 * M_PI) ? theta : (theta - 2 * M_PI);
      goalref.x = rvalidpt.at(num - 1).x + defaultlane * cos(theta);
      goalref.y = rvalidpt.at(num - 1).y + defaultlane * sin(theta);
    }
#endif

    //     double dx = rvalidpt.at(num - 1).x - rvalidpt.at(num - 2).x;
    //     double dy = rvalidpt.at(num - 1).y - rvalidpt.at(num - 2).y;
    //     rad = atan2(dy, dx);
    //     if (rad < 0)
    //       rad = 2 * M_PI + rad;
    //     goalref.z = M_PI - rad;
    //     double l = defaultlane / sqrt(dx * dx + dy * dy);
    //     goalref.x = rvalidpt.at(num - 1).x - l * dy;
    //     goalref.y = rvalidpt.at(num - 1).y + l * dx;
    //     if (goalref.y < 0 || goalref.y > height || goalref.x < 0 || goalref.x > width)
    //     {
    //       return 0;
    //     }

    cv::Point temp;
    temp.x = rvalidpt.at(num - 2).x;
    temp.y = rvalidpt.at(num - 2).y;
    points.push_back(temp);
    temp.x = rvalidpt.at(num - 1).x;
    temp.y = rvalidpt.at(num - 1).y;
    points.push_back(temp);
    if (points.size() > 0)
    {
      cv::Vec4f line;
      cv::fitLine(
          points,
          line,
          CV_DIST_HUBER,
          0,
          0.01,
          0.01);
      //The direction vector
      float cos_theta = line[0];
      float sin_theta = line[1];
      //A single point on the line
      float x0 = line[2];
      float y0 = line[3];
      float phi = atan2(sin_theta, cos_theta);
      //The line heading
      phi = (phi >= 0.0) ? phi : (phi + 2 * M_PI);
      goalref.z = M_PI - phi;
      //The line function
      float rho = y0 * cos_theta - x0 * sin_theta;
      //The perpendicular line through the last point in the point cluster
      float perpendicular = rvalidpt.at(num - 1).y * sin_theta + rvalidpt.at(num - 1).x * cos_theta;

      float theta = phi + M_PI / 2.0;
      theta = (theta <= 2 * M_PI) ? theta : (theta - 2 * M_PI);
      goalref.x = rvalidpt.at(num - 1).x + defaultlane * cos(theta);
      goalref.y = rvalidpt.at(num - 1).y + defaultlane * sin(theta);
    }

    if (rfilterpt.size() < 4)
    {
      rfilterpt.push_back(goalref);
    }
    else
    {
      geometry_msgs::Point temp;
      for (int i = 0; i < rfilterpt.size(); ++i)
      {
        temp.y += rfilterpt.at(i).y * coeff.at(i);
        temp.z += rfilterpt.at(i).z * coeff.at(i);
      }
      rfilterpt.pop_front();
      goalref.y = 0.4 * goalref.y + 0.6 * temp.y;
      goalref.z = 0.4 * goalref.z + 0.6 * temp.z;
      rfilterpt.push_back(goalref);
    }
    return 1;
  }
  return 0;
}

int lidarpath::traceboundary(const std::vector<std::vector<unsigned char>> &tempmap)
{
  m_pts.clear();
  bresenham(0, 0, radius);
  sortpt(eighthpt);
  if (tracecount < 3)
  {
    tracecount += 1;
  }
  geometry_msgs::Point ltemp, rtemp;
  //The usage of bitset takes less memory cost!!!
  std::bitset<2> exist;
  exist.reset();
  //Get the left path boundary.
  lvalidpt.clear();
  // for (int j = startpt.x; j >= startpt.x -3; --j)
  // {
  //   for (int i = startpt.y; i > 0; --i)
  //   {
  for (int i = startpt.y; i > 0; --i)
  {
    for (int j = startpt.x; j >= startpt.x - 3; --j)
    {
      if (tempmap[i][j] > 250)
      {
        ltemp.x = j;
        ltemp.y = i;
        ROS_INFO_STREAM("Temp value: x " << ltemp.x << "; y " << ltemp.y);
        exist.set(0);
        break;
      }
    }
    if (exist[0])
    {
      break;
    }
  }
  //Get the right path boundary.
  rvalidpt.clear();
  // for (int j = startpt.x; j >= startpt.x -3; --j)
  // {
  //   for (int i = startpt.y; i < height; ++i)
  //   {
  for (int i = startpt.y; i < height; ++i)
  {
    for (int j = startpt.x; j >= startpt.x - 3; --j)
    {
      if (tempmap[i][j] > 250)
      {
        rtemp.x = j;
        rtemp.y = i;
        ROS_INFO_STREAM("Temp value: x " << rtemp.x << "; y " << rtemp.y);
        exist.set(1);
        break;
      }
    }
    if (exist[1])
    {
      break;
    }
  }

  validpath.clear();
  if (exist[0])
  {
    // searchlgoal(ltemp, tempmap);
    searchgoal(ltemp, tempmap, LEFT_LANE);
    validpath.push_back(lvalidpt);
  }
  else
  {
    ROS_INFO_STREAM("The left road path size is null!!!");
  }

  if (exist[1])
  {
    // searchrgoal(rtemp, tempmap);
    searchgoal(rtemp, tempmap, RIGHT_LANE);
    validpath.push_back(rvalidpt);
  }
  else
  {
    ROS_INFO_STREAM("The right road path size is null!!!");
  }
  ROS_WARN_STREAM("The size of the validpath: " << validpath.size());
  for (int i = 0; i < validpath.size(); ++i)
  {
    // for (int j = 0; j < validpath.at(i).size(); ++j)
    // {
    //   ROS_INFO_STREAM("validpath["<<i<<"]["<<j<<"].x: "<<validpath.at(i).at(j).x<<"; "<<"validpath["<<i<<"]["<<j<<"].y: "<<validpath[i][j].y);
    // }
    //This can raise the overflow debug information!!!
    ROS_WARN_STREAM("validpath[" << i << "]"
                                 << ".x: " << validpath.at(i).back().x << "; "
                                 << "validpath[" << i << "]"
                                 << ".y: " << validpath.at(i).back().y);
  }
  if (tracecount == 1)
  {
    if (lvalidpt.size() > 0)
    {
      llaneoff = resolution * sqrt((lvalidpt.front().x - startpt.x) * (lvalidpt.front().x - startpt.x) + (lvalidpt.front().y - startpt.y) * (lvalidpt.front().y - startpt.y));
    }
    if (rvalidpt.size() > 0)
    {
      rlaneoff = resolution * sqrt((rvalidpt.front().x - startpt.x) * (rvalidpt.front().x - startpt.x) + (rvalidpt.front().y - startpt.y) * (rvalidpt.front().y - startpt.y));
    }
  }

#if CVDEBUG
  pt_trace = Mat(height, width, CV_8UC3, Scalar(100, 200, 100));
  for (int i = 0; i < m_pts.size(); ++i)
  {
    //     pt_trace.at<unsigned char>(height - m_pts.at(i).y, 3 * m_pts.at(i).x) = 0;
    //     pt_trace.at<unsigned char>(height - m_pts.at(i).y, 3 * m_pts.at(i).x + 1) = 0;
    //     pt_trace.at<unsigned char>(height - m_pts.at(i).y, 3 * m_pts.at(i).x + 2) = 255;
    pt_trace.at<Vec3b>(height - m_pts.at(i).y, m_pts.at(i).x)[0] = 0;
    pt_trace.at<Vec3b>(height - m_pts.at(i).y, m_pts.at(i).x)[1] = 0;
    pt_trace.at<Vec3b>(height - m_pts.at(i).y, m_pts.at(i).x)[2] = 255;
  }
  try
  {
    namedWindow("circle", CV_WINDOW_NORMAL);
    imshow("circle", pt_trace);
    waitKey(1);
  }
  catch (cv::Exception &e)
  {
    std::cout << e.what() << std::endl;
    std::cerr << e.what() << std::endl;
  }
#endif
  return 1;
}

//This algorithm is awesome!!!
int lidarpath::searchgoal(const geometry_msgs::Point &currentpt, const std::vector<std::vector<unsigned char>> &tempmap, emLane lane)
{
  // ROS_ERROR_STREAM("currentpt.x "<<currentpt.x<<" currentpt.y "<<currentpt.y);
  std::vector<geometry_msgs::Point> gridpt;
  switch (lane)
  {
  case LEFT_LANE:
    maprhdpt(currentpt, gridpt);
    lvalidpt.push_back(currentpt);
    break;
  case RIGHT_LANE:
    maplhdpt(currentpt, gridpt);
    rvalidpt.push_back(currentpt);
    break;
  default:
    ROS_ERROR_STREAM("Invalid lane info!!!");
    return 0;
  }
  bool exist = false;
  geometry_msgs::Point nextpt;
  count = gridpt.size();
  int i = curindex;
  int judgeindex = curindex - count / 2;
  judgeindex = (judgeindex < 0) ? (count + judgeindex) : judgeindex;
  while (i != judgeindex)
  {
    // while((i <= count / 3) || ( i >= 2 * count / 3) )
    // {
    i = (i >= count) ? (i - count) : i;
    if (0 <= gridpt.at(i).y && gridpt.at(i).y < height && gridpt.at(i).x < width && 0 <= gridpt.at(i).x)
    {
#if CVDEBUG
      geometry_msgs::Point temp;
      temp.x = gridpt.at(i).x;
      temp.y = gridpt.at(i).y;
      m_pts.push_back(temp);
#endif
      // std::vector<geometry_msgs::Point> v;
      // linecasting(currentpt, gridpt.at(i), v);
      // for (int i = 0; i < v.size(); ++i)
      // {
      //   if (tempmap[v.at(i).y][v.at(i).x] > 250)
      //   {
      //     nextpt.x = v.at(i).x;
      //     nextpt.y = v.at(i).y;
      //     curindex = i - count/4;
      //     curindex = (curindex < 0)? (count + curindex) : curindex;
      //     exist = true;
      //     break;
      //   }
      // }
      if (tempmap[gridpt.at(i).y][gridpt.at(i).x] > 250)
      {
        nextpt.x = gridpt.at(i).x;
        nextpt.y = gridpt.at(i).y;
        curindex = i - count / 4;
        curindex = (curindex < 0) ? (count + curindex) : curindex;
        exist = true;
        break;
      }
    }
    i += 1;
  }
  if (exist)
  {
    searchgoal(nextpt, tempmap, lane);
  }
  else
  {
    switch (lane)
    {
    case LEFT_LANE:
      lendindex = curindex;
      break;
    case RIGHT_LANE:
      rendindex = curindex;
      break;
    default:
      endindex = curindex;
      break;
    }
    curindex = 0;
    return 1;
  }
}

void lidarpath::searchlgoal(const geometry_msgs::Point &currentpt, const std::vector<std::vector<unsigned char>> &tempmap)
{
  // ROS_ERROR_STREAM("currentpt.x "<<currentpt.x<<" currentpt.y "<<currentpt.y);
  lvalidpt.push_back(currentpt);
  bool exist = false;
  geometry_msgs::Point nextpt;
  rgridpt.clear();
  maprhdpt(currentpt, rgridpt);
  count = rgridpt.size();
  // ROS_ERROR_STREAM("count = rgridpt.size(): "<<count);
  int i = lcurindex;
  int judgeindex = lcurindex - count / 2;
  judgeindex = (judgeindex < 0) ? (count + judgeindex) : judgeindex;
  while (i != judgeindex)
  {
    // while((i <= count / 3) || ( i >= 2 * count / 3) )
    // {
    i = (i >= count) ? (i - count) : i;
    if (0 <= rgridpt.at(i).y && rgridpt.at(i).y < height && rgridpt.at(i).x < width && 0 <= rgridpt.at(i).x)
    {
#if CVDEBUG
      geometry_msgs::Point temp;
      temp.x = rgridpt.at(i).x;
      temp.y = rgridpt.at(i).y;
      m_pts.push_back(temp);
#endif
      // std::vector<geometry_msgs::Point> v;
      // linecasting(currentpt, rgridpt.at(i), v);
      // for (int i = 0; i < v.size(); ++i)
      // {
      //   if (tempmap[v.at(i).y][v.at(i).x] > 250)
      //   {
      //     nextpt.x = v.at(i).x;
      //     nextpt.y = v.at(i).y;
      //     lcurindex = i - count/4;
      //     lcurindex = (lcurindex < 0)? (count + lcurindex) : lcurindex;
      //     exist = true;
      //     break;
      //   }
      // }

      if (tempmap[rgridpt.at(i).y][rgridpt.at(i).x] > 250)
      {
        nextpt.x = rgridpt.at(i).x;
        nextpt.y = rgridpt.at(i).y;
        lcurindex = i - count / 4;
        lcurindex = (lcurindex < 0) ? (count + lcurindex) : lcurindex;
        exist = true;
        break;
      }
    }
    i += 1;
  }
  if (exist)
  {
    searchlgoal(nextpt, tempmap);
  }
  else
  {
    lendindex = lcurindex;
    lcurindex = 0;
    return;
  }
}

void lidarpath::searchrgoal(const geometry_msgs::Point &currentpt, const std::vector<std::vector<unsigned char>> &tempmap)
{
  // ROS_ERROR_STREAM("currentpt.x "<<currentpt.x<<" currentpt.y "<<currentpt.y<<" !!!");
  rvalidpt.push_back(currentpt);
  bool exist = false;
  geometry_msgs::Point nextpt;
  lgridpt.clear();
  maplhdpt(currentpt, lgridpt);
  count = lgridpt.size();
  // ROS_WARN_STREAM("count = lgridpt.size(): "<<count);
  // for (int i = 0; i < lgridpt.size(); ++i)
  // {
  //   ROS_DEBUG_STREAM("i: "<<i<<"; lgridpt.at(i).x: "<<lgridpt.at(i).x<<"; lgridpt.at(i).y: "<<lgridpt.at(i).y);
  // }
  int i = rcurindex;
  int judgeindex = rcurindex - count / 2;
  judgeindex = (judgeindex < 0) ? (count + judgeindex) : judgeindex;
  while (i != judgeindex)
  {
    // while((i <= count / 3) || ( i >= 2 * count / 3) )
    // {
    i = (i >= count) ? (i - count) : i;
    if (0 <= lgridpt.at(i).y && lgridpt.at(i).y < height && lgridpt.at(i).x >= 0 && lgridpt.at(i).x < width)
    {
#if CVDEBUG
      geometry_msgs::Point temp;
      temp.x = lgridpt.at(i).x;
      temp.y = lgridpt.at(i).y;
      m_pts.push_back(temp);
#endif
      // std::vector<geometry_msgs::Point> v;
      // linecasting(currentpt, lgridpt.at(i), v);
      // for (int i = 0; i < v.size(); ++i)
      // {
      //   if (tempmap[v.at(i).y][v.at(i).x] > 250)
      //   {
      //     nextpt.x = v.at(i).x;
      //     nextpt.y = v.at(i).y;
      //     rcurindex = i - count/4;
      //     rcurindex = (rcurindex < 0)? (count + rcurindex) : rcurindex;
      //     exist = true;
      //     break;
      //   }
      // }

      if (tempmap[lgridpt.at(i).y][lgridpt.at(i).x] > 250)
      {
        nextpt.x = lgridpt.at(i).x;
        nextpt.y = lgridpt.at(i).y;
        rcurindex = i - count / 4;
        rcurindex = (rcurindex < 0) ? (count + rcurindex) : rcurindex;
        exist = true;
        break;
      }
    }
    i += 1;
  }
  if (exist)
  {
    searchrgoal(nextpt, tempmap);
  }
  else
  {
    rendindex = rcurindex;
    rcurindex = 0;
    return;
  }
}

void lidarpath::maprhdpt(const geometry_msgs::Point &currentpt, std::vector<geometry_msgs::Point> &temppt)
{
  geometry_msgs::Point temp;
  for (int i = 0; i < rhd.size(); ++i)
  {
    temp.x = currentpt.x - rhd.at(i).y;
    temp.y = currentpt.y + rhd.at(i).x;
    temppt.push_back(temp);
  }
  return;
}

void lidarpath::maplhdpt(const geometry_msgs::Point &currentpt, std::vector<geometry_msgs::Point> &temppt)
{
  geometry_msgs::Point temp;
  for (int i = 0; i < lhd.size(); ++i)
  {
    temp.x = currentpt.x + lhd.at(i).y;
    temp.y = currentpt.y - lhd.at(i).x;
    temppt.push_back(temp);
  }
  return;
}

void lidarpath::lidarcallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  lasermsg = *msg;
  laser2pl.projectLaser(lasermsg, cloudmsg);
}

//Get the clock wise coordinate system circle
void lidarpath::sortpt(const std::vector<geometry_msgs::Point> &temppt)
{
  rhd.clear();
  geometry_msgs::Point temp;
  int num;
  num = temppt.size();
  //Get the quarter circle
  for (int i = 0; i < num; ++i)
  {
    if (temppt.at(i).x != temppt.at(i).y)
    {
      temp.x = temppt.at(i).y;
      temp.y = temppt.at(i).x;
      rhd.push_back(temp);
    }
  }
  for (int i = num - 1; i > 0; --i)
  {
    rhd.push_back(temppt.at(i));
  }
  //Get the half circle
  //Rotate 90 degrees
  num = rhd.size();
  for (int i = 0; i < num; ++i)
  {
    temp.x = -rhd.at(i).y;
    temp.y = rhd.at(i).x;
    rhd.push_back(temp);
  }
  //Get the full circle
  //Rotate 180 degrees
  num = rhd.size();
  for (int i = 0; i < num; ++i)
  {
    temp.x = -rhd.at(i).x;
    temp.y = -rhd.at(i).y;
    rhd.push_back(temp);
  }
  lhd.assign(rhd.begin(), rhd.end());
  std::reverse(lhd.begin(), lhd.end());
}

void lidarpath::point2map(const sensor_msgs::PointCloud &pl)
{
  int tempx, tempy;
  vArrayCreate(lidarmap, height, width);
  for (int i = 0; i < pl.points.size(); ++i)
  {
    tempx = static_cast<int>(startpt.x - pl.points.at(i).x / resolution + 0.5);
    tempy = static_cast<int>(startpt.y - pl.points.at(i).y / resolution + 0.5);
    int a = 0;
    int b = 0;
    for (a = -1; a <= 1; ++a)
    {
      for (b = -1; b <= 1; ++b)
      {
        tempx = tempx + b;
        tempy = tempy + a;
        if (0 <= tempx && 0 <= tempy && tempx < width && tempy < height)
        {
          lidarmap[tempy][tempx] = 254;
        }
      }
    }
  }
}

void lidarpath::createmap(const char *str)
{
  std::ofstream in;
  in.open(str, std::ios::trunc);
  //   in.open(str, std::ios::app);
  for (int i = 0; i < height; ++i)
  {
    for (int j = 0; j < width; ++j)
    {
      in << (int)lidarmap[i][j] << " ";
    }
    in << "\n";
  }
  in.close();
  return;
}

//This is the improved Bresenham algorithms!!!
void lidarpath::bresenham(int xc, int yc, int r)
{
  geometry_msgs::Point temp;
  //  circlept.clear();
  //  quarterpt.clear();
  eighthpt.clear();
  int x = 0;
  int y = r;
  int p = 4 * r * r - 4 * (x + 1) * (x + 1) - 4 * y * y + 4 * y - 1;
  int px = -8 * x - 12;
  int py = 8 * y - 8;
  //  circleplot(xc, yc, x, y);
  //  quarterplot(xc, yc, x, y);
  temp.x = x;
  temp.y = y;
  eighthpt.push_back(temp);
  while (x < y)
  {
    if (p >= 0)
    {
      x += 1;
      p = p + px;
      px = px - 8;
    }
    else
    {
      x += 1;
      y -= 1;
      p = p + px + py;
      px = px - 8;
      py = py - 8;
    }
    //    circleplot(xc, yc, x, y);
    //    quarterplot(xc, yc, x, y);
    temp.x = x;
    temp.y = y;
    eighthpt.push_back(temp);
  }
}

//Get the quarterpoints
void lidarpath::quarterplot(int xc, int yc, int x, int y)
{
  geometry_msgs::Point temp;
  if (x == y)
  {
    temp.x = x + xc;
    temp.y = y + yc;
    quarterpt.push_back(temp);
  }
  else
  {
    temp.x = x + xc;
    temp.y = y + yc;
    quarterpt.push_back(temp);
    temp.x = y + xc;
    temp.y = x + yc;
    quarterpt.push_back(temp);
  }
  return;
}

//Get the whole circlepoints
void lidarpath::circleplot(int xc, int yc, int x, int y)
{
  geometry_msgs::Point temp;
  if (x == 0)
  {
    temp.x = x + xc;
    temp.y = y + yc;
    circlept.push_back(temp);
    temp.x = x + xc;
    temp.y = -y + yc;
    circlept.push_back(temp);

    temp.x = y + xc;
    temp.y = x + yc;
    circlept.push_back(temp);
    temp.x = -y + xc;
    temp.y = x + yc;
    circlept.push_back(temp);
  }
  else if (y == 0)
  {
    temp.x = x + xc;
    temp.y = y + yc;

    temp.x = -x + xc;
    temp.y = y + yc;
    circlept.push_back(temp);

    temp.x = y + xc;
    temp.y = x + yc;
    circlept.push_back(temp);

    temp.x = y + xc;
    temp.y = -x + yc;
    circlept.push_back(temp);
  }
  else if (x == y)
  {
    temp.x = x + xc;
    temp.y = y + yc;
    circlept.push_back(temp);
    temp.x = x + xc;
    temp.y = -y + yc;
    circlept.push_back(temp);

    temp.x = -x + xc;
    temp.y = y + yc;
    circlept.push_back(temp);
    temp.x = -x + xc;
    temp.y = -y + yc;
    circlept.push_back(temp);
  }
  else
  {
    temp.x = x + xc;
    temp.y = y + yc;
    circlept.push_back(temp);
    temp.x = x + xc;
    temp.y = -y + yc;
    circlept.push_back(temp);

    temp.x = -x + xc;
    temp.y = y + yc;
    circlept.push_back(temp);
    temp.x = -x + xc;
    temp.y = -y + yc;
    circlept.push_back(temp);

    temp.x = y + xc;
    temp.y = x + yc;
    circlept.push_back(temp);
    temp.x = -y + xc;
    temp.y = x + yc;
    circlept.push_back(temp);

    temp.x = y + xc;
    temp.y = -x + yc;
    circlept.push_back(temp);
    temp.x = -y + xc;
    temp.y = -x + yc;
    circlept.push_back(temp);
  }
  return;
}

void lidarpath::vArrayCreate(std::vector<std::vector<unsigned char>> &array, int m, int n)
{
  //   array.resize(m);
  //   for (int i = 0; i < m; i++)
  //   {
  //     array[i].resize(n);
  //   }
  //Another way to create two dimenshin array!!!
  std::vector<unsigned char> temparr;
  temparr.assign(n, 0);
  array.assign(m, temparr);
  return;
}

void lidarpath::setresolution(const float res)
{
  resolution = res;
  startpt.x = 60 / res;
  startpt.y = 10 / res;
  startpt.z = M_PI;
  return;
}

void lidarpath::setsideinfo(const int w, const int h)
{
  width = static_cast<int>(w / resolution + 0.5);
  height = static_cast<int>(h / resolution + 0.5);
  return;
}

void lidarpath::setradius(const int r)
{
  radius = r;
  return;
}

//The following method apply to the normal right hand system coordinate
void lidarpath::linecasting(geometry_msgs::Point src, geometry_msgs::Point dst, std::vector<geometry_msgs::Point> &rtn)
{
  ROS_ERROR_STREAM("src.x: " << src.x << "; src.x: " << src.y << "; dst.x: " << dst.x << "; dst.y: " << dst.y);
  int dy = dst.y - src.y;
  int dx = dst.x - src.x;
  int q = 0;
  double theta = atan2(dy, dx);
  std::vector<geometry_msgs::Point> ptv;
  if (theta >= 0.0)
  {
    q = 4.0 * theta / M_PI;
    ROS_WARN_STREAM("q: " << q);
  }
  else
  {
    q = 7 + 4.0 * theta / M_PI;
    ROS_WARN_STREAM("q: " << q);
  }
  switch (q)
  {
  case 0:
    generateline(src.x, src.y, dst.x, dst.y, rtn);
    // ROS_ERROR_STREAM("The size of rtn: "<<rtn.size());
    for (int i = 0; i < rtn.size(); ++i)
    {
      ROS_WARN_STREAM("i: " << i << "; ptv.at(i).x: " << ptv.at(i).x << "; ptv.at(i).y: " << ptv.at(i).y);
    }
    break;
  case 1:
    generateline(0, 0, dy, dx, ptv);
    // ROS_ERROR_STREAM("The size of ptv: "<<ptv.size());
    for (int i = 0; i < ptv.size(); ++i)
    {
      // ROS_WARN_STREAM("i: "<<i<<"; ptv.at(i).x: "<<ptv.at(i).x<<"; ptv.at(i).y: "<<ptv.at(i).y);
      geometry_msgs::Point temp;
      temp.x = src.x + ptv.at(i).y;
      temp.y = src.y + ptv.at(i).x;
      ROS_WARN_STREAM("i: " << i << "; temp.at(i).x: " << temp.x << "; temp.at(i).y: " << temp.y);
      rtn.push_back(temp);
    }
    break;
  case 2:
    generateline(0, 0, dy, -dx, ptv);
    // ROS_ERROR_STREAM("The size of ptv: "<<ptv.size());
    for (int i = 0; i < ptv.size(); ++i)
    {
      // ROS_WARN_STREAM("i: "<<i<<"; ptv.at(i).x: "<<ptv.at(i).x<<"; ptv.at(i).y: "<<ptv.at(i).y);
      geometry_msgs::Point temp;
      temp.x = src.x - ptv.at(i).y;
      temp.y = src.y + ptv.at(i).x;
      ROS_WARN_STREAM("i: " << i << "; temp.at(i).x: " << temp.x << "; temp.at(i).y: " << temp.y);
      rtn.push_back(temp);
    }
    break;
  case 3:
    generateline(0, 0, -dx, dy, ptv);
    // ROS_ERROR_STREAM("The size of ptv: "<<ptv.size());
    for (int i = 0; i < ptv.size(); ++i)
    {
      // ROS_WARN_STREAM("i: "<<i<<"; ptv.at(i).x: "<<ptv.at(i).x<<"; ptv.at(i).y: "<<ptv.at(i).y);
      geometry_msgs::Point temp;
      temp.x = src.x - ptv.at(i).x;
      temp.y = src.y + ptv.at(i).y;
      ROS_WARN_STREAM("i: " << i << "; temp.at(i).x: " << temp.x << "; temp.at(i).y: " << temp.y);
      rtn.push_back(temp);
    }
    break;
  case 4:
    generateline(0, 0, -dx, -dy, ptv);
    // ROS_ERROR_STREAM("The size of ptv: "<<ptv.size());
    for (int i = 0; i < ptv.size(); ++i)
    {
      // ROS_WARN_STREAM("i: "<<i<<"; ptv.at(i).x: "<<ptv.at(i).x<<"; ptv.at(i).y: "<<ptv.at(i).y);
      geometry_msgs::Point temp;
      temp.x = src.x - ptv.at(i).x;
      temp.y = src.y - ptv.at(i).y;
      ROS_WARN_STREAM("i: " << i << "; temp.at(i).x: " << temp.x << "; temp.at(i).y: " << temp.y);
      rtn.push_back(temp);
    }
    break;
  case 5:
    generateline(0, 0, -dy, -dx, ptv);
    // ROS_ERROR_STREAM("The size of ptv: "<<ptv.size());
    for (int i = 0; i < ptv.size(); ++i)
    {
      // ROS_WARN_STREAM("i: "<<i<<"; ptv.at(i).x: "<<ptv.at(i).x<<"; ptv.at(i).y: "<<ptv.at(i).y);
      geometry_msgs::Point temp;
      temp.x = src.x - ptv.at(i).y;
      temp.y = src.y - ptv.at(i).x;
      ROS_WARN_STREAM("i: " << i << "; temp.at(i).x: " << temp.x << "; temp.at(i).y: " << temp.y);
      rtn.push_back(temp);
    }
    break;
  case 6:
    generateline(0, 0, -dy, dx, ptv);
    // ROS_ERROR_STREAM("The size of ptv: "<<ptv.size());
    for (int i = 0; i < ptv.size(); ++i)
    {
      // ROS_WARN_STREAM("i: "<<i<<"; ptv.at(i).x: "<<ptv.at(i).x<<"; ptv.at(i).y: "<<ptv.at(i).y);
      geometry_msgs::Point temp;
      temp.x = src.x + ptv.at(i).y;
      temp.y = src.y - ptv.at(i).x;
      ROS_WARN_STREAM("i: " << i << "; temp.at(i).x: " << temp.x << "; temp.at(i).y: " << temp.y);
      rtn.push_back(temp);
    }
    break;
  case 7:
    generateline(0, 0, dx, -dy, ptv);
    // ROS_ERROR_STREAM("The size of ptv: "<<ptv.size());
    for (int i = 0; i < ptv.size(); ++i)
    {
      // ROS_WARN_STREAM("i: "<<i<<"; ptv.at(i).x: "<<ptv.at(i).x<<"; ptv.at(i).y: "<<ptv.at(i).y);
      geometry_msgs::Point temp;
      temp.x = src.x + ptv.at(i).x;
      temp.y = src.y - ptv.at(i).y;
      ROS_WARN_STREAM("i: " << i << "; temp.at(i).x: " << temp.x << "; temp.at(i).y: " << temp.y);
      rtn.push_back(temp);
    }
    break;
  default:
    ROS_ERROR_STREAM("Invalid value!!!");
    break;
  }

#if CVDEBUG
  //   Mat image = Mat(height, width, CV_8UC3, Scalar::all(255));
  Mat image = Mat(height, width, CV_8UC3, Scalar(50, 150, 100));
  for (int i = 0; i < rtn.size(); ++i)
  {
    //     image.at<unsigned char>(height - rtn.at(i).y, 3 * rtn.at(i).x) = 0;
    //     image.at<unsigned char>(height - rtn.at(i).y, 3 * rtn.at(i).x + 1) = 255;
    //     image.at<unsigned char>(height - rtn.at(i).y, 3 * rtn.at(i).x + 2) = 0;
    image.at<Vec3b>(height - rtn.at(i).y, rtn.at(i).x)[0] = 0;
    image.at<Vec3b>(height - rtn.at(i).y, rtn.at(i).x)[1] = 255;
    image.at<Vec3b>(height - rtn.at(i).y, rtn.at(i).x)[2] = 0;
  }
  //   for (int c = 0; c < 3; ++c)
  //   {
  //     std::cout<<(int)image.at<Vec3b>(0, 0)[c]<<std::endl;
  //   }
  try
  {
    //     char buffer[20];
    //     double t = static_cast<double>(ros::Time::now().toSec());
    //     sprintf(buffer, "%lf", t);
    //     std::string str = buffer;
    //     str = "./pic/" + str + ".jpg";
    // //     ROS_INFO_STREAM(str);
    //     Mat gray_image;
    //     cvtColor(image, gray_image, CV_BGR2GRAY);
    //     imwrite(str,gray_image);
    namedWindow("linecasting", CV_WINDOW_NORMAL);
    imshow("linecasting", image);
    waitKey(1);
  }
  catch (cv::Exception &e)
  {
    std::cout << e.what() << std::endl;
    std::cerr << e.what() << std::endl;
  }
#endif

  return;
}

/*This improved bresenham line generating method is wonderful!!!*/
void lidarpath::generateline(int x0, int y0, int x1, int y1, std::vector<geometry_msgs::Point> &lineseg)
{
  std::vector<geometry_msgs::Point> hl, tl;
  geometry_msgs::Point temp;
  int dx, dy, incre, incrne, d, x, y, n, m, inc;
  dx = x1 - x0;
  dy = y1 - y0;
  d = dy * 2 - dx;
  incre = 2 * dy;
  incrne = 2 * (dy - dx);
  x = x0;
  y = y0;

  temp.x = x1;
  temp.y = y1;
  lineseg.push_back(temp);

  while ((y < y0 + 1) && (x < x1))
  {
    if (d <= 0)
    {
      d += incre;
      x++;
      x1--;
    }
    else
    {
      x++;
      y++;
      y1--;
    }
    temp.x = x;
    temp.y = y;
    hl.push_back(temp);
    temp.x = x1;
    temp.y = y1;
    tl.push_back(temp);
  }
  n = x - x0;
  inc = incrne + (2 * n - 3) * incre;
  while (x < x1)
  {
    d += inc;
    if (d > 0)
    {
      m = 2 * n - 2;
      temp.x = x;
      temp.y = y;
      hl.push_back(temp);
      x += m;
      x1 -= m;
      temp.x = x1;
      temp.y = y1;
      tl.push_back(temp);
    }
    else
    {
      if (d > -incre)
      {
        m = 2 * n - 1;
        temp.x = x;
        temp.y = y;
        hl.push_back(temp);
        x += m;
        x1 -= m;
        temp.x = x1;
        temp.y = y1;
        tl.push_back(temp);
        d += incre;
      }
      else
      {
        m = 2 * n;
        temp.x = x;
        temp.y = y;
        hl.push_back(temp);
        x += m;
        x1 -= m;
        temp.x = x1;
        temp.y = y1;
        tl.push_back(temp);
        d += 2 * incre;
      }
    }
    y += 1;
    y1 -= 1;
  }
  std::reverse(hl.begin(), hl.end());
  for (int i = 1; i < tl.size(); ++i)
  {
    lineseg.push_back(tl.at(i));
  }
  for (int i = 0; i < hl.size(); ++i)
  {
    lineseg.push_back(hl.at(i));
  }

//   std::cout<<"------------------------"<<std::endl;
#if LINEDEBUG
  //   Mat image = Mat(height, width, CV_8UC3, Scalar::all(255));
  Mat image = Mat(height, width, CV_8UC3, Scalar(100, 200, 100));
  //   std::cout<<image.rows<<"; "<<image.cols<<std::endl;
  for (int i = 0; i < lineseg.size(); ++i)
  {
    //     image.at<unsigned char>(height - lineseg.at(i).y, 3 * lineseg.at(i).x) = 0;
    //     image.at<unsigned char>(height - lineseg.at(i).y, 3 * lineseg.at(i).x + 1) = 0;
    //     image.at<unsigned char>(height - lineseg.at(i).y, 3 * lineseg.at(i).x + 2) = 255;
    image.at<Vec3b>(height - lineseg.at(i).y, lineseg.at(i).x)[0] = 0;
    image.at<Vec3b>(height - lineseg.at(i).y, lineseg.at(i).x)[1] = 0;
    image.at<Vec3b>(height - lineseg.at(i).y, lineseg.at(i).x)[2] = 255;
  }
  try
  {
    namedWindow("generateline", CV_WINDOW_NORMAL);
    imshow("generateline", image);
    waitKey(1);
  }
  catch (cv::Exception &e)
  {
    std::cout << e.what() << std::endl;
    std::cerr << e.what() << std::endl;
  }
#endif
  return;
}
