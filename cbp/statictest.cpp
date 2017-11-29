
#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include <boost/shared_ptr.hpp>
#include <ctime>
#include "ivmap/ivmapmsglocpos.h"
#include "ivpathplanner/ivmsgpath.h"
#include "ivmap/ivmapmsgobj.h"
#include "ivsensorgps/ivsensorgps.h"
#include "ivpathplanner/path.h"
#include "sensor_msgs/Imu.h"

#include <boost/filesystem.hpp>
#include <cstdio>
//c++ lib
#include <sstream>
#include <iostream>
#include <fstream>
#include "visualize.h"
#include "functions.h"

#include "../../../../avoslib/geotool.h"
#include "../../../../avoslib/iabasemaptool.h"

#include "refpath.h"

#define DEBUG 1
#define DYNAMIC 1

using namespace std;

ivsensorgps::ivsensorgps fakegps;

void gpscallback(const ivsensorgps::ivsensorgps::ConstPtr msg)
{
    fakegps = *msg;
}

void obstacleinit(std::vector<sPointOfGCCS>& ops, std::vector<geometry_msgs::Point32>& dyn, refpath RefPath)
{
	//Initial obstacle positions
	ops.clear();
	dyn.clear();
	int c;
    float disoff;
    float velocity;
    float radian1, radian2;

	geometry_msgs::Point32 temp;
	sPointOfGCCS global_position;

    //Crossing mode
	c = 1000;
    disoff = 0.0;
    velocity = 0.0;
    radian1 = (RefPath.PathPointsRaw.at(c).angle + 90.0) * M_PI / 180.0;
    radian2 = (RefPath.PathPointsRaw.at(c).angle - 90.0) * M_PI / 180.0;
	global_position.xg = RefPath.PathPointsRaw.at(c).xg + disoff * cos(radian1);
	global_position.yg = RefPath.PathPointsRaw.at(c).yg + disoff * sin(radian1);
	global_position.angle = RefPath.PathPointsRaw.at(c).angle;
	ops.push_back(global_position);
    temp.x = velocity * cos(radian2);
    temp.y = velocity * sin(radian2);
    dyn.push_back(temp);

    //Skew crossing mode
    c = 1200;
    disoff = -5.0;
    velocity = -1.0;
    radian1 = (RefPath.PathPointsRaw.at(c).angle + 90.0) * M_PI / 180.0;
    radian2 = (RefPath.PathPointsRaw.at(c).angle - 45.0) * M_PI / 180.0;
	global_position.xg = RefPath.PathPointsRaw.at(c).xg + disoff * cos(radian1);
	global_position.yg = RefPath.PathPointsRaw.at(c).yg + disoff * sin(radian1);
	global_position.angle = RefPath.PathPointsRaw.at(c).angle;
	ops.push_back(global_position);
    temp.x = velocity * cos(radian2);
    temp.y = velocity * sin(radian2);
    dyn.push_back(temp);

    //Acc simulation mode
    c = 1400;
    disoff = 5.0;
    velocity = 1.8;
    radian1 = (RefPath.PathPointsRaw.at(c).angle + 90.0) * M_PI / 180.0;
    radian2 = (RefPath.PathPointsRaw.at(c).angle - 90.0) * M_PI / 180.0;
	global_position.xg = RefPath.PathPointsRaw.at(c).xg + disoff * cos(radian1);
	global_position.yg = RefPath.PathPointsRaw.at(c).yg + disoff * sin(radian1);
	global_position.angle = RefPath.PathPointsRaw.at(c).angle;
	ops.push_back(global_position);
    temp.x = velocity * cos(radian2);
    temp.y = velocity * sin(radian2);
    dyn.push_back(temp);

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "statictest");
	ros::NodeHandle nh;

	refpath RefPath;
	RefPath.load();

    int acc = 1400;
    int cc = 0;
    int count = 0;
    int hz = 0;
    int inc = 0;
    double dt = 0.0;
    float t = 0.0;

    sPointOfVCS local_position;
    sPointOfGCCS global_position;

    ros::Time pretime;
    ros::Time currentime;
    ros::Duration lst;


    ivmap::ivmapmsgobj fakeobj;
    ivmap::mapobject tempobs;

    std::vector<sPointOfGCCS> ops;
    std::vector<geometry_msgs::Point32> dyn;

    obstacleinit(ops, dyn, RefPath);

    sensor_msgs::Imu imu;
    imu.orientation_covariance = {0, 1, 2, 3, 4, 5, 6, 7, 8};

#if DEBUG
    ros::Publisher pub = nh.advertise<ivsensorgps::ivsensorgps>("ivsensorgps", 5);
#else
    ros::Subscriber sub = nh.subscribe("ivsensorgps", 10, gpscallback);
#endif

	ros::Publisher pub_ = nh.advertise<ivmap::ivmapmsgobj>("ivmapobj", 5);

	ros::Rate loop(20.0);

    pretime = ros::Time::now();
	while (ros::ok())
	{
		ros::spinOnce();
		clock_t start, finish;
		double duration;
		start = clock();

#if DEBUG
		#if DYNAMIC
		fakegps.lon = RefPath.PathPointsRaw.at(cc).lon - 0.000025;
		fakegps.lat = RefPath.PathPointsRaw.at(cc).lat - 0.000025;
		fakegps.heading = RefPath.PathPointsRaw.at(cc).heading;
		fakegps.velocity = RefPath.PathPointsRaw.at(cc).velocity;
        fakegps.status = 4;
        fakegps.isvalid = 1;
		#else
		int ind = 920;
		fakegps.lon = RefPath.PathPointsRaw.at(ind).lon;
		fakegps.lat = RefPath.PathPointsRaw.at(ind).lat - 0.00005;
		fakegps.heading = RefPath.PathPointsRaw.at(ind).heading;
		fakegps.velocity = RefPath.PathPointsRaw.at(ind).velocity;
        fakegps.status = 4;
        fakegps.isvalid = 1;
		#endif
		pub.publish(fakegps);
#endif

	/*
    * The obstacle simulation mode!!!
    */
    fakeobj.obj.clear();

    currentime = ros::Time::now();
    lst = currentime - pretime;
    pretime = currentime;
    dt = lst.toSec();
    for (int i = 0; i < ops.size(); ++i)
    {
    	global_position = ops.at(i);
    	local_position = RefPath.gt.GCCS2VCS(fakegps, global_position);
    	tempobs.x = local_position.x;
    	tempobs.y = local_position.y;
    	tempobs.xg = global_position.xg;
    	tempobs.yg = global_position.yg;
    	tempobs.width = tempobs.height = 0.8;
    	tempobs.id = i + ops.size() * count;
    	fakeobj.obj.push_back(tempobs);
    	ops.at(i).xg += dyn.at(i).x * dt;
    	ops.at(i).yg += dyn.at(i).y * dt;
    }
    global_position.xg = RefPath.PathPointsRaw.at(acc + inc).xg;
    global_position.yg = RefPath.PathPointsRaw.at(acc + inc).yg;
    global_position.angle = RefPath.PathPointsRaw.at(acc + inc).angle;
    local_position = RefPath.gt.GCCS2VCS(fakegps, global_position);
    tempobs.x = local_position.x;
    tempobs.y = local_position.y;
    tempobs.xg = global_position.xg;
    tempobs.yg = global_position.yg;
    tempobs.width = tempobs.height = 0.8;
    tempobs.id = 200;
    // fakeobj.obj.push_back(tempobs);

    // pub_.publish(fakeobj);

    // This is for the car position updation
    if (hz >= 0)
    {
    	cc++;
        inc++;
    	hz = 0;
    }
    hz++;
    if (cc == RefPath.PathPointsRaw.size())
    {
    	std::cout << "Start new loop!!!\n";
    	cc = 0;
    }
    if (inc + acc == RefPath.PathPointsRaw.size())
    {
        std::cout << "Start new acc model loop!!!\n";
        inc = 0;
    }

    t += dt;
    if (t > 12.0)
    {
    	obstacleinit(ops, dyn, RefPath);
    	count++;
    	t = 0.0;
    }

    finish = clock();
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    loop.sleep();
}
return 0;
}
