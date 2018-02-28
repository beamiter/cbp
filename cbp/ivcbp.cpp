#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include <boost/shared_ptr.hpp>
#include <ctime>
#include <opencv2//opencv.hpp>

#include "dwaplanner.h"
#include "dubins.h"
#include "sehs.h"

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ivcbp");
    ros::NodeHandle nh;
    
    // std::unique_ptr<dwaplanner> dwa(new dwaplanner(nh));
    std::unique_ptr<sehs> heuristic(new sehs);

    ros::Rate loop(1);
    while (ros::ok())
    {
        ros::spinOnce();
        clock_t start, finish;
        double duration;
        start = clock();
        // dwa->testModule();
        heuristic->testModule();
        finish = clock();
        duration = (double)(finish - start) / CLOCKS_PER_SEC;
        std::cout<<"The algorithm costs: "<<duration * 1000.0<<" ms"<<std::endl;
        loop.sleep();
    }

    return 0;
}
