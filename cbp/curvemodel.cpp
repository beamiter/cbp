#include "curvemodel.h"

std::vector<geometry_msgs::Point32> dubinspatharray;
std::vector<geometry_msgs::Point32> reedspatharray;

int ReedsSheppPathCallback(double q[3], void *user_data)
{
    geometry_msgs::Point32 temp;
    temp.x = q[0];
    temp.y = q[1];
    temp.z = q[2];
    reedspatharray.push_back(temp);
    return 0;
}

int DubinsPathCallback(double q[3], double x, void *user_data)
{
    geometry_msgs::Point32 temp;
    temp.x = q[0];
    temp.y = q[1];
    temp.z = q[2];
    dubinspatharray.push_back(temp);
    return 0;
}

curvemodel::curvemodel()
    :radius(2)
    ,count(2)
{
    //The C macro is marvelous
    dirname = __FILE__;
    boost::filesystem::path pathname(dirname);
    pathname.remove_filename();
    dirname = pathname.string();

    visdubins_path = std::unique_ptr<visualize_path>(new visualize_path("/dubins"));
    visreeds_pose = std::unique_ptr<visualize_pose>(new visualize_pose("/reeds"));
    visreeds_path = std::unique_ptr<visualize_path>(new visualize_path("/reeds"));
    visreeds_marker = std::unique_ptr<visualize_marker>(new visualize_marker("/reeds"));
    vislines = std::unique_ptr<visualize_polygon>(new visualize_polygon("/footprint"));
    memset(q0, 0, sizeof(q0));
    memset(q1, 0, sizeof(q1));
    q1[0] = 4;
    q1[1] = 4;
    q1[2] = 0;
    std::tuple<const char *, int, int, int> tp = make_tuple("yinjian", 24, 170, 75);
    std::string a;
    int b, c, d;
    std::tie(a, b, c, d) = tp;
    std::cout << std::get<0>(tp) << std::endl;
}

curvemodel::~curvemodel()
{
}

//Evaluate the path in the cluster!!!
void curvemodel::evaluatecurve(std::vector<ReedsSheppStateSpace::ReedsSheppPath> path, indexnode<int, float>::boostqueue &bq,
                               double goal[3], std::vector<geometry_msgs::Point32> goalarray)
{
    for (int i = 0; i < path.size(); ++i)
    {
        indexnode<int, float> num;
        ReedsSheppStateSpace::ReedsSheppPath temp = path.at(i);
        num.node.first = i;
        if (temp.length_[0] < 0.0)
        {
            temp.length_[0] = 1.5 * temp.length_[0];
        }
        for (int j = 0; j < 5; ++j)
        {
            num.node.second += (temp.length_[j] < 0.0) ? fabs(temp.length_[j] * 2.0) : temp.length_[j];
        }
        //The new goal nearest to the original goal comes first!!!
        num.node.second += sqrt((goal[0] - goalarray.at(i).x) * (goal[0] - goalarray.at(i).x) +
                                (goal[1] - goalarray.at(i).y) * (goal[1] - goalarray.at(i).y));
        bq.push(num);
    }
    return;
}

void curvemodel::mypolygon(Mat &img, geometry_msgs::Polygon poly)
{
    int lineType = 8;

    /** Create some points */
    Point rook_points[1][poly.points.size()];
    for (int i = 0; i < poly.points.size(); ++i)
    {
        rook_points[0][i] = Point(poly.points.at(i).y, poly.points.at(i).x);
        //     std::cout<<poly.points.at(i).y<<"; "<<poly.points.at(i).x<<std::endl;
    }

    const Point *ppt[1] = {rook_points[0]};
    int npt[] = {static_cast<int>(poly.points.size())};

    fillPoly(img,
             ppt,
             npt,
             1,
             Scalar(50, 100, 150),
             lineType);

    return;
}

void curvemodel::samplegoal(geometry_msgs::Point32 goal, vector<geometry_msgs::Point32> &rtn)
{
    float londiff = 1.0; // meter
    float latdiff = 1.0; // meter
    int lonseed[4] = {0, 1, 2, 3};
    int latseed[3] = {0, -1, 1};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            geometry_msgs::Point32 temp;
            temp.x = goal.x + latseed[j] * londiff;
            temp.y = goal.y - lonseed[i] * latdiff;
            temp.z = goal.z;
            rtn.push_back(temp);
        }
    }
    return;
}

void curvemodel::run()
{
    std::vector<std::vector<geometry_msgs::Point32>> samples;
    std::vector<std::vector<geometry_msgs::Polygon>> polygons;
    radius += 0.1;
    if (radius > 3.0)
        radius = 2.5;
    dubinspatharray.clear();
    reedspatharray.clear();
    std::vector<ReedsSheppStateSpace::ReedsSheppPath> reeds_path;
    dubins_curve::DubinsPath dubins_path;
    ReedsSheppStateSpace reeds_shepp(2.5);
    if (visreeds_pose->updatestate())
    {
//         geometry_msgs::PoseStamped temps;
        geometry_msgs::PoseStamped tempg;
//         visreeds_pose->setstart(temps);
//         q0[0] = temps.pose.position.x;
//         q0[1] = temps.pose.position.y;
//         q0[2] = tf::getYaw(temps.pose.orientation);
        visreeds_pose->setgoal(tempg);
        q1[0] = tempg.pose.position.x;
        q1[1] = tempg.pose.position.y;
        q1[2] = tf::getYaw(tempg.pose.orientation);
    }
    dubins_curve::dubins_init(q0, q1, 2.5, &dubins_path);
    dubins_curve::dubins_path_sample_many(&dubins_path, DubinsPathCallback, 0.1, nullptr);
    visdubins_path->publishpath(dubinspatharray);
    //     reeds_shepp.sample(q0, q1, 0.1, ReedsSheppPathCallback, nullptr);
    geometry_msgs::Point32 origin;
    std::vector<geometry_msgs::Point32> goals;
    std::vector<geometry_msgs::Point32> goalarray;
    origin.x = q1[0];
    origin.y = q1[1];
    origin.z = q1[2];
    goals.push_back(origin);
    //     samplegoal(origin, goals);
    for (int i = 0; i < goals.size(); ++i)
    {
        double qnew[3] = {};
        qnew[0] = goals.at(i).x;
        qnew[1] = goals.at(i).y;
        qnew[2] = goals.at(i).z;
        std::vector<ReedsSheppStateSpace::ReedsSheppPath> new_path = reeds_shepp.sample_many(q0, qnew, 0.1, samples,
                                                                                             polygons);
        for (int a = 0; a < new_path.size(); ++a)
        {
            goalarray.push_back(goals.at(i));
        }
        reeds_path.insert(reeds_path.end(), new_path.begin(), new_path.end());
    }
    //     std::cout<<"reeds_shepp sample number: "<<samples.size()<<"; polygons sample number: "<<polygons.size()<<std::endl;
    if (count >= samples.size())
    {
        count = 0;
    }
    visreeds_marker->publishmultipath(samples, 0);
    indexnode<int, float>::boostqueue tempqueue;
    evaluatecurve(reeds_path, tempqueue, q1, goalarray);
    indexnode<int, float> index = tempqueue.top();
    visreeds_path->publishpath(samples.at(index.node.first));
    vislines->publishmultipolygon(polygons, index.node.first);

    std::vector<geometry_msgs::Polygon> tt = polygons.at(index.node.first);
    Mat rook_image = Mat(200, 800, CV_8UC3, Scalar::all(255)); //Mat::zeros(200, 800, CV_8UC3);
    for (auto &t : tt)
    {
        for (int i = 0; i < t.points.size(); ++i)
        {
            t.points.at(i).x = 100 + (t.points.at(i).x - q0[0]) * 10;
            t.points.at(i).y = 600 - (t.points.at(i).y - q0[1]) * 10;
        }
    }
    for (auto l : tt)
    {
        mypolygon(rook_image, l);
    }
    //The C macro is marvelous
    std::string dirname = __FILE__;
    boost::filesystem::path pathname(dirname);
    pathname.remove_filename();
    dirname = pathname.string();
    Mat srcImage, grayImage, out_Canny;
    std::vector<std::vector<Point>> g_vContours;
    std::vector<Vec4i> g_vHierarchy;
    cvtColor(rook_image, grayImage, cv::COLOR_BGR2GRAY);
    //     blur(grayImage, grayImage, Size(3, 3));
    Canny(grayImage, out_Canny, 50, 500, 3);
    findContours(out_Canny, g_vContours, g_vHierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    //     ROS_ERROR_STREAM("The size of the g_vContours: "<<g_vContours.size());
    srcImage = Mat(200, 800, CV_8UC3, Scalar(255, 255, 255));

    //lambda function as the compare function for the multiset data structure!!!
    auto cmpy = [](const Point &lhs, const Point &rhs) -> bool {
        return lhs.y > rhs.y;
    };
    std::multiset<Point, decltype(cmpy)> pts(cmpy);

    for (auto elem : g_vContours.front())
    {
        srcImage.at<Vec3b>(elem.y, elem.x)[0] = 0;
        srcImage.at<Vec3b>(elem.y, elem.x)[1] = 0;
        srcImage.at<Vec3b>(elem.y, elem.x)[2] = 0;
        pts.emplace(elem.y, elem.x);
    }
    imwrite(dirname.append("/scripts/wtf.jpg"), srcImage);

    //lambda function as the compare function for the multiset data structure!!!
    auto cmpx = [](const Point &lhs, const Point &rhs) -> bool {
        return lhs.x > rhs.x;
    };

    std::vector<std::pair<Point, Point>> bdpt;

    while (!pts.empty())
    {
        //       std::cout<<(*pts.begin()).x<<"; "<<(*pts.begin()).y<<std::endl;
        int refy = (*pts.begin()).y;
        std::multiset<Point, decltype(cmpx)> boundary(cmpx);
        while (refy == (*pts.begin()).y)
        {
            boundary.emplace((*pts.begin()).y, (*pts.begin()).x);
            pts.erase(pts.begin());
        }
        std::pair<Point, Point> tpt;
        tpt.first = *boundary.begin();
        tpt.second = *boundary.rbegin();
        bdpt.push_back(tpt);
    }
    
    //Compare with the filled polygon coordinate to get the inner point of the contour!!!
//     for (auto ha : bdpt)
//     {
//               std::cout<< ha.first.x<<"; "<< ha.first.y <<"; "<<ha.second.x << "; "<<ha.second.y<<std::endl;
//     }

    count++;
}
