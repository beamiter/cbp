#include "dwaplanner.h"

dwaplanner::dwaplanner(ros::NodeHandle nh)
: stepsize(0.15)
, steplength(0.6)
, controlCycle(0.9)
, yawtolerance(9.0)
, disttolerance(0.6)
, REVERSE(false)
, reachedGoal(false)
, gicsCellSize(0.05)
{
    visdwa_marker = std::unique_ptr<visualize_marker>(new visualize_marker("/dwa"));

    for (int i = 0; i <= 20; ++i)
    {
        samplenum.emplace(i, i);
    }

    initialstatus.x = 0.0;
    initialstatus.y = 0.0;
    initialstatus.heading = functions::deg2Rad(0.0);
    initialstatus.linear_velocity = 0.0;
    initialstatus.angular_velocity = 0.0;

    goalstatus.x = 12.0;
    goalstatus.y = 12.0;
    goalstatus.heading = functions::deg2Rad(0.0);
    goalstatus.angular_velocity = functions::deg2Rad(0.0);
    goalstatus.linear_velocity = 0.0;

    //This is the recommended centrifugal_acceleration
    dyn.centrifugal_acceleration = 1.5;
    dyn.max_linear_velocity = 2.0;
    dyn.max_linear_acceleration = 1.0;
    dyn.min_turn_radius = functions::radius;

    nh.param ( "basemap", mapPath, mapPath );
    nh.param ( "gicscellsize", gicsCellSize, gicsCellSize );
    nh.param ( "iabasemaptllon", iabasemapTL.lon, iabasemapTL.lon );
    nh.param ( "iabasemaptllat", iabasemapTL.lat, iabasemapTL.lat );
    nh.param ( "iabasemapbrlon", iabasemapBR.lon, iabasemapBR.lon );
    nh.param ( "iabasemapbrlat", iabasemapBR.lat, iabasemapBR.lat );

    mapTool.initParam ( mapPath, gicsCellSize, iabasemapTL, iabasemapBR);

    // std::cout << __FILE__ << ": " << __FUNCTION__<<std::endl;

}

dwaplanner::dwaplanner()
{
}

dwaplanner::~dwaplanner() {}

int dwaplanner::collisiondetection(std::vector<geometry_msgs::Point32> ego, sorted_tree &obspts, sVehicleElem vehiclemodel)
{
    geometry_msgs::Point32 p, q;
    geometry_msgs::Polygon poly, littlepoly;
    if (obspts.size() == 0)
    {
        return 0;
    }
    for (auto tp : ego)
    {
        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);

        sPointOfVCS modelPt;
        modelPt.x = tp.x;
        modelPt.y = tp.y;

        #if USE_FREE_SPACE
        if (false == mapTool.isInFreeSpace (egoPos, modelPt))
        {
            return 1;
        }

        functions::carmodel(tp, littlepoly, 0.5, 0.3, 0.3);
        for (auto a : littlepoly.points)
        {
            modelPt.x = a.x;
            modelPt.y = a.y;
            if (false == mapTool.isInFreeSpace (egoPos, modelPt))
            {
                return 1;
            }
        }

        #endif


        auto ha = obspts.lower_bound(p.x);
        if (!obspts.count(ha->first) || ha->first > q.x)
        {
            continue;
        }
        else
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int dwaplanner::collisiondetection(ivpathplanner::path ego, sorted_tree &obspts, sVehicleElem vehiclemodel)
{
    geometry_msgs::Point32 p, q, tp;
    geometry_msgs::Polygon poly;

    if (obspts.size() == 0)
    {
        return 0;
    }

    for (int i = 0; i < ego.points.size(); ++i)
    {
        tp.x = ego.points.at(i).x;
        tp.y = ego.points.at(i).y;
        tp.z = ego.points.at(i).angle * M_PI / 180.0;
        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);

        // sPointOfVCS modelPt;
        // modelPt.x = tp.x;
        // modelPt.y = tp.y;
        // if (false == mapTool.isInFreeSpace (egoPos, modelPt))
        // {
        //     return 1;
        // }

        // sPointOfVCS modelPt;
        // for (auto a : poly.points)
        // {
        //     modelPt.x = a.x;
        //     modelPt.y = a.y;
        //     if (false == mapTool.isInFreeSpace (egoPos, modelPt))
        //     {
        //         return 1;
        //     }
        // }

        auto ha = obspts.lower_bound(p.x);
        if (!obspts.count(ha->first) || ha->first > q.x)
        {
            continue;
        }
        else
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int dwaplanner::collisiondetection(std::vector<sVehicleStatus> ego, sorted_tree &obspts,
                                   sVehicleStatus goalvs, bool& reachedGoal, sVehicleElem vehiclemodel)
{
    geometry_msgs::Point32 p, q, tp;
    geometry_msgs::Polygon poly, littlepoly;
    if (obspts.size() == 0)
    {
        return 0;
    }
    if (ego.empty())
    {
        return 0;
    }
    for (auto tt : ego)
    {
        reachedGoal = reachgoal(tt, goalvs);
        if (true == reachedGoal)
        {
            return 0;
        }
        tp.x = tt.x;
        tp.y = tt.y;
        tp.z = tt.heading;

        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);

        sPointOfVCS modelPt;
        modelPt.x = tp.x;
        modelPt.y = tp.y;

        #if USE_FREE_SPACE
        if (false == mapTool.isInFreeSpace(egoPos, modelPt))
        {
            return 1;
        }

        functions::carmodel(tp, littlepoly, 0.5, 0.3, 0.3);
        for (auto a : littlepoly.points)
        {
            modelPt.x = a.x;
            modelPt.y = a.y;
            if (false == mapTool.isInFreeSpace(egoPos, modelPt))
            {
                return 1;
            }
        }
        #endif

        auto ha = obspts.lower_bound(p.x);

        if (!obspts.count(ha->first) || ha->first > q.x)
        {
            continue;
        }
        else
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }

    /* For the emergency braking! */
    /**********************************************/
    
    sVehicleStatus tempvs = ego.back();
    /*So as to avoid the near obstacles!!!*/
    if (std::hypot(tempvs.x, tempvs.y) < 2.5)
    {
        return 0;
    }
    
    float braking_distance = std::pow(tempvs.linear_velocity, 2) * 1.0;
    if (braking_distance <= steplength)
    {
        braking_distance = steplength;
    }
    float chip_step = 0.2;
    int diffnum = std::ceil(static_cast<int>(braking_distance / chip_step));
    float timestep = (chip_step / tempvs.linear_velocity);

    for (int a = 0; a < diffnum; ++a)
    {
        // if (fabs(tempvs.angular_velocity) < 1e-2)
        if (1)
        {
            tempvs.x += tempvs.linear_velocity * timestep * cos(tempvs.heading);
            tempvs.y += tempvs.linear_velocity * timestep * sin(tempvs.heading);
        }
        else
        {
            //@brief refer to the <<Probabilistic robotics>>, table 5.3, 5.9
            tempvs.heading += tempvs.angular_velocity * timestep;
            tempvs.x += (tempvs.linear_velocity / tempvs.angular_velocity) *
            (sin(tempvs.heading + tempvs.angular_velocity * timestep) - sin(tempvs.heading));
            tempvs.y += (tempvs.linear_velocity / tempvs.angular_velocity) *
            (-cos(tempvs.heading + tempvs.angular_velocity * timestep) + cos(tempvs.heading));
        }

        tp.x = tempvs.x;
        tp.y = tempvs.y;
        tp.z = tempvs.heading;

        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);

        auto ha = obspts.lower_bound(p.x);

        if (!obspts.count(ha->first) || ha->first > q.x)
        {
            continue;
        }
        else
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }

   /**********************************************/

    return 0;
}


std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>>
dwaplanner::GenerateTrajectory(sVehicleStatus currentvs, sKinematic sk)
{
    trajectorys.clear();

    sVehicleStatus subsequent;

    // linear_velocity, angular_velocity, and G cost
    std::vector<std::tuple<float, float, float>> speedarray;
    std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>> rtn;

    int angular_num = 4;
    int speed_num = 1;
    float angular_sample = 0.0;
    float speed_sample = 0.0;
    float acceleration_coefficient = 0.4;
    float linear_velocity, angular_velocity;

    subsequent = currentvs;
    
    // linear_velocity = std::sqrt(std::pow(currentvs.linear_velocity, 2) + 2 * acceleration_coefficient * sk.max_linear_acceleration * steplength);

    /*    the reversing mode sampling     */
    if (REVERSE)
    {
        std::cout << "Reversing mode!!!" << std::endl;
    }
    /*    the normal mode sampling    */
    else
    {
        speed_num = samplenum[1];
        speed_sample = acceleration_coefficient * sk.max_linear_acceleration * controlCycle / speed_num;
        for (int i = speed_num; i >= -speed_num; i += -1)
        {
            linear_velocity = currentvs.linear_velocity + speed_sample * i;
            if (linear_velocity < (initialstatus.linear_velocity - 1e-2) || linear_velocity > (goalstatus.linear_velocity + 1e-2))
            {
                continue;
            }
            // cout<<"linear_velocity: "<<linear_velocity<<endl;
            sk.max_angular_velocity = fabs(linear_velocity) / sk.min_turn_radius;
            if (fabs(linear_velocity * sk.max_angular_velocity) > (sk.centrifugal_acceleration + 1e-3))
            {
                sk.max_angular_velocity = sk.centrifugal_acceleration / linear_velocity;
            }
            if (linear_velocity <= 0.5)
            {
                angular_num = samplenum[2];
            }
            else if (linear_velocity <= 2.0)
            {
                angular_num = samplenum[3];
            }
            else
            {
                angular_num = samplenum[4];
            }
    
            angular_sample = sk.max_angular_velocity / angular_num;
            for (int j = -angular_num; j <= angular_num; ++j)
            {
                angular_velocity = currentvs.angular_velocity + angular_sample * j;
                if (fabs(angular_velocity) < 1e-2)
                {
                    angular_velocity = 0.0;
                }
                if (fabs(angular_velocity) > (sk.max_angular_velocity + 1e-3))
                {
                    continue;
                }
                subsequent.linear_velocity = linear_velocity;
                subsequent.angular_velocity = angular_velocity;
                speedarray.emplace_back(
                std::forward_as_tuple(
                linear_velocity, angular_velocity, gValUpdate(currentvs, subsequent)
                ));
            }
        }
    }

    /**The trajectory samping process**/
    for (auto spd : speedarray)
    {
        std::vector<sVehicleStatus> path;
        sVehicleStatus tempvs = currentvs;
        path.push_back(tempvs);

        tempvs.linear_velocity = std::get<0>(spd);
        tempvs.angular_velocity = std::get<1>(spd);

        int diffnum = static_cast<int>(tempvs.linear_velocity * controlCycle / stepsize);
        float timestep = stepsize / tempvs.linear_velocity;

        for (int a = 0; a < diffnum; ++a)
        {
            // Vehicle state sampling
            /** A little bug causes great pain !!! **/
            if (fabs(tempvs.angular_velocity) < 1e-2)
            {
                tempvs.x += tempvs.linear_velocity * timestep * cos(tempvs.heading);
                tempvs.y += tempvs.linear_velocity * timestep * sin(tempvs.heading);
            }
            else
            {
                //@brief refer to the <<Probabilistic robotics>>, table 5.3, 5.9
                tempvs.heading += tempvs.angular_velocity * timestep;
                tempvs.x += (tempvs.linear_velocity / tempvs.angular_velocity) *
                (sin(tempvs.heading + tempvs.angular_velocity * timestep) - sin(tempvs.heading));
                tempvs.y += (tempvs.linear_velocity / tempvs.angular_velocity) *
                (-cos(tempvs.heading + tempvs.angular_velocity * timestep) + cos(tempvs.heading));
            }
            path.push_back(tempvs);
        }
        trajectorys.push_back(path);
        rtn.emplace_back(std::forward_as_tuple(path.back(), path, std::get<2>(spd)));
    }

    #if 0
    std::vector<std::vector<geometry_msgs::Point32>> multipath;
    for (int i = 0; i < trajectorys.size(); ++i)
    {
        std::vector<geometry_msgs::Point32> tempv;
        for (int j = 0; j < trajectorys.at(i).size(); ++j)
        {
            geometry_msgs::Point32 temp;
            temp.x = trajectorys[i][j].x;
            temp.y = trajectorys[i][j].y;
            temp.z = trajectorys[i][j].heading;
            tempv.push_back(temp);
        }
        multipath.push_back(tempv);
    }
    visdwa_marker->publishmultipath(multipath, 0);
    #endif

    // exit(0);

    return rtn;
}

float dwaplanner::gValUpdate(sVehicleStatus cur, sVehicleStatus next)
{
    float gval = 0.0;
    // float distance = steplength;
    // float angular_diff = cur.angular_velocity - next.angular_velocity;
    // if (fabs(angular_diff) < 1e-1)
    // {
    //     gval = distance;
    // }
    // else
    // {
    //     gval = distance * (functions::penaltyTurning + 0.1 * distance / next.linear_velocity * angular_diff / (2 * M_PI));
    // }
    return gval;
}

//This H-value updating function bind the dubins_curve and the reeds_shepp as hValUpdate!!!
float dwaplanner::hValUpdate(sVehicleStatus cur, sVehicleStatus goal)
{
    float hvalue = 0.0;
    float circle_radius = functions::radius;
    geometry_msgs::Point32 p1, p2;
    std::vector<geometry_msgs::Point32> arr;
    p1.x = cur.x;
    p1.y = cur.y;
    p1.z = cur.heading;
    p2.x = goal.x;
    p2.y = goal.y;
    p2.z = goal.heading;
    double q0[3] = {cur.x, cur.y, cur.heading};
    double q1[3] = {goal.x, goal.y, goal.heading};
    auto cb = [](double q[3], double x, void *user_data) -> int {
        return 1;
    };

    if (true == REVERSE)
    {
        ReedsSheppStateSpace reeds_shepp(circle_radius);
        hvalue = reeds_shepp.distance(q0, q1);
    }
    else
    {
        dubins_curve::DubinsPath dubins_path;
        dubins_curve::dubins_init(q0, q1, circle_radius, &dubins_path);
        dubins_curve::dubins_path_sample_many(&dubins_path, cb, 0.2, nullptr);
        hvalue = dubins_curve::dubins_path_length(&dubins_path);     
    }

    return hvalue;
}

//The kernel function of this god-like algorithm
sVehicleStatus dwaplanner::dynamicWindowApproach(sVehicleStatus start, sVehicleStatus goal, unordered_t<sVehicleStatus> &came_from,
unordered_t<float> &cost_so_far, unordered_t<std::vector<sVehicleStatus>> &pathtable, sorted_tree sortedobjs)
    {
        // cout<<__LINE__<<endl;
        reachedGoal = false;
        int remark = 0;
        dyn.max_linear_velocity = goal.linear_velocity;

        sVehicleStatus current;
        functions::PriorityNode<sVehicleStatus, float> frontier;
        frontier.elements.emplace(start, 0.0);
        came_from[start] = start;
        cost_so_far[start] = 0.0;

        while (!frontier.elements.empty())
        {
            current = frontier.elements.top().first;
            frontier.elements.pop();
            //This algorithm is like holy shit, and perfectly wonderful!!!
            sVehicleElem vehiclemodel;
            vehiclemodel.frontoverhang = 1.5;
            vehiclemodel.backoverhang = 0.5;
            vehiclemodel.halfwheeltrack = 0.90;
            int collisionflag = collisiondetection(pathtable[current], sortedobjs, goal, reachedGoal, vehiclemodel);
            if (true == collisionflag)
            {
                continue;
            }
            if (remark >= 200 || true == reachedGoal)
            {
                break;
            }
            remark++;
            for (auto next : GenerateTrajectory(current, dyn))
            {
                //Compare the algorithm efficiency!!!
                auto nextpath = std::get<1>(next);
                sVehicleStatus nextvs = std::get<0>(next);
                float new_cost = cost_so_far[current] + std::get<2>(next);

                // Only if the successor is not in the close table
                if (!came_from.count(nextvs))
                {
                    // If the successor is not in the open table, If the successor is in the open table
                    if (!cost_so_far.count(nextvs) || (new_cost < cost_so_far[nextvs]))
                    {
                        float temph = hValUpdate(nextvs, goal);
                        float priority = new_cost + temph;
                        frontier.elements.emplace(nextvs, priority);
                        cost_so_far[nextvs] = new_cost;
                        came_from[nextvs] = current;
                        pathtable[nextvs] = nextpath;
                    }
                }
            }
        }
        //std::cout << "The loop sequences >>>>>>>>>>>> " << remark << std::endl;
        return current;
    }

void dwaplanner::obstaclemodify(ivmap::ivmapmsgstcell objs, sorted_tree &nearestobs, sorted_tree &rawobs, sorted_tree &dilatedobs)
{
    geometry_msgs::Point32 tt, tp;
    tp.x = -0.1;
    tp.y = 0.0;
    tp.z = 0.0;
    geometry_msgs::Polygon poly;
    functions::carmodel(tp, poly, 0.1, 1.6, 0.8);

    for (auto b : objs.cell)
    {
        tt.x = b.xc * 0.1;
        tt.y = b.yc * 0.1;
        float dis = std::hypot(tt.x, tt.y);
        if (dis > 12.0)
        {
            continue;
        }
        else
        {
            if (dis < 4.0)
            {
                if (!nearestobs[tt.x].count(tt))
                {
                    nearestobs[tt.x].insert(tt);
                }
            }

            if (true == functions::IsInsideFootprint(tt, poly))
            {
                continue;
            }

            if (!rawobs[tt.x].count(tt))
            {
                rawobs[tt.x].insert(tt);
            }
        }
    }

    return;
}

void dwaplanner::obstaclemodify(ivpredict::ivmsgpredict objs, sorted_tree &nearestobs,
                                sorted_tree &rawobs, sorted_tree &dilatedobs, float carspeed)
{
    geometry_msgs::Point32 tt, tp;
    tp.x = -0.1;
    tp.y = 0.0;
    tp.z = 0.0;
    geometry_msgs::Polygon poly;
    functions::carmodel(tp, poly, 0.1, 1.6, 0.8);

    for (auto a : objs.objects)
    {
        for (auto b : a.cell)
        {
            tt.x = b.xc * 0.1;
            tt.y = b.yc * 0.1;
            float dis = std::hypot(tt.x, tt.y);
            if (dis > 12.0)
            {
                continue;
            }
            else
            {
                if (dis < 4.0)
                {
                    if (!nearestobs[tt.x].count(tt))
                    {
                        nearestobs[tt.x].insert(tt);
                    }
                }

                #if STOP_AND_WAIT
                if (fabs(carspeed) > 0.10 || fabs(a.v_abs) > 0.10)
                {
                    continue;
                }
                #else
                if (fabs(a.v_abs) > 0.2)
                {
                    continue;
                }
                #endif

                if (true == functions::IsInsideFootprint(tt, poly))
                {
                    continue;
                }

                if (!rawobs[tt.x].count(tt))
                {
                    rawobs[tt.x].insert(tt);
                }

                if (a.width > 1.5)
                {
                    geometry_msgs::Point32 temp;

                    temp = tt;
                    temp.x -= 0.1;
                    if (!rawobs[temp.x].count(temp))
                    {
                        rawobs[temp.x].insert(temp);
                    } 

                    temp = tt;
                    temp.x += 0.2;
                    if (!rawobs[temp.x].count(temp))
                    {
                        rawobs[temp.x].insert(temp);
                    }  

                }
            }
        }
    }
    return;
}


/**
* @brief
* @param
*/
int dwaplanner::reachgoal(sVehicleStatus currentvs, sVehicleStatus goalvs)
{
    float dis, theta;
    dis = std::hypot(currentvs.x - goalvs.x, currentvs.y - goalvs.y);
    theta = functions::rad2Deg(fabs(currentvs.heading - goalvs.heading));
    if (dis <= disttolerance && theta <= yawtolerance)
    {
        return 1;
    }
    return 0;
}

std::vector<sVehicleStatus>
dwaplanner::reconstructPath(sVehicleStatus start, sVehicleStatus goal,
unordered_t<sVehicleStatus> &came_from, sVehicleStatus last)
{
    std::vector<sVehicleStatus> path;
    sVehicleStatus current = last;
    path.push_back(current);
    while (!(current == start))
    {
        current = came_from[current];
        path.push_back(current);
    }
    //   path.push_back(start);
    std::reverse(path.begin(), path.end());
    //   path.push_back(goal);
    return path;
}

//Interface to collision avoiding module of path planning
ivpathplanner::path dwaplanner::pathinterface(ivmap::ivmapmsglocpos ivlocpos,
                    ivpathplanner::path current, ivpathplanner::path rawpath,
                    sorted_tree sortedobjs)
{
    if (sortedobjs.empty())
    {
        return current;
    }
    egoPos.xg = ivlocpos.xg;
    egoPos.yg = ivlocpos.yg;
    egoPos.angle = ivlocpos.angle;

    std::vector<int> roadSegId;
    ivpathplanner::path ivpath;

    float dis = 0.0;
    float totaldis = 0.0;
    if (rawpath.points.size() > 10)
    {
        for (int i = 0; i < rawpath.points.size() - 2; ++i)
        {
            float tempdis = 0.0;
            tempdis = std::hypot(rawpath.points.at(i + 1).x - rawpath.points.at(i).x, rawpath.points.at(i + 1).y - rawpath.points.at(i).y);
            totaldis += tempdis;
            if (totaldis < 12.0)
            {
                continue;
            }
            dis += tempdis;
            if (dis > 1.0)
            {
                roadSegId.push_back(i);
                dis = 0.0;
            }
        }
        initialstatus.x = -1.2;
        initialstatus.y = 0.0;
        initialstatus.heading = 0.0;
        initialstatus.linear_velocity = 0.4;
        initialstatus.angular_velocity = 0.0;
    }

    const int chip_num = 4;
    int chip_size = roadSegId.size() / (chip_num - 1);
    if (chip_size <= 1)
    {
        chip_size = 2;
    }

    std::vector<int> point_id;
    int chip_pos = 0;
    while(chip_pos < roadSegId.size())
    {
        point_id.push_back(roadSegId.at(chip_pos));
        chip_pos += chip_size;
    }

    for (auto cc : point_id)
    {
        goalstatus.x = rawpath.points.at(cc).x;
        goalstatus.y = rawpath.points.at(cc).y;
        goalstatus.heading = rawpath.points.at(cc).angle * M_PI / 180.0;

        goalstatus.linear_velocity = 2.0;
        goalstatus.angular_velocity = 0.0;

        std::vector<sVehicleStatus> rtnpath;
        sVehicleStatus lst;
        unordered_t<sVehicleStatus> came_from;
        unordered_t<float> cost_so_far;
        unordered_t<std::vector<sVehicleStatus> > pathtable;
        lst = dynamicWindowApproach(initialstatus, goalstatus, came_from, cost_so_far, pathtable, sortedobjs);
        rtnpath = reconstructPath(initialstatus, goalstatus, came_from, lst);

        if (true == reachedGoal)
        {
            for (int i = 0; i < rtnpath.size(); ++i)
            {
                for (auto val : pathtable[rtnpath.at(i)])
                {
                    ivpathplanner::pathpoint temp;
                    temp.x = val.x;
                    temp.y = val.y;
                    temp.angle = val.heading * 180.0 / M_PI;
                    temp.velocity = val.linear_velocity;
                    temp.velocity = -88.0;
                    ivpath.points.push_back(temp);
                }
            }
            return ivpath;
        }
        else
        {
	        tf::Quaternion tfquater;
            geometry_msgs::Point32 st, gt, tmp;
            geometry_msgs::Pose start_pose, end_pose;
            std::vector<geometry_msgs::Point32> link_curve;
            auto backpt = pathtable[rtnpath.back()].back();
            if (std::hypot(backpt.x - goalstatus.x, backpt.y - goalstatus.y) > 2.0)
            {
                continue;
            }
            st.x = backpt.x;
            st.y = backpt.y;
            st.z = backpt.heading;
            start_pose.position.x = backpt.x;
            start_pose.position.y = backpt.y;
            start_pose.orientation = tf::createQuaternionMsgFromYaw(backpt.heading);
            gt.x = goalstatus.x;
            gt.y = goalstatus.y;
            gt.z = goalstatus.heading;
            end_pose.position.x = goalstatus.x;
            end_pose.position.y = goalstatus.y;
            end_pose.orientation = tf::createQuaternionMsgFromYaw(goalstatus.heading);

            // functions::arcpath(st, gt, link_curve);
            std::vector<geometry_msgs::Pose> raw_path;
            raw_path = generatecurve::generateHermiteCurveForROS(start_pose, end_pose, 2.0);
            for (auto a : raw_path)
            {
                tmp.x = a.position.x;
                tmp.y = a.position.y;
                tf::quaternionMsgToTF(a.orientation, tfquater);
                tmp.z = tf::getYaw(tfquater);
                link_curve.push_back(tmp);
            }

            sVehicleElem vehiclemodel;
            vehiclemodel.frontoverhang = 1.5;
            vehiclemodel.backoverhang = 0.5;
            vehiclemodel.halfwheeltrack = 0.9;
            bool collidflag = collisiondetection(link_curve, sortedobjs, vehiclemodel);
            if (false == collidflag)
            {
                for (int i = 0; i < rtnpath.size(); ++i)
                {
                    for (auto val : pathtable[rtnpath.at(i)])
                    {
                        ivpathplanner::pathpoint temp;
                        temp.x = val.x;
                        temp.y = val.y;
                        temp.angle = val.heading * 180.0 / M_PI;
                        temp.velocity = val.linear_velocity;
                        temp.velocity = -88.0;
                        ivpath.points.push_back(temp);
                    }
                }
                for (auto val : link_curve)
                {
                    ivpathplanner::pathpoint temp;
                    temp.x = val.x;
                    temp.y = val.y;
                    temp.angle = val.z * 180.0 / M_PI;
                    temp.velocity = -88.0;
                    ivpath.points.push_back(temp);
                }
                return ivpath;
            }
        }
    }
    // ROS_ERROR_STREAM("dwaplanner failed!!!");
    return ivpath;
}

void dwaplanner::testModule()
{
    geometry_msgs::Point32 tt;
    ivpathplanner::path updatepath;
    ivmap::ivmapmsglocpos ivlocpos;
    ivpathplanner::path LocalRefPath, LocalRefPathRaw;
    ivpredict::ivmsgpredict predictmsg;
    ivpredict::predictobj objmsg;
    ivpredict::mapobjcell objcell;

    std::vector< geometry_msgs::Point32 > road, obs;
    ivlocpos.xg = 0.0;
    ivlocpos.yg = 0.0;
    ivlocpos.angle = 0.0;
    for (float a = -1.2; a <= 30.0; a+= 0.1)
    {
        ivpathplanner::pathpoint point;
        point.x = a;
        point.y = 0.0;
        point.angle = 0.0;
        LocalRefPath.points.push_back(point);
    }
    LocalRefPathRaw = LocalRefPath;

    static float inc = 0.0;

#if 1

    for (float b = -1.5; b <= 1.5; b += 0.5)
    {
        tt.x = 8.0;
        tt.y = b;
        obs.push_back(tt);

        objcell.xc = tt.x * 10;
        objcell.yc = tt.y * 10;
        objmsg.cell.push_back(objcell);


        tt.x = 9.0;
        tt.y = b + 0.5;
        // obs.push_back(tt);

        objcell.xc = tt.x * 10;
        objcell.yc = tt.y * 10;
        // objmsg.cell.push_back(objcell);
    }

    tt.x = 4.0;
    tt.y = 1.0;
    obs.push_back(tt);

    objcell.xc = tt.x * 10;
    objcell.yc = tt.y * 10;
    objmsg.cell.push_back(objcell);

    tt.x = 4.0;
    tt.y = 2.0;
    obs.push_back(tt);

    objcell.xc = tt.x * 10;
    objcell.yc = tt.y * 10;
    objmsg.cell.push_back(objcell);

    tt.x = 6.0;
    tt.y = -2.0;
    obs.push_back(tt);

    objcell.xc = tt.x * 10;
    objcell.yc = tt.y * 10;
    objmsg.cell.push_back(objcell);


#endif

    objmsg.width = 2.0;
    predictmsg.objects.push_back(objmsg);

    visdwa_marker->publishobstacle(obs, 1, 0.1);

    inc += 0.5;
    if (inc > 15.0)
    {
        inc = 0.0;
    }

    sorted_tree emergencyObsTree, rawObsTree, dilatedObsTree;
    obstaclemodify(predictmsg, emergencyObsTree, rawObsTree, dilatedObsTree, 0.0);

    sVehicleElem vehiclemodel;
    vehiclemodel.frontoverhang = 1.5;
    vehiclemodel.backoverhang = 0.5;
    vehiclemodel.halfwheeltrack = 0.9;
    collisiondetection(LocalRefPath, rawObsTree, vehiclemodel);

    updatepath = pathinterface(ivlocpos, LocalRefPath, LocalRefPathRaw, rawObsTree);

    // ivpathplanner::path rawpath = updatepath;
    // float len = 0.0;
    // auto iter = updatepath.points.begin();
    // for (int i = 1; i < updatepath.points.size(); ++i)
    // {
    //     len += std::hypot(-iter->x + rawpath.points.at(i).x, -iter->y + rawpath.points.at(i).y);
    //     if (len <= 0.8)
    //     {
    //         iter = updatepath.points.erase(iter);
    //     }
    //     else
    //     {
    //         iter ++;
    //     }
    // }
    for (auto c : updatepath.points)
    {
        tt.x = c.x;
        tt.y = c.y;
        road.push_back(tt);
    }
    visdwa_marker->publishobstacle(road, 2, 0.1);

    // road.clear();
    // for (auto c : LocalRefPath.points)
    // {
    //     tt.x = c.x;
    //     tt.y = c.y;
    //     road.push_back(tt);
    // }
    // visdwa_marker->publishobstacle(road, 3, 0.1);

}

