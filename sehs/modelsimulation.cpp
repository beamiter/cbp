/* @brief combining space exploration and heuristic search
 *in online motion planning for nonholonomic vehicles
 */

#include "sehs.h"

/* @brief: refers to the essya
 * Motion Planning in Complex Environments Using Closed-loop Prediction
 */

std::vector<sFreeCircle> sehs::LookAheadDistance(sKinodynamic current)
{
    std::vector<sFreeCircle> rtn;
    int heuristic_index = 0;
    std::tie(heuristic_index, std::ignore) = HeuristicIndex(heuristic_index, current, circle_path);
    float distance_threshold = 0.0;

    if (current.speed <= 2)
    {
        distance_threshold = 4.0;
    }
    else if (current.speed <= 4.0)
    {
        distance_threshold = 4 * current.speed - 4.0;
    }
    else
    {
        distance_threshold = 12.0;
    }

    for(int i = heuristic_index; i < circle_path.size(); ++i)
    {
        rtn.push_back(circle_path.at(i).first);
        if (std::hypot(current.x - circle_path.at(i).first.center.x, current.y - circle_path.at(i).first.center.y) >= distance_threshold)
        {
            break;
        }
    }
    return rtn;
}

float sehs::ForwardDrive(sKinodynamic current, std::vector<sFreeCircle> bubbles)
{
    assert(!bubbles.empty());
    sKinodynamic anchorpoint = current;
    anchorpoint.x = current.x + omega.forwardAnchor * cos(current.theta);
    anchorpoint.y = current.y + omega.forwardAnchor * sin(current.theta);
    auto backcircle = bubbles.back();
    float lenforward = std::hypot(anchorpoint.x - backcircle.center.x, anchorpoint.y - backcircle.center.y);
    float ygap = current.y - backcircle.center.y;
    float eta = current.theta + asin((ygap + omega.forwardAnchor * sin(current.theta)) / lenforward);
    float delta = -atan2((omega.wheelBase * sin(eta)) , (0.5 * lenforward + omega.forwardAnchor * cos(eta)));

    if (delta > dynamic_constraints.maxSteeringAngle)
    {
        delta = dynamic_constraints.maxSteeringAngle;
    }
    else if (delta < -dynamic_constraints.maxSteeringAngle)
    {
        delta = -dynamic_constraints.maxSteeringAngle;
    }
}

std::vector<std::tuple<sKinodynamic, std::vector<sKinodynamic>, float>>
sehs::ClosedLoopPrediction(sKinodynamic current)
{
    std::vector<std::tuple<sKinodynamic, std::vector<sKinodynamic>, float>> rtn;
    float characteristic_velocity = 20.0;
    float side_slip_g = 1.0 / (1.0 + std::pow(current.speed / characteristic_velocity, 2));
    // first-order lag with time constants for the steering rate and the acceleration respectively
    float delta_t = 0.15;
    float looprate = 0.9;

    std::vector<sFreeCircle> bubbles = LookAheadDistance(current);
    float delta_command = ForwardDrive(current, bubbles);
    auto current_circle = bubbles.front();
    auto back_circle = bubbles.back();

    // This will never overflow 
    float steering_speed = (delta_command - current.steeringAngle) / looprate;
    // if (steering_speed > dynamic_constraints.maxSteeringSpeed)
    // {
    //     steering_speed = dynamic_constraints.maxSteeringSpeed;
    // }
    // else if (steering_speed < -dynamic_constraints.maxSteeringSpeed)
    // {
    //     steering_speed = -dynamic_constraints.maxSteeringSpeed;
    // }
    float turning_radius = fabs(omega.wheelBase * tan(M_PI * 0.5 - delta_command));
    float shrink_ratio = back_circle.radius / current_circle.radius;

    float acceleration = 0.0;
    if (shrink_ratio >= 1.3)
    {
        acceleration = 0.4 * dynamic_constraints.maxAcceleration;
    }
    else if (shrink_ratio >= 1.0)
    {
        acceleration = 0.2 * dynamic_constraints.maxAcceleration;
    }
    else if (shrink_ratio >= 0.8)
    {
        acceleration = 0.2 * dynamic_constraints.maxDecceleration;
    }
    else
    {
        acceleration = 0.4 * dynamic_constraints.maxDecceleration;
    }

    float check_speed = current.speed + acceleration * looprate;
    if (check_speed > dynamic_constraints.maxSpeed || check_speed < dynamic_constraints.minSpeed ||
        std::pow(check_speed, 2) > turning_radius * (dynamic_constraints.centrifugalAcceleration + 1e-3))
    {
        acceleration = 0.0;
    }

    sKinodynamic tempvs = current;
    std::vector<sKinodynamic> path;
    path.push_back(tempvs);

    bool checkflag = false;
    if (std::hypot(current.x - goal.x, current.y - goal.y) < 3.0)
    {
        checkflag = true;
    }

    float integration_time = 0.0;
    while(integration_time <= looprate)
    {
        integration_time += delta_t;
        tempvs.x += tempvs.speed * cos(tempvs.theta) * cos(tempvs.steeringAngle) * delta_t;
        tempvs.y += tempvs.speed * sin(tempvs.theta) * cos(tempvs.steeringAngle) * delta_t;
        tempvs.theta += tempvs.speed * tan(tempvs.steeringAngle) / omega.wheelBase * delta_t;
        tempvs.speed += acceleration * delta_t;
        tempvs.steeringAngle += steering_speed * delta_t;
        path.push_back(tempvs);
        if (true == checkflag)
        {
            if (true == ReachedGoal(tempvs, goal))
            {
                if (!virtualgoal.count(tempvs))
                {
                    virtualgoal.insert(std::make_pair(tempvs, tempvs));
                }
                looprate = integration_time;
                break;
            }                    
        }
    }

    if (!path.empty())
    {
        rtn.emplace_back(std::forward_as_tuple(path.back(), path, looprate));
    }

    return rtn;
}

ivpathplanner::path sehs::planning(ivmap::ivmapmsglocpos ivlocpos, ivpathplanner::path current, 
ivpathplanner::path rawpath, sortedTree sortedobjs)
{
    if (sortedobjs.empty())
    {
        return current;
    }
    egoPos.xg = ivlocpos.xg;
    egoPos.yg = ivlocpos.yg;
    egoPos.angle = ivlocpos.angle;

    ivpathplanner::path ivpath;

    float totaldis = 0.0;
    bool get_goal_status = false;
    if (rawpath.points.size() > 10)
    {
        for (int i = 0; i < rawpath.points.size() - 3; ++i)
        {
            float tempdis = 0.0;
            tempdis = std::hypot(rawpath.points.at(i + 1).x - rawpath.points.at(i).x, rawpath.points.at(i + 1).y - rawpath.points.at(i).y);
            totaldis += tempdis;
            if (totaldis < 12.0)
            {
                continue;
            }
            else if (totaldis > 18.0)
            {
                goal.x = rawpath.points.at(i).x;
                goal.y = rawpath.points.at(i).y;
                goal.speed = 1.0;
                goal.theta = rawpath.points.at(i).angle * M_PI / 180.0;
                goal.steeringAngle = 0.0;
                break;
            }
            goal.x = rawpath.points.at(i).x;
            goal.y = rawpath.points.at(i).y;
            goal.speed = 1.0;
            goal.theta = rawpath.points.at(i).angle * M_PI / 180.0;
            goal.steeringAngle = 0.0;

            get_goal_status = true;
        }
        start.x = -1.2;
        start.y = 0.0;
        start.speed = 0.5;
        start.theta = 0.0;
        start.steeringAngle = 0.0;
    }

    if (true == get_goal_status)
    {
        bool ok = WavefrontExpansion(start, goal, 5);
        cout<<"WavefrontExpansion status: "<<ok<<endl;
        
        if (false == ok)
        {
            return current;
        }

        std::vector<sFreeCircle> bubbles;
        std::vector<geometry_msgs::Point32> road;
        road.clear();
        bubbles.clear();
        for (auto a : circle_path)
        {
            bubbles.push_back(a.first);
        }
        
        // road = functions::curvefitting(road);

        road.clear();
        // Optimizer opt;
        // opt.Solve(bubbles);
        // road = functions::curvefitting(opt.path);
        // visdwa_marker->publishobstacle(road, 4, 0.1);

        ivpathplanner::pathpoint temp;
        for (auto val : road)
        {
            temp.x = val.x;
            temp.y = val.y;
            temp.angle = val.z * 180.0 / M_PI;
            temp.velocity = -88.0;
            ivpath.points.push_back(temp);         
        }
        road.clear();
        for (auto a : circle_path)
        {
            geometry_msgs::Point32 obs;
            obs.x = a.first.center.x;
            obs.y = a.first.center.y;
            obs.z = a.first.radius;
            road.push_back(obs);
        }
        visdwa_marker->publishcircles(road);

        return ivpath;
    }
    else
    {
        return current;
    }
}

bool sehs::SnakeCrawl(sKinodynamic start, sKinodynamic goal, std::vector<sKinodynamic>& trajectory)
{
    KinodynamicMap<sKinodynamic> came_from;
    KinodynamicMap<std::vector<sKinodynamic>> pathtable; 
    std::stack<sKinodynamic> frontier;

    frontier.push(start);
    came_from[start] = start;
    virtualgoal.clear();

    int heuristic_index = 0;

    ros::Time timePre = ros::Time::now();
    while(!frontier.empty())
    {
        auto current = frontier.top();
        frontier.pop();

        bool fake_val = false;
        // if ((ros::Time::now() - timePre).toSec() > 0.040 || heuristic_index >= 4)
        if ((ros::Time::now() - timePre).toSec() > 0.080)
        {
            ROS_ERROR_STREAM("Snake Crawling failed!!!");
            fake_val = true;
            // return false;
        }

        if (virtualgoal.count(current) || true == fake_val)
        {
            ROS_ERROR_STREAM("########");
            trajectory.clear();
            came_from[goal] = current;
            // cout<<"came_from.size(): "<<came_from.size()<<endl;
            sKinodynamic reversevs = goal;
            waypoint.clear();
            waypoint.push_back(reversevs);
            while(!(reversevs == start))
            {
                reversevs = came_from[reversevs];
                waypoint.push_back(reversevs);
            }

            // cout<<"waypoint.size(): "<<waypoint.size()<<endl;
            std::reverse(waypoint.begin(), waypoint.end());

            for (int i = 0; i < waypoint.size(); ++i)
            {
                auto points = pathtable[waypoint.at(i)];
                trajectory.insert(trajectory.end(), points.begin(), points.end());
            }
            return true;
        }

        //Update the heuristic_index in the while loop
        std::tie(heuristic_index, std::ignore) = HeuristicIndex(heuristic_index, current, circle_path);
        cout<<"heuristic_index: "<<heuristic_index<<endl;
        float step = 0.0;
        step = circle_path.at(heuristic_index).first.radius * 0.8;
        // if (heuristic_index == circle_path.size() - 1)
        // {
        //     step = std::hypot(current.x - goal.x, current.y - goal.y);
        // }
        // else
        // {
        //     step = std::hypot(current.x - circle_path.at(heuristic_index + 1).first.center.x,
        //                       current.y - circle_path.at(heuristic_index + 1).first.center.y);
        // }
        // cout<<"step: "<<step<<endl;

        functions::PriorityNode<sKinodynamic, float> cluster; 
        sKinodynamic leading_points;
        for (auto next : PrimitiveActions(current, step))
        {
            auto nextvs = std::get<0>(next);
            auto next_trajectory = std::get<1>(next);
            auto bundle = HeuristicIndex(heuristic_index, nextvs, circle_path);
            //The car tial is outof free circle;
            if (std::get<0>(bundle) < 0)
            {
                continue;
            }
            sKinodynamic head;
            head.x = nextvs.x + omega.wheelBase * cos(nextvs.theta);
            head.y = nextvs.y + omega.wheelBase * sin(nextvs.theta);
            //The car header is outof free circle;
            // if (std::get<0>(HeuristicIndex(heuristic_index, head, circle_path)) < 0)
            // {
            //     continue;
            // }

            cluster.elements.emplace(nextvs, std::get<1>(bundle));
            came_from[nextvs] = current;
            pathtable[nextvs] = next_trajectory;
        }
        while(!cluster.elements.empty())
        {
            auto top_elements = cluster.elements.top().first;
            cluster.elements.pop();
            frontier.push(top_elements);
        }
    }
    return false;
}