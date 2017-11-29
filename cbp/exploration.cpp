/* @brief combining space exploration and heuristic search
 *in online motion planning for nonholonomic vehicles
 */

#include "sehs.h"

bool sehs::SpaceExploration(sKinodynamic start, sKinodynamic goal)
{
    FreeCircleMap<sFreeCircle> came_from;
    OpenMultiset<sFreeCircle> openlist;
    std::vector<sFreeCircle> closed_vector;
    sFreeCircle next_circle;

    sFreeCircle start_circle, goal_circle;
    GenerateCircle(start, start_circle);
    GenerateCircle(goal, goal_circle);

    stparam::target = goal_circle;

    openlist.nearest.insert(start_circle);
    openlist.largest.insert(start_circle);

    came_from[start_circle] = start_circle;

    while(!openlist.nearest.empty())
    {
        // cout<<"--------------------********************--------------------"<<endl;

        auto near = *openlist.nearest.begin();
        // std::cout<<"near.center.x: "<<near.center.x<<std::endl;
        if(OverLap(near, goal_circle, 0.6))
        {
            ROS_ERROR_STREAM("Reached goal!!!");

            closed_vector.push_back(near);
            closed_vector.push_back(goal_circle);
            came_from[goal_circle] = near;
            sFreeCircle reverse_circle = goal_circle;
            while(!(reverse_circle == start_circle))
            {
                // cout<<"reverse_circle.center.x: "<<reverse_circle.center.x<<"; reverse_circle.center.y: "
                //     <<reverse_circle.center.y<<endl;
                // cout<<"reverse_circle.radius: "<<reverse_circle.radius<<endl;
                reverse_circle = came_from[reverse_circle];
            }
            // cout<<"reverse_circle.center.x: "<<reverse_circle.center.x<<"; reverse_circle.center.y: "
            //     <<reverse_circle.center.y<<endl;
            // cout<<"reverse_circle.radius: "<<reverse_circle.radius<<endl;
            return true;
        }
        else
        {
            auto large = *openlist.largest.begin();

            openlist.nearest.erase(openlist.nearest.begin());
            openlist.largest.erase(openlist.largest.begin());

            auto iter = openlist.largest.find(near);
            if (iter != openlist.largest.end())
            {
                openlist.largest.erase(iter);
            } 
            iter = openlist.nearest.find(large);
            if (iter != openlist.nearest.end())
            {
                openlist.nearest.erase(iter);
            }

            if (true == NotExist(near, closed_vector))
            {
                // std::cout<<"1. near.center.x: "<<near.center.x<<"; near.center.y: "<<near.center.y<<std::endl;
                if (true == Expand(near, goal_circle, 6, next_circle))
                {
                    // std::cout<<"next_circle.radius: "<<next_circle.radius<<std::endl;
                    closed_vector.push_back(near);
                    came_from[next_circle] = near;
                    openlist.nearest.insert(next_circle);
                    openlist.largest.insert(next_circle);
                }
                else
                {
                    ROS_ERROR_STREAM("1. Expand the circle failed!!!");
                }
            }
            else
            {
                ROS_ERROR_STREAM("1. Already existed in the close table!!!");
                // std::cout<<"------1. near.center.x: "<<near.center.x<<"; near.center.y: "<<near.center.y<<std::endl;
            }
            if (true == NotExist(large, closed_vector))
            {
                // std::cout<<"2. large.center.x: "<<large.center.x<<std::endl;
                if (true == Expand(large, goal_circle, 4, next_circle))
                {
                    // std::cout<<"2. next_circle.center.x: "<<next_circle.center.x<<std::endl;
                    closed_vector.push_back(large);
                    came_from[next_circle] = large;
                    openlist.largest.insert(next_circle);
                    openlist.nearest.insert(next_circle);
                }
            }
        }
    }

    return false;
}

/*
* @brief: samples the border of circle
*/
bool sehs::Expand(sFreeCircle current, sFreeCircle goal, int sample_number, sFreeCircle& target)
{
    if (0 == sample_number)
    {
        return false;
    }

    sKinodynamic circle;
    float included_angle = 0.0;
    float sample_angle = 2 * M_PI / sample_number;
    included_angle = atan2(goal.center.y - current.center.y, goal.center.x - current.center.x);
    
    for (int i = 0; i < sample_number; ++i)
    {
        float angle = included_angle + i * sample_angle;
        circle.x = current.center.x + current.radius * cos(angle);
        circle.y = current.center.y + current.radius * sin(angle);
        // cout<<"circle.x: "<<circle.x<<"; circle.y: "<<circle.y<<endl;
        if (true == GenerateCircle(circle, target))
        {
            return true;
        }
    }
    return false;
}

bool sehs::WavefrontExpansion(sKinodynamic start, sKinodynamic goal, int sample_number)
{
    FreeCircleMap<sFreeCircle> came_from;
    functions::PriorityNode<sFreeCircle, float> frontier;
    std::vector<sFreeCircle> closed_vector;
    sFreeCircle next_circle, current;

    sKinodynamic final;
    sFreeCircle start_circle, goal_circle, final_circle;
    GenerateCircle(start, start_circle);
    GenerateCircle(goal, goal_circle);
    
    final.x = goal_circle.center.x - goal_circle.radius * cos(goal_circle.center.theta);
    final.y = goal_circle.center.y - goal_circle.radius * sin(goal_circle.center.theta);
    GenerateCircle(final, final_circle);

    stparam::target = final_circle;

    frontier.elements.emplace(start_circle, 0.0);
    came_from[start_circle] = start_circle;

    while(!frontier.elements.empty())
    {
        // cout<<"--------------------********************--------------------"<<endl;
        current = frontier.elements.top().first;
        frontier.elements.pop();
        
        if (OverLap(current, final_circle, 0.6))
        {
            // ROS_ERROR_STREAM("Reached goal!!!");
            path_points.clear();
            circle_path.clear();
            geometry_msgs::Point32 obs;
            sFreeCircle reverse_circle, previous_circle;

            /*This helps lead to smooth the final path angle*/
            came_from[goal_circle] = final_circle;
            came_from[final_circle] = current;
            
            circle_path.emplace_back(goal_circle, 0.0);
            reverse_circle = goal_circle;
            
            // cout<<"reverse_circle.center.x: "<<reverse_circle.center.x<<"; reverse_circle.center.y: "
            //     <<reverse_circle.center.y<<endl;
            // cout<<"reverse_circle.radius: "<<reverse_circle.radius<<endl;

            auto circle_iter = came_from.begin();
            while(circle_iter != came_from.end())
            {
                obs.x = circle_iter->first.center.x;
                obs.y = circle_iter->first.center.y;
                obs.z = circle_iter->first.radius;
                circle_iter++;
                if (obs.x > 3.0)
                {
                    // continue;
                }
                path_points.push_back(obs);
            }

            float circle_distance = 0.0; 
            while(!(reverse_circle == start_circle))
            {
                previous_circle = reverse_circle;
                reverse_circle = came_from[reverse_circle];
                circle_distance += std::hypot(previous_circle.center.x - reverse_circle.center.x,
                                              previous_circle.center.y - reverse_circle.center.y);
                circle_path.emplace_back(reverse_circle, circle_distance);
                // cout<<"********"<<endl;
                // cout<<"reverse_circle.center.x: "<<reverse_circle.center.x<<"; reverse_circle.center.y: "
                //     <<reverse_circle.center.y<<endl;
                // cout<<"reverse_circle.radius: "<<reverse_circle.radius<<endl;
                // cout<<"circle_distance: "<<circle_distance<<endl;
            }
            std::reverse(circle_path.begin(), circle_path.end());
            cout<<"came_from.size(): "<<came_from.size()<<endl;
            cout<<"closed_vector.size(): "<<closed_vector.size()<<endl;
            cout<<"circle_path.size(): "<<circle_path.size()<<endl;
            cout<<"********************************"<<endl;
            return true;
        }
        else
        {
            // std::cout<<"1. current.center.x: "<<current.center.x<<"; current.center.y: "<<current.center.y<<std::endl;
            float angle = 0.0;
            float priority_value = 0.0;
            float sample_area = 45.0 * M_PI / 180.0;
            float sample_angle = sample_area / sample_number;
            bool success_flag = false;
            /*ensure the completeness for the hybird sampling*/
            for (int a = 0; a < 8; ++a)
            {
                float theta = a * sample_area;
                /*1. regular sampling process*/
                angle = theta;
                next_circle.center.x = current.center.x + current.radius * cos(angle);
                next_circle.center.y = current.center.y + current.radius * sin(angle);
                next_circle.radius = current.radius;
                if (true == SampleCircle(next_circle, next_circle))
                {
                    if (true == NotExist(next_circle, closed_vector))
                    {     
                        success_flag = true;  
                        came_from[next_circle] = current;
                        priority_value = std::hypot(next_circle.center.x - goal_circle.center.x, 
                                         next_circle.center.y - goal_circle.center.y) - next_circle.radius;
                        frontier.elements.emplace(next_circle, priority_value); 
                    }
                }     
                   
                /*2. shrink sampling process*/ 
                functions::PriorityNode<sFreeCircle, float>::PriorityQueue area_frontier;   
                for (int i = 1; i < sample_number; ++i)
                {
                    angle = theta + i * sample_angle;
                    next_circle.center.x = current.center.x + current.radius * cos(angle);
                    next_circle.center.y = current.center.y + current.radius * sin(angle);
                    next_circle.radius = current.radius;
                    if (true == SampleCircle(next_circle, next_circle))
                    {
                        if (true == NotExist(next_circle, closed_vector))
                        {     
                            area_frontier.emplace(next_circle, next_circle.radius);
                        }
                    }
                }
                if (!area_frontier.empty())
                {
                    success_flag = true;  
                    next_circle = area_frontier.top().node.first;
                    came_from[next_circle] = current;
                    priority_value = std::hypot(next_circle.center.x - goal_circle.center.x, 
                                     next_circle.center.y - goal_circle.center.y) - next_circle.radius;
                    frontier.elements.emplace(next_circle, priority_value);       
                }
            }
            if (true == success_flag)
            {
                closed_vector.push_back(current);
            }  
        }
    }
    ROS_ERROR_STREAM("WavefrontExpansion failed!!!");
    return false;
}


/*
* @brief: creates a free-space circle centered at current position
*         with the radius equal to the distance to the nearest obstacles
*         minus the inner radius of the vehicle
*/
bool sehs::GenerateCircle(sKinodynamic current, sFreeCircle& cfree)
{
    float max_radius = omega.halfWidth + 2.4;
    float min_radius = omega.halfWidth + 0.2;

    cfree.center = current;
    cfree.radius = min_radius;
    if (false == FreeSpaceEstimate(cfree))
    {
        return false;
    }
    cfree.radius = max_radius;
    if (true == FreeSpaceEstimate(cfree))
    {
        cfree.radius -= omega.halfWidth;
        return true;
    }

    while(max_radius - min_radius >= 0.1)
    {
        float middle_radius = (max_radius + min_radius) / 2.0;
        cfree.radius = middle_radius;
        if (true == FreeSpaceEstimate(cfree))
        {
            min_radius = middle_radius;
        }
        else
        {
            max_radius = middle_radius;
        }
    }
    //min_radius will always be the safe radius
    cfree.radius = min_radius - omega.halfWidth;
    return true;
}

/*
* @brief: Ensure that the generated circle meet the not 
*         overlapping requirement
*  just in case that the radius of the circle is 0;
*/
bool sehs::SampleCircle(sFreeCircle current, sFreeCircle& cfree)
{
    cfree.center = current.center;
    float max_radius = omega.halfWidth + 2.4;
    float min_radius = omega.halfWidth + 0.2;
    /*the coefficient 1.6 must be smaller than the coefficient in notexist function*/
    float increasing_rate = omega.halfWidth + 1.6 * current.radius;

    cfree.radius = min_radius;
    if (false == FreeSpaceEstimate(cfree))
    {
        return false;
    }
    if (max_radius > increasing_rate + 1e-3)
    {
        max_radius = increasing_rate;
    }
    cfree.radius = max_radius;
    if (true == FreeSpaceEstimate(cfree))
    {
        cfree.radius -= omega.halfWidth;
        return true;
    }

    float middle_radius = current.radius + omega.halfWidth;
    while(max_radius - min_radius >= 0.1)
    {  
        cfree.radius = middle_radius;
        if (true == FreeSpaceEstimate(cfree))
        {
            min_radius = middle_radius;
        }
        else
        {
            max_radius = middle_radius;
        }  
        middle_radius = (max_radius + min_radius) / 2.0;
    }
    cfree.radius = min_radius - omega.halfWidth;
    return true;
}

/**
* @brief
* @param
*/
bool sehs::ReachedGoal(sKinodynamic currentvs, sKinodynamic goalvs)
{
    float dis, theta;
    dis = std::hypot(currentvs.x - goalvs.x, currentvs.y - goalvs.y);
    theta = functions::rad2Deg(fabs(currentvs.theta - goalvs.theta));
    if (dis <= disttolerance && theta <= yawtolerance)
    {
        return true;
    }
    return false;
}

/*
* @brief: check if two circles over lap with each other in a certan margin
*/
bool sehs::OverLap(sFreeCircle current, sFreeCircle comparison, float margin)
{
    margin = (margin > 1.95)? 1.95 : margin;
    margin = (margin < 0.0)? 0.0 : margin;
    float min_radius = (current.radius <= comparison.radius)? current.radius : comparison.radius;
    float distance = std::hypot(current.center.x - comparison.center.x, current.center.y - comparison.center.y);
    if (distance <= current.radius + comparison.radius - margin * min_radius)
    {
        return true;
    }
    return false;
}

bool sehs::NotExist(sFreeCircle current, std::vector<sFreeCircle> closed_vector)
{
    for (auto a : closed_vector)
    {
        if (true == OverLap(current, a, 1.8))
        {
            return false;
        }
    }
    return true;
}

bool sehs::FreeSpaceEstimate(sFreeCircle current)
{
    /*Vehicle coordination*/
    geometry_msgs::Point32 bottomright, upleft;
    if(cObs.empty())
    {
        return true;
    }
    bottomright.x =  current.center.x - current.radius;
    bottomright.y = current.center.y - current.radius;
    upleft.x = current.center.x + current.radius;
    upleft.y = current.center.y + current.radius;
    auto br = cObs.lower_bound(bottomright.x);
    if (br == cObs.end() || br->first > upleft.x)
    {
        return true;
    }
    else
    {
        for(; br->first <= upleft.x && br != cObs.end(); ++br)
        {
            auto ul = cObs[br->first].lower_bound(bottomright);
            if (ul == cObs[br->first].end() || ul->y > upleft.y)
            {
                continue;
            }
            else
            {
                for (; ul->y <= upleft.y && ul != cObs[br->first].end(); ++ul)
                {
                    if (std::hypot(current.center.x - ul->x, current.center.y - ul->y) < current.radius + 1e-3)
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}