
/* @brief combining space exploration and heuristic search
 *in online motion planning for nonholonomic vehicles
 */

#include "sehs.h"

float aim_angle,next1_angle,next2_angle,next1_dist,next2_dist;
std::vector<geometry_msgs::Point32> roadtemp;



sehs::sehs()
: disttolerance(0.5)
, yawtolerance(9.0)
{
    visdwa_marker = std::unique_ptr<visualize_marker>(new visualize_marker("/dwa"));

    omega.halfWidth = 0.5;
    omega.halfLength = 0.9;
    omega.wheelBase = 1.0;
    omega.frontAxle = 1.2;
    omega.tailAxle = 0.6;
    omega.forwardAnchor = 0.4;
    omega.rearwardAnchor = 0.4;

    /*The vehicle dynamic model differs from each other*/
    dynamic_constraints.maxSpeed = 3.0;
    dynamic_constraints.minSpeed = 0.4;
    dynamic_constraints.maxSteeringSpeed = 0.6; //0.406
    dynamic_constraints.maxAcceleration = 2.0;
    dynamic_constraints.maxDecceleration = -3.0;
    dynamic_constraints.maxSteeringAngle =  0.60;
    dynamic_constraints.minTurningRadius = 2.2;
    dynamic_constraints.centrifugalAcceleration = 1.2;

    ros::NodeHandle nh;
    nh.param ( "basemap", mapPath, mapPath );
    nh.param ( "gicscellsize", gicsCellSize, gicsCellSize );
    nh.param ( "iabasemaptllon", iabasemapTL.lon, iabasemapTL.lon );
    nh.param ( "iabasemaptllat", iabasemapTL.lat, iabasemapTL.lat );
    nh.param ( "iabasemapbrlon", iabasemapBR.lon, iabasemapBR.lon );
    nh.param ( "iabasemapbrlat", iabasemapBR.lat, iabasemapBR.lat );

    mapTool.initParam ( mapPath, gicsCellSize, iabasemapTL, iabasemapBR);
}

sehs::~sehs()
{

}

std::tuple<int, float> sehs::HeuristicIndex(int current_index, sKinodynamic current_status, 
const std::vector<std::pair<sFreeCircle, float> >& circle_array)
{
    float min_distance = 10086.0;
    int heuristic_index = current_index;
    for (int i = -1; i <= 6; ++i)
    {
        int index = current_index + i;
        if (index < 0 || index >= circle_array.size())
        {
            continue;
        }
        float distance = std::hypot(current_status.x - circle_array.at(index).first.center.x, 
                                    current_status.y - circle_array.at(index).first.center.y);
        if (min_distance > distance)
        {
            min_distance = distance;
            heuristic_index = index;
        }
        //Only if the distance increased!
        else
        {
            break;
        } 
    }
    if (min_distance > circle_array.at(heuristic_index).first.radius)
    {
        //The status not lie in the circle
        return std::forward_as_tuple(-1, min_distance);
    }
    return std::forward_as_tuple(heuristic_index, min_distance);
}

std::vector<std::tuple<sKinodynamic, std::vector<sKinodynamic>, float>>
sehs::PrimitiveActions(sKinodynamic currentvs, float step_length)
{
    std::vector<std::tuple<sKinodynamic, std::vector<sKinodynamic>, float>> rtn;
    
    bool REVERSE = false;

    int steering_num = 1;
    int speed_num = 1;

    float min_speed = 0.1;
    float min_speed_square = 0.159;
    float controlCycle = 0.0;
    float steering_sample = 0.0;
    float positive_steering = 0.0;
    float negative_steering = 0.0;
    float positive_max_steering_velocity = 0.0;
    float negetive_max_steering_velocity = 0.0;
    float speed_sample = 0.1;
    float delta_t = 0.1;
    float acceleration_coefficient = 0.2;
    float steering_coefficient = 0.9;
    float linear_velocity = 0.0;
    float steering_velocity = 0.0;
    float transfer_angle = 0.0;

    /*    the reversing mode sampling     */
    if (REVERSE)
    {
        std::cout << "Reversing mode!!!" << std::endl;
    }
    /*    the normal mode sampling    */
    else
    {
        speed_num = 1;
        controlCycle = step_length / currentvs.speed;

        float speed_sample = acceleration_coefficient * dynamic_constraints.maxAcceleration;
        for (int i = -speed_num; i <= speed_num; ++i)
        {
            float acceleration = speed_sample * i;

            // //The following speed_square may be negative
            // float speed_square = std::pow(currentvs.speed, 2) + 2 * acceleration * step_length;
            // if (speed_square < min_speed_square)
            // {
            //     continue;
            // }
            // linear_velocity = std::sqrt(speed_square); 

            linear_velocity = currentvs.speed + acceleration * controlCycle;

            if (linear_velocity > dynamic_constraints.maxSpeed || linear_velocity <= 0.1)
            {
                continue;
            }

            #if BDEBUG
            cout<<"linear_velocity: "<<linear_velocity<<endl;
            #endif

            // if (0 == i)
            // {
            //     controlCycle = step_length / currentvs.speed;
            // }
            // else
            // {
            //     controlCycle = (linear_velocity - currentvs.speed) / acceleration;
            // }
            
            #if BDEBUG
            cout<<"controlCycle: "<<controlCycle<<endl;
            #endif
            
            //steering_num must be greater than  0!!!
            steering_num = 5;

            float max_steering_velocity = 0.0;
            float max_steering_angle = 0.0;

            //This restrict centrifugal acceleration!
            max_steering_angle = atan2(omega.wheelBase * dynamic_constraints.centrifugalAcceleration,
                                                               std::pow(linear_velocity, 2));
            if (max_steering_angle > dynamic_constraints.maxSteeringAngle)
            {
                max_steering_angle = dynamic_constraints.maxSteeringAngle;
            }

            /*Generate different negative and positive steering sampling speed
             *The following is the constrain boundary
             */
            //currentvs.steeringAngle
            //Sample the rhs steering speed
            max_steering_velocity = (max_steering_angle - currentvs.steeringAngle) / controlCycle;
            if (max_steering_velocity >= steering_coefficient * dynamic_constraints.maxSteeringSpeed)
            {
                max_steering_velocity = steering_coefficient * dynamic_constraints.maxSteeringSpeed;
            }
            positive_max_steering_velocity = max_steering_velocity;
            
            //Sample the lhs steering speed
            max_steering_velocity = (-max_steering_angle - currentvs.steeringAngle) / controlCycle;
            if (max_steering_velocity <= -steering_coefficient * dynamic_constraints.maxSteeringSpeed)
            {
                max_steering_velocity = -steering_coefficient * dynamic_constraints.maxSteeringSpeed;
            }
            negetive_max_steering_velocity = max_steering_velocity;

            for (int j = -steering_num; j <= steering_num; ++j)
            {
                // if (j <= 0)
                // {
                //     steering_velocity = aim_steering + negative_steering * j;
                // }
                // else
                // {
                //     steering_velocity = aim_steering + positive_steering * j;
                // }

                steering_velocity = negetive_max_steering_velocity + (positive_max_steering_velocity - negetive_max_steering_velocity) * (steering_num + j) /(2 * steering_num);

                if (fabs(steering_velocity) < 1e-3)
                {
                    steering_velocity = 0.0;
                }
                
                /*THis is the trajectory generation process*/
                std::vector<sKinodynamic> path;
                sKinodynamic tempvs = currentvs;
                path.push_back(tempvs); 
 
                bool checkflag = false;
                if (std::hypot(currentvs.x - goal.x, currentvs.y - goal.y) < 3.0)
                {
                    checkflag = true;
                }

                float integration_time = 0.0;
                float time_eval = controlCycle;
                while(integration_time <= controlCycle)
                {
                    integration_time += delta_t;
                    tempvs.x += tempvs.speed * cos(tempvs.theta) * cos(tempvs.steeringAngle) * delta_t;
                    tempvs.y += tempvs.speed * sin(tempvs.theta) * cos(tempvs.steeringAngle) * delta_t;
                    tempvs.theta += tempvs.speed * sin(tempvs.steeringAngle) / omega.wheelBase * delta_t;
                    tempvs.speed += acceleration * delta_t;
                    tempvs.steeringAngle += steering_velocity * delta_t;
                    path.push_back(tempvs);
                
                    if (true == checkflag)
                    {
                        if (true == ReachedGoal(tempvs, goal))
                        {
                            if (!virtualgoal.count(tempvs))
                            {
                                virtualgoal.insert(std::make_pair(tempvs, tempvs));
                            }
                            time_eval = integration_time;
                            break;
                        }                    
                    }
                }

                #if BDEBUG
                // cout<<"tempvs.theta: "<<tempvs.theta<<"; tempvs.steeringAngle: "<<tempvs.steeringAngle<<endl;
                // cout<<"tempvs.x: "<<tempvs.x<<"; tempvs.y: "<<tempvs.y<<endl;
                #endif

                if (!path.empty())
                {
                    rtn.emplace_back(std::forward_as_tuple(path.back(), path, time_eval * (path.back().speed) * 0.5));
                }
            }
        }
    }

    return rtn;
}

bool sehs::HeuristicSearch(sKinodynamic start, sKinodynamic goal, std::vector<sKinodynamic>& trajectory)
{
    KinodynamicMap<float> cost_so_far;
    KinodynamicMap<sKinodynamic> came_from;
    KinodynamicMap<std::vector<sKinodynamic>> pathtable;
    functions::PriorityNode<sKinodynamic, float> frontier;

    frontier.elements.emplace(start, 0.0);
    came_from[start] = start;
    cost_so_far[start] = 0.0;
    virtualgoal.clear();

    int heuristic_index = 0;
    float min_step = 1.0;
    float alpha = 1.0;
    float beta = 0.8;

    while(!frontier.elements.empty())
    {
        auto current = frontier.elements.top().first;
        frontier.elements.pop();

        // This is the equivalent judgment
        if (virtualgoal.count(current))
        {
            trajectory.clear();
            came_from[goal] = current;
            ROS_ERROR_STREAM("########");
            cout<<"came_from.size(): "<<came_from.size()<<endl;
            sKinodynamic reversevs = goal;
            waypoint.clear();
            waypoint.push_back(reversevs);
            while(!(reversevs == start))
            {
                reversevs = came_from[reversevs];
                waypoint.push_back(reversevs);
                // cout<<"reversevs.x: "<<reversevs.x<<"; reversevs.y: "<<reversevs.y<<endl;
            }

            cout<<"waypoint.size(): "<<waypoint.size()<<endl;
            std::reverse(waypoint.begin(), waypoint.end());

            for (int i = 0; i < waypoint.size(); ++i)
            {
                auto points = pathtable[waypoint.at(i)];
                trajectory.insert(trajectory.end(), points.begin(), points.end());
            }
            return true;
        }
        
        std::tie(heuristic_index, std::ignore) = HeuristicIndex(heuristic_index, current, circle_path);
        
        float step = 0;

        aim_angle = 0;
        if(heuristic_index == -1)
        {
            continue;
        }

        if(heuristic_index < circle_path.size() - 2) 
        {
            next2_angle = atan2(circle_path.at(heuristic_index + 2).first.center.y - current.y , circle_path.at(heuristic_index + 2).first.center.x - current.x);
            next1_angle = atan2(circle_path.at(heuristic_index + 1).first.center.y - current.y , circle_path.at(heuristic_index + 1).first.center.x - current.x);
            next2_dist = pow(circle_path.at(heuristic_index + 2).first.center.y - current.y , 2) + pow(circle_path.at(heuristic_index + 2).first.center.x - current.x , 2);
            next1_dist = pow(circle_path.at(heuristic_index + 1).first.center.y - current.y , 2) + pow(circle_path.at(heuristic_index + 1).first.center.x - current.x , 2);
              
            double k = 1.8 - circle_path.at(heuristic_index + 1).first.radius;
              
            if(next1_dist > pow(k * circle_path.at(heuristic_index + 1).first.radius, 2))
            {
                aim_angle = next1_angle;
                
            }
            else
            {
                aim_angle = next2_angle + (next1_angle - next2_angle) * (sqrt(next1_dist) /( k * circle_path.at(heuristic_index + 1).first.radius));
                
            }
        }
        else 
        {
            aim_angle = atan2(goal.y - current.y , goal.x - current.x);
        }
        aim_angle = aim_angle * 1.1;

        if(heuristic_index < circle_path.size())
        {
            step = std::min(std::min(alpha * circle_path.at(heuristic_index).first.radius, beta * circle_path.at(heuristic_index).second), min_step);
        }
        // else
        // {
        //     step = std::min(std::min(alpha * circle_path.at(heuristic_index).first.radius, beta * circle_path.at(heuristic_index).second), min_step);
        // }

        step = min_step;
        //std::cout<<step<<std::endl;

        for (auto next : PrimitiveActions(current, step))
        {
            auto next_trajectory = std::get<1>(next);
            sKinodynamic nextvs = std::get<0>(next);
            float new_cost = 0.0;
            float priority = 0.0;
            float let_devia_angle = 0.0;
            new_cost = cost_so_far[current] + std::get<2>(next);

            int current_index = 0;
            //std::cout<<heuristic_index<<std::endl;
            std::tie(current_index, std::ignore) = HeuristicIndex(heuristic_index, nextvs, circle_path);
            if (current_index < 0)
                continue;

            geometry_msgs::Point32 tt;
            {
                tt.x = nextvs.x;
                tt.y = nextvs.y;
                roadtemp.push_back(tt);
            }

            /*If only the successor is not in the close table*/
            if (!came_from.count(nextvs))
            {
                if (!cost_so_far.count(nextvs) || (new_cost < cost_so_far[nextvs]))
                {
                    float minus_distance = 10086.0;
                    for (int i = 0; i <= 2; ++i)
                    {
                        int id = current_index + i;
                        if (id >= circle_path.size() || id < 0)
                        {
                            continue;
                        }

                         float dis = 0.0;
                        // if (id < circle_path.size() - 2)
                        // {
                        //     dis = std::hypot(nextvs.x - circle_path.at(id).first.center.x, 
                        //                        nextvs.y - circle_path.at(id).first.center.y) + std::hypot(nextvs.x - circle_path.at(id + 3).first.center.x, 
                        //                        nextvs.y - circle_path.at(id + 3).first.center.y);
                        // }                  //0.8 * circle_path.at(id).second;
                        
                        float how_far = 0.7 + 0.5 * id / circle_path.size();  
                        let_devia_angle = std::fabs(atan2(circle_path.at(id).first.center.y - nextvs.y , circle_path.at(id).first.center.x - nextvs.x) - nextvs.theta);
                
                        {
                            dis = std::hypot(nextvs.x - circle_path.at(id).first.center.x, nextvs.y - circle_path.at(id).first.center.y)  + 
                                                        circle_path.at(id).second;
                            // dis = std::hypot(nextvs.x - circle_path.at(id).first.center.x, nextvs.y - circle_path.at(id).first.center.y)  + sin(let_devia_angle) + 
                            //                             circle_path.at(id).second;
                        }                  
                        

                        if (minus_distance > dis)
                        {
                            minus_distance = dis;
                        }
                    }
                    priority = new_cost + minus_distance ;// nextvs.speed;
                    //priority = minus_distance;

                    frontier.elements.emplace(nextvs, priority);
                    cost_so_far[nextvs] = new_cost;
                    came_from[nextvs] = current;
                    pathtable[nextvs] = next_trajectory;
                }
            }
        }
    }
    return false;
}

void sehs::testModule()
{
    static int cc = 0;

    cObs.clear();
    geometry_msgs::Point32 obs, tt;
    std::vector<geometry_msgs::Point32> road, temp_road;

    #if 1
    obs.x = 4.0;
    obs.y = 0.5;
    cObs[obs.x].insert(obs);
    road.push_back(obs); 

    // obs.x = 6.0;
    // obs.y = -0.5;
    // cObs[obs.x].insert(obs);
    // road.push_back(obs); 

    obs.x = 8.0;
    obs.y = 0.5;
    cObs[obs.x].insert(obs);
    road.push_back(obs); 

    // obs.x = 10.0;
    // obs.y = -0.5;
    // cObs[obs.x].insert(obs);
    // road.push_back(obs); 

    obs.x = 12.0;
    obs.y = 0.5;
    cObs[obs.x].insert(obs);
    road.push_back(obs); 

    // obs.x = 14.0;
    // obs.y = -0.5;
    // cObs[obs.x].insert(obs);
    // road.push_back(obs); 

    for (float a = -2.0; a <= 20.0; a += 0.2)
    {
        obs.x = a;
        obs.y =  -1.0;
        cObs[obs.x].insert(obs);
        road.push_back(obs); 
        obs.x = a;
        obs.y = +1.0;
        cObs[obs.x].insert(obs);
        road.push_back(obs); 
    }

    for (float a = 6.0; a <= 13.0; a += 1)
    {
        // obs.x = a;
        // obs.y = -0.5;
        // cObs[obs.x].insert(obs);
        // road.push_back(obs); 
    }

    #endif

    #if 0
    obs.x = 7;
    obs.y = 0.3;
    cObs[obs.x].insert(obs);
    road.push_back(obs); 
    obs.x = 7;
    obs.y = 0.5;
    cObs[obs.x].insert(obs);
    road.push_back(obs); 

    for (float a = -2.0; a <= 2.0; a += 0.5)
    {
        obs.x = 7;
        obs.y = a;
        cObs[obs.x].insert(obs);
        road.push_back(obs); 
    }

    for (float a = 7.0; a <= 10.0; a += 0.5)
    {
        obs.x = a;
        obs.y = -2;
        cObs[obs.x].insert(obs);
        road.push_back(obs); 
    }
    #endif

    visdwa_marker->publishobstacle(road, 1, 0.1);

    start.x = -1.2;
    start.y = start.theta = 0.0;
    start.speed = 1.0;
    start.steeringAngle = 0.0;

    goal.x = 20.0;
    goal.y = 0.0;
    goal.theta = 0.0 * M_PI / 180.0;
    bool ok = false;
    std::vector<sKinodynamic> trajectory, newtrajectory;

    #if 1
    ok = SpaceExploration(start, goal);

    #else
    ok = WavefrontExpansion(start, goal, 4);
    #endif
    cout<<"Expansion status: "<<ok<<endl;
    
    // for (auto a : circle_path)
    // {
    //     std::cout<<a.first.radius<<std::endl;        
    // }

    std::vector<sFreeCircle> bubbles;
    bubbles.clear();
    road.clear();
    for (auto a : circle_path)
    {
        obs.x = a.first.center.x;
        obs.y = a.first.center.y;
        obs.z = a.first.radius;
        road.push_back(obs);
        bubbles.push_back(a.first);
    }
    visdwa_marker->publishcircles(road);
    
    
    road = functions::curvefitting(road);
    ok = HeuristicSearch(start, goal, newtrajectory);

    visdwa_marker->publishobstacle(roadtemp, 3, 0.02);

    for (auto a : newtrajectory)
    {
        tt.x = a.x;
        tt.y = a.y;
        temp_road.push_back(tt);
    }
    
     visdwa_marker->publishobstacle(temp_road, 4, 0.05);
    
    // visdwa_marker->publishobstacle(temp_road, 4, 0.15);

    // Optimizer opt;
    // temp_road.clear();
    // opt.Solve(bubbles);
    // cout << "opt.path.size(): " << opt.path.size() << endl;

    // temp_road = functions::curvefitting(opt.path);
    // // temp_road = opt.path;
    // visdwa_marker->publishobstacle(temp_road, 4, 0.1);

    /*This requires that the x value is monotone increasing 
     */
    // std::vector<double> X(road.size()), Y(road.size());
    // for (auto a : road)
    // {
    //     X.push_back(a.x);
    //     Y.push_back(a.y);
    // } 
    // tk::spline s;
    // s.set_points(X, Y);

#if 0
    if (true == ok)
    {
        // ok = HeuristicSearch(start, goal, trajectory);
        ok = SnakeCrawl(start, goal, trajectory);
        cout<<"SnakeCrawl status: "<<ok<<endl;       
    }

    road.clear();
    for (auto a : waypoint)
    {
        obs.x = a.x;
        obs.y = a.y;
        road.push_back(obs);
    }
    visdwa_marker->publishobstacle(road, 5, 0.1);

    road.clear();
    for (auto a : trajectory)
    {
        tt.x = a.x;
        tt.y = a.y;
        road.push_back(tt);
    }
    visdwa_marker->publishobstacle(road, 6, 0.05);
#endif

#if 0
    std::vector<std::tuple<sKinodynamic, std::vector<sKinodynamic>, float>> actions;
    start.speed = 2.0;
    // actions = ClosedLoopPrediction(start);
    actions = PrimitiveActions(start, 3.0);
    std::cout << "actions.size(): " << actions.size() << std::endl;
    if (cc >= actions.size())
    {
        cc = 0;
    }
    road.clear(); 
    for (auto a : actions)
    {
        for (auto b : std::get<1>(a))
        {
            tt.x = b.x;
            tt.y = b.y;
            road.push_back(tt);
        }
    } 
    road.clear();   
    for (auto c : std::get<1>(actions.at(cc)))
    {
        tt.x = c.x;
        tt.y = c.y;
        road.push_back(tt);
    }
    visdwa_marker->publishobstacle(road, 8, 0.05);
    cc++;  
#endif
    std::cout << "---------------------------" << std::endl;
}
