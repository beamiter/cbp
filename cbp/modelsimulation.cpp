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
    heuristic_index = HeuristicIndex(heuristic_index, current, circle_path);
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