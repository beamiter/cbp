#include "globalplanner.h"

globalplanner::globalplanner()
{
    ros::NodeHandle nh;
    std::string routePath;
    nh.param("routemap", routePath, routePath);
    routePath.append("/pathseg.yaml");
    YAML::Node doc = YAML::LoadFile(routePath.c_str());
    for (unsigned int i = 0; i < doc.size(); ++i)
    {
        YAML::Node cfg = doc[i];
        YAML::Node cfg0 = cfg["children"];
        for (unsigned int j = 0; j < cfg0.size(); ++j)
        {
            YAML::Node cfg1 = cfg0[j];
            edges[cfg["parent"].as<int>()].push_back(cfg1["number"].as<int>());
        }
    }
    for (auto a = edges.begin(); a != edges.end(); ++a)
    {
        std::cout<<a->first<<std::endl; 
        for (auto b : a->second)
        {
            std::cout<<"--> "<<b<<std::endl; 
        }
    }

}

globalplanner::~globalplanner()
{

}

void globalplanner::initParam()
{
    return;
}

void globalplanner::run()
{
    return;
}

bool globalplanner::globalpath(int start, int goal, 
    std::unordered_map<int, int>& came_from,
    std::unordered_map<int, float>& cost_so_far)
{
    int current;
    int mark_count = 0;
    functions::PriorityNode<int, float> frontier;

    frontier.elements.emplace(start, 0);
    came_from[start] = start;
    cost_so_far[start] = 0.0;

    while (!frontier.elements.empty())
    {
        current = frontier.elements.top().first;
        frontier.elements.pop();
        //This algorithm is like holy shit, and perfectly wonderful!!!

        int collisionflag = false;
        if (true == collisionflag)
        {
            continue;
        }
        if (current == goal)
        {
            return true;
        }

        mark_count ++;
        if (mark_count > 200)
        {
            return false;
        }

        if (edges.count(current))
        {
            for (auto next : edges[current])
            {
                float new_cost = cost_so_far[current] + 1;

                // Only if the successor is not in the close table
                if (!came_from.count(next))
                {
                    // If the successor is not in the open table, If the successor is in the open table
                    if (!cost_so_far.count(next) || (new_cost < cost_so_far[next]))
                    {
                        float hValue = 0;
                        float priority = new_cost + hValue;
                        frontier.elements.emplace(next, priority);
                        cost_so_far[next] = new_cost;
                        came_from[next] = current;
                    }
                }
            }            
        }
        else
        {
            continue;
        }
    }
    return false;
}

int globalplanner::globalpath(int start, int goal, 
    std::unordered_map<int, std::vector<int> > edges,
    std::unordered_map<int, int>& came_from,
    std::unordered_map<int, float>& cost_so_far)
{
    int current;
    functions::PriorityNode<int, float> frontier;

    frontier.elements.emplace(start, 0);
    came_from[start] = start;
    cost_so_far[start] = 0.0;

    while (!frontier.elements.empty())
    {
        current = frontier.elements.top().first;
        frontier.elements.pop();
        //This algorithm is like holy shit, and perfectly wonderful!!!

        int collisionflag = false;
        if (true == collisionflag)
        {
            continue;
        }
        if (current == goal)
        {
            break;
        }

        if (edges.count(current))
        {
            for (auto next : edges[current])
            {
                float new_cost = cost_so_far[current] + 1;

                // Only if the successor is not in the close table
                if (!came_from.count(next))
                {
                    // If the successor is not in the open table, If the successor is in the open table
                    if (!cost_so_far.count(next) || (new_cost < cost_so_far[next]))
                    {
                        float hValue = 0;
                        float priority = new_cost + hValue;
                        frontier.elements.emplace(next, priority);
                        cost_so_far[next] = new_cost;
                        came_from[next] = current;
                    }
                }
            }            
        }
        else
        {
            continue;
        }

    }
    return current;
}

std::vector<int> globalplanner::reconstruct_path(
int start, int goal, std::unordered_map<int, int> came_from)
{
    std::vector<int> path;
    int current = goal;
    path.push_back(current);
    while (!(current == start))
    {
        current = came_from[current];
        path.push_back(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
}