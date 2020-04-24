#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) 
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    current_node->FindNeighbors();
    for (auto& neighbour : current_node->neighbors)
    {
        neighbour->parent = current_node;
        neighbour->g_value = current_node->g_value + current_node->distance(*neighbour);
        neighbour->h_value = CalculateHValue(neighbour);
        open_list.emplace_back(neighbour);
        neighbour->visited = true;
    }
}

void RoutePlanner::Sort()
{
    std::sort(open_list.begin(), open_list.end(), [] (auto const& begin, auto const& end) 
                {
                    return (begin->g_value + begin->h_value) < (end->g_value + end->h_value);
                }
             );
}

RouteModel::Node *RoutePlanner::NextNode() 
{
    Sort();    
    RouteModel::Node* next_node = open_list.front();
    open_list.erase(open_list.begin());
    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) 
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found{*current_node};

    while (current_node != start_node) 
    {
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
        path_found.emplace_back(*current_node);
    }
    std::reverse(path_found.begin(), path_found.end());

    distance = distance * m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() 
{
    start_node->visited = true;
    open_list.emplace_back(start_node);

    while (!open_list.empty()) 
    {
        RouteModel::Node *current_node = NextNode();
        if (current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        } 
        else 
        {
            AddNeighbors(current_node);
        }
    }
}