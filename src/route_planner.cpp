#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*this->end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor: current_node->neighbors){
        if (!neighbor->visited) {
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            neighbor->visited = true;
            open_queue.emplace(neighbor);
        }
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    auto node = open_queue.top();
    open_queue.pop();
    return node;
}


/**
 * Construct final path.
 *
 * @param pointer to the final node in the path.
 * @return vector of nodes represents the path, where the start node is the first element and the end node is the last element.
 */

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node){
        path_found.push_back(*current_node);
        if (current_node->parent) {
            distance += current_node->distance(*current_node->parent);
        }
        current_node = current_node->parent;
    }
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    current_node->visited = true;
    open_queue.emplace(current_node);
    while (!open_queue.empty()){
        AddNeighbors(current_node);
        current_node = NextNode();
        if (current_node == end_node){
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }
    }
}