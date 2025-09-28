#include <iostream>
#include <cmath>
#include <queue>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <Eigen/Core>
#include <unordered_set>
//for downsampling and filtering
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <cstdlib>
#include <rerun/demo_utils.hpp>
#include "pathplanning.h"

using namespace std;
using namespace mapping;
using namespace quadtree;

namespace planning {
// Heuristic function (Euclidean Distance)
double heuristic_astar(int x1, int y1, int x2, int y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


vector<Node> astarsparse(const mapping::Gridmap& gridmap, Node start, Node goal) {
    priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, comparenode> openList;
    unordered_map<pair<int, int>, shared_ptr<Node>, pair_hash> allNodes;
    unordered_set<pair<int, int>, pair_hash> visited;

    start.h_cost = planning::heuristic_astar(start.x, start.y, goal.x, goal.y);
    start.f_cost = start.g_cost + start.h_cost;

    auto startNode = make_shared<Node>(start);
    allNodes[{start.x, start.y}] = startNode;
    openList.push(startNode);

    // Movement directions (8 directions: N, S, E, W, NE, NW, SE, SW)
    const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dy[] = {0, 0, 1, -1, 1, -1, -1, 1};

    while (!openList.empty()) {
        auto current = openList.top();
        openList.pop();
        pair<int, int> currentKey = {current->x, current->y};

        // **Goal check**
        if (current->x == goal.x && current->y == goal.y) {
            vector<Node> path;
            for (auto n = current; n != nullptr; n = n->parent) {
                path.push_back(*n);
            }
            reverse(path.begin(), path.end());
            return path; 
        }

              for (int i = 0; i < 8; i++) {
    int newX = current->x + dx[i];
    int newY = current->y + dy[i];
    double movementCost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
    // Get obstacle cost
    double obstacleCost = 1e6;  // Default: unexplored = very high cost (impassable)
    auto it = gridmap.occupancy_grid.find({newX, newY});
    if (it != gridmap.occupancy_grid.end()) {
        obstacleCost = it->second.cost;  // Explored, with known obstacle cost
    } 
    else if (newX >= gridmap.min_x && newX <= gridmap.max_x &&
             newY >= gridmap.min_y && newY <= gridmap.max_y) {
        obstacleCost = 0.0;  // Explored, but free space
    } 
    else {
        continue;  // Unexplored or out of bounds → skip this neighbor
    }
    if (obstacleCost >= 1e6) continue;  // Skip impassable cells
    double newGCost = current->g_cost + movementCost + obstacleCost;
    if (current->parent && (newX - current->x != current->x - current->parent->x || newY - current->y != current->y - current->parent->y)) {
        newGCost += 0.1;  // Penalize unnecessary turns slightly
    }
    pair<int, int> neighborKey = {newX, newY};
    //Ensure only better paths update the node
    if (visited.find(neighborKey) == visited.end() &&
        (allNodes.find(neighborKey) == allNodes.end() || newGCost < allNodes[neighborKey]->g_cost)) {

        auto neighbor = make_shared<Node>(newX, newY, newGCost, planning::heuristic_astar(newX, newY, goal.x, goal.y), current);
        neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;

        openList.push(neighbor);
        allNodes[neighborKey] = neighbor;
        visited.insert(neighborKey);  // Move this AFTER confirming a better path
    }
}

    }

    return {};  // No path found
}
}
