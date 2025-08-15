#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <cstdlib>
#include <rerun/demo_utils.hpp>
#include <unordered_set>
#include <set>
#include "rerun.h"
#include "quadtree.h"
#include <limits>
#include "pathplanning.h"

using namespace std;

double heuristic(float x1, float y1, float x2, float y2) {
    return hypot(x2 - x1, y2 - y1);
}

vector<Node> reconstruct_path(shared_ptr<Node> current) {
    vector<Node> path;
    while (current != nullptr) {
        path.push_back(*current);
        current = current->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

int roundToGrid(float coord, float resolution) {
    return static_cast<int>(round(coord / resolution));
}

int getCostAtPoint(Point p, QuadtreeNode* low, QuadtreeNode* mid, QuadtreeNode* high) {
    int cost = 1; // Default cost for free space
    if (low->inBounds(p)) cost = max(cost, low->getCostAtPoint(p));
    if (mid->inBounds(p)) cost = max(cost, mid->getCostAtPoint(p));
    if (high->inBounds(p)) cost = max(cost, high->getCostAtPoint(p));
    return cost;
}


vector<Node> astarquad(QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree,
                       Node start, Node goal, float resolution) {
    auto cmp = [](const shared_ptr<Node>& a, const shared_ptr<Node>& b) {
        return a->f_cost > b->f_cost;
    };
    priority_queue<shared_ptr<Node>, vector<shared_ptr<Node>>, decltype(cmp)> openSet(cmp);

    unordered_map<pair<int, int>, double, NodeHasher> gScore;
    unordered_map<pair<int, int>, shared_ptr<Node>, NodeHasher> cameFrom;
    unordered_map<pair<int, int>, bool, NodeHasher> closedSet;

    pair<int, int> startKey = {roundToGrid(start.x, resolution), roundToGrid(start.y, resolution)};
    shared_ptr<Node> startNode = make_shared<Node>(start.x, start.y, 0, heuristic(start.x, start.y, goal.x, goal.y));
    openSet.push(startNode);
    gScore[startKey] = 0.0;

    const float dx[8] = {resolution, -resolution, 0, 0, resolution, -resolution, resolution, -resolution};
    const float dy[8] = {0, 0, resolution, -resolution, resolution, -resolution, -resolution, resolution};

    while (!openSet.empty()) {
        shared_ptr<Node> current = openSet.top();
        openSet.pop();

        pair<int, int> currentKey = {roundToGrid(current->x, resolution), roundToGrid(current->y, resolution)};
        if (closedSet[currentKey]) continue;
        closedSet[currentKey] = true;

        if (heuristic(current->x, current->y, goal.x, goal.y) < resolution) {
            return reconstruct_path(current);
        }

        for (int dir = 0; dir < 8; dir++) {
            float newX = current->x + dx[dir];
            float newY = current->y + dy[dir];
            pair<int, int> neighborKey = {roundToGrid(newX, resolution), roundToGrid(newY, resolution)};
            if (closedSet[neighborKey]) continue;

            Point neighborP = {newX, newY};
            int cost = getCostAtPoint(neighborP, lowQuadtree, midQuadtree, highQuadtree);
            if (cost >= 10000) continue;

            double tentative_g = current->g_cost + resolution * cost;
            if (!gScore.count(neighborKey) || tentative_g < gScore[neighborKey]) {
                gScore[neighborKey] = tentative_g;
                auto neighbor = make_shared<Node>(newX, newY, tentative_g, heuristic(newX, newY, goal.x, goal.y), current);
                openSet.push(neighbor);
                cameFrom[neighborKey] = neighbor;
            }
        }
    }

    return {};
}

