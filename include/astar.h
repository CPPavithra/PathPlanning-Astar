#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <unordered_map> 
#include <string>
#include <utility> // For std::pair
#include <functional> // For std::hash
#include <cmath>
#include <deque>
#include <memory>
#include "common.h"

using namespace std;

struct Node
{
    float x, y;
    double g_cost, h_cost, f_cost;
    shared_ptr<Node> parent;  // ✅ Use shared_ptr instead of raw pointer

    // ✅ Corrected constructor using shared_ptr
    Node(float x, float y, double g_cost = 0, double h_cost = 0, shared_ptr<Node> parent = nullptr)
        : x(x), y(y), g_cost(g_cost), h_cost(h_cost), f_cost(g_cost + h_cost), parent(move(parent)) {}

    bool operator>(const Node& other) const {
        return f_cost > other.f_cost;
    }
     bool operator==(const Node& other) const {
        return (x == other.x) && (y == other.y);
    }
};

// ✅ Fix comparison function for priority_queue
struct comparenode {
    bool operator()(const shared_ptr<Node>& a, const shared_ptr<Node>& b) const {
        return a->f_cost > b->f_cost;
    }
};

double heuristic(int x1, int y1, int x2, int y2);

// ✅ Use shared_ptr for consistency
vector<Node> astar(const unordered_map<pair<int, int>, CellCost, pair_hash>& occupancyGrid, Node start, Node goal);

#endif //ASTAR_H

