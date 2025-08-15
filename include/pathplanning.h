#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#include "common.h"      // Already has pair_hash, CellCost, Gridmap, Point
#include "quadtree.h"
#include <vector>
#include <unordered_map>
#include <set>
#include <deque>
#include <memory>
#include <cmath>

// From astar.h
struct Node {
    float x, y;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<Node> parent;

    Node(float x, float y, double g_cost = 0, double h_cost = 0, std::shared_ptr<Node> parent = nullptr)
        : x(x), y(y), g_cost(g_cost), h_cost(h_cost), f_cost(g_cost + h_cost), parent(std::move(parent)) {}

    bool operator>(const Node& other) const { return f_cost > other.f_cost; }
    bool operator==(const Node& other) const { return (x == other.x) && (y == other.y); }
};

struct comparenode {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->f_cost > b->f_cost;
    }
};

double heuristic_astar(int x1, int y1, int x2, int y2);
std::vector<Node> astarsparse(const std::unordered_map<std::pair<int, int>, CellCost, pair_hash>& occupancyGrid, Node start, Node goal);

// From astarquadtree.h
struct NodeHasher {
    size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

double heuristic(float x1, float y1, float x2, float y2);
std::vector<Node> reconstruct_path(Node* current);
int roundToGrid(float coord, float resolution);
int getCostAtPoint(Point p, QuadtreeNode* low, QuadtreeNode* mid, QuadtreeNode* high);
std::vector<Node> astarquad(QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree, Node start, Node goal, float resolution = 1.0f);

#endif

