#ifndef ASTARQUADTREE_H
#define ASTARQUADTREE_H

#include <unordered_set>
#include <set>
#include <unordered_map>
#include <functional>
#include "quadtree.h"

using namespace std;

struct Node {
    float x, y;
    double g, h;
    Node* parent;
    Node(float _x, float _y, double _g = 0, double _h = 0, Node* _parent = nullptr)
        : x(_x), y(_y), g(_g), h(_h), parent(_parent) {}
    double f() const { return g + h; }
    bool operator==(const Node& other) const {
        return abs(x - other.x) < 1e-3 && abs(y - other.y) < 1e-3;
    }
};

struct NodeHasher {
    size_t operator()(const pair<int, int>& p) const {
        return hash<int>()(p.first) ^ hash<int>()(p.second << 1);
    }
};

double heuristic(float x1, float y1, float x2, float y2);
vector<Node> reconstruct_path(Node* current);
int roundToGrid(float coord, float resolution); 
int getCostAtPoint(Point p, QuadtreeNode* low, QuadtreeNode* mid, QuadtreeNode* high);
vector<Node> astarquad(QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree, Node start, Node goal, float resolution = 1.0f);

 

#endif //ASTARQUADTREE_H
