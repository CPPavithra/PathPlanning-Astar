#ifndef QUADTREE_H
#define QUADTREE_H

#include <vector>
#include "common.h"  // For Point
#include <cmath>     // For atan2, cos, sin, M_PI_2
#include <Eigen/Dense> // For Vector3f, assuming you're using Eigen

// Forward declaration if you use rerun
namespace rerun {
    class RecordingStream;
    struct Color;
    struct Position3D;
    class Points3D;
}

class QuadtreeNode {
public:
    Point center;
    float size;
    bool isLeaf;
    bool hasObstacle;
    int cost;
    int pointCount;
    QuadtreeNode* children[4];

    QuadtreeNode(Point c, float s, int co);
    ~QuadtreeNode();
    
    void subdivide();
    void insert(Point p);
    void setObstaclesBasedOnDensity(int threshold);
    void assignCostToObstacles(int assignedCost);
    void collectObstaclePoints(std::vector<Point>& obstacles) const;
    bool containsPoint(const Eigen::Vector3f& point) const;
    bool isObstacleAtPoint(const Eigen::Vector3f& point) const;
    bool inBounds(Point p) const;
    void clear();

    void collectObstaclePointsWithColor(std::vector<Point>& points, std::vector<rerun::Color>& colors, rerun::Color color) const;

    static void updateQuadtreesWithPointCloud(
        QuadtreeNode* lowQuadtree,
        QuadtreeNode* midQuadtree,
        QuadtreeNode* highQuadtree,
        const std::vector<Eigen::Vector3f>& point_vectors,
        const Pose& roverpose);

    static void rerunvisualisation(QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree, rerun::RecordingStream& rec);
};

#endif // QUADTREE_H

