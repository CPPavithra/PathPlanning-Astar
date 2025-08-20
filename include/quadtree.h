#ifndef QUADTREE_H
#define QUADTREE_H

#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include "mapping.h" // Includes Point and Pose definitions

class QuadtreeNode {
public:
    // --- Member Variables ---
    Point center;
    float size;
    bool isLeaf;
    bool hasObstacle;
    int cost;
    int pointCount;
    QuadtreeNode* children[4];

    // --- Constructor & Destructor ---
    QuadtreeNode(Point c, float s, int co);
    ~QuadtreeNode();

    // --- Core Quadtree Logic ---
    void insert(Point p);
    void setObstaclesBasedOnDensity(int threshold);
    void assignCostToObstacles(int assignedCost);
    int getCostAtPoint(Point p) const;
    void collectObstaclePoints(std::vector<Point>& obstacles) const;
    bool containsPoint(const Eigen::Vector3f& point) const;
    bool inBounds(Point p) const;
    void subdivide();
    void clear();
    bool isObstacleAtPoint(const Eigen::Vector3f& point) const;
    void collectObstaclePointsWithColor(std::vector<Point>& points, 
                                       std::vector<rerun::Color>& colors, 
                                       rerun::Color color) const;

  
};

// --- Helper Functions ---

// Updates the three quadtree layers based on a new point cloud
void updateQuadtreesWithPointCloud(
    QuadtreeNode* lowQuadtree,
    QuadtreeNode* midQuadtree,
    QuadtreeNode* highQuadtree,
    const std::vector<Eigen::Vector3f>& point_vectors,
    const Pose& roverpose
);

// Visualizes the three quadtree layers in Rerun
void rerunvisualisation(
    QuadtreeNode* lowQuadtree,
    QuadtreeNode* midQuadtree,
    QuadtreeNode* highQuadtree,
    rerun::RecordingStream& rec
);

#endif // QUADTREE_H
