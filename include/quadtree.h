#ifndef QUADTREE_H
#define QUADTREE_H

#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include "mapping.h" // Includes Point and Pose definitions

namespace quadtree{
class QuadtreeNode {
public:
    // --- Member Variables ---
    mapping::Point center;
    float size;
    bool isLeaf;
    bool hasObstacle;
    int cost;
    int pointCount;
    QuadtreeNode* children[4];

    // --- Constructor & Destructor ---
    QuadtreeNode(mapping::Point c, float s, int co);
    ~QuadtreeNode();

    // --- Core Quadtree Logic ---
    void insert(mapping::Point p);
    void setObstaclesBasedOnDensity(int threshold);
    void assignCostToObstacles(int assignedCost);
    int getCostAtPoint(mapping::Point p) const;
    void collectObstaclePoints(std::vector<mapping::Point>& obstacles) const;
    bool containsPoint(const Eigen::Vector3f& point) const;
    bool inBounds(mapping::Point p) const;
    void subdivide();
    void clear();
    bool isObstacleAtPoint(const Eigen::Vector3f& point) const;
    void collectObstaclePointsWithColor(std::vector<mapping::Point>& points, 
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
    const mapping::Slam_Pose& slam_pose,
    float height=1.5f
);

// Visualizes the three quadtree layers in Rerun
void rerunvisualisation(
    QuadtreeNode* lowQuadtree,
    QuadtreeNode* midQuadtree,
    QuadtreeNode* highQuadtree,
    rerun::RecordingStream& rec
);
}
#endif // QUADTREE_H
