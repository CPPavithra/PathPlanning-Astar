#ifndef QUADTREE_H
#define QUADTREE_H

#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <cmath>
#include "rerun.h"

using namespace rerun;

/*namespace rerun {
    class RecordingStream;
    class Points3D;
    class Color;
    class Position3D;
    struct Pose;
}*/
/*struct Point {
    float x, y;
*/

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
     ~QuadtreeNode() { clear(); }

    void subdivide();
    void insert(Point p);
    void setObstaclesBasedOnDensity(int threshold);
    void assignCostToObstacles(int assignedCost);
    int getCostAtPoint(Point p) const;
    void collectObstaclePoints(std::vector<Point>& obstacles) const;
    bool containsPoint(const Eigen::Vector3f& point) const;
    bool inBounds(Point p) const;
    void clear();
    bool isObstacleAtPoint(const Eigen::Vector3f& point) const;
    void collectObstaclePointsWithColor(std::vector<Point>& points, 
                                       std::vector<rerun::Color>& colors, 
                                       rerun::Color color) const;

  };
void updateQuadtreesWithPointCloud(
    QuadtreeNode *lowQuadtree,
    QuadtreeNode *midQuadtree,
    QuadtreeNode *highQuadtree,
    const std::vector<Eigen::Vector3f>& point_vectors,
    const Pose& roverpose);

void rerunvisualisation(QuadtreeNode *lowQuadtree, 
                       QuadtreeNode *midQuadtree, 
                       QuadtreeNode *highQuadtree, 
                       rerun::RecordingStream& rec);

#endif // QUADTREE_H
