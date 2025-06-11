#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <Eigen/Core>
//for downsampling and filtering
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <cstdlib>
#include <rerun/demo_utils.hpp>
#include <unordered_set>
#include <sstream>
#include "../include/rerun.h"
#include "quadtree.h"

using namespace rerun;

/*struct Point {
    float x, y;
};*/
/*class QuadtreeNode {
public:
    Point center;
    float size;
    bool isLeaf;
    bool hasObstacle;
    int cost;
    int pointCount = 0; 
    QuadtreeNode* children[4]; *///one child for each direction
    QuadtreeNode::QuadtreeNode(Point c, float s, int co) : center(c),size(s),isLeaf(true),hasObstacle(false),cost(co) {
        for (int i = 0; i < 4; i++) children[i] = nullptr;
    }
    void QuadtreeNode::subdivide() {
        float h=size/2;
        children[0] = new QuadtreeNode({center.x-h/2, center.y+h/2}, h, cost); //up-left->will be -h/2 from center in -x direction and +h/2 in +y direction
        children[1] = new QuadtreeNode({center.x+h/2, center.y+h/2}, h, cost); //up-right +h/2 in +x and +h/2 in +y
        children[2] = new QuadtreeNode({center.x-h/2, center.y-h/2}, h, cost); //down-left -h/2 in -x and -h/2 in -y
        children[3] = new QuadtreeNode({center.x+h/2, center.y-h/2}, h, cost); //down-right +h/2 in +x and -h/2 in -y
        isLeaf = false;
    }
    //minimum leaf size 0.5 is the map resolution
  void QuadtreeNode::insert(Point p) {
    if (!inBounds(p)) return;
    if (size <= 0.5) {
        pointCount++;
        return;
    }
    if (isLeaf) {
        subdivide();
    }
    for (int i = 0; i < 4; i++) {
        if (children[i]->inBounds(p)) {
            children[i]->insert(p);
            return;
        }
    }
}
    void QuadtreeNode::setObstaclesBasedOnDensity(int threshold) {
    if (isLeaf) {
        hasObstacle = (pointCount >= threshold);
        return;
    }
    for (int i = 0; i < 4; i++) {
        if (children[i]) {
            children[i]->setObstaclesBasedOnDensity(threshold);
        }
    }
    }
void QuadtreeNode::assignCostToObstacles(int assignedCost) {
    if (isLeaf) {
        if (hasObstacle) {
            cost = assignedCost;
        }
        return;
    }
    for (int i = 0; i < 4; i++) {
        if (children[i]) {
            children[i]->assignCostToObstacles(assignedCost);
        }
    }
}
int QuadtreeNode::getCostAtPoint(Point p) const {
    if (!inBounds(p)) return 10000;  // Very high cost for out-of-bounds

    if (isLeaf) {
        return cost;
    }

    for (int i = 0; i < 4; i++) {
        if (children[i] && children[i]->inBounds(p)) {
            return children[i]->getCostAtPoint(p);
        }
    }

    return cost;  // Fallback, though ideally it should never hit this
}
void QuadtreeNode::collectObstaclePoints(std::vector<Point>& obstacles) const {
    if (isLeaf) {
        if (hasObstacle) obstacles.push_back(center);
        return;
    }
    for (int i = 0; i < 4; ++i) {
        if (children[i]) {
            children[i]->collectObstaclePoints(obstacles);
        }
    }
}

bool QuadtreeNode::containsPoint(const Vector3f& point) const {
    return (point.x() >= center.x - size/2 && point.x() <= center.x + size/2 &&
            point.y() >= center.y - size/2 && point.y() <= center.y + size/2);
}

void updateQuadtreesWithPointCloud(
    QuadtreeNode* lowQuadtree,
    QuadtreeNode* midQuadtree,
    QuadtreeNode* highQuadtree,
    const std::vector<Vector3f>& point_vectors,
    const Pose& roverpose) {

    float rover_x = roverpose.position.x();
    float rover_y = roverpose.position.y();

    float theta = atan2(2.0f * (roverpose.orientation.w() * roverpose.orientation.z() +
                                roverpose.orientation.x() * roverpose.orientation.y()),
                        1.0f - 2.0f * (roverpose.orientation.y() * roverpose.orientation.y() +
                                       roverpose.orientation.z() * roverpose.orientation.z()));
    float ned_theta = -(theta - M_PI_2);

    for (const auto& point : point_vectors) {
        float dx = point.z();      //forward
        float dy = -point.x();     //x becomes -y
        float height = point.y();  //y=height

        float rotated_x = cos(ned_theta) * dx - sin(ned_theta) * dy;
        float rotated_y = sin(ned_theta) * dx + cos(ned_theta) * dy;

        Point p = {rover_x + rotated_x, rover_y + rotated_y};

        if (height < 0.2f) {
            lowQuadtree->insert(p);
        } else if (height < 0.6f) {
            midQuadtree->insert(p);
        } else {
            highQuadtree->insert(p);
        }
    }

    int densityThreshold = 30;//pass it as a parameter here
    lowQuadtree->setObstaclesBasedOnDensity(densityThreshold);
    midQuadtree->setObstaclesBasedOnDensity(densityThreshold);
    highQuadtree->setObstaclesBasedOnDensity(densityThreshold);
    lowQuadtree->assignCostToObstacles(2);
    midQuadtree->assignCostToObstacles(5);
    highQuadtree->assignCostToObstacles(10);
    }
    bool QuadtreeNode::inBounds(Point p) const{
        return (p.x >= center.x - size/2 && p.x <= center.x + size/2 &&
                p.y >= center.y - size/2 && p.y <= center.y + size/2);
    }
    void QuadtreeNode::clear() {
    for (int i = 0; i < 4; i++) {
        if (children[i]) {
            children[i]->clear();
            delete children[i];
            children[i] = nullptr;
        }
    }
    isLeaf = true;
    hasObstacle = false;
    pointCount = 0;
}

// Fixed: Removed extra qualification 'QuadtreeNode::'
bool QuadtreeNode::isObstacleAtPoint(const Vector3f& point) const {
    if (!containsPoint(point)) return false;
    if (isLeaf) {
        return hasObstacle;
    }
    for (int i = 0; i < 4; i++) {
        if (children[i] && children[i]->containsPoint(point)) {
            return children[i]->isObstacleAtPoint(point);
        }
    }
    return false;
}


void QuadtreeNode::collectObstaclePointsWithColor(std::vector<Point>& points, std::vector<rerun::Color>& colors, rerun::Color color) const {
    if (isLeaf) {
        if (hasObstacle) {
            points.push_back(center);
            colors.push_back(color);
        }
        return;
    }
    for (int i = 0; i < 4; i++) {
        if (children[i]) {
            children[i]->collectObstaclePointsWithColor(points, colors, color);
        }
    }
}

void rerunvisualisation(QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree, rerun::RecordingStream& rec) {
    std::vector<Point> all_points;
    std::vector<rerun::Color> all_colors;

    lowQuadtree->collectObstaclePointsWithColor(all_points, all_colors, rerun::Color{0, 255, 0});
    midQuadtree->collectObstaclePointsWithColor(all_points, all_colors, rerun::Color{255, 255, 0});
    highQuadtree->collectObstaclePointsWithColor(all_points, all_colors, rerun::Color{255, 0, 0});

    std::vector<rerun::Position3D> positions;
    positions.reserve(all_points.size());

    for (const auto& p : all_points) {
        positions.push_back(rerun::Position3D{p.x, 0.0f, p.y});  // Adjust height if needed
    }

    // Fixed: Use move semantics properly with rerun Points3D
    auto points3d = rerun::Points3D(positions).with_colors(all_colors);
    // Optional: points3d = std::move(points3d).with_radii(std::vector<float>(positions.size(), 0.5f));
    rec.log("obstacles", points3d);
}
