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
#include "quadtree.h"

using namespace rerun;
using namespace Eigen;
using namespace std;
using namespace mapping;


namespace quadtree {

mapping::Point center = {0.0f, 0.0f};
float rootSize = 100.0f;

/* QuadtreeNode* lowQuadtree  = new QuadtreeNode(center, rootSize, 1); */
/* QuadtreeNode* midQuadtree  = new QuadtreeNode(center, rootSize, 1); */
/* QuadtreeNode* highQuadtree = new QuadtreeNode(center, rootSize, 1); */
/**/ 

QuadtreeNode::QuadtreeNode(mapping::Point c, float s, int co) : center(c),size(s),isLeaf(true),hasObstacle(false),cost(co) {
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
void QuadtreeNode::insert(mapping::Point p) {
    if (!inBounds(p)) return;
    if (size <= 1.0) {
        pointCount++;
        return;
    }
    if (isLeaf) {
        subdivide();
    }
    for (int i = 0; i < 4; i++) {
        if (children[i] && children[i]->inBounds(p)) {
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

int QuadtreeNode::getCostAtPoint(mapping::Point p) const {
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

void QuadtreeNode::collectObstaclePoints(std::vector<mapping::Point>& obstacles) const {
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
    QuadtreeNode *lowQuadtree, 
    QuadtreeNode *midQuadtree, 
    QuadtreeNode *highQuadtree, 
    const std::vector<Vector3f>& point_vectors,
    const mapping::Slam_Pose& slam_pose,
    float height) 
{
  // Calculate orientation (same as hashmap version)
  // float ned_theta = -(theta - M_PI_2);
   //same logic as the one in grid map code
    float ground_sum =0.0f;
    int ground_count =0;
    float ground_window =5.0f; //5m radius around rover for ground estimation
    for (const auto& p : point_vectors) {
        float dx= p.x()-slam_pose.x;
        float dy= p.y()-slam_pose.y;
        float dist= std::sqrt(dx*dx+dy*dy);
        if (dist < ground_window && p.z()<height/4) {
            ground_sum+=p.z();
            ground_count++;
        }
    }
    //check!!
    float ground_level = (ground_count > 0) ? (ground_sum / ground_count) : 0.0f;

    for (const auto& point : point_vectors) {
        // Same coordinate transformation as hashmap

        float local_x = point.x()-slam_pose.x;
        float local_y = point.y()-slam_pose.y;
        float height = point.z()-ground_level; //check if ground_level needs to be included or not

        mapping::Point p = {local_x, local_y};  // Now in global coordinates

        // Use same height thresholds as hashmap cost assignments
        if (height < 0.2f) {
            lowQuadtree->insert(p);
        } 
        else if (height < 0.6f) {
            midQuadtree->insert(p);
        } 
        else {
            highQuadtree->insert(p);
        }
    }
    // Keep density-based obstacle detection
    int densityThreshold = 30;
    lowQuadtree->setObstaclesBasedOnDensity(densityThreshold);
    midQuadtree->setObstaclesBasedOnDensity(densityThreshold);
    highQuadtree->setObstaclesBasedOnDensity(densityThreshold);
    // Assign costs to match hashmap's cost structure
    lowQuadtree->assignCostToObstacles(1);   // Matches hashmap's 1.0f cost
    midQuadtree->assignCostToObstacles(5);   // Matches hashmap's 5.0f cost
    highQuadtree->assignCostToObstacles(10); // Matches hashmap's 10.0f cost
}

    bool QuadtreeNode::inBounds(mapping::Point p) const{
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


void QuadtreeNode::collectObstaclePointsWithColor(std::vector<mapping::Point>& points, std::vector<rerun::Color>& colors, rerun::Color color) const {
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

void rerunvisualisation( QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree, rerun::RecordingStream& rec) {
    std::vector<Point> low_points, mid_points, high_points;
    std::vector<rerun::Color> low_colors, mid_colors, high_colors;
    // Collect points separately for each quadtree
    lowQuadtree->collectObstaclePointsWithColor(low_points, low_colors, rerun::Color{0, 255, 0});   // Green
    midQuadtree->collectObstaclePointsWithColor(mid_points, mid_colors, rerun::Color{255, 255, 0}); // Yellow
    highQuadtree->collectObstaclePointsWithColor(high_points, high_colors, rerun::Color{255, 0, 0}); // Red

    // Convert to Position3D arrays
    auto convert_points = [](const std::vector<Point>& points) {
        std::vector<rerun::Position3D> result;
        result.reserve(points.size());
        for (const auto& p : points) {
            result.emplace_back(p.x, p.y, 0.0f);  // X-Z plane with Y=0
        }
        return result;
    };

    // Log each level separately with distinct colors
    rec.log("world/obstacles/low",
        rerun::Points3D(convert_points(low_points))
        .with_colors(low_colors)
        .with_radii({0.5f}));

    rec.log("world/obstacles/mid",
        rerun::Points3D(convert_points(mid_points))
        .with_colors(mid_colors)
        .with_radii({0.5f}));  // Slightly larger for mid-level

    rec.log("world/obstacles/high",
        rerun::Points3D(convert_points(high_points))
        .with_colors(high_colors)
        .with_radii({0.5f}));  // Largest for high obstacles
}
QuadtreeNode::~QuadtreeNode() {
    for (int i = 0; i < 4; ++i) {
        delete children[i]; // This calls the destructor for each child node
    }
}
}
