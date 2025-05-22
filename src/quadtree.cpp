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
#include "common.h"

struct Point {
    float x, y;
};
class QuadtreeNode {
public:
    Point center;
    float size;
    bool isLeaf;
    bool hasObstacle;
    QuadtreeNode* children[4]; //one child for each direction
    QuadtreeNode(Point c, float s) : center(c),size(s),isLeaf(true),hasObstacle(false) {
        for (int i = 0; i < 4; i++) children[i] = nullptr;
    }
    void subdivide() {
        float h=size/2;
        children[0] = new QuadtreeNode({center.x-h/2, center.y+h/2}, h); //up-left->will be -h/2 from center in -x direction and +h/2 in +y direction
        children[1] = new QuadtreeNode({center.x+h/2, center.y+h/2}, h); //up-right +h/2 in +x and +h/2 in +y
        children[2] = new QuadtreeNode({center.x-h/2, center.y-h/2}, h); //down-left -h/2 in -x and -h/2 in -y
        children[3] = new QuadtreeNode({center.x+h/2, center.y-h/2}, h); //down-right +h/2 in +x and -h/2 in -y
        isLeaf = false;
    }
  void insert(Point p) {
    int pointCount;
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
    void setObstaclesBasedOnDensity(int threshold) {
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
void updateQuadtreeWithPointCloud(QuadtreeNode* root, const std::vector<Vector3f>& point_vectors, const Pose& roverpose) {
    float rover_x = roverpose.position.x();
    float rover_y = roverpose.position.y();
    float theta = atan2(2.0f * (roverpose.orientation.w() * roverpose.orientation.z() + roverpose.orientation.x() * roverpose.orientation.y()),
                        1.0f - 2.0f * (roverpose.orientation.y() * roverpose.orientation.y() + roverpose.orientation.z() * roverpose.orientation.z()));
    float ned_theta = -(theta - M_PI_2);

    for (const auto& point : point_vectors) {
        float dx = point.z();  // Forward motion
        float dy = -point.x(); // X becomes -Y
        float rotated_x = cos(ned_theta) * dx - sin(ned_theta) * dy;
        float rotated_y = sin(ned_theta) * dx + cos(ned_theta) * dy;

        Point p = {rover_x + rotated_x, rover_y + rotated_y};
        root->insert(p);
    }

    int densityThreshold = 30; // Define your own threshold
    root->setObstaclesBasedOnDensity(densityThreshold);
}

    bool inBounds(Point p) {
        return (p.x >= center.x - size/2 && p.x <= center.x + size/2 &&
                p.y >= center.y - size/2 && p.y <= center.y + size/2);
    }
    void clear() {
        for (int i = 0; i < 4; i++) {
            if (children[i]) {
                children[i]->clear();
                delete children[i];
                children[i] = nullptr;
            }
        }
        isLeaf = true;
        hasObstacle = false;
    }
}
};

