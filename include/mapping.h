#pragma once

// ==================== CORE DEPENDENCIES ====================
#include <vector>
#include <unordered_map>
#include <utility>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <rerun.hpp>  // External rerun SDK

#include "common.h"   // Defines Point, Gridmap, CellCost, pair_hash

// ==================== POSE STRUCT ====================
struct Pose {
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Quaternionf orientation;
};

// ==================== QUADTREE NODE ====================
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
    void collectObstaclePointsWithColor(
        std::vector<Point>& points,
        std::vector<rerun::Color>& colors,
        rerun::Color color
    ) const;
};

// ==================== GLOBAL QUADTREE VARIABLES ====================
extern Point center;
extern float rootSize;
extern QuadtreeNode* lowQuadtree;
extern QuadtreeNode* midQuadtree;
extern QuadtreeNode* highQuadtree;

// ==================== QUADTREE UPDATE & VISUALIZATION ====================
void updateQuadtreesWithPointCloud(
    QuadtreeNode* lowQuadtree,
    QuadtreeNode* midQuadtree,
    QuadtreeNode* highQuadtree,
    const std::vector<Eigen::Vector3f>& point_vectors,
    const Pose& roverpose
);

void rerunvisualisation(
    QuadtreeNode* lowQuadtree,
    QuadtreeNode* midQuadtree,
    QuadtreeNode* highQuadtree,
    rerun::RecordingStream& rec
);

// ==================== GLOBAL MAPPING VARIABLES ====================
extern Gridmap gridmap;
extern float grid_resolution;
extern int batch_threshold;

// ==================== MAPPING FUNCTIONS ====================
void create_gridmap(
    Gridmap& gridmap,
    const std::vector<Eigen::Vector3f>& points,
    const Pose& roverpose,
    float grid_resolution = 0.001f,
    float height = 1.5f,
    float proxfactor = 0.5f
);

rerun::components::Color get_color_for_cost(const CellCost& cell);

void draw_gridmap(
    const Gridmap& gridmap,
    const std::vector<Eigen::Vector3f>& point_vectors,
    const Pose& roverpose,
    float grid_resolution,
    rerun::RecordingStream& rec
);

void update_rover_pose(
    Pose& pose,
    const Eigen::Vector3f& accel_data,
    const Eigen::Vector3f& gyro_data,
    float delta_time
);

void log_navigation_pane(
    rerun::RecordingStream& rec,
    const Pose& roverpose
);

// ==================== CONVERSIONS ====================
Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec);

pcl::PointCloud<pcl::PointXYZ>::Ptr convert_to_pcl(
    const std::vector<Eigen::Vector3f>& point_vectors
);
