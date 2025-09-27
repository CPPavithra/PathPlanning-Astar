#include "motionplanner.h"
#include <cwchar>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;
using namespace mapping;
using namespace planning;

bool MotionPlanner::getSensorData(rs2::frameset& frameset, rs2_vector& accel_raw, rs2_vector& gyro_raw, float& dt) {
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    dt = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;
    try {
        frameset = pipe.wait_for_frames();
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return false;
    }

    // Get raw accelerometer data
    if (auto accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL).as<rs2::motion_frame>()) {
        accel_raw = accel_frame.get_motion_data();
    } else {
        std::cerr << "Warning: Failed to get accelerometer data." << std::endl;
    }
    // Get raw gyroscope data
    if (auto gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO).as<rs2::motion_frame>()) {
        gyro_raw = gyro_frame.get_motion_data();
    } else {
        std::cerr << "Warning: Failed to get gyroscope data." << std::endl;
    }
    return true;
}


// Handles all point cloud generation and filtering.
std::vector<Eigen::Vector3f> MotionPlanner::processPointCloud(const rs2::frameset& frameset,const Slam_Pose& slam_pose) {
    rs2::points points = pc.calculate(frameset.get_depth_frame());
    std::vector<Eigen::Vector3f> raw_points;
    raw_points.reserve(points.size());
    const rs2::vertex* vertices = points.get_vertices();
    for (size_t i = 0; i < points.size(); ++i) {
        if (vertices[i].z) {
            // Transform point from camera frame to world frame
            raw_points.push_back(slam_pose.yaw * Eigen::Vector3f(vertices[i].x, vertices[i].y, vertices[i].z));
        }
    }
    // Convert to PCL, filter, and convert back
    auto pcl_cloud = convert_to_pcl(raw_points);

    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcl_cloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(0.5, 5.0);
    passthrough.filter(*pcl_cloud); // Filter in-place for efficiency

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(pcl_cloud);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel.filter(*pcl_cloud); // Filter in-place again

    // Convert back to Eigen vectors
    std::vector<Eigen::Vector3f> filtered_points;
    filtered_points.reserve(pcl_cloud->points.size());
    for (const auto& point : pcl_cloud->points) {
        filtered_points.emplace_back(point.x, point.y, point.z);
    }
    return filtered_points;
}


// Updates the gridmap and quadtrees.
void MotionPlanner::updateMaps(const std::vector<Eigen::Vector3f>& points) {
    create_gridmap(gridmap,  points, slam_pose, grid_resolution);
    updateQuadtreesWithPointCloud(lowQuadtree, midQuadtree, highQuadtree, points, slam_pose);
    if (gridmap.occupancy_grid.size() >= batch_threshold) {
        draw_gridmap(gridmap, grid_resolution, rec, slam_pose);
        batch_threshold += gridmap.occupancy_grid.size();
    }
}


// Checks the condition to switch from mapping to path planning.
bool MotionPlanner::checkPlanningTrigger() {
    counter = gridmap.occupancy_grid.size() - adder;
    if (counter >= limit) {
        std::cout << "Mapping paused. Switching to path planning." << std::endl;
        pathplanning_flag = true;
        adder += limit; // Increment adder so the condition isn't met again immediately
    }
  return pathplanning_flag;
}

bool MotionPlanner::findpath(const planning::Node& current_start, const planning::Node& current_goal, std::vector<planning::Node>& dense_path) {
    cout<<"Current Start: ("<<current_start.x<< "," <<current_start.y <<")" <<endl;
    cout<<"Selected Intermediate Goal: (" <<current_goal.x << "," <<current_goal.y <<")" <<endl;

    vector<Node> sparse_path = astarsparse(gridmap, current_start, current_goal);

    if (sparse_path.empty()) {
            std::cout << "Sparse A* failed. Attempting Dense A*..." << std::endl;
            dense_path = astarquad(lowQuadtree, midQuadtree, highQuadtree, current_start, current_goal, 1.0f);
    } else {
        cout << "Sparse A* succeeded. Refining path with Dense A*..." << endl;
        dense_path.push_back(sparse_path[0]); // Start with the first node
        for (size_t i = 1; i < sparse_path.size(); ++i) {
            std::vector<Node> segment=astarquad(lowQuadtree, midQuadtree, highQuadtree, sparse_path[i - 1],sparse_path[i], 1.0f);
            if (!segment.empty()) {
                dense_path.insert(dense_path.end(), segment.begin() + 1, segment.end());
            } else {
                std::cout << "Warning: Dense A* failed for a sparse segment. Path may be incomplete." << std::endl;
            }
        }
    }
    return !dense_path.empty();
}

std::vector<Node> MotionPlanner::prunepath(const std::vector<Node>& path) {
    if (path.empty()) {
        return {};
    }
    std::vector<Node> pruned_path;
    pruned_path.push_back(path[0]);
    for (size_t i = 1; i < path.size(); ++i) {
        if (!(path[i] == path[i - 1])) {
            pruned_path.push_back(path[i]);
        }
    }
    return pruned_path;
}

void MotionPlanner::executepath(const std::vector<Node>& path, bool& stuck) {
    stuck = true;
    if (path.size() < 2) return;
    Node previous_start=current_start;
    bool found_start=false;
    for (size_t i = 0; i < path.size() - 1; ++i) {
            if (!found_start) {
            if (path[i] == current_start) {
                found_start = true;
            } 
            else {
                continue; // keep skipping
             }
             }
        Node local_start = path[i];
        Node local_goal  = path[i + 1];
        // Ensure we only start executing from where the rover currently is
        if (local_start.x != current_start.x || local_start.y != current_start.y) {
            continue;
        }
        cout << "Moving from (" << local_start.x << "," << local_start.y << ") to ("
                  << local_goal.x << "," << local_goal.y << ")" <<endl;

        vector<Node> segment = {local_start, local_goal};
        moveRoverAlongPath(segment); //DROVE FUNC

       std::vector<rerun::Position3D> subpath;
        for (const Node& node : segment) {
        subpath.push_back(rerun::Position3D{node.x, node.y, 0.0f});
        if (full_path.empty() || !(full_path.back().x == node.x && full_path.back().y == node.y)) {
            full_path.push_back(node);
        }
        visited_nodes.insert({node.x, node.y});
        }
        rec.log("full_path", rerun::Points3D(subpath)
                        .with_colors({rerun::Color(0, 0, 255)})
                        .with_radii({0.5f}));   
        current_start = local_goal;
        if (current_start.x != previous_start.x || current_start.y != previous_start.y) {
        stuck = false;
        }
       previous_start = current_start; 
       if (current_start == final_goal) {
            std::cout << "FINAL GOAL REACHED!" << std::endl;
            pathplanning_flag = false;
            break; // Exit movement loop
        }
    }
}

void MotionPlanner::if_stuck(const Node& failed_goal, int& retry_attempts, const int MAX_RETRIES, bool& pathplanning_flag, std::set<std::pair<int, int>> &failed_goals){
    std::cout << "Rover is stuck or path failed. Marking goal as failed." << std::endl;
    failed_goals.insert({failed_goal.x, failed_goal.y});
    retry_attempts++;
    if (retry_attempts >= MAX_RETRIES) {
        std::cout << "Too many retries. Aborting." << std::endl;
        pathplanning_flag = false;
    }
}
