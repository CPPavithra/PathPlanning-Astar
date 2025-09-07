#include "motionplanner.h"
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

MotionPlanner::MotionPlanner(rerun::RecordingStream& rec) : 
    rec(rec),
    start(0, 0),
    goal(0,0),
    final_goal(0, 0),
    current_start(0, 0)
{
    // Initialize RealSense pipeline
    rs2::config cfg;
    //Uncomment and use a bag file for testing if needed
    //cfg.enable_device_from_file("actualgoodvideo.bag");
    cfg.enable_stream(RS2_STREAM_DEPTH);
   // cfg.enable_stream(RS2_STREAM_GYRO);
   // cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1);
    pipe.start(cfg);

    // Initialize rover pose
       lowQuadtree = new QuadtreeNode(center, rootSize, 1);
    midQuadtree = new QuadtreeNode(center, rootSize, 1);
    highQuadtree = new QuadtreeNode(center, rootSize, 1);

    // Initialize cost table text
    cost_table_text =
        "Color   | Signifies      | Cost\n\n"
        "--------|----------------|-------------\n\n"
        "Blue    | PATH           | 0 - Free\n\n"
        "--------|----------------|-------------\n\n"
        "Black   | Tall obstacles | 10\n\n"
        "--------|----------------|-------------\n\n"
        "Red     | Mid-obstacles  | 5\n\n"
        "--------|----------------|-------------\n\n"
        "Yellow  | Easy obstacles | 1\n\n";
}

MotionPlanner::~MotionPlanner() {
    // Clean up dynamically allocated memory to prevent leaks
    delete lowQuadtree;
    delete midQuadtree;
    delete highQuadtree;
    lowQuadtree = midQuadtree = highQuadtree = nullptr;
}

//Function to initialize everything and Setting up the variables
void MotionPlanner::setup() {
    log_views();
    /*rover_pose.position = Eigen::Vector3f(0, 0, 0);
    rover_pose.orientation = Eigen::Matrix3f::Identity();
    rover_pose.velocity = Eigen::Vector3f(0, 0, 0);*/
    rec.log("cost_table", rerun::archetypes::TextDocument(cost_table_text));
    
    cout <<"Setting boundaries...\n";
    cout <<"Enter goal coordinates (x y): ";
    int goalx, goaly;
    cin >>goalx>>goaly;
    cout <<"\nGoal: ("<<goalx<<", "<<goaly<< ")";
    int startx = 0;
    int starty = 0;
    
    start =Node(startx, starty);
    goal =Node(goalx, goaly);
    current_start = start;
    final_goal = goal;
}

//Entire mapping workflow
bool MotionPlanner::runMapping() {
    rs2::frameset frameset;
    float delta_time;
    rs2_vector accel_raw = {0.0f, 0.0f, 0.0f};
    rs2_vector gyro_raw = {0.0f, 0.0f, 0.0f};
    if (!getSensorData(frameset, accel_raw, gyro_raw, delta_time)) {
        return false; // Exit if sensor data fails
    }
    // Convert Data and Update Pose
    // The conversion happens here, where it's needed.
    Eigen::Vector3f accel_eigen = convert_to_eigen_vector(accel_raw);
    Eigen::Vector3f gyro_eigen = convert_to_eigen_vector(gyro_raw);
    //NOT NEEDED update_rover_pose(rover_pose, accel_eigen, gyro_eigen, delta_time);

    log_camera_frames(frameset);
    std::vector<Eigen::Vector3f> filtered_points = processPointCloud(frameset);
    updateMaps(filtered_points);
    checkPlanningTrigger();
    return true;
}

//Entire mapping workflow
void MotionPlanner::runPathPlanning() {
   const int MAX_RETRIES = 5;
    int retry_attempts = 0;
    while (pathplanning_flag) {
        Node current_goal = findcurrentgoal();
        if (!pathplanning_flag) break;
        std::vector<Node> dense_path;
        if (findpath(current_start, current_goal, dense_path)) {
            vector<Node> pruned_path = prunepath(dense_path);
            bool stuck = false;
            executepath(pruned_path, stuck);
  
            if (pathplanning_flag && stuck) {
                if_stuck(current_goal, retry_attempts, MAX_RETRIES);
            } else if (pathplanning_flag) {
                std::cout << "Intermediate goal reached. Planning next segment..." << std::endl;
            }
        } else {
            if_stuck(current_goal, retry_attempts, MAX_RETRIES);
        }
    }
}
// --- Helper Function Implementations ---

void MotionPlanner::log_views() {
    rec.log_static("grid_map", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("rgb_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("heat_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("rover_feedback", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("cost_table", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
}

void MotionPlanner::log_camera_frames(const rs2::frameset& frameset) {
    auto color_frame = frameset.get_color_frame();
    auto ir_frame = frameset.get_infrared_frame();

    if (color_frame) {
        uint32_t width = color_frame.get_width();
        uint32_t height = color_frame.get_height();
        const uint8_t* data = static_cast<const uint8_t*>(color_frame.get_data());
        rec.log("rgb_camera", rerun::Image::from_rgb24(std::vector<uint8_t>(data, data + (width * height * 3)), {width, height}));
    }

    if (ir_frame) {
        uint32_t ir_width = ir_frame.get_width();
        uint32_t ir_height = ir_frame.get_height();
        const uint8_t* ir_data = static_cast<const uint8_t*>(ir_frame.get_data());
        rec.log("heat_camera", rerun::DepthImage(ir_data, {ir_width, ir_height}, rerun::datatypes::ChannelDatatype::U8));
    }
}

void MotionPlanner::moveRoverAlongPath(const std::vector<Node>& path) {//Drive func with grid logic
    if (path.size() < 2) return;

    for (size_t i = 1; i < path.size(); i++) {
        int dx = path[i].x - path[i - 1].x;
        int dy = path[i].y - path[i - 1].y;

        if (dx == 0 && dy == 0) continue;

        if (dx == 0 && dy > 0)       dir = 0;
        else if (dx > 0 && dy > 0)   dir = 1;
        else if (dx > 0 && dy == 0)  dir = 2;
        else if (dx > 0 && dy < 0)   dir = 3;
        else if (dx == 0 && dy < 0)  dir = 4;
        else if (dx < 0 && dy < 0)   dir = 5;
        else if (dx < 0 && dy == 0)  dir = 6;
        else if (dx < 0 && dy > 0)   dir = 7;

        float distance = sqrt(dx * dx + dy * dy) * grid_resolution; // Scale by resolution for real distance
        float time_needed = distance / 0.2; // Assuming 0.2 m/s

        Drive(dir, time_needed, prev_dir);
        prev_dir = dir; // Update previous direction
    }
}

Node MotionPlanner::findcurrentgoal() {
    Node best_node = current_start;
    double best_score = std::numeric_limits<double>::max();
    bool found = false;
    int margin = 6;//change the margin if wanted. It is better to pass it by reference.

    for (int x = current_start.x - margin; x <= current_start.x + margin; ++x) {
        for (int y = current_start.y; y <= current_start.y + margin; ++y) {
            std::pair<int, int> cell = {x, y};
            Node candidate(x, y);

            if (x < gridmap.min_x || x > gridmap.max_x || y < gridmap.min_y || y > gridmap.max_y) continue;
            if (gridmap.occupancy_grid.count(cell) || visited_nodes.count(cell) || failed_goals.count(cell)) continue;
            if (std::find(recent_goals.begin(), recent_goals.end(), candidate) != recent_goals.end()) continue;

            double angle_to_node = atan2(y - current_start.y, x - current_start.x);
            double angle_to_goal = atan2(final_goal.y - current_start.y, final_goal.x - current_start.x);
            double angle_diff = fabs(angle_to_node - angle_to_goal);
            if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
            if (angle_diff > M_PI/3) continue; //only the starting arc in front of the rover.

            double dist_from_start = heuristic(current_start.x, current_start.y, x, y);
            if (dist_from_start < 1.0) continue;

            double obstacle_cost = gridmap.occupancy_grid.count(cell) ? gridmap.occupancy_grid.at(cell).cost : 0.0;
            double alignment_penalty = angle_diff * 2.0;
            double score = dist_from_start + heuristic(x, y, final_goal.x, final_goal.y) + obstacle_cost + alignment_penalty;

            if (score < best_score) {
                best_score = score;
                best_node = candidate;
                found = true;
            }
        }
    }

    if (found) {
        auto dry_path = astarquad(lowQuadtree, midQuadtree, highQuadtree, current_start, best_node, 1.0f);
        if (!dry_path.empty()) {
            std::cout << "Best reachable intermediate goal: (" << best_node.x << "," << best_node.y << ")\n";
            recent_goals.push_back(best_node);
            if (recent_goals.size() > 10) recent_goals.pop_front();
            return best_node;
        }
    }

    std::cout << "No good intermediate goal found.\n";
    pathplanning_flag = false;
    return current_start;
}
