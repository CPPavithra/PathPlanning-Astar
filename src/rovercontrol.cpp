#include "rovercontrol.h"
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

RoverControl::RoverControl(rerun::RecordingStream& rec) : 
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
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1);
    pipe.start(cfg);

    // Initialize rover pose
    rover_pose.position = Eigen::Vector3f(0, 0, 0);
    rover_pose.orientation = Eigen::Matrix3f::Identity();
    rover_pose.velocity = Eigen::Vector3f(0, 0, 0);

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

RoverControl::~RoverControl() {
    // Clean up dynamically allocated memory to prevent leaks
    delete lowQuadtree;
    delete midQuadtree;
    delete highQuadtree;
    lowQuadtree = midQuadtree = highQuadtree = nullptr;
}

//Function to initialize everything and Setting up the variables
void RoverControl::setup() {
    log_views();
    rover_pose.position = Eigen::Vector3f(0, 0, 0);
    rover_pose.orientation = Eigen::Matrix3f::Identity();
    rover_pose.velocity = Eigen::Vector3f(0, 0, 0);
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
bool RoverControl::runMapping() {
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    float delta_time = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;
    rs2::frameset frameset;
    try {
        frameset = pipe.wait_for_frames();
    } catch (const rs2::error& e) {
        cerr<<"RealSense error: "<<e.what()<<endl;
        return false;
    }

    //Get accelerometer data
    rs2_vector accel_data = {0.0f, 0.0f, 0.0f};
    if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL)) {
        accel_data = accel_frame.get_motion_data();
    } else {
        cerr << "Failed to retrieve accelerometer data" << endl;
    }

    //Get gyroscope data
     rs2_vector gyro_data = {0.0f, 0.0f, 0.0f};
    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO)) {
        gyro_data = gyro_frame.get_motion_data();
    } else {
        cerr << "Failed to retrieve gyroscope data" << endl;
    }
    update_rover_pose(rover_pose, convert_to_eigen_vector(accel_data), convert_to_eigen_vector(gyro_data), delta_time);

    //POINT CLOUD PROCESSING
    rs2::depth_frame depth_frame = frameset.get_depth_frame();
    rs2::points points = pc.calculate(depth_frame);
    log_camera_frames(frameset);
    vector<Eigen::Vector3f> point_vectors;
    for (size_t i = 0; i < points.size(); ++i) {
        auto point = points.get_vertices()[i];
        if (point.z) {
            Eigen::Vector3f transformed_point = rover_pose.orientation * Eigen::Vector3f(point.x, point.y, point.z) + rover_pose.position;
            point_vectors.push_back(transformed_point);
        }
    }

    //point cloud->PCL->FILTER->conver back
    auto pcl_cloud = convert_to_pcl(point_vectors);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcl_cloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(0.5, 5.0);
    passthrough.filter(*passthrough_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(passthrough_cloud);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f);
    voxel.filter(*filtered_cloud);

    point_vectors.clear();
    for (const auto& point : filtered_cloud->points) {
        point_vectors.emplace_back(point.x, point.y, point.z);
    }

    create_gridmap(gridmap, point_vectors, rover_pose, grid_resolution);
    updateQuadtreesWithPointCloud(lowQuadtree, midQuadtree, highQuadtree, point_vectors, rover_pose);
    //rerunvisualisation(lowQuadtree, midQuadtree, highQuadtree, rec);
    if (gridmap.occupancy_grid.size() >= batch_threshold) {
        draw_gridmap(gridmap,rover_pose,grid_resolution, rec);
        batch_threshold += gridmap.occupancy_grid.size();
    }
    counter = gridmap.occupancy_grid.size() - adder;
    if (counter >= limit) {
        cout <<"Mapping paused. Switching to path planning." << std::endl;
        pathplanning_flag = true;
        adder = 20;//this is just to add to the batch threshold calculation during every calculation
    }

    return true;
}

//ENTIRE WORKFLOW FOR PATH PLANNING
void RoverControl::runPathPlanning() {
    const int MAX_RETRIES = 5;
    int retry_attempts = 0;

    while (pathplanning_flag) {
        // --- 1. SETUP FOR CURRENT ITERATION ---
        visited_nodes.insert({current_start.x, current_start.y});
        Node current_goal = findcurrentgoal();
        if (!pathplanning_flag) break; // Exit if findcurrentgoal stops the process

        std::cout << "Current Start: (" << current_start.x << "," << current_start.y << ")" << std::endl;
        std::cout << "Selected Intermediate Goal: (" << current_goal.x << "," << current_goal.y << ")" << std::endl;

        // --- 2. HIERARCHICAL PATHFINDING ---
        std::vector<Node> dense_path;
        std::vector<Node> sparse_path = astarsparse(gridmap, current_start, current_goal);

        if (sparse_path.empty()) {
            // Sparse A* failed, fall back to Dense A* for the whole segment
            std::cout << "Sparse A* failed. Attempting Dense A*..." << std::endl;
            dense_path = astarquad(lowQuadtree, midQuadtree, highQuadtree, current_start, current_goal, 1.0f);
            if (dense_path.empty()) {
            std::cout << "Dense A* also failed. Marking goal as failed.\n";
            failed_goals.insert({current_goal.x, current_goal.y});
            retry_attempts++;
            if (retry_attempts >= MAX_RETRIES) {
                std::cout << "Too many failed attempts. Aborting path planning.\n";
                pathplanning_flag = false;
            }
            continue;
        }
        } else {
            // Sparse A* succeeded, refine its path with Dense A*
            std::cout << "Sparse A* succeeded. Refining path with Dense A*..." << std::endl;
            dense_path.push_back(sparse_path[0]); // Start with the first node
            for (size_t i = 1; i < sparse_path.size(); ++i) {
                std::vector<Node> segment = astarquad(lowQuadtree, midQuadtree, highQuadtree,
                                                      sparse_path[i - 1], sparse_path[i], 1.0f);
                if (!segment.empty()) {
                    // Skip the first node of the segment as it's the same as the last node of the previous segment
                    dense_path.insert(dense_path.end(), segment.begin() + 1, segment.end());
                } else {
                    std::cout << "Warning: Dense A* failed for a sparse segment. Path may be incomplete." << std::endl;
                }
            }
        }
            // Step 3: Prune duplicates
    std::vector<Node> pruned_path;
    if (!dense_path.empty()) pruned_path.push_back(dense_path[0]);
    for (int i = 1; i < dense_path.size(); ++i) {
        if (!(dense_path[i] == dense_path[i - 1])) {
            pruned_path.push_back(dense_path[i]);
        }
    }
     
 

        // Execute the pruned path segment by segment
        bool stuck = true;
        Node previous_start= current_start;
        bool found_start=false;
        for (size_t i = 0; i < pruned_path.size() - 1; ++i) {
        if (!found_start) {
            if (pruned_path[i] == current_start) {
                found_start = true;
            } else {
                continue; // keep skipping
             }
          }
            Node local_start = pruned_path[i];
            Node local_goal  = pruned_path[i + 1];
            
            //std::vector<Node> remaining_path(pruned_path.begin() + i, pruned_path.end());
            //moveRoverAlongPath(remaining_path); // Pass the rest of the path to the drive function

            
    std::cout << "Path segment:\n";
    std::cout << "(" << local_start.x << "," << local_start.y << ") -> ("
              << local_goal.x << "," << local_goal.y << ")\n";

    std::vector<Node> segment = {local_start, local_goal};
    moveRoverAlongPath(segment);
            
            // Log just the segment that was traversed for visualization
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
                // ArucoDetect();
                // sendfinalsignal();
                pathplanning_flag = false;
                break; // Exit the for loop
            }
        }

        // --- 4. POST-EXECUTION CHECKS ---
        if (pathplanning_flag && stuck) {
            std::cout << "Rover is stuck. Marking goal as failed." << std::endl;
            failed_goals.insert({current_goal.x, current_goal.y});
            retry_attempts++;
            if (retry_attempts >= MAX_RETRIES) {
                std::cout << "Too many retries. Aborting." << std::endl;
                pathplanning_flag = false;
            }
            continue; // Try the main while loop again
        }

        if (pathplanning_flag) {
            std::cout << "Intermediate goal reached. Planning next segment..." << std::endl;
        }
    } // End of while(pathplanning_flag)
}
// --- Helper Function Implementations ---

void RoverControl::log_views() {
    rec.log_static("grid_map", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("rgb_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("heat_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("rover_feedback", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("cost_table", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
}

void RoverControl::log_camera_frames(const rs2::frameset& frameset) {
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

void RoverControl::moveRoverAlongPath(const std::vector<Node>& path) {//Drive func with grid logic
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

Node RoverControl::findcurrentgoal() {
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
