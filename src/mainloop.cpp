#include "mainloop.h"
#include <iostream>
#include <vector>
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

Gridmap gridmap;
float grid_resolution = 0.001f;
int batch_threshold = 1;
int startx, starty, goalx, goaly;
std::set<std::pair<int, int>> visited_nodes;
std::set<std::pair<int, int>> failed_goals;
Pose rover_pose;
int limit = 20;
int last_index = 0;
int prev_dir = 0;
std::deque<Node> recent_goals;
int dir = 0;

// Initialize Node objects here to solve the constructor error
Node start(0, 0), goal(0, 0), current_start(0, 0), final_goal(0, 0);
vector<rerun::Position3D> full_path_points;
set<std::pair<int, int>> tried_goals;
vector<Node> full_path;

std::string table_text =
    "Color   | Signifies      | Cost\n\n"
    "--------|----------------|-------------\n\n"
    "Blue     | PATH           | 0 - Free\n\n"
    "--------|----------------|-------------\n\n"
    "Black    | Tall obstacles | 10\n\n"
    "--------|----------------|-------------\n\n"
    "Red      | Mid-obstacles  | 5\n\n"
    "--------|----------------|-------------\n\n"
    "Yellow   | Easy obstacles | 1\n\n";

// --- Function Implementations follow ---

// Make sure your setup function implementation matches the new prototype
void setup(rerun::RecordingStream &rec) {
    log_views(rec);

    // Initialize rover pose
    rover_pose.position = Eigen::Vector3f(0, 0, 0);
    rover_pose.orientation = Eigen::Matrix3f::Identity();
    rover_pose.velocity = Eigen::Vector3f(0, 0, 0);

    rec.log("cost_table", rerun::archetypes::TextDocument(table_text));

    // Path planning-setting the starting node
    cout <<"Setting boundaries...\n";
    cout <<"Enter goal coordinates (x y): ";
    cin >>goalx>>goaly;
    cout <<"\nGoal: ("<<goalx<<", "<<goaly<< ")";
    startx = 0;
    starty = 0;

    //Create start and goal nodes
    start =Node(startx, starty);
    goal =Node(goalx, goaly);
    current_start = start;
    final_goal = goal;
}
//Forward declaration for a helper function used in setup
void log_views(rerun::RecordingStream& rec);

//the entire workflow and function for MAPPING.
bool mapping(
    rs2::pipeline &pipe,
    rs2::pointcloud &pc,
    Pose &rover_pose,
    Gridmap &gridmap,
    QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree,
    float &grid_resolution,
    int &batch_threshold,
    int &counter,
    int &adder,
    int limit,
    rerun::RecordingStream &rec,
    bool &pathplanning_flag
) {
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    float delta_time = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;
    //this holds the sensor data- we are going to use SLAM instead of this
    rs2_vector accel_data = {0.0f, 0.0f, 0.0f};
    rs2_vector gyro_data = {0.0f, 0.0f, 0.0f};

    rs2::frameset frameset;
    try {
        frameset = pipe.wait_for_frames();
    } catch (const rs2::error& e) {
        cerr<<"RealSense error: "<<e.what()<<endl;
        return false;
    }

    //Get accelerometer data
    if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL)) {
        accel_data = accel_frame.get_motion_data();
    } else {
        cerr << "Failed to retrieve accelerometer data" << endl;
    }

    //Get gyroscope data
    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO)) {
        gyro_data = gyro_frame.get_motion_data();
    } else {
        cerr << "Failed to retrieve gyroscope data" << endl;
    }

    //Update rover pose- TAKE FROM SLAM LATER
    Eigen::Vector3f accel_eigen = convert_to_eigen_vector(accel_data);
    Eigen::Vector3f gyro_eigen = convert_to_eigen_vector(gyro_data);
    update_rover_pose(rover_pose, accel_eigen, gyro_eigen, delta_time);

    //Process depth data to create point cloud
    rs2::depth_frame depth_frame = frameset.get_depth_frame();
    rs2::points points = pc.calculate(depth_frame);

    log_camera_frames(rec, frameset);
  

    //Collect and transform point cloud data to Eigen
    vector<Eigen::Vector3f> point_vectors;
    for (size_t i = 0; i < points.size(); ++i) {
        auto point = points.get_vertices()[i];
        if (point.z) {
            Eigen::Vector3f transformed_point = rover_pose.orientation * Eigen::Vector3f(point.x, point.y, point.z) + rover_pose.position;
            point_vectors.push_back(transformed_point);
        }
    }

    //Convert to PCL and filter out the noise using passthrough and voxel
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
    //we are just not LOGGING the quadtree cuz it looks messy. We are making it.
    if (gridmap.occupancy_grid.size() >= batch_threshold) {
        draw_gridmap(gridmap, point_vectors, rover_pose, grid_resolution, rec);
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
void pathPlanning(
    Gridmap &gridmap,
    QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree,
    Node &current_start,
    Node &final_goal,
    vector<Node> &full_path,
    set<std::pair<int, int>> &visited_nodes,
    set<std::pair<int, int>> &failed_goals,
    deque<Node> &recent_goals,
    bool &pathplanning_flag,
    rerun::RecordingStream &rec
) {
    const int MAX_RETRIES = 5;//THIS IS JUST FOR A RETRY LOGIC, CAN AVOID THIS LATER
    int retry_attempts = 0;

    while (pathplanning_flag) {
        visited_nodes.insert({current_start.x, current_start.y});
        //WE FIRST CHOOSE OUR GOAL WITH THIS FUNC.
        Node current_goal = findcurrentgoal(gridmap, current_start, final_goal, visited_nodes, failed_goals, recent_goals, pathplanning_flag);
        if (!pathplanning_flag) break;

        cout << "Selected intermediate goal: ("<< current_goal.x<<","<<current_goal.y << ")" << std::endl;

        vector<Node> sparse_path = astarsparse(gridmap.occupancy_grid, current_start, current_goal);
        vector<Node> dense_path;
        //FIRST FIND SPARSE PATH, IF SPARSE FAILS FIND DENSE BUT IN FIND CURRENT GOAL WE RUN IT ACROSS THE DENSE ONE
        if (sparse_path.empty()) {
            cout <<"Sparse A* failed. Attempting Dense A*...\n";
            dense_path = astarquad(lowQuadtree, midQuadtree, highQuadtree, current_start, current_goal, 1.0f);
        } else {
            for (size_t i = 1; i < sparse_path.size(); ++i) {
                vector<Node> segment = astarquad(lowQuadtree, midQuadtree, highQuadtree, sparse_path[i - 1], sparse_path[i], 1.0f);
                if (!segment.empty()) {
                    dense_path.insert(dense_path.end(), segment.begin(), segment.end());
                } else {
                    cout << "Dense segment failed between sparse nodes. Skipping segment.\n";
                }
            }
        }

        if (dense_path.empty()) {
            cout << "Pathfinding failed for this goal. Marking as failed.\n";
            failed_goals.insert({current_goal.x, current_goal.y});
            retry_attempts++;
            if (retry_attempts >= MAX_RETRIES) {
                cout << "Too many failed attempts. Aborting path planning.\n";
                pathplanning_flag = false;
            }
            continue;
        }

        // Prune duplicates- THIS IS TO AVOID TAKING THE LAST ONE OF THE PREVIOUS ITERATION
        // eg- (0,0),(0,1) IN FIRST ITERATION. In the second one it should not include (0,1)
        std::vector<Node> pruned_path;
        if (!dense_path.empty()) {
            pruned_path.push_back(dense_path[0]);
            for (size_t i = 1; i < dense_path.size(); ++i) {
                if (!(dense_path[i] == dense_path[i - 1])) {
                    pruned_path.push_back(dense_path[i]);
                }
            }
        }

        //Execute path
        bool stuck = true;
        for (size_t i = 0; i < pruned_path.size() - 1; ++i) {
            Node local_start = pruned_path[i];
            Node local_goal  = pruned_path[i + 1];
            std::vector<Node> segment = {local_start, local_goal};
            
            moveRoverAlongPath(segment);//DRIVE FUNC TO MOVE ROVER

            std::vector<rerun::Position3D> subpath;
            for (const Node& node : segment) {
                subpath.push_back(rerun::Position3D{(float)node.x, (float)node.y, 0.0f});
                if (full_path.empty() || !(full_path.back() == node)) {
                    full_path.push_back(node);
                }
                visited_nodes.insert({node.x, node.y});
            }

            rec.log("full_path", rerun::Points3D(subpath)
                                     .with_colors({rerun::Color(0, 0, 255)})
                                     .with_radii({0.5f}));
            
            current_start = local_goal;
            stuck = false; //If we moved at all, we are not stuck

            if (current_start == final_goal) {
                std::cout << "GOAL REACHED!\n";
                ArucoDetect();
                sendfinalsignal();
                pathplanning_flag = false;
                break;
            }
        }
        
        if (pathplanning_flag && stuck) {
            std::cout << "Rover stuck. Retrying with a different goal.\n";
            failed_goals.insert({current_goal.x, current_goal.y});
            retry_attempts++;
            if (retry_attempts >= MAX_RETRIES) {
                std::cout << "Too many retries. Aborting planning.\n";
                pathplanning_flag = false;
            }
            continue;
        }

        if (pathplanning_flag) {
            std::cout << "Intermediate goal reached. Continuing planning...\n";
        }
    }
}


// --- Helper Function Implementations ---

void log_views(rerun::RecordingStream& rec) {
    rec.log_static("grid_map", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("rgb_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("heat_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("rover_feedback", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
    rec.log_static("cost_table", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);
}

void log_camera_frames(rerun::RecordingStream& rec, const rs2::frameset& frameset) {
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

void moveRoverAlongPath(const std::vector<Node>& path) {//Drive func with grid logic
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

Node findcurrentgoal(const Gridmap& gridmap, const Node& current_start, const Node& final_goal,
                     const std::set<std::pair<int, int>>& visited_nodes, const std::set<std::pair<int, int>>& failed_goals,
                     std::deque<Node>& recent_goals, bool& pathplanning_flag) {
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
