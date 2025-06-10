#include <iostream>
#include "astar.h"
#include <ostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include <Eigen/Core>
// For downsampling and filtering
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <cstdlib>
#include <rerun/demo_utils.hpp>
#include "rerun.h"
#include "common.h"  
#include "imu.h" 
#include <boost/asio.hpp> 
#include "ArucoDetect.h"
#include <set>

/***********************************************
 * DEFINING GLOBAL VARIABLES HERE*
 ***********************************************/

Gridmap gridmap;          // Define and initialize here
float grid_resolution = 0.001f; // Initialize with a value
int batch_threshold = 1;      // Initialize with a value
int startx, starty, goalx, goaly;
using namespace std;
Pose rover_pose;
bool input_ready = false;
int limit = 20;
int last_index = 0;
int prev_dir = 0;
int dir = 0;
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

/********************************************************
 * ******************************************************/

// Function to find the next checkpoint/goal
Node findcurrentgoal(const Gridmap& gridmap, const Node& current_start, const Node& final_goal, const std::set<std::pair<int, int>>& visited_nodes) {
    Node best_node = current_start;
    double min_total_cost = std::numeric_limits<double>::max();
    double current_to_goal_heuristic = heuristic(current_start.x, current_start.y, final_goal.x, final_goal.y);

    for (int x = gridmap.min_x; x <= gridmap.max_x; ++x) {
        for (int y = gridmap.min_y; y <= gridmap.max_y; ++y) {
            // Skip if already visited or occupied
            if (gridmap.occupancy_grid.find({x, y}) != gridmap.occupancy_grid.end() ||
                visited_nodes.find({x, y}) != visited_nodes.end()) {
                continue;
            }
            // Must be closer to goal than current position
            double new_to_goal = heuristic(x, y, final_goal.x, final_goal.y);
            if (new_to_goal >= current_to_goal_heuristic) {
                continue;
            }
            
            // Check directional constraint (optional - remove if not needed)
            double angle_to_node = atan2(y - current_start.y, x - current_start.x);
            double angle_to_goal = atan2(final_goal.y - current_start.y, final_goal.x - current_start.x);
            double angle_diff = fabs(angle_to_node - angle_to_goal);
            // Normalize angle_diff to [0, ¿]
            if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;

            if (angle_diff > (M_PI / 4)) { // > 45 degrees off
                continue;
            }

            double obstacle_cost = 0;
            if (gridmap.occupancy_grid.find({x, y}) != gridmap.occupancy_grid.end()) {
                obstacle_cost = gridmap.occupancy_grid.at({x, y}).cost;
            }

            double cost_to_node = heuristic(current_start.x, current_start.y, x, y) + obstacle_cost;
            double total_cost = cost_to_node + new_to_goal;

            // Penalize zero or backward motion
            if (x == current_start.x && y == current_start.y) {
                total_cost += 1000;
            }
            // Select lowest cost valid node
            if (total_cost < min_total_cost) {
                min_total_cost = total_cost;
                best_node = Node(x, y);
            }
        }
    }

    return best_node;
}

/*****************************************************************************
 * Functions to log views in the rerun viewer.
 * The Blueprint attribute is not being recognised so we have to manually adjust the screen view.
 * Add more in the future if needed.
 * Now it includes- 1. Grid map 
                    2. Cost table
                    3. Rover feedback
                    4. Stereo camera
                    5. Realsense color stream
  ****************************************************************************/

void log_views(rerun::RecordingStream& rec) {
    // Define spatial views
    rec.log_static("grid_map", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);  // Left 3/4th pane
    rec.log_static("rgb_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN); // Top-right
    rec.log_static("heat_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN); // Below RGB camera
    rec.log_static("rover_feedback", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN); // Bottom-right log
    rec.log_static("cost_table", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN); // Bottom-right navigation
}

void log_camera_frames(rerun::RecordingStream& rec, const rs2::frameset& frameset) {
    // Get color frame
    auto color_frame = frameset.get_color_frame();
    auto ir_frame = frameset.get_infrared_frame();  // Get IR frame from frameset

    // Get color frame properties
    uint32_t width = color_frame.get_width();
    uint32_t height = color_frame.get_height();
    const uint8_t* color_data = static_cast<const uint8_t*>(color_frame.get_data());

    // Log color (RGB) frame
    rec.log("rgb_camera",
            rerun::Image::from_rgb24(std::vector<uint8_t>(color_data, color_data + (width * height * 3)),  
                { width, height }));

    // Get IR frame properties
    uint32_t ir_width = ir_frame.get_width();
    uint32_t ir_height = ir_frame.get_height();
    const uint8_t* ir_data = static_cast<const uint8_t*>(ir_frame.get_data());

    // Stereo cam
    rec.log("heat_camera",
        rerun::DepthImage(ir_data, {ir_width, ir_height}, rerun::datatypes::ChannelDatatype::U8));
}

void log_rover_feedback(rerun::RecordingStream& rec) {
    rec.log("rover_feedback", rerun::TextLog("Rover movement tracking enabled."));
}

/******************************************************************************************
 * FUNCTION TO DETERMINE THE DIRECTION FOR THE ROVER TO MOVE
 * ****************************************************************************************/

/*void moveRoverAlongPath(const std::vector<Node>& path) {
    static size_t last_index = 0;
    if (path.size() < 2 || last_index >= path.size() - 1)
        return; // No movement needed
    
    for (size_t i = last_index + 1; i < path.size(); i++) {
        int goal_x = path[i].x;
        int goal_y = path[i].y;
        int prev_x = path[i-1].x;
        int prev_y = path[i-1].y;
        float goal_theta = atan2(goal_y - prev_y, goal_x - prev_x);
        move(goal_x, goal_y, goal_theta);
        last_index = i;
    }
}*/
void moveRoverAlongPath(const std::vector<Node>& path) {
    static size_t last_index = 0; // Track last moved index persistently

    if (path.size() < 2 || last_index >= path.size() - 1) return;  // No movement needed  

    for (size_t i = last_index + 1; i < path.size(); i++) {  
        int dx = path[i].x - path[i - 1].x;
        int dy = path[i].y - path[i - 1].y;

        if (dx == 0 && dy == 0) continue; // Skip redundant moves

        // Determine direction index (0-7 for 45-degree increments)
        //int dir = 0;
        if (dx == 0 && dy == 0) dir = 0;  // front
        else if (dx > 0 && dy > 0) dir = 1;  // top-right
        else if (dx > 0 && dy == 0) dir = 2;  // right
        else if (dx < 0 && dy > 0) dir = 7;  // Top-left
        else if (dx < 0 && dy == 0) dir = 6;  // Left

        if (dir == -1) continue; // Just in case, prevent an invalid move

        float distance = sqrt(dx * dx + dy * dy) * 1.0f; // Grid resolution = 1.0f
        float time_needed = distance / 0.2;  // Assuming speed is 0.2 units per second

        Drive(dir, time_needed, prev_dir); // Call to movement function in imu.cpp

        last_index = i; // Update last moved position
       // prev_dir=dir;

    }
}
/***********************************************************************************************/

int main() {
    auto rec = rerun::RecordingStream("gridmap");
    // auto ret = rec.connect_tcp("192.168.80.73");
    rec.spawn().exit_on_failure(); // This is for realsense viewer - can be avoided
    log_views(rec);
    
    rs2::pipeline pipe;
    rs2::config cfg;  
    // cfg.enable_device_from_file("actualgoodvideo.bag"); 
    
    // SERIAL CONNECTION
    // initSerial("/dev/ttyACM0", 9600);
    initSerial("/dev/serial/by-id/usb-ZEPHYR_Team_RUDRA_Tarzan_3339511100350023-if00", 9600);

    cfg.enable_stream(RS2_STREAM_DEPTH); 
    cfg.enable_stream(RS2_STREAM_GYRO);   
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2); // Right IR - Enable only if available
 
    pipe.start(cfg);

    rs2::pointcloud pc;
    rs2::points points;

    // Initialize rover pose
    rover_pose.position = Eigen::Vector3f(0, 0, 0);
    rover_pose.orientation = Eigen::Matrix3f::Identity();
    rover_pose.velocity = Eigen::Vector3f(0, 0, 0);
      
    // For time tracking
    auto last_time = std::chrono::high_resolution_clock::now();

    vector<Vector3f> point_vectors;

    static int frame_counter = 0;
    const int maxgrid = (3/grid_resolution) * (3/grid_resolution);
    bool pathplanning_flag = false;
    int counter = 0; // To check when path planning is to be called
    int adder = 0;

    rec.log("cost_table", rerun::archetypes::TextDocument(table_text));

    // Path planning - setting the starting node
    cout << "Setting boundaries...\n";
    cout << "Enter goal coordinates (x y): ";
    cin >> goalx >> goaly;
    cout << "\nGoal: (" << goalx << ", " << goaly << ")";
    int startx = 0;
    int starty = 0; // Take goal as a user input later
   
    // Create start and goal nodes
    Node start(startx, starty);
    Node goal(goalx, goaly);
    Node current_start = start;
    std::vector<rerun::Position3D> full_path_points;
    std::set<std::pair<int, int>> tried_goals;
    Node final_goal = goal;
    std::vector<Node> full_path;
    std::set<std::pair<int, int>> visited_nodes;

    while (gridmap.occupancy_grid.size() < maxgrid) {
        if (!pathplanning_flag) {
            auto current_time = std::chrono::high_resolution_clock::now();
            float delta_time = std::chrono::duration<float>(current_time - last_time).count();
            last_time = current_time;
            
            // Declare variables to hold sensor data
            rs2_vector accel_data = {0.0f, 0.0f, 0.0f};
            rs2_vector gyro_data = {0.0f, 0.0f, 0.0f};

            // Get frames from the RealSense camera
            rs2::frameset frameset;
            try {
                frameset = pipe.wait_for_frames();
            } catch (const rs2::error& e) {
                std::cerr << "RealSense error: " << e.what() << std::endl;
                continue;
            }

            // Get accelerometer data
            if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL)) {
                accel_data = accel_frame.get_motion_data();
            } else {
                cerr << "Failed to retrieve accelerometer data" << endl;
            }
            
            // Get gyroscope data
            if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO)) {
                gyro_data = gyro_frame.get_motion_data();
            } else {
                cerr << "Failed to retrieve gyroscope data" << endl;
            }
            
            // Convert accelerometer and gyroscope data to Eigen vectors
            Eigen::Vector3f accel_eigen = convert_to_eigen_vector(accel_data);
            Eigen::Vector3f gyro_eigen = convert_to_eigen_vector(gyro_data);
            
            // Update the rover pose with accelerometer and gyroscope data
            update_rover_pose(rover_pose, accel_eigen, gyro_eigen, delta_time);

            // Process depth data to create point cloud
            rs2::depth_frame depth_frame = frameset.get_depth_frame();
            points = pc.calculate(depth_frame);
            
            // Log the frameset
            log_camera_frames(rec, frameset);
            log_rover_feedback(rec);

            // Collect point cloud data
            point_vectors.clear();
            for (size_t i = 0; i < points.size(); ++i) {
                auto point = points.get_vertices()[i];
                if (point.z) {
                    Eigen::Vector3f transformed_point = rover_pose.orientation * Eigen::Vector3f(point.x, point.y, point.z) + rover_pose.position;
                    point_vectors.push_back(transformed_point);
                }
            }
            
            // Convert the realsense points to pcl point cloud
            auto pcl_cloud = convert_to_pcl(point_vectors);

            if (depth_frame) {
                static int processed_frames = 0;  // Tracks how many frames you process
                processed_frames++;
            }
            
            // Passthrough filter
            pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PassThrough<pcl::PointXYZ> passthrough;
            passthrough.setInputCloud(pcl_cloud);
            passthrough.setFilterFieldName("z");  // This is based on depth
            passthrough.setFilterLimits(0.5, 5.0); // Range in metres
            passthrough.filter(*passthrough_cloud);

            // Voxel grid filter
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(passthrough_cloud);
            voxel.setLeafSize(0.05f, 0.05f, 0.05f); // Size of each voxel in xyz dimension
            voxel.filter(*filtered_cloud);

            // Clear and rebuild point vectors
            std::vector<Eigen::Vector3f>().swap(point_vectors); // Forces memory deallocation
            std::vector<float> z_values;
            z_values.reserve(filtered_cloud->points.size());

            for (const auto& point : filtered_cloud->points) {
                point_vectors.emplace_back(point.x, point.y, point.z);
            }
            
            create_gridmap(gridmap, point_vectors, rover_pose, grid_resolution);

            if (gridmap.occupancy_grid.size() >= batch_threshold) {
                draw_gridmap(gridmap, point_vectors, rover_pose, grid_resolution, rec);
                batch_threshold = batch_threshold + gridmap.occupancy_grid.size();
            }

            counter = gridmap.occupancy_grid.size() - adder;
            if (counter >= limit) {
                cout << "Mapping paused. Switching to path planning." << std::endl;
                pathplanning_flag = true;    // Switching to path planning
                adder = 2;
            }
        } else {
            // Path planning phase
            while (pathplanning_flag) {
                visited_nodes.insert({current_start.x, current_start.y});
                std::vector<rerun::Position3D> subpath;
                
                // Select next goal using sparse hash map
                Node current_goal = findcurrentgoal(gridmap, current_start, final_goal, visited_nodes);
                std::cout << "Selected intermediate goal: (" << current_goal.x << "," << current_goal.y << ")" << std::endl;

                // Check if that goal is valid
                if (gridmap.occupancy_grid.find({current_goal.x, current_goal.y}) != gridmap.occupancy_grid.end()) {
                    std::cout << "Selected goal is occupied. Planning failed." << std::endl;
                    break;
                }

                // Run A* on gridmap (should be gridmap, not quadtree)
                std::vector<Node> path = astar(gridmap.occupancy_grid, current_start, current_goal);

                if (path.empty()) {
                    std::cout << "No path found. Will replan after collecting more data.\n";
                    break;  // Allow new sensor data
                }

                // Log path and move
                std::cout << "Path found:\n";
                for (const Node& node : path) {
                    std::cout << "(" << node.x << "," << node.y << ") ";
                    subpath.push_back(rerun::Position3D{node.x, node.y, 0.0f});
                }
                std::cout << "\n";

                // Log and move
                rec.log("full_path", rerun::Points3D(subpath)
                                      .with_colors({rerun::Color(0, 0, 255)})
                                      .with_radii({0.5f}));

                moveRoverAlongPath(path);  // Moves rover to current_goal
                current_start = path.back();  // Update position
                visited_nodes.insert({current_start.x, current_start.y});

                // Final Goal Check
                if (current_goal == final_goal) {
                    std::cout << "GOAL REACHED!\n";
                    ArucoDetect();         // Optional signal
                    sendfinalsignal();     // Stop or notify base
                    break;
                }

                pathplanning_flag = false;
                break;
            }
        }
    }
    return 0;
}
