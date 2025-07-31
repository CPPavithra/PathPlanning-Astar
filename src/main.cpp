#include <iostream>
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
#include "astar.h"
#include "quadtreecommon.h"
#include "quadtree.h"
#include "astarquadtree.h"
#include "imu.h"
#include "rerun.h"
#include "common.h"
#include "ArucoDetect.h"
#include <boost/asio.hpp> 
#include <set>

/***********************************************
 * DEFINING GLOBAL VARIABLES HERE*
 ***********************************************/

using namespace std;
Gridmap gridmap;          // Define and initialize here
float grid_resolution = 0.001f; // Initialize with a value
int batch_threshold = 1;      // Initialize with a value
int startx, starty, goalx, goaly;
using namespace std;
std::set<std::pair<int, int>> visited_nodes;
std::set<std::pair<int, int>> failed_goals;
Pose rover_pose;
bool input_ready = false;
int limit = 20;
int last_index = 0;
int prev_dir = 0;
std::deque<Node> recent_goals;
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

Node findcurrentgoal(const Gridmap& gridmap, 
                     const Node& current_start, 
                     const Node& final_goal, 
                     const std::set<std::pair<int, int>>& visited_nodes,
                     const std::set<std::pair<int, int>>& failed_goals,
                     std::deque<Node>& recent_goals,
                     bool& pathplanning_flag) 
{
    double dist_to_final=heuristic(current_start.x,current_start.y,final_goal.x,final_goal.y);
    std::pair<int,int> goal_cell={final_goal.x,final_goal.y};
    /*if (dist_to_final<1.5&& !gridmap.occupancy_grid.count(goal_cell) && !visited_nodes.count(goal_cell) && !failed_goals.count(goal_cell)) 
    {
        cout << "Final goal close enough. Trying snap...\n";
        auto dry_path = astarquad(lowQuadtree, midQuadtree, highQuadtree, current_start, final_goal, 1.0f);
        if (!dry_path.empty()) return final_goal;
        cout << "Snap to final goal failed. Skipping.\n";
    }*/  //CHECK IF WE HAVE TO DO THIS

    //Intermediate goal search
    Node best_node =current_start;
    double min_total_cost =std::numeric_limits<double>::max();
    bool found_candidate =false;

    //Search within a cropped frontier window (not entire grid!)
    int margin = 6; // Search range from current_start- PASS IT AS A VARIABLE AND NOT AS A CONST 
    for (int x = current_start.x - margin; x <= current_start.x + margin; ++x) {
        for (int y = current_start.y; y <= current_start.y + margin; ++y) {
            std::pair<int, int> cell ={x, y};

            //Skip occupied, visited, failed, or recent
            if (gridmap.occupancy_grid.count(cell) || visited_nodes.count(cell) || failed_goals.count(cell))
                continue;

            Node candidate(x,y);
            if (std::find(recent_goals.begin(),recent_goals.end(),candidate)!=recent_goals.end())
                continue;

            //To not go out og bounds 
            if (!(x >= gridmap.min_x && x <= gridmap.max_x &&
                  y >= gridmap.min_y && y <= gridmap.max_y))
                continue;

            // Skip unreachable outer boundary points
            /*if (gridmap.occupancy_grid.find({x + 1, y}) == gridmap.occupancy_grid.end() &&
                gridmap.occupancy_grid.find({x - 1, y}) == gridmap.occupancy_grid.end() &&
                gridmap.occupancy_grid.find({x, y + 1}) == gridmap.occupancy_grid.end() &&
                gridmap.occupancy_grid.find({x, y - 1}) == gridmap.occupancy_grid.end())
                continue;*/

            //REPLACE THIS WITH A SMARTER LOGIC
            /*double to_goal = heuristic(x, y, final_goal.x, final_goal.y);
            if (to_goal>=dist_to_final) continue;*/

            //Directional alignment with final goal
            double angle_to_node = atan2(y-current_start.y, x-current_start.x);
            double angle_to_goal = atan2(final_goal.y-current_start.y, final_goal.x-current_start.x);
            double angle_diff = fabs(angle_to_node-angle_to_goal);
            if (angle_diff > M_PI) angle_diff = 2*M_PI - angle_diff;
            if (angle_diff > M_PI / 2) continue;

            double obstacle_cost = gridmap.occupancy_grid.count(cell) ? gridmap.occupancy_grid.at(cell).cost : 0.0;
            double cost_to_node = heuristic(current_start.x, current_start.y, x, y) + obstacle_cost;
            double total_cost = cost_to_node + to_goal;

            if (x == current_start.x && y == current_start.y) total_cost += 10.0;

            // 🧪 Validate reachability
            auto dry_path = astarquad(lowQuadtree, midQuadtree, highQuadtree, current_start, candidate, 1.0f);
            if (!dry_path.empty() && total_cost < min_total_cost) {
                best_node = candidate;
                min_total_cost = total_cost;
                found_candidate = true;
            }
        }
    }

    if (found_candidate) {
        std::cout << "🧠 Best reachable intermediate goal: (" << best_node.x << "," << best_node.y << ")\n";
        recent_goals.push_back(best_node);
        if (recent_goals.size() > 10) recent_goals.pop_front();
        return best_node;
    }

    std::cout << "⚠️ No good intermediate goal found.\n";
    pathplanning_flag = false;
    return current_start;
}
/*********************************************************************
 * LIKE THE PYTHON SIMULATION- MORE MODULAR AND BETTER 
 * ****************************************************************/  
/*void update_visibility(Gridmap& gridmap, const Node& rover_pos, int visibility_range) {
    for (int dx = -visibility_range; dx <= visibility_range; ++dx) {
        for (int dy = -visibility_range; dy <= visibility_range; ++dy) {
            int x = rover_pos.x + dx;
            int y = rover_pos.y + dy;
            if (dx * dx + dy * dy <= visibility_range * visibility_range) {
                if (x >= gridmap.min_x && x <= gridmap.max_x && y >= gridmap.min_y && y <= gridmap.max_y) {
                    auto key = std::make_pair(x, y);
                    // Only mark as known if not already in occupancy grid
                    if (gridmap.occupancy_grid.find(key) == gridmap.occupancy_grid.end()) {
                        gridmap.occupancy_grid[key] = CellCost{0, true};
 /*                   }
                }
            }
        }
    }
}

std::optional<Node> find_closest_unknown(const Gridmap& gridmap, const Node& rover_pos, int visibility_range) {
    double min_distance = std::numeric_limits<double>::max();
    std::optional<Node> closest_unknown;

    // Search in a spiral pattern outward from the rover position
    for (int radius = visibility_range + 1; radius <= 2 * visibility_range; radius++) {
        for (int dx = -radius; dx <= radius; dx++) {
            for (int dy = -radius; dy <= radius; dy++) {
                if (dx * dx + dy * dy <= radius * radius && 
                    dx * dx + dy * dy > (radius - 1) * (radius - 1)) {
                    
                    int x = rover_pos.x + dx;
                    int y = rover_pos.y + dy;
                    
                    if (x >= gridmap.min_x && x <= gridmap.max_x && 
                        y >= gridmap.min_y && y <= gridmap.max_y) {
                        
                        auto key = std::make_pair(x, y);
                        if (gridmap.occupancy_grid.find(key) == gridmap.occupancy_grid.end()) {
                            double distance = std::hypot(dx, dy);
                            if (distance < min_distance) {
                                min_distance = distance;
                                closest_unknown = Node(x, y);
                            }
                        }
                    }
                }
            }
        }
        if (closest_unknown.has_value()) {
            return closest_unknown;
        }
    }

    return closest_unknown;
}

bool can_see_goal(const Node& rover_pos, const Node& goal, int visibility_range) {
    // Check line of sight using Bresenham's algorithm
    int x0 = rover_pos.x, y0 = rover_pos.y;
    int x1 = goal.x, y1 = goal.y;
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        // Check if current point is occupied
        auto key = std::make_pair(x0, y0);
        if (gridmap.occupancy_grid.find(key) != gridmap.occupancy_grid.end() && 
            gridmap.occupancy_grid.at(key).cost > 0) {
            return false; // Obstacle in the way
        }
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
        
        // Check if we've gone beyond visibility range
        if (std::hypot(x0 - rover_pos.x, y0 - rover_pos.y) > visibility_range) {
            return false;
        }
    }
    
    return true;
}

bool plan_next_move(Gridmap& gridmap, const Node& rover_pos, const Node& final_goal,
                   std::vector<Node>& path, Node& selected_target, int visibility_range) {
    // First check if we can see and reach the final goal
    if (can_see_goal(rover_pos, final_goal, visibility_range)) {
        path = astarsparse(gridmap.occupancy_grid, rover_pos, final_goal);
        if (!path.empty()) {
            selected_target = final_goal;
            return true;
        }
    }

    // If not, look for closest unknown area
    auto unknown = find_closest_unknown(gridmap, rover_pos, visibility_range);
    if (unknown.has_value()) {
        path = astarsparse(gridmap.occupancy_grid, rover_pos, unknown.value());
        if (!path.empty()) {
            selected_target = unknown.value();
            return true;
        }
    }

    // Fallback: try to make progress toward final goal
    path = astarsparse(gridmap.occupancy_grid, rover_pos, final_goal);
    if (!path.empty()) {
        selected_target = final_goal;
        return true;
    }

    return false;
}*/
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
    //cfg.enable_device_from_file("actualgoodvideo.bag"); 
    
    // SERIAL CONNECTION
     //initSerial("/dev/ttyACM0", 9600);
    //initSerial("/dev/serial/by-id/usb-ZEPHYR_Team_RUDRA_Tarzan_3339511100350023-if00", 9600);


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
            updateQuadtreesWithPointCloud(lowQuadtree, midQuadtree, highQuadtree, point_vectors, rover_pose);
            rerunvisualisation(lowQuadtree, midQuadtree, highQuadtree, rec);

            if (gridmap.occupancy_grid.size() >= batch_threshold) {
                draw_gridmap(gridmap, point_vectors, rover_pose, grid_resolution, rec);
                batch_threshold = batch_threshold + gridmap.occupancy_grid.size();
            }

            counter = gridmap.occupancy_grid.size() - adder;
            if (counter >= limit) {
                cout << "Mapping paused. Switching to path planning." << std::endl;
                pathplanning_flag = true;    // Switching to path planning
                adder = 20;
            }
        } else {
            
int retry_attempts = 0;
const int MAX_RETRIES = 5;

while (pathplanning_flag) {
    visited_nodes.insert({current_start.x, current_start.y});

    Node current_goal = findcurrentgoal(gridmap, current_start, final_goal,
                                        visited_nodes, failed_goals, recent_goals, pathplanning_flag);
    if (!pathplanning_flag) break;  // findcurrentgoal might disable it
    std::cout << "Selected intermediate goal: (" << current_goal.x << "," << current_goal.y << ")" << std::endl;
    // Check if selected goal is occupied
    if (gridmap.occupancy_grid.count({current_goal.x, current_goal.y})) {
        std::cout << "Selected goal is occupied. Marking as failed.\n";
        failed_goals.insert({current_goal.x, current_goal.y});
        retry_attempts++;
        if (retry_attempts >= MAX_RETRIES) {
            std::cout << "Too many failed attempts. Aborting path planning.\n";
            pathplanning_flag = false;
        }
        continue;
    }

    // Step 1: Try Global Sparse A*
    std::vector<Node> sparse_path = astarsparse(gridmap.occupancy_grid, current_start, current_goal);
    std::vector<Node> dense_path;

    if (sparse_path.empty()) {
        std::cout << "Sparse A* failed. Attempting Dense A*...\n";
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
        // Step 2: Refine Sparse path segments using Dense A*
        for (int i = 1; i < sparse_path.size(); ++i) {
            std::vector<Node> segment = astarquad(lowQuadtree, midQuadtree, highQuadtree,
                                                  sparse_path[i - 1], sparse_path[i], 1.0f);
            if (!segment.empty()) {
                dense_path.insert(dense_path.end(), segment.begin(), segment.end());
            } else {
                std::cout << "Dense segment failed between sparse nodes. Skipping segment.\n";
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

    // Safety check
    if (pruned_path.size() <= 1) {
        std::cout << "Pruned path too short. Marking goal as failed.\n";
        failed_goals.insert({current_goal.x, current_goal.y});
        retry_attempts++;
        continue;
    }

    // Step 4: Execute the path
    bool stuck = true;
    Node previous_start = current_start;

    for (int i = 1; i < pruned_path.size(); ++i) {
        Node local_start = pruned_path[i - 1];
        Node local_goal = pruned_path[i];

        std::cout << "Path segment:\n";
        std::cout << "(" << local_start.x << "," << local_start.y << ") -> ("
                  << local_goal.x << "," << local_goal.y << ")\n";

        std::vector<Node> segment = {local_start, local_goal};
        moveRoverAlongPath(segment);

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
            std::cout << "🏁 GOAL REACHED!\n";
            ArucoDetect();
            sendfinalsignal();
            pathplanning_flag = false;
            break;
        }
    }

    // Step 5: Handle failure or retry
    if (pathplanning_flag && stuck) {
        std::cout << "Rover stuck. Retrying with a different goal.\n";
        failed_goals.insert({current_goal.x, current_goal.y});
        retry_attempts++;
        if (retry_attempts >= MAX_RETRIES) {
            std::cout << "Too many retries. Aborting planning.\n";
            pathplanning_flag = false;
            break;
        }
        continue;
    }

    if (pathplanning_flag) {
        std::cout << "Intermediate goal reached. Continuing planning...\n";
    }
}
}
}
    return 0;
}
