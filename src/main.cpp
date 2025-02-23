#include <iostream>
#include "astar.h"
#include <ostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include <Eigen/Core>
//for downsampling and filtering
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
float grid_resolution=0.001f; // Initialize with a value
int batch_threshold=1;      // Initialize with a value
int startx, starty, goalx, goaly;
using namespace std;
Pose rover_pose;
bool input_ready=false;
int limit=2;
int last_index=0;
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
// TO FIND THE CHECKPOINTS
Node findcurrentgoal(const Gridmap& gridmap, const Node& current_start, const Node& final_goal, const std::set<std::pair<int, int>>& visited_nodes) {
    Node best_node = current_start;
 initSerial("/dev/ttyACM0", 9600);
    double min_total_cost = std::numeric_limits<double>::max();

    for (int x = gridmap.min_x; x <= gridmap.max_x; ++x) {
        for (int y = gridmap.min_y; y <= gridmap.max_y; ++y) {
          //CHECK IF ALREADY VISITED OR IN THE OCCUPANCY GRID
          if (gridmap.occupancy_grid.find({x, y}) != gridmap.occupancy_grid.end() ||
                visited_nodes.find({x, y}) != visited_nodes.end()) {
                continue;
            }

            // calculate cost from start to that node
            double cost_to_node = heuristic(current_start.x, current_start.y, x, y);
            // alculate cost from this node to the final goal
            double cost_to_goal = heuristic(x, y, final_goal.x, final_goal.y);

            // Total cost = cost to node + cost to goal
            double total_cost = cost_to_node + cost_to_goal;

            // Select the node with the minimum total cost
            if (total_cost < min_total_cost) {
                min_total_cost = total_cost;
                best_node = Node(x, y);
            }
        }
    }

    return best_node;
}

/*****************************************************************************
 * There are a few functions to log views in the rerun viewer.
 * The Blueprint attribute is not being recognised so we have to manually adjust the screen view.
 * Add more in the future if needed.
 * Now it includes- 1. Grid map 
                    2. Cost table
                    3. Rover feedback
                    4. Stereo camera
                    5. Realsense color enable_stream
  ****************************************************************************/

void log_views(rerun::RecordingStream& rec) {
    // Define spatial views
    rec.log_static("grid_map", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN);  // Left 3/4th pane

    rec.log_static("rgb_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN); // Top-right
    rec.log_static("heat_camera", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN); // Below RGB camera

    rec.log_static("rover_feedback", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN); // Bottom-right log
    rec.log_static("cost_table", rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN); // Bottom-right navigation

    // Define views manually using SpaceView
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

    //stero cam
    rec.log("heat_camera",
        rerun::DepthImage(ir_data, {ir_width, ir_height}, rerun::datatypes::ChannelDatatype::U8));
    
}


void log_rover_feedback(rerun::RecordingStream& rec) {
    rec.log("rover_feedback", rerun::TextLog("Rover movement tracking enabled."));
}

/********************************************************************************************
 * ******************************************************************************************/

/******************************************************************************************
 * FUNCTION TO DETERMINE THE DIRECTION FOR THE ROVER TO MOVE (HARDCODED POSITIONS)
 * ****************************************************************************************/

void moveRoverAlongPath(const std::vector<Node>& path) {
    if (path.size() < 2) return;  // No movement needed

    int prev_dir = 0;  // Start with no direction

    for (size_t i = 1; i < path.size(); i++) {
        int dx = path[i].x - path[i - 1].x;
        int dy = path[i].y - path[i - 1].y;

        // Determine direction index (0-7 for 45-degree increments)
        int dir = 0;
        if (dx > 0 && dy == 0) dir = 0;  // Right
        else if (dx > 0 && dy > 0) dir = 1;  // Top-right
        else if (dx == 0 && dy > 0) dir = 2;  // Up
        else if (dx < 0 && dy > 0) dir = 3;  // Top-left
        else if (dx < 0 && dy == 0) dir = 4;  // Left
        
        float distance = sqrt(dx*dx + dy*dy)*1.0f;//grid resolution=1.0f
        float time_needed = distance / 0.2;  

        Drive(dir, time_needed, prev_dir);//this function is in imu.cpp
      //  std::this_thread::sleep_for(std::chrono::milliseconds(int(time_needed * 1000)));

        prev_dir = dir;  //dont update it here as the rover does not know if the rotation has been updated or not. It should be updated in the drive function
    }
}
/***********************************************************************************************/

int main()
{
      	auto rec = rerun::RecordingStream("gridmap");
        rec.spawn().exit_on_failure(); //this is for realsense viewer- can be avoided
        log_views(rec);
        //std::system("realsense-viewer &");
        rs2::pipeline pipe;
        rs2::config cfg;  
       // cfg.enable_device_from_file("actualgoodvideo.bag"); 
        
        //SERIAL CONNECTION
        initSerial("/dev/ttyACM0", 9600);

        cfg.enable_stream(RS2_STREAM_DEPTH); 
        cfg.enable_stream(RS2_STREAM_GYRO);   
        cfg.enable_stream(RS2_STREAM_ACCEL);
        cfg.enable_stream(RS2_STREAM_COLOR);
      

       cfg.enable_stream(RS2_STREAM_INFRARED, 1); // Left IR
       cfg.enable_stream(RS2_STREAM_INFRARED, 2); // Right IR/ Enable only if available
          // Enable Right IR (optional)
 
    
        pipe.start(cfg);

        rs2::pointcloud pc;
        rs2::points points;

        //Pose rover_pose;
        rover_pose.position = Eigen::Vector3f(0, 0, 0);
        rover_pose.orientation = Eigen::Matrix3f::Identity();
        rover_pose.velocity = Eigen::Vector3f(0, 0, 0);

      
        //for time
        auto last_time = std::chrono::high_resolution_clock::now();

        vector<Vector3f> point_vectors;

        static int frame_counter = 0;
        const int maxgrid=(3/grid_resolution)*(3/grid_resolution);
        bool pathplanning_flag=false;
        int counter=0; //to check when path planning is to be called
        int adder=0;

          rec.log("cost_table",rerun::archetypes::TextDocument(table_text));


        //path planning- just setting the starting node
        cout<<"Setting boundaries...\n";
        cout<<"Enter starting coordinates (x y): ";
        cin>>startx>>starty;
        cout<<"\nStart: ("<<startx<< ", " <<starty<< ")";
        Node goal(15,15); //take goal as a user input later
       // cout << "Goal: (" << goal.x << ", " << goal.y << ")\n";  
     
   
        //create start and goal nodes
        Node start(startx, starty);
        Node current_start = start;
        std::vector<rerun::Position3D> full_path_points;
        std::set<std::pair<int, int>> tried_goals;
        Node final_goal = goal;
        std::vector<Node> full_path;
        std::set<std::pair<int, int>> visited_nodes;

       while (gridmap.occupancy_grid.size()<maxgrid)
       {
           if(!pathplanning_flag)
           {
             auto current_time = std::chrono::high_resolution_clock::now();
             float delta_time = std::chrono::duration<float>(current_time - last_time).count();
             last_time = current_time;
             //declare variables to hold sensor data
             rs2_vector accel_data = {0.0f, 0.0f, 0.0f};
             rs2_vector gyro_data = {0.0f, 0.0f, 0.0f};

             //get frames from the RealSense camera
             rs2::frameset frameset;
             try {
                  frameset = pipe.wait_for_frames();
             } catch (const rs2::error& e) {
                  std::cerr << "RealSense error: " << e.what() << std::endl;
                  continue;
             }


              //get accelerometer data
             if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
             {
                   accel_data = accel_frame.get_motion_data();
             }
             else
             {
                 cerr<< "Failed to retrieve accelerometer data" << endl;
             }
             //get gyroscope data
             if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
             {
                gyro_data = gyro_frame.get_motion_data();
             }
             else
             {
               cerr<< "Failed to retrieve gyroscope data"<< endl;
             }
             //convert accelerometer and gyroscope data to Eigen vectors
             Eigen::Vector3f accel_eigen = convert_to_eigen_vector(accel_data);
             Eigen::Vector3f gyro_eigen = convert_to_eigen_vector(gyro_data);
             //update the rover pose with accelerometer and gyroscope data
             update_rover_pose(rover_pose, accel_eigen, gyro_eigen, delta_time);

             //process depth data to create point cloud
             rs2::depth_frame depth_frame = frameset.get_depth_frame();
             points = pc.calculate(depth_frame);
            
             //LOGGING THE frameset
              log_camera_frames(rec, frameset);
              log_rover_feedback(rec);
              //log_table("table.png",rec);
                          //log_navigation_pane(rec,rover_pose);

             //collect point cloud data
             point_vectors.clear();
             for (size_t i=0; i<points.size(); ++i)
             {
                auto point=points.get_vertices()[i];
                if (point.z) 
                {
                       Eigen::Vector3f transformed_point=rover_pose.orientation*Eigen::Vector3f(point.x, point.y, point.z)+rover_pose.position;
                       point_vectors.push_back(transformed_point);
                }
             }
                //convert the realsense points to pcl point
             auto pcl_cloud=convert_to_pcl(point_vectors);

             if (depth_frame) 
             {
             static int processed_frames = 0;  // Tracks how many frames you process
             processed_frames++;
             //auto frame_number = depth_frame.get_frame_number();

             //std::cout << "Iteration: " << processed_frames << ", Frame Number: " << frame_number << std::endl;
             }
          //CHECK IF THE IMPLEMENTATIONS OF PASSTHROUGH AND VOXEL GRID IS CORRECT  
             //passthrough filter
             pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>());
             pcl::PassThrough<pcl::PointXYZ> passthrough;
             passthrough.setInputCloud(pcl_cloud);
             passthrough.setFilterFieldName("z");  //this is based on depth
             passthrough.setFilterLimits(0.5, 5.0); //range in metres.
             passthrough.filter(*passthrough_cloud);
             //cout<<"After filtering AFTER PASSTHROUGH: "<<passthrough_cloud->size()<<" points."<<"\n"<<endl;

             //voxelgrid
             pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
             pcl::VoxelGrid<pcl::PointXYZ> voxel;
             voxel.setInputCloud(passthrough_cloud);
             voxel.setLeafSize(0.05f, 0.05f, 0.05f); //this is the size of each voxel in xyz dimension
             voxel.filter(*filtered_cloud);

             point_vectors.clear();
             std::vector<float> z_values;
             z_values.reserve(filtered_cloud->points.size());

             for (const auto& point : filtered_cloud->points) 
             {
                 point_vectors.emplace_back(point.x, point.y, point.z);
             }
             create_gridmap(gridmap,point_vectors , rover_pose, grid_resolution);

             if(gridmap.occupancy_grid.size()>=batch_threshold)
             {
                draw_gridmap(gridmap,point_vectors, rover_pose, grid_resolution, rec);
                batch_threshold=batch_threshold+gridmap.occupancy_grid.size();
             }  //std::this_thread::sleep_for(std::chrono::milliseconds(30));

              counter=gridmap.occupancy_grid.size()-adder;
             if (counter >= limit) 
             {
                cout<<"Mapping paused. Switching to path planning." << std::endl;
                pathplanning_flag =true;    //switching to path planning
                adder=2;
             }
          }
          else
          {
            while(pathplanning_flag)
            {
               // Mark the current start as visited
             visited_nodes.insert({current_start.x, current_start.y});
             std::vector<rerun::Position3D> subpath; // Store all nodes for logging
        // Find the best intermediate goal
             Node current_goal = findcurrentgoal(gridmap, current_start, final_goal, visited_nodes);

            // std::cout << "Current position: (" << current_start.x << "," << current_start.y << ")" << std::endl;
             //std::cout << "Selected intermediate goal: (" << current_goal.x << "," << current_goal.y << ")" << std::endl;

             // Check if the selected goal is valid
             if (gridmap.occupancy_grid.find({current_goal.x, current_goal.y}) != gridmap.occupancy_grid.end()) 
             {
                 std::cout << "Selected goal is occupied. Planning failed." << std::endl;
                 break;
             }

             // Find path using A*
             std::vector<Node> path = astar(gridmap.occupancy_grid, current_start, current_goal);

             if (path.empty())
             {
                 std::cout << "No path found. Check grid or start/goal positions." << std::endl;
                 break;
        
             } 
             else
             {
                 std::cout << "Path found:" << std::endl;

                 // Clear subpath before using it
                 subpath.clear();

                 /*for (const Node& node : path) 
                 {
                    std::cout << "(" << node.x << "," << node.y << ") ";
                    subpath.push_back(rerun::Position3D{node.x, node.y, 0.0f}); // z=0 for 2D
                 }*/
                 moveRoverAlongPath(path);
                 for (size_t i = 0; i < path.size(); ++i) {
                      cout << "(" << path[i].x << "," << path[i].y << ") ";
                      subpath.push_back(rerun::Position3D{path[i].x, path[i].y, 0.0f}); // z=0 for 2D
                                            // Move the rover every 5 points or at the last point
                     /* if ((i + 1) % 5 == 0 || i == path.size() - 1) {
                           cout << "\nMoving rover along path segment..." << std::endl;
                            if (last_index < path.size() && last_index <= i) {
                                 moveRoverAlongPath({path.begin() + last_index, path.begin() + i + 1});
                                 last_index = i + 1;
                            }         */                //stop path planning to allow mapping updates
                          /* if(path[i]==goal)
                           {
                             break; //break out of this while loop to check if current goal is equal to final goal but keeping pathplanning flag as true only
                           }
                           else
                           {
                           pathplanning_flag = false;
                           continue;  // Exit to update grid and replan
                           }*/
                      //}
                  }

                 std::cout << std::endl;

                 // Log all nodes together instead of per node
                 rec.log("full_path",rerun::Points3D(subpath)
                        .with_colors({rerun::Color(0, 0, 255)}) // Blue color
                        .with_radii({0.5f})); // Radius 0.5);
                                              //
                  current_start = path.back();
             }

             // Update current start to the end of the current path
             //current_start = path.back();

             if(current_goal==final_goal)
             {
                cout<<"GOAL REACHED"<<endl;
                ArucoDetect();
                sendfinalsignal();
                //serial.close();
                break;
             }
             pathplanning_flag=false;
            }

        }
    }
return 0;
}        //create start and goal nodes



