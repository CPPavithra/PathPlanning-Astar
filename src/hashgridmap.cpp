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
#include "mapping.h"

using namespace rerun;
using namespace rs2;
using namespace std;
using namespace Eigen;
using namespace mapping;

void create_gridmap(Gridmap& gridmap, const vector<Vector3f>& point_vectors, const Slam_Pose& slam_pose, float grid_resolution, float height, float proxfactor)
{
    // Update grid boundaries based on current SLAM pose
    float boundary_threshold=0.01f;
    if (slam_pose.x < gridmap.min_x + boundary_threshold) 
    {
      gridmap.min_x-=(grid_resolution*50.0f);
    }
    if (slam_pose.x > gridmap.max_x - boundary_threshold) 
    { 
      gridmap.max_x+=(grid_resolution*50.0f);
    }
    if (slam_pose.y < gridmap.min_y + boundary_threshold)
    {
      gridmap.min_y-=(grid_resolution*50.0f);
    }
    if (slam_pose.y>gridmap.max_y-boundary_threshold)
    {
      gridmap.max_y+=(grid_resolution*50.0f);
    }

    unordered_map<pair<int,int>,CellCost,pair_hash> updated_occupancy_grid =gridmap.occupancy_grid;
    
    int proxradius=3; //for padding. CHANGE IT LATER.
    float rover_x =slam_pose.x;
    float rover_y =slam_pose.y;
    float yaw =slam_pose.yaw;

    //We are estimating the local ground level for rough terrain.
    //Here, we check for the local ground (near the rover only)
    float ground_sum =0.0f;
    int ground_count =0;
    float ground_window =5.0f; //5m radius around rover for ground estimation
    for (const auto& point : point_vectors) {
        float dz = -point.y(); 
        float dx = point.z();
        float dy = -point.x(); //forward motion
        float dist = sqrt(dx*dx + dy*dy); //5x5m
        if (dist < ground_window && dz < height/4) {//we ignore the tall points
            ground_sum += dz;
            ground_count++;
        }
    }
    //check!!
    float ground_level = (ground_count > 0) ? (ground_sum / ground_count) : 0.0f;

    for (const auto& point : point_vectors) {
        float dx =point.z();  
        float dy =-point.x(); 
        float dz =-point.y();  

        //float ned_theta =-(theta -M_PI_2); 
        //slam is returning yaw-M_PI_2 we have to check if it is oriented correctly */
        //or if we have to put -ve on it!! 
        float rotated_x =cos(yaw)*dx - sin(yaw)*dy;
        float rotated_y =sin(yaw)*dx + cos(yaw)*dy;

        int grid_x = static_cast<int>(rotated_x /1.0f); //replace with grid resolution
        int grid_y = static_cast<int>(rotated_y /1.0f);

        float adjusted_height = dz;
        float cost = 0.0f;

        // Positive obstacle
        if (adjusted_height > height) 
        { 
          cost = 10.0f;
        }
        else if (adjusted_height > (height/2) && adjusted_height <= height) 
        {
          cost = 5.0f;
        }
        else if (adjusted_height > (height/4) && adjusted_height <= (height/2))
        { 
          cost = 1.0f;
        }

        //NEGATIVE OBSTACLE- CHANGE THRESHOLD IF NEEDED
        float drop_threshold = 1.0f; // 1m drop considered as ditch
        if (std::isnan(dz) || std::isinf(dz)) cost = 10.0f;
        else if ((ground_level - adjusted_height) > drop_threshold) cost = 10.0f;

        //Update occupancy grid
        pair<int, int> current = {grid_x, grid_y};
        CellCost& cell = updated_occupancy_grid[current];

        if (!cell.visited && !cell.proxvisited) {
            cell.cost += cost;
            cell.visited = true;
        } else {
            if (cost > 0.0f) updated_occupancy_grid[current] = CellCost(cost, cell.proxcost, true, cell.proxvisited);
        }

        // ----- Optional: Proximity cost padding -----
        /*int proxradius = 3;
        for (int dxn = -proxradius; dxn <= proxradius; ++dxn) {
            for (int dyn = -proxradius; dyn <= proxradius; ++dyn) {
                if (dxn == 0 && dyn == 0) continue;

                pair<int,int> neighbor = {grid_x + dxn, grid_y + dyn};
                CellCost& neighbor_cell = updated_occupancy_grid[neighbor];
                float dist = sqrt(dxn*dxn + dyn*dyn);
                float proxcost = (proxfactor * 2.0f) / (0.1f + dist);

                if (!neighbor_cell.proxvisited) {
                    neighbor_cell.proxcost = proxcost;
                    neighbor_cell.cost += proxcost;
                    neighbor_cell.proxvisited = true;
                }
            }
        }*/
    }

    //Updating the gridmap boundaries
    int min_x = INT_MAX, max_x = INT_MIN, min_y = INT_MAX, max_y = INT_MIN;
    for (const auto& cell : updated_occupancy_grid) {
        int x = cell.first.first;
        int y = cell.first.second;
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }
    gridmap.min_x = min_x;
    gridmap.max_x = max_x;
    gridmap.min_y = min_y;
    gridmap.max_y = max_y;
    gridmap.occupancy_grid = updated_occupancy_grid;
}

//color based on cost
components::Color get_color_for_cost(const CellCost& cell) 
{
  if (cell.proxvisited && cell.cost > 0.0f && cell.cost<1.0f) {
          return components::Color{
              static_cast<uint8_t>(0.0f * 255),
              static_cast<uint8_t>(0.0f * 255),
              static_cast<uint8_t>(1.0f * 255)
          };  // BLUE = Proxvisited and cost > 0
 }
   else	if (cell.cost >= 10.0f)
 {
           return components::Color{
              static_cast<uint8_t>(0.0f * 255), 
              static_cast<uint8_t>(0.0f * 255), 
              static_cast<uint8_t>(0.0f * 255)
            };
}  // BLACK=FULLY OCCUPIED
 else if (cell.cost >= 5.0f) 
 {
           return components::Color{
             static_cast<uint8_t>(1.0f * 255), 
             static_cast<uint8_t>(0.0f * 255), 
             static_cast<uint8_t>(0.0f * 255)
           };
 }// RED=MILD }
 else if (cell.cost >= 1.0f)
{
          return components::Color{
             static_cast<uint8_t>(1.0f * 255), 
             static_cast<uint8_t>(0.8f * 255), 
             static_cast<uint8_t>(0.4f * 255)
          };
}	// ORANGE=CAN GO } 
else
{
    return components::Color{
          static_cast<uint8_t>(0.6f * 255), 
          static_cast<uint8_t>(1.0f * 255), 
          static_cast<uint8_t>(0.6f * 255)
    };
}	// LIGHT GREEN } 
}
     
void draw_gridmap(const Gridmap& gridmap, float grid_resolution, rerun::RecordingStream& rec, const Slam_Pose& slam_pose)
{
    float min_x=(slam_pose.x-5.0f)/grid_resolution;
    float max_x=(slam_pose.x+5.0f)/grid_resolution;
    float min_y=(slam_pose.y-5.0f)/grid_resolution;
    float max_y=(slam_pose.y+5.0)/grid_resolution;
    float scale_factor = 1000.0f;  //Put 1000 so that it is in mm
    std::cout << "Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
    if (gridmap.occupancy_grid.empty())
    {
        std::cout << "Error: Occupancy grid is empty!" << std::endl;
    }
    else
    {
        std::cout << "Occupancy grid has data!" << std::endl;
    }
    vector<rerun::Color> colors;
    vector<rerun::Position3D> roverposition;
    ostringstream table_data;
    //table_data << "Position (x, y) | Color (r, g, b) | Cost\n";
    for (const auto& entry : gridmap.occupancy_grid)
   {
    const auto& [coord, value] = entry;
    float grid_x = coord.first;
    float grid_y = coord.second;

     rerun::Color color = mapping::get_color_for_cost(value);
    
     vector<rerun::Position3D> points = {rerun::Position3D{grid_x, grid_y, 0.0f}};

      
     colors.push_back(color);
     string cell_id = "gridcell_(" + std::to_string(grid_x) + "," + std::to_string(grid_y) + ")_" + std::to_string(value.cost);
     string tag = "grid_map" + cell_id;
     rec.log(tag, rerun::Points3D(points).with_colors({color}).with_radii({0.5f}));
   }
  colors.clear();
}

Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec)
{	return Eigen::Vector3f(rs2_vec.x, rs2_vec.y, rs2_vec.z); }//helper function to convert rs2 to eigen vector3f

/*Function to update the rover's pose using IMU data
update orientation using a small-angle quaternion
Quaternionf delta_q=Quaternionf(AngleAxisf(angular_velocity.norm(),angular_velocity.normalized()));
pose.orientation=delta_q*pose.orientation;
pose.orientation.normalize(); //normalize quaternion to prevent drift*/ 

//convert the realsense points to pcl point
   pcl::PointCloud<pcl::PointXYZ>::Ptr convert_to_pcl(const std::vector<Eigen::Vector3f>& point_vectors) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : point_vectors) {
        cloud->points.emplace_back(point.x(), point.y(), point.z());
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; //Unorganized cloud
    return cloud;
}

