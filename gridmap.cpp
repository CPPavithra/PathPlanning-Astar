#include <iostream>
#include "include/gridmap.h"
#include <vector>
#include <string>
#include <limits>
#include <fstream>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

Gridmap create_gridmap(const string& ply_file, float grid_resolution, float height) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    if (io::loadPLYFile(ply_file, *cloud) == -1) {
        cerr << "Couldn't read the PLY file" << endl;
        exit(EXIT_FAILURE);
    }

    vector<Vector3f> points; 
    for (size_t i=0; i<cloud->size(); ++i) {
        points.push_back(Vector3f(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z));
    }

    float min_x=numeric_limits<float>::max();
    float max_x=numeric_limits<float>::lowest();
    float min_y=numeric_limits<float>::max();
    float max_y=numeric_limits<float>::lowest();

    // Iterate over the points to compute boundaries
    for (const auto& point : points) {
        min_x=min(min_x, point[0]); // x-coordinate
        max_x=max(max_x, point[0]);
        min_y=min(min_y, point[1]); // y-coordinate
        max_y=max(max_y, point[1]);
    }

    // Define grid size
    int x_bins=static_cast<int>((max_x-min_x)/grid_resolution)+1;
    int y_bins=static_cast<int>((max_y-min_y)/grid_resolution)+1;

    // Initialize the occupancy grid
    vector<vector<bool>>occupancy_grid(x_bins,vector<bool>(y_bins, false));

    // Fill the occupancy grid
    for (const auto& point:points) {
        int x_idx=static_cast<int>((point(0)-min_x)/grid_resolution);
        int y_idx=static_cast<int>((point(1)-min_y)/grid_resolution);

        if (x_idx >= 0 && x_idx < x_bins && y_idx >= 0 && y_idx < y_bins) {
            if (point(2) > height) { // Check height condition
                occupancy_grid[x_idx][y_idx] = true;
            }
        }
    }

    // Mark grid cells outside the defined boundaries as occupied
    for (int i = 0; i < x_bins; ++i) {
        for (int j = 0; j < y_bins; ++j) {
            if ((min_x + i * grid_resolution < min_x || min_x + i * grid_resolution > max_x ||
                 min_y + j * grid_resolution < min_y || min_y + j * grid_resolution > max_y)) {
                occupancy_grid[i][j] = true;
            }
        }
    }

    // Return the grid map and boundaries
    /*Gridmap grid_map = {occupancy_grid, min_x, min_y, max_x, max_y};
    return grid_map;*/
    return {occupancy_grid, min_x, min_y, max_x, max_y};
}

/*int main() {
    // Example usage of the create_gridmap function
    string ply_file = "pointcloud.ply"; // Path to your PLY file
    float grid_resolution = 0.1;           // Define grid resolution

    Gridmap grid_map = create_gridmap(ply_file, grid_resolution);

    // Print the grid map dimensions and boundary values
    cout << "Grid boundaries: x(" << grid_map.min_x << " to " << grid_map.max_x << "), "
         << "y(" << grid_map.min_y << " to " << grid_map.max_y << ")" << endl;
    cout << "Grid dimensions: " << grid_map.occupancy_grid.size() << "x"
         << grid_map.occupancy_grid[0].size() << endl;

    return 0;
}*/
