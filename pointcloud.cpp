#include <librealsense2/rs.hpp> // RealSense Cross Platform API
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>

using namespace std;
using namespace pcl;
using namespace Eigen;

// Grid map structure to hold occupancy grid and boundaries
struct Gridmap {
    vector<vector<bool>> occupancy_grid;
    float min_x, min_y, max_x, max_y;
};

// Function to create grid map from point cloud
Gridmap create_gridmap(const vector<Vector3f>& points, float grid_resolution, float height=2.0) {
    // Calculate boundaries based on points
    float min_x = numeric_limits<float>::max();
    float max_x = numeric_limits<float>::lowest();
    float min_y = numeric_limits<float>::max();
    float max_y = numeric_limits<float>::lowest();

    for (const auto& point : points) {
        min_x = min(min_x, point[0]);
        max_x = max(max_x, point[0]);
        min_y = min(min_y, point[1]);
        max_y = max(max_y, point[1]);
    }

    // Define grid size
    int x_bins = static_cast<int>((max_x - min_x) / grid_resolution) + 1;
    int y_bins = static_cast<int>((max_y - min_y) / grid_resolution) + 1;

    // Initialize the occupancy grid
    vector<vector<bool>> occupancy_grid(x_bins, vector<bool>(y_bins, false));

    // Fill the occupancy grid based on the height of points
    for (const auto& point : points) {
        int x_idx = static_cast<int>((point(0) - min_x) / grid_resolution);
        int y_idx = static_cast<int>((point(1) - min_y) / grid_resolution);

        // Ensure indices are within the valid range
        if (x_idx >= 0 && x_idx < x_bins && y_idx >= 0 && y_idx < y_bins) {
            if (point(2) > height) { // Only mark occupied if point is above height threshold
                occupancy_grid[x_idx][y_idx] = true;
            }
        }
    }

    // Return grid map with boundaries
    return {occupancy_grid, min_x, min_y, max_x, max_y};
}

int main() {
    // Create a RealSense pipeline and start streaming
    rs2::pipeline pipe;
    //pipe.start();
    
    rs2::config cfg;
    cfg.enable_device_from_file("outdoors.bag");
    pipe.start(cfg); // Load from file

    // Declare pointcloud object and points for mapping
    rs2::pointcloud pc;
    rs2::points points;
    vector<Vector3f> point_vectors; // Vector to hold 3D points

    while (true) {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        auto color = frames.get_color_frame();
        auto depth = frames.get_depth_frame();

        // Map point cloud to color frame
        pc.map_to(color);
        points = pc.calculate(depth);

        // Collect points from the pointcloud
        point_vectors.clear(); // Clear previous points
        for (size_t i = 0; i < points.size(); ++i) {
            auto point = points.get_vertices()[i];
            point_vectors.push_back(Vector3f(point.x, point.y, point.z));
        }

        // Define grid resolution
        float grid_resolution = 0.1f;

        // Generate grid map
        Gridmap grid_map = create_gridmap(point_vectors, grid_resolution);

        // Output grid boundaries and grid size
        cout << "Grid boundaries: x(" << grid_map.min_x << " to " << grid_map.max_x << "), "
             << "y(" << grid_map.min_y << " to " << grid_map.max_y << ")" << endl;
        cout << "Grid dimensions: " << grid_map.occupancy_grid.size() << "x"
             << grid_map.occupancy_grid[0].size() << endl;

        // For demonstration purposes, print a portion of the grid map
        // (to avoid printing too much data, you could improve this logic as needed)
        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 10; ++j) {
                cout << grid_map.occupancy_grid[i][j] << " ";
            }
            cout << endl;
        }

        // A small delay (this would be handled by your application, if needed)
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 0.5 seconds
    }

    return 0;
}

