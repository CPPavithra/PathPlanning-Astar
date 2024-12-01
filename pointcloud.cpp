#include <librealsense2/rs.hpp> // RealSense Cross Platform API
#include <iostream>
#include <vector>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>

using namespace std;
using namespace Eigen;

// Grid map structure to hold occupancy grid and boundaries
struct Gridmap {
    vector<vector<bool>> occupancy_grid;
    float min_x, min_y, max_x, max_y;
};

// Function to create grid map from point cloud
Gridmap create_gridmap(const vector<Vector3f>& points, float grid_resolution, float height = 2.0) {
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

    int x_bins = static_cast<int>((max_x - min_x) / grid_resolution) + 1;
    int y_bins = static_cast<int>((max_y - min_y) / grid_resolution) + 1;

    vector<vector<bool>> occupancy_grid(x_bins, vector<bool>(y_bins, false));

    for (const auto& point : points) {
        int x_idx = static_cast<int>((point(0) - min_x) / grid_resolution);
        int y_idx = static_cast<int>((point(1) - min_y) / grid_resolution);

        if (x_idx >= 0 && x_idx < x_bins && y_idx >= 0 && y_idx < y_bins) {
            if (point(2) > height) {
                occupancy_grid[x_idx][y_idx] = true;
            }
        }
    }
    return {occupancy_grid, min_x, min_y, max_x, max_y};
}

void draw_axes() {
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex2f(-1.0f, 0.0f);
    glVertex2f(1.0f, 0.0f);
    glVertex2f(0.0f, -1.0f);
    glVertex2f(0.0f, 1.0f);
    glEnd();
}

void draw_gridmap(const Gridmap& gridmap) {
    float cell_width = 2.0f / gridmap.occupancy_grid.size();
    float cell_height = 2.0f / gridmap.occupancy_grid[0].size();

    glBegin(GL_QUADS);
    for (size_t i = 0; i < gridmap.occupancy_grid.size(); i++) {
        for (size_t j = 0; j < gridmap.occupancy_grid[0].size(); j++) {
            if (gridmap.occupancy_grid[i][j]) {
                glColor3f(1.0f, 0.0f, 0.0f); // Red for occupied
            } else {
                glColor3f(1.0f, 1.0f, 1.0f); // White for free
            }

            float x_start = -1.0f + i * cell_width;
            float y_start = -1.0f + j * cell_height;

            glVertex2f(x_start, y_start);
            glVertex2f(x_start + cell_width, y_start);
            glVertex2f(x_start + cell_width, y_start + cell_height);
            glVertex2f(x_start, y_start + cell_height);
        }
    }
    glEnd();
}

int main() {
    // Spawn RealSense Viewer
    std::thread viewer_thread([]() {
        std::system("realsense-viewer &");
    });
    viewer_thread.detach();

    // Initialize GLFW
    if (!glfwInit()) {
        cerr << "Failed to initialize GLFW" << endl;
        return -1;
    }

    // Create a GLFW window
    GLFWwindow* window = glfwCreateWindow(800, 800, "Real-Time Grid Map", nullptr, nullptr);
    if (!window) {
        cerr << "Failed to create GLFW window" << endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Create a RealSense pipeline
    rs2::pipeline pipe;
    pipe.start();

    // Declare pointcloud object
    rs2::pointcloud pc;
    rs2::points points;

    vector<Vector3f> point_vectors; // Vector to hold 3D points
    Gridmap gridmap;

    float grid_resolution = 0.1f;

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT);

        // Get the next frame
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        pc.map_to(frames.get_color_frame());
        points = pc.calculate(depth);

        // Collect point cloud data
        point_vectors.clear();
        for (size_t i = 0; i < points.size(); ++i) {
            auto point = points.get_vertices()[i];
            if (point.z) {
                point_vectors.push_back(Vector3f(point.x, point.y, point.z));
            }
        }

        // Create gridmap
        gridmap = create_gridmap(point_vectors, grid_resolution);

        // Draw gridmap and axes
        draw_gridmap(gridmap);
        draw_axes();

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

