#include "/usr/local/include/librealsense2/rs.hpp"   // Include RealSense Cross Platform API
#include <GLFW/glfw3.h>           // Include OpenGL and GLFW
#include <pcl/point_cloud.h>      // PCL for PointCloud handling
#include <pcl/point_types.h>      // PCL point types
#include <pcl/io/ply_io.h>        // PCL to save PointCloud as .ply file
#include "include/example.hpp"            // Convenience functions for rendering
#include <algorithm>
#include <cstdlib>                // For system calls (used to launch RealSense Viewer)
#include <GL/glu.h>

using namespace rs2;

// Struct for managing rotation of pointcloud view
struct state {
    double yaw, pitch, last_x, last_y;
    bool ml;
    float offset_x, offset_y;
    texture tex;

    state() : yaw(0), pitch(0), last_x(0), last_y(0), ml(false), offset_x(0), offset_y(0), tex() {}
};

// Helper function declarations
//void register_glfw_callbacks(window& app, state& app_state);
//void draw_pointcloud(window& , state& app_state, rs2::points& points);

int main() {
    try {
        // Create a simple OpenGL window for rendering
        window app(1280, 720, "RealSense Pointcloud Example");

        // Construct an object to manage view state
        state app_state;

        // Register callbacks to allow manipulation of the pointcloud
        register_glfw_callbacks(app, app_state);

        // Declare pointcloud object for calculating pointclouds and texture mappings
        rs2::pointcloud pc;

        // We want the points object to be persistent so we can display the last cloud when a frame drops
        rs2::points points;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;

        // Start streaming with default recommended configuration
        pipe.start();

        // Launch the RealSense Viewer GUI (optional)
        std::system("realsense-viewer &");

        while (app) {  // Loop until the user closes the window
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames();

            // Get the depth and color frames
            auto depth = frames.get_depth_frame();
            auto color = frames.get_color_frame();

            // Generate the pointcloud and texture mappings
            points = pc.calculate(depth);

            // Map the point cloud to the color frame
            pc.map_to(color);

            // Upload the color frame to OpenGL
            app_state.tex.upload(color);

            // Render the pointcloud to the window
            draw_pointcloud(app, app_state, points);

            // Get vertices and texture coordinates
            auto vertices = points.get_vertices();
            auto tex_coords = points.get_texture_coordinates();

            // Convert to PCL point cloud format for saving
            pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
            pcl_cloud.width = points.size();
            pcl_cloud.height = 1;
            pcl_cloud.is_dense = false;
            pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);

            for (size_t i = 0; i < points.size(); ++i) {
                if (vertices[i].z) {  // Only process valid points
                    pcl_cloud.points[i].x = vertices[i].x;
                    pcl_cloud.points[i].y = vertices[i].y;
                    pcl_cloud.points[i].z = vertices[i].z;

                    const uint8_t* color_data = reinterpret_cast<const uint8_t*>(color.get_data());
                    pcl_cloud.points[i].r = color_data[i * 3];
                    pcl_cloud.points[i].g = color_data[i * 3 + 1];
                    pcl_cloud.points[i].b = color_data[i * 3 + 2];
                }
            }

            // Save the point cloud to a .ply file
            pcl::io::savePLYFileASCII("pointcloud.ply", pcl_cloud);

            // Break out of the loop after saving the point cloud
            break;  // Remove or replace with custom loop control logic
        }
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "("
                  << e.get_failed_args() << "):\n" << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}

