#ifndef MAPPING_H
#define MAPPING_H

#include <vector>
#include <unordered_map>
#include <utility>
#include <functional>
#include <rerun.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <Eigen/Dense>

// --- Core Data Structures (formerly in common.h) ---

// A hash function to allow std::pair to be used as a key in unordered_map
struct pair_hash {
    template <typename T1, typename T2>
    std::size_t operator () (const std::pair<T1,T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        // A common way to combine hash values
        return h1 ^ (h2 << 1);
    }
};

// Represents the cost and state of a single cell in the gridmap
struct CellCost {
    float cost;
    float proxcost;
    bool visited;
    bool proxvisited;

    CellCost(float c = 0.0f, float pc = 0.0f, bool v = false, bool p = false)
        : cost(c), proxcost(pc), visited(v), proxvisited(p) {}
};

// The main gridmap structure
struct Gridmap {
    std::unordered_map<std::pair<int, int>, CellCost, pair_hash> occupancy_grid;
    float min_x = 0.0f, min_y = 0.0f, max_x = 0.0f, max_y = 0.0f;

    // Default constructor
    Gridmap() = default;

    // Parameterized constructor
    Gridmap(std::unordered_map<std::pair<int, int>, CellCost, pair_hash> grid,
            float p_min_x, float p_min_y, float p_max_x, float p_max_y)
        : occupancy_grid(std::move(grid)), min_x(p_min_x), min_y(p_min_y), max_x(p_max_x), max_y(p_max_y) {}
};

// A simple 2D point, used by Quadtree and Gridmap
struct Point {
    float x, y;
};

// Represents the rover's physical state (position, velocity, orientation)
struct Pose {
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Quaternionf orientation;
};


// --- Function Prototypes for Mapping, Visualization, and Utilities ---

// Creates or updates the gridmap from a point cloud
void create_gridmap(
    Gridmap& gridmap,
    const std::vector<Eigen::Vector3f>& points,
    float grid_resolution= 0.001f,
    float height = 1.5f,
    float proxfactor = 0.5f,
    const Slam_Pose& slam_pose
);

// Draws the current state of the gridmap to the Rerun stream
void draw_gridmap(
    const Gridmap& gridmap,
    float grid_resolution, // <-- ADDED: Needed to calculate cell positions
    rerun::RecordingStream& rec,
    const Slam_Pose& slam_pose
);

// Helper function for draw_gridmap to determine cell color based on cost
rerun::components::Color get_color_for_cost(const CellCost& cell);

// Updates the rover's pose based on IMU data
void update_rover_pose(
    Pose& pose,
    const Eigen::Vector3f& accel_data,
    const Eigen::Vector3f& gyro_data,
    float delta_time
);

// --- Data Conversion Utilities ---

Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec);
pcl::PointCloud<pcl::PointXYZ>::Ptr convert_to_pcl(const std::vector<Eigen::Vector3f>& point_vectors);

#endif // MAPPING_H
