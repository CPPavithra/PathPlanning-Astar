#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <vector>
#include <string>


// Define the Gridmap structure
struct Gridmap {
    std::vector<std::vector<bool>> occupancy_grid;
    float min_x, min_y, max_x, max_y;
};

struct Pose {
    Quaternionf orientation;
    Vector3f position;
};

// Function declaration for creating a grid map
Gridmap create_gridmap(const std::string& ply_file, float grid_resolution, float height = 2.0);

#endif // GRIDMAP_H

