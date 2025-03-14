#ifndef RERUN_H
#define RERUN_H

#include <vector>
#include<unordered_map>
#include <utility>
#include <iostream>
#include "common.h"

using namespace Eigen;
using namespace std;

//It must contain all necessary declarations of structures, constants, and function prototypes while excluding implementation details

// Define the `rerun` namespace for custom components
namespace rerun {

template <>
struct AsComponents<float> {
    static std::vector<float> serialize(float value);
};

template <>
struct AsComponents<std::vector<float>> {
    static std::vector<ComponentBatch> serialize(const std::vector<float>& data);
};

/*namespace components {
struct Color {
    uint8_t r, g, b;
};
}*/ // namespace components

} // namespace rerun
 

struct Pose
{
        Vector3f position;
        Vector3f velocity;
        Quaternionf orientation;
};

//GLOBALLY
extern Gridmap gridmap;
extern float grid_resolution; //because the distance is in mm and we have to convert it o metre
extern int batch_threshold;

// Function prototypes
void create_gridmap(Gridmap& gridmap, const std::vector<Vector3f>& points,
<<<<<<< HEAD
                    const Pose& roverpose, float grid_resolution = 0.001f, float height = 1.0f, float proxfactor = 0.5f);
=======
                    const Pose& roverpose, float grid_resolution = 0.001f, float height = 1.5f, float proxfactor = 0.5f);
>>>>>>> final


rerun::components::Color get_color_for_cost(const CellCost& cell);

void draw_gridmap(const Gridmap& gridmap, const std::vector<Vector3f>& point_vectors,
                  const Pose& roverpose, float grid_resolution, rerun::RecordingStream& rec);

void update_rover_pose(Pose& pose, const Vector3f& accel_data, const Vector3f& gyro_data, float delta_time);

void log_navigation_pane(rerun::RecordingStream& rec, const Pose& roverpose);

Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec);

pcl::PointCloud<pcl::PointXYZ>::Ptr convert_to_pcl(const std::vector<Eigen::Vector3f>& point_vectors);

#endif // RERUN_H

