#ifndef RERUN_H
#define RERUN_H

#include <vector>
#include<unordered_map>
#include <utility>
#include <iostream>

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

namespace components {
struct Color {
    uint8_t r, g, b;
};
} // namespace components

} // namespace rerun
 
struct pair_hash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);//to combine the 2 hashes
        }
};
struct CellCost {
    float cost;  //cost of the cell
  float proxcost;  //cost based on proximity
    bool visited;  //flag indicating if the cell has been visited
    bool proxvisited;

    //edit this if needed
    CellCost(float c = 0.0f,float pc=0.0f,  bool v = false, bool p = false) : cost(c),proxcost(pc), visited(v),proxvisited(p) {}
};
struct Gridmap {
        unordered_map<pair<int,int>,CellCost,pair_hash>occupancy_grid;
        float min_x,min_y,max_x,max_y;
        Gridmap()
        {
        //
        }
        Gridmap(unordered_map<pair<int,int>,CellCost,pair_hash> grid, float min_x, float min_y, float max_x, float max_y)
        : occupancy_grid(grid), min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) {
        //
        }
};


struct Pose
{
        Vector3f position;
        Vector3f velocity;
        Matrix3f orientation;
};

//GLOBALLY
extern Gridmap gridmap;
extern float grid_resolution; //because the distance is in mm and we have to convert it o metre
extern int batch_threshold;

// Function prototypes
void create_gridmap(Gridmap& gridmap, const std::vector<Vector3f>& points,
                    const Pose& roverpose, float grid_resolution, float height = 0.5f, float proxfactor = 0.5f);

rerun::components::Color get_color_for_cost(const CellCost& cell);

void draw_gridmap(const Gridmap& gridmap, const std::vector<Vector3f>& point_vectors,
                  const Pose& roverpose, float grid_resolution, rerun::RecordingStream& rec);

#endif // RERUN_H

