#ifndef COMMON_H
#define COMMON_H

#include <unordered_map>
#include <utility> // For std::pair
#include <functional> // For std::hash

struct pair_hash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1); // Combine the two hashes
    }
};

struct CellCost {
    float cost;
    float proxcost;
    bool visited;
    bool proxvisited;

    CellCost(float c = 0.0f, float pc = 0.0f, bool v = false, bool p = false)
        : cost(c), proxcost(pc), visited(v), proxvisited(p) {}
};

struct Gridmap {
    std::unordered_map<std::pair<int, int>, CellCost, pair_hash> occupancy_grid;
    float min_x, min_y, max_x, max_y;

    Gridmap() : min_x(0), min_y(0), max_x(0), max_y(0) {}

    Gridmap(std::unordered_map<std::pair<int, int>, CellCost, pair_hash> grid,
            float min_x, float min_y, float max_x, float max_y)
        : occupancy_grid(grid), min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) {}
};

struct Point {
  float x,y;
};

#endif // COMMON_H

