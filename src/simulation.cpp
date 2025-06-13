#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <memory>
#include <random>
#include <thread>
#include <chrono>
#include <algorithm>
#include <rerun.hpp>

struct Node {
    int x, y;
    double g_cost, h_cost, f_cost;
    std::shared_ptr<Node> parent;

    Node(int x, int y, double g = 0.0, double h = 0.0, std::shared_ptr<Node> p = nullptr)
        : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}

    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template<>
    struct hash<Node> {
        size_t operator()(const Node& node) const {
            return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
        }
    };
}
struct CompareNode {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->f_cost > b->f_cost;
    }
};

const int GRID_SIZE = 20;

class CostMap {
private:
    std::vector<std::vector<double>> cost_data;
    std::vector<std::vector<bool>> visited;

public:
    CostMap() : cost_data(GRID_SIZE, std::vector<double>(GRID_SIZE, 0.0)),
                visited(GRID_SIZE, std::vector<bool>(GRID_SIZE, false)) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.1, 10.0);

        for (int i = 0; i < GRID_SIZE; ++i) {
            for (int j = 0; j < GRID_SIZE; ++j) {
                cost_data[i][j] = (rand() % 100 < 10) ? dis(gen) * 5.0 : dis(gen);
            }
        }
    }

    bool isInsideGrid(int x, int y) const {
        return x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE;
    }

    double getCost(int x, int y) const {
        return cost_data[x][y];
    }

    bool isVisited(int x, int y) const {
        return visited[x][y];
    }

    void setVisited(int x, int y) {
        visited[x][y] = true;
    }

    void resetVisited() {
        for (auto& row : visited)
            std::fill(row.begin(), row.end(), false);
    }

    const std::vector<std::vector<double>>& getCostData() const {
        return cost_data;
    }
};

double heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

std::vector<std::shared_ptr<Node>> astar(CostMap& cost_map, int sx, int sy, int gx, int gy) {
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> openList;
    auto start = std::make_shared<Node>(sx, sy, 0.0, heuristic(sx, sy, gx, gy));
    openList.push(start);

    std::unordered_map<Node, std::shared_ptr<Node>> allNodes;
    allNodes[*start] = start;

    const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

    while (!openList.empty()) {
        auto current = openList.top();
        openList.pop();
        int x = current->x, y = current->y;

        if (x == gx && y == gy) {
            std::vector<std::shared_ptr<Node>> path;
            while (current) {
                path.push_back(current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        cost_map.setVisited(x, y);

        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i], ny = y + dy[i];

            if (!cost_map.isInsideGrid(nx, ny) || cost_map.isVisited(nx, ny)) continue;

            double move_cost = (dx[i] == 0 || dy[i] == 0) ? 1.0 : 1.414;
            double terrain_cost = cost_map.getCost(nx, ny);
            double new_g = current->g_cost + move_cost + terrain_cost;

            Node neighbor_node(nx, ny);
            if (!allNodes.count(neighbor_node) || new_g < allNodes[neighbor_node]->g_cost) {
                auto neighbor = std::make_shared<Node>(nx, ny, new_g, heuristic(nx, ny, gx, gy), current);
                openList.push(neighbor);
                allNodes[neighbor_node] = neighbor;
            }
        }
    }

    return {};
}

void visualizeSimulation(const std::shared_ptr<Node>& start, 
                         const std::shared_ptr<Node>& goal, 
                         const std::vector<std::shared_ptr<Node>>& path,
                         const CostMap& cost_map) {
    auto rec = rerun::RecordingStream("Path Planning Simulation");
    rec.spawn().exit_on_failure();

    // Visualize terrain
    std::vector<rerun::Position3D> grid_points;
    std::vector<rerun::Color> grid_colors;

    const auto& cost_data = cost_map.getCostData();
    for (int i = 0; i < GRID_SIZE; ++i) {
        for (int j = 0; j < GRID_SIZE; ++j) {
            grid_points.emplace_back(static_cast<float>(i), static_cast<float>(j), 0.0f);

            double normalized = std::min(1.0, cost_data[i][j] / 10.0);
            uint8_t r = static_cast<uint8_t>(255 * normalized);
            uint8_t g = static_cast<uint8_t>(255 * (1.0 - normalized));
            grid_colors.emplace_back(r, g, 0);
        }
    }

    rec.log_static("world/terrain", rerun::Points3D(grid_points)
        .with_colors(grid_colors)
        .with_radii({0.5f}));

    // Visualize start and goal
    rec.log_static("world/start", rerun::Points3D({{
        static_cast<float>(start->x), 
        static_cast<float>(start->y), 
        1.0f}})
        .with_colors({{0, 0, 255}})
        .with_radii({0.5f}));

    rec.log_static("world/goal", rerun::Points3D({{
        static_cast<float>(goal->x), 
        static_cast<float>(goal->y), 
        1.0f}})
        .with_colors({{0, 255, 0}})
        .with_radii({0.5f}));

    // Visualize path
    std::vector<rerun::Position3D> path_points;
    for (const auto& node : path) {
        path_points.emplace_back(
            static_cast<float>(node->x), 
            static_cast<float>(node->y), 
            0.5f
        );
    }

    rec.log_static("world/path", rerun::LineStrips3D({path_points})
        .with_colors({{255, 0, 255}}));

    // Animate rover movement
    for (size_t i = 0; i < path.size(); ++i) {
        int cx = path[i]->x;
        int cy = path[i]->y;

        std::cout << "Moved to (" << cx << "," << cy << ")\n";

        rec.set_time_sequence("frame", static_cast<int64_t>(i));
        
        rec.log("world/rover", rerun::Points3D({{
            static_cast<float>(cx), 
            static_cast<float>(cy), 
            1.0f}})
            .with_colors({{255, 255, 0}})
            .with_radii({0.5f}));

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Reached goal at (" << goal->x << "," << goal->y << ")\n";
}

int main() {
    CostMap cost_map;
    cost_map.resetVisited();

    int sx = 0, sy = 0;
    int gx = 18, gy = 18;

    auto path = astar(cost_map, sx, sy, gx, gy);

    if (path.empty()) {
        std::cout << "No path found.\n";
        return 1;
    }

    auto start = path.front();
    auto goal = path.back();

    visualizeSimulation(start, goal, path, cost_map);

    return 0;
}
