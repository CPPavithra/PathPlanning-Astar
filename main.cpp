#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "include/gridmap.h" // Ensure these files are correctly implemented
#include "include/astar.h"

using namespace std;

// Function to save the grid map to a CSV file
void save_gridmap_to_csv(const vector<vector<int>>& grid, const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr << "Could not open file to save grid map." << endl;
        exit(EXIT_FAILURE);
    }
    for (const auto& row : grid) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) file << ",";
        }
        file << "\n";
    }
    file.close();
}

// Function to read a grid map from a CSV file
vector<vector<int>> readGridFromCSV(const string& filename) {
    vector<vector<int>> grid;
    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Could not open CSV file.");
    }
    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string value;
        vector<int> row;
        while (getline(ss, value, ',')) {
            row.push_back(stoi(value));
        }
        grid.push_back(row);
    }
    file.close();
    return grid;
}

int main() {
    string ply_file = "pointcloud.ply";
    float grid_resolution = 0.1;

    // Generate grid map
    Gridmap grid_map = create_gridmap(ply_file, grid_resolution);

    // Convert the occupancy grid (bool) to an int grid
    vector<vector<int>> grid;
    for (const auto& row : grid_map.occupancy_grid) {
        vector<int> int_row(row.begin(), row.end());
        grid.push_back(int_row);
    }

    // Save the grid to a CSV file
    string csv_file = "gridmap2.csv";
    save_gridmap_to_csv(grid, csv_file);

    // Read the grid back from the CSV file
    try {
        grid = readGridFromCSV(csv_file);
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    cout << "Grid map loaded successfully." << endl;

    // Print grid dimensions
    double min_x = 0;
    double max_x = grid[0].size() - 1;
    double min_y = 0;
    double max_y = grid.size() - 1;

    cout << "Grid boundaries: x(" << min_x << " to " << max_x << "), "
         << "y(" << min_y << " to " << max_y << ")" << endl;

    // Define start and goal nodes (ensure they are within grid bounds)
    Node start(0, 0);
    //Node goal(grid.size() - 1, grid[0].size() - 1);
    Node goal(5, 6);

    // Perform A* pathfinding
    vector<Node> path = astar(grid, start, goal);

    // Display the result
    if (path.empty()) {
        cout << "No path found." << endl;
    } else {
        cout << "Path found:" << endl;
        for (const auto& node : path) {
            cout << "(" << node.x << ", " << node.y << ")" << endl;
        }
    }

    return 0;
}

