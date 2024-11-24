#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include "gridmap.cpp"
#include "astar.cpp"

using namespace std;

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
    // Step 1: Create Gridmap
    string ply_file = "pointcloud.ply";
    float grid_resolution = 0.1;
    Gridmap grid_map = create_gridmap(ply_file, grid_resolution);

    // Step 2: Save Gridmap to CSV
    string csv_file = "gridmap2csv";
    save_gridmap_to_csv(grid, csv_file);

    // Step 3: Run A* Algorithm
    // Read the grid map from a CSV file
string filename = "gridmap2.csv"; // Replace with your CSV file name
vector<vector<int>> grid;
try {
    grid = readGridFromCSV(filename);
} catch (const exception& e) {
    cerr << "Error: " << e.what() << endl;
    return 1;
}
cout << "Grid map loaded" << endl;

    
double min_x = 0;
double max_x = grid[0].size();
double min_y = 0;
double max_y = grid.size();

cout<<"Setting boundaries"<<endl;

    Node start(0, 0);
    Node goal(5, 6); // Adjust based on your requirements
    vector<Node> path = astar(grid, start, goal);

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

