#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "include/gridmap.h" 
#include "include/astar.h"

using namespace std;
void save_gridmap_to_csv(const vector<vector<int>>& grid, const string& filename) {
    ofstream file(filename);
    if (!file.is_open()) {
        cerr<<"Could not open"<<endl;
        exit(EXIT_FAILURE);
    }
    for (const auto& row:grid) {
        for (size_t i=0; i<row.size(); ++i) {
            file << row[i];
            if (i<row.size()-1) file<<",";
        }
        file<<"\n";
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

int main()
{
    string ply_file="pointcloud.ply";
    float grid_resolution=0.1;
    //to form the grid map
    Gridmap grid_map=create_gridmap(ply_file,grid_resolution);

    //converting boolean to anint grid
    vector<vector<int>>grid;
    for (const auto& row:grid_map.occupancy_grid)
    {
        vector<int> int_row(row.begin(),row.end());
        grid.push_back(int_row);
    }

    //saving it to a csv file
    string csv_file = "gridmap2.csv";
    save_gridmap_to_csv(grid,csv_file);
    try
    {
        grid=readGridFromCSV(csv_file);
    }
    catch (const exception& e)
    {
        cerr<<"Error:"<<e.what()<<endl;
        return 1;
    }
    cout << "Grid map loaded successfully." << endl;
    double min_x=0;
    double max_y=grid[0].size()-1;
    double min_y=0;
    double max_y=grid.size()-1;
    //start and end
    Node start(0,0);
    //Node goal(grid.size() - 1, grid[0].size() - 1);
    Node goal(5,6);

    //a star
    vector<Node> path = astar(grid, start, goal);
   
    if (path.empty()) {
        cout<<"No path"<<endl;
    } else {
        cout<<"Path found:"<<endl;
        for (const auto& node:path) {
            cout<<"("<<node.x<<", "<<node.y<<")"<<endl;
        }
    }

    return 0;
}

