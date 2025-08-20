#ifndef ROVERCONTROL_H
#define ROVERCONTROL_H

#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rerun.hpp>
#include <vector>
#include <set>
#include <deque>
#include <string>

// --- Forward-include project headers ---
#include "pathplanning.h"
#include "quadtree.h"
#include "mapping.h"
#include "imu.h"
#include "ArucoDetect.h"

class QuadtreeNode;

class RoverControl {
  public:
    RoverControl(rerun::RecordingStream& rec);
    ~RoverControl();
    void setup();
    bool runMapping();
    void runPathPlanning();
    //void log_views();
    //void log_camera_frames();
    
  private:
    Node findcurrentgoal();
    void log_camera_frames(const rs2::frameset& frameset);
    void log_views();
    void moveRoverAlongPath(const std::vector<Node>&path);

    rs2::pipeline pipe;
    rs2::pointcloud pc;
    rerun::RecordingStream &rec;
    // Rover State
    Pose rover_pose;
    Gridmap gridmap;
    // Path Planning State
    Node start;
    Node goal;
    Node current_start;
    Node final_goal;
    std::vector<Node> full_path;
    std::set<std::pair<int, int>> visited_nodes;
    std::set<std::pair<int, int>> failed_goals;
    std::deque<Node> recent_goals;
    std::set<std::pair<int, int>> tried_goals;
    std::vector<rerun::Position3D> full_path_points;
    // Quadtree for spatial partitioning
    QuadtreeNode* lowQuadtree{nullptr};
    QuadtreeNode* midQuadtree{nullptr};
    QuadtreeNode* highQuadtree{nullptr};
    // Configuration & Control Flags
    float grid_resolution{0.001f};
    int batch_threshold{1};
    int limit{20}; 
    Point center;
    float rootSize;
    int dir;
    int adder{0};
    int counter{0};
    bool pathplanning_flag{false};

    // Movement state
    int prev_dir{0}; // Formerly 'prev_dir'
    // Static data
    std::string cost_table_text;
};
#endif
