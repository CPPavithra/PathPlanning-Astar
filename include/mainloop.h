#ifndef MAINLOOP_H
#define MAINLOOP_H

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
#include "quadtreecommon.h"
#include "quadtree.h"
#include "common.h"
#include "imu.h"
#include "rerun.h"
#include "ArucoDetect.h"

// The actual DEFINITION will be in mainloop.cpp
extern Gridmap gridmap;
extern float grid_resolution;
extern int batch_threshold;
extern int startx, starty, goalx, goaly;
extern std::set<std::pair<int, int>> visited_nodes;
extern std::set<std::pair<int, int>> failed_goals;
extern Pose rover_pose;
extern int limit;
extern int last_index;
extern int prev_dir;
extern std::deque<Node> recent_goals;
extern int dir;
class Quadtree;

//extern std::string table_text;
extern Node start, goal, current_start, final_goal;
extern std::vector<rerun::Position3D> full_path_points;
extern std::set<std::pair<int, int>> tried_goals;
extern std::vector<Node> full_path;
extern std::string table_text;

Node findcurrentgoal(const Gridmap& gridmap, const Node& current_start, const Node& final_goal,
                      const std::set<std::pair<int, int>>& visited_nodes, const std::set<std::pair<int, int>>& failed_goals,
                      std::deque<Node>& recent_goals, bool& pathplanning_flag);

void log_camera_frames(rerun::RecordingStream& rec, const rs2::frameset& frameset);
void log_views(rerun::RecordingStream& rec);
void moveRoverAlongPath(const std::vector<Node>& path);

//Simplified setup, as other variables are global now
void setup(rerun::RecordingStream &rec);

bool mapping(
    rs2::pipeline &pipe,
    rs2::pointcloud &pc,
    Pose &rover_pose,
    Gridmap &gridmap,
    QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree,
    float &grid_resolution,
    int &batch_threshold,
    int &counter,
    int &adder,
    int limit,
    rerun::RecordingStream &rec,
    bool &pathplanning_flag
);

void pathPlanning(
    Gridmap &gridmap,
    QuadtreeNode* lowQuadtree, QuadtreeNode* midQuadtree, QuadtreeNode* highQuadtree,
    Node &current_start,
    Node &final_goal,
    std::vector<Node> &full_path,
    std::set<std::pair<int, int>> &visited_nodes,
    std::set<std::pair<int, int>> &failed_goals,
    std::deque<Node> &recent_goals, //Corrected from pair<int, int> to Node
    bool &pathplanning_flag,
    rerun::RecordingStream &rec
);

#endif
