#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

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

class MotionPlanner {
  public:
    MotionPlanner(rerun::RecordingStream& rec);
    ~MotionPlanner();
    void setup();
    bool runMapping();
    void runPathPlanning();
    //void log_views();
    //void log_camera_frames();
    planning::Node findcurrentgoal();
    void log_camera_frames(const rs2::frameset& frameset);
    void log_views();
    void moveRoverAlongPath(const std::vector<planning::Node>&path);
    bool getSensorData(rs2::frameset& frameset, rs2_vector& accel_raw, rs2_vector& gyro_raw, float& dt);
    bool findpath(const planning::Node& start, const planning::Node& goal, std::vector<planning::Node>& dense_path);
    std::vector<planning::Node> prunepath(const std::vector<planning::Node>& path);
    void executepath(const std::vector<planning::Node>& path, bool& stuck);
    void if_stuck(const planning::Node& failed_goal, int& retry_attempts, const int MAX_RETRIES, bool& pathplanning_flag, std::set<std::pair<int, int>>& failed_goals);
    std::vector<Eigen::Vector3f> processPointCloud(const rs2::frameset& frameset,const mapping::Slam_Pose& slam_pose);
    void updateMaps(const std::vector<Eigen::Vector3f>& points);
    bool checkPlanningTrigger();


    
  private:
    rs2::pipeline pipe;
    rs2::pointcloud pc;
    rerun::RecordingStream &rec;
    // Rover State
    mapping::Slam_Pose slam_pose;
    mapping::Gridmap gridmap;
    // Path Planning State
    planning::Node start;
    planning::Node goal;
    planning::Node current_start;
    planning::Node final_goal;
    std::vector<planning::Node> full_path;
    std::set<std::pair<int, int>> visited_nodes;
    std::set<std::pair<int, int>> failed_goals;
    std::deque<planning::Node> recent_goals;
    std::set<std::pair<int, int>> tried_goals;
    std::vector<rerun::Position3D> full_path_points;
    // Quadtree for spatial partitioning
    quadtree::QuadtreeNode* lowQuadtree{nullptr};
    quadtree::QuadtreeNode* midQuadtree{nullptr};
    quadtree::QuadtreeNode* highQuadtree{nullptr};
    // Configuration & Control Flags
    float grid_resolution{0.001f};
    int batch_threshold{1};
    int limit{20}; 
    mapping::Point center;
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
