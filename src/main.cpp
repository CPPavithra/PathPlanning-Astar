#include "rovercontrol.h"
#include <librealsense2/rs.hpp>
#include <rerun.hpp>

int main() {
    auto rec = rerun::RecordingStream("gridmap");
    rec.spawn().exit_on_failure();
    rs2::pipeline pipe;
    rs2::config cfg;

    //cfg.enable_device_from_file("actualgoodvideo.bag"); 
    // SERIAL CONNECTION
     //initSerial("/dev/ttyACM0", 9600);
    //initSerial("/dev/serial/by-id/usb-ZEPHYR_Team_RUDRA_Tarzan_3339511100350023-if00", 9600);

    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1); //Use stream index 1 for IR
    pipe.start(cfg);

    rs2::pointcloud pc;
    
    //setup(goalx, goaly, start, goal, current_start, final_goal, rec);
    setup(rec);
    const int maxgrid = (3 / grid_resolution) * (3 / grid_resolution);
    bool pathplanning_flag = false;
    int counter = 0;
    int adder = 0;

    //this is the main loop
    while (gridmap.occupancy_grid.size() < maxgrid) {
        if (!pathplanning_flag) {
            mapping(pipe, pc, rover_pose, gridmap, lowQuadtree, midQuadtree, highQuadtree,
                    grid_resolution, batch_threshold, counter, adder, limit, rec, pathplanning_flag);
        } else {
            pathPlanning(gridmap, lowQuadtree, midQuadtree, highQuadtree,
                         current_start, final_goal, full_path,
                         visited_nodes, failed_goals, recent_goals,
                         pathplanning_flag, rec);
        }
    }

    return 0;
}
