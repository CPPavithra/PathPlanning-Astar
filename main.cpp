#include <iostream>
#include "include/astar.h"
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <Eigen/Core>
//for downsampling and filtering
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <cstdlib>
#include <rerun/demo_utils.hpp>
#include <unordered_set>
#include <sstream>
#include "include/rerun.h"
#include <deque>
#include "include/common.h"
//for threads
#include <thread>
#include <mutex>
#include <condition_variable>

Gridmap gridmap;          // Define and initialize here
float grid_resolution=0.001f; // Initialize with a value
int batch_threshold=100;      // Initialize with a value
std::mutex input_mutex;
std::condition_variable cv;
int startx, starty, goalx, goaly;

Pose rover_pose;
rover_pose.position = Eigen::Vector3f(0, 0, 0);
rover_pose.orientation = Eigen::Matrix3f::Identity();
rover_pose.velocity = Eigen::Vector3f(0, 0, 0);

void realsense()
{
	auto rec = rerun::RecordingStream("gridmap");
        rec.spawn().exit_on_failure();

        //realsense pipeline
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_device_from_file("video2.bag");
        cfg.enable_stream(RS2_STREAM_GYRO);
        cfg.enable_stream(RS2_STREAM_ACCEL);
        cfg.enable_stream(RS2_STREAM_DEPTH);

       try {
                pipe.start(cfg);
       } catch (const rs2::error &e) {
                cerr << "Error: Failed to start the pipeline: " << e.what() << endl;
      }
      rs2::pointcloud pc;
      rs2::points points;

      /*Pose rover_pose;
      rover_pose.position = Eigen::Vector3f(0, 0, 0);
      rover_pose.orientation = Eigen::Matrix3f::Identity();
      rover_pose.velocity = Eigen::Vector3f(0, 0, 0);*/


        //for time
     auto last_time = std::chrono::high_resolution_clock::now();

     vector<Vector3f> point_vectors;

     static int frame_counter = 0;
     
     while (true){
        //get the current time and compute the delta time
        auto current_time = std::chrono::high_resolution_clock::now();
        float delta_time = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;
       //declare variables to hold sensor data
        rs2_vector accel_data = {0.0f, 0.0f, 0.0f};
        rs2_vector gyro_data = {0.0f, 0.0f, 0.0f};

        //get frames from the RealSense camera
        rs2::frameset frameset;
        try {
                frameset = pipe.wait_for_frames();
        } catch (const rs2::error& e) {
                std::cerr << "RealSense error: " << e.what() << std::endl;
                continue;
        }


        //get accelerometer data
        if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
        {
                accel_data = accel_frame.get_motion_data();
        }
        else
        {
                cerr<< "Failed to retrieve accelerometer data" << endl;
        }
        //get gyroscope data
        if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
        {
                gyro_data = gyro_frame.get_motion_data();
        }
        else
        {
                cerr<< "Failed to retrieve gyroscope data"<< endl;
        }
                //convert accelerometer and gyroscope data to Eigen vectors
        Eigen::Vector3f accel_eigen = convert_to_eigen_vector(accel_data);
        Eigen::Vector3f gyro_eigen = convert_to_eigen_vector(gyro_data);
               //update the rover pose with accelerometer and gyroscope data
        update_rover_pose(rover_pose, accel_eigen, gyro_eigen, delta_time);

        //process depth data to create point cloud
        rs2::depth_frame depth_frame = frameset.get_depth_frame();
                points = pc.calculate(depth_frame);

                //collect point cloud data
                point_vectors.clear();
                for (size_t i=0; i<points.size(); ++i) {
                auto point=points.get_vertices()[i];
                if (point.z) {
                        Eigen::Vector3f transformed_point=rover_pose.orientation*Eigen::Vector3f(point.x, point.y, point.z)+rover_pose.position;
                        point_vectors.push_back(transformed_point);
                }
 }
                //convert the realsense points to pcl point
                auto pcl_cloud=convert_to_pcl(point_vectors);

  if (depth_frame) {
    static int processed_frames = 0;  // Tracks how many frames you process
    processed_frames++;
    auto frame_number = depth_frame.get_frame_number();

    std::cout << "Iteration: " << processed_frames
              << ", Frame Number: " << frame_number << std::endl;
}

    //passthrough filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcl_cloud);
    passthrough.setFilterFieldName("z");  //this is based on depth
    passthrough.setFilterLimits(0.5, 5.0); //range in metres.
    passthrough.filter(*passthrough_cloud);
    //cout<<"After filtering AFTER PASSTHROUGH: "<<passthrough_cloud->size()<<" points."<<"\n"<<endl;

    //voxelgrid
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
   voxel.setInputCloud(passthrough_cloud);
    voxel.setLeafSize(0.05f, 0.05f, 0.05f); //this is the size of each voxel in xyz dimension
    voxel.filter(*filtered_cloud);

    point_vectors.clear();
    for (const auto& point : filtered_cloud->points) {
        point_vectors.emplace_back(point.x, point.y, point.z);
    }
create_gridmap(gridmap, point_vectors, rover_pose, grid_resolution);
}

}

void gridmap_thread()
{
 while(true)
 {
     if(gridmap.occupancy_grid.size()>=batch_threshold)
     {
        draw_gridmap(gridmap,point_vectors, rover_pose, grid_resolution, rec);
        batch_threshold=batch_threshold+gridmap.occupancy_grid.size();
     }  //std::this_thread::sleep_for(std::chrono::milliseconds(30));

 }
}

void pathplanning()
{
        while(true)
	{
	    if(gridmap.occupancy_grid.size()>=batch_threshold)
     {
        cout<<"Setting boundaries"<<endl;
	cin>>startx;
        cout<<" , ";
        cin>>starty;
        cout<<")"<<"\n"<<endl;
        cout<<"Enter goal: (";
        cin>>goalx;
        cout<<" , ";
        cin>>goaly;
        cout<<")"<<"\n"<<endl;
         {
            // Lock the mutex and update the shared variables
            std::lock_guard<std::mutex> lock(input_mutex);
            input_ready = true;
        }

        // Notify the path planning thread
        cv.notify_one();
     }
     }
}

void userinput()
{
	while(true)
	{
        std::unique_lock<std::mutex> lock(input_mutex);
        cv.wait(lock, [] { return input_ready; });
        if(startx<gridmap.min_x || starty<gridmap.min_y || goalx>gridmap.max_x || goaly>gridmap.max_y)
        {
             cout << "Out of bound query. Valid range: ("<< gridmap.min_x << ", " << gridmap.min_y << ") to (" << gridmap.max_x << ", " << gridmap.max_y << ")" << endl;
        }
         else
        {
        Node start(startx,starty);
        Node goal(goalx,goaly);

        cout<<"Start and goal node set astar starts"<<endl;

        vector<Node>path = astar(gridmap.occupancy_grid,start,goal);
        if(path.empty())
        {
            cout<<"No path found"<<endl;
        }
        else
        {
        cout<<"Path found"<<endl;
        for(const Node&node:path)
        {
                cout<<"("<<node.x<<","<<node.y<<")";
        }
        cout<<endl;
}

        }
input_ready = false;
	}
}
int main()
{
    // Start the threads
    std::thread realsense_thread(realsense);
    std::thread gridmap_thread_func(gridmap_thread);
    std::thread pathplanning_thread(pathplanning);
    std::thread userinput_thread(userinput);

    // Join threads
    realsense_thread.join();
    gridmap_thread_func.join();
    pathplanning_thread.join();
    userinput_thread.join();
return 0;

}
