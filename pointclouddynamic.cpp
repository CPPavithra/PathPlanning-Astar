#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <chrono>

//chrono is for time

using namespace std;
using namespace Eigen;
using Eigen::Vector3f;
using Eigen::Matrix3f;

struct Gridmap {
    vector<vector<bool>> occupancy_grid;
    float min_x,min_y,max_x,max_y;
};

struct Pose 
{
	Eigen::Vector3f position;
	Eigen::Matrix3f orientation;
};

/*struct state { double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; texture tex; };*/

Gridmap create_gridmap(const vector<Vector3f>& points, const Pose& roverpose,float grid_resolution, float height=2.0)
{
	float min_x=roverpose.position[0]-5.0f;
	float min_y=roverpose.position[0]+5.0f;
	float max_x=roverpose.position[1]-5.0f;
	float max_y=roverpose.position[1]+5.0f;
	
   int xgridnum,ygridnum;
   xgridnum=static_cast<int>((max_x-min_x)/grid_resolution)+1; //adding one to ensure that all of the grids are counted
   ygridnum=static_cast<int>((max_y-min_y)/grid_resolution)+1;

   vector<vector<bool>>occupancy_grid(xgridnum, vector<bool>(ygridnum,false)); //everything is initialised to be false at the start
   
   for(const auto& point :points)
   {
	   int xpos=static_cast<int>((point(0)-min_x)/grid_resolution);
	   int ypos=static_cast<int>((point(1)-min_y)/grid_resolution); //to check which grid is it currently in
           
	   if(xpos>=0 && xpos<xgridnum && ypos>=0 && ypos<ygridnum)
	   {
		   if(point(2)>height)
		   {
			   occupancy_grid[xpos][ypos]=true; //height threshhold to be two
		   }
	   }
   }	   
   return Gridmap{occupancy_grid,min_x,min_y,max_x,max_y};
}

void draw_gridmap(const Gridmap& gridmap)
{
   float grid_resolution=(gridmap.max_x - gridmap.min_x)/gridmap.occupancy_grid.size();

   for(size_t x=0;x<gridmap.occupancy_grid.size();++x)
   {
	   for(size_t y=0;y<gridmap.occupancy_grid[x].size();++y)
	   {
		   if(gridmap.occupancy_grid[x][y])
		   {
			   float x_pos=gridmap.min_x+x*grid_resolution;
			   float y_pos=gridmap.min_y+y*grid_resolution;

			   glBegin(GL_QUADS);
                           glColor3f(1.0f, 0.0f, 0.0f); // Red for occupied
                           glVertex2f(x_pos, y_pos);
                           glVertex2f(x_pos + grid_resolution, y_pos);
                           glVertex2f(x_pos + grid_resolution, y_pos + grid_resolution);
                           glVertex2f(x_pos, y_pos + grid_resolution);
                           glEnd();
		   }
	   }
   }
}

/*void draw_gridmap(const Gridmap& gridmap) {
    float cell_width = 2.0f / gridmap.occupancy_grid.size();
    float cell_height = 2.0f / gridmap.occupancy_grid[0].size();

    glBegin(GL_QUADS);
    for (size_t i = 0; i < gridmap.occupancy_grid.size(); i++) {
        for (size_t j = 0; j < gridmap.occupancy_grid[0].size(); j++) {
            if (gridmap.occupancy_grid[i][j]) {
                glColor3f(1.0f, 0.0f, 0.0f); // Red for occupied
            } else {
                glColor3f(1.0f, 1.0f, 1.0f); // White for free
            }

            float x_start = -1.0f + i * cell_width;
            float y_start = -1.0f + j * cell_height;

            glVertex2f(x_start, y_start);
            glVertex2f(x_start + cell_width, y_start);
            glVertex2f(x_start + cell_width, y_start + cell_height);
            glVertex2f(x_start, y_start + cell_height);
        }
    }
    glEnd();
}*/

void draw_axes()
{
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex2f(-1.0f, 0.0f);
    glVertex2f(1.0f, 0.0f);
    glVertex2f(0.0f, -1.0f);
    glVertex2f(0.0f, 1.0f);
    glEnd();
}

void update_rover_pose(Pose& pose, const rs2_pose& pose_data, float delta_time) {
    // Update position using translation data from pose_data
    pose.position += Eigen::Vector3f(pose_data.translation.x, 
                                     pose_data.translation.y, 
                                     pose_data.translation.z);

    // Update orientation using quaternion rotation from pose_data
    Eigen::Quaternionf rotation_quat(pose_data.rotation.w, 
                                      pose_data.rotation.x, 
                                      pose_data.rotation.y, 
                                      pose_data.rotation.z);

    // Convert quaternion to rotation matrix
    Eigen::Matrix3f rotation_matrix = rotation_quat.toRotationMatrix();

    // Combine the current orientation with the new rotation
    pose.orientation = rotation_matrix * pose.orientation;

    // Log the updated pose for debugging
    std::cout << "Updated Position: " << pose.position.transpose() << " (meters)\n";
    std::cout << "Updated Orientation:\n" << pose.orientation << "\n";
}

 int main() {
    if (!glfwInit()) {
        cerr << "Failed to initialize GLFW" << endl;
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(800, 600, "Dynamic Grid Map", nullptr, nullptr);
    if (!window) {
        cerr << "Failed to create GLFW window" << endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    rs2::pipeline pipe;
    rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    /*cfg.enable_stream(RS2_STREAM_POSE);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_device_from_file("outdoors.bag");
    pipe.start(cfg);
    rs2::pointcloud pc;
    rs2::points points;*/

        // Enable device from file first
        cfg.enable_device_from_file("outdoors.bag");

        // Then enable the streams you need
        cfg.enable_stream(RS2_STREAM_POSE);
        cfg.enable_stream(RS2_STREAM_DEPTH);

        pipe.start(cfg);

        rs2::pointcloud pc;
        rs2::points points;

    Pose rover_pose;
    rover_pose.position = Eigen::Vector3f(0, 0, 0);
    rover_pose.orientation = Eigen::Matrix3f::Identity();
    
    auto last_time = std::chrono::high_resolution_clock::now();


    vector<Vector3f> point_vectors;
    Gridmap gridmap;

    float grid_resolution = 0.1f;
    //float last_time = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        /*float current_time = glfwGetTime();
        float delta_time = current_time - last_time;
        last_time = current_time;*/

        glClear(GL_COLOR_BUFFER_BIT);
        glLoadIdentity();

        // Update rover pose
        //roverupdate(rover_pose, delta_time);

        // Get the next frame
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
	auto f = frames.first_or_default(RS2_STREAM_POSE); //FROM DOCUMENTATION - Get a frame from the pose stream
	// Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
	
	//UPDATE
	auto current_time = std::chrono::high_resolution_clock::now();
        float delta_time = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;

        // Update rover pose
        update_rover_pose(rover_pose, pose_data, delta_time);

        pc.map_to(frames.get_color_frame());
        points = pc.calculate(depth);
      // Collect point cloud data
        point_vectors.clear();
        for (size_t i = 0; i < points.size(); ++i) {
            auto point = points.get_vertices()[i];
            if (point.z) {
                Eigen::Vector3f transformed_point = rover_pose.orientation * Vector3f(point.x, point.y, point.z) + rover_pose.position;
                point_vectors.push_back(transformed_point);
            }
        }

        // Create gridmap
        gridmap = create_gridmap(point_vectors, rover_pose, grid_resolution);

        // Draw gridmap and axes
        draw_gridmap(gridmap);
        draw_axes();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;

}
