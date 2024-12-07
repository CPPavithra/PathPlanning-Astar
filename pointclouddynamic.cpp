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
using namespace rs2;

struct Gridmap {
    vector<vector<bool>> occupancy_grid;
    float min_x,min_y,max_x,max_y;
};

struct Pose 
{
	Vector3f position;
	Matrix3f orientation;
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

void update_rover_pose(Pose& pose,Vector3f &accel_data,Vector3f &gyro_data, float delta_time) {

    pose.velocity+=accel_data*delta_time //assume that the initial velocity can be non zero
    Vector3f del_position = (pose.velocity*delta_time)+(0.5f*accel_data*delta_time*delta_time); //(using the motion eq ut+1/2at^2)

    pose.position=pose.position+del_position; //for position
					      
    Vector3f ang_velocity= gyro_data*delta_time;
    
    AngleAxisf roll(ang_velocity.x(), Vector3f::UnitX());
    AngleAxisf pitch(ang_velocity.y(), Vector3f::UnitY());
    AngleAxisf yaw(ang_velocity.z(), Vector3f::UnitZ());
    Matrix3f rotation_matrix = (yaw*pitch*roll).toRotationMatrix(); //changing it to a rot matrix
    
    pose.orientation = rotation_matrix * pose.orientation;

    //FOR INCREASING ACCURACY- but idk if I need to use it yet. Just in case;
    JacobiSVD<Matrix3f> svd(pose.orientation, ComputeFullU | ComputeFullV);
    pose.orientation = svd.matrixU() * svd.matrixV().transpose();

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

    cfg.enable_device_from_file("video2.bag");
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    pipe.start(cfg);

    rs2::pointcloud pc;
    rs2::points points;

    /*Pose rover_pose;
    rover_pose.position = Eigen::Vector3f(0, 0, 0);
    rover_pose.orientation = Eigen::Matrix3f::Identity();*/
    
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
	//auto f = frames.first_or_default(RS2_STREAM_POSE); //FROM DOCUMENTATION - Get a frame from the pose stream
	// Cast the frame to pose_frame and get its data
        //auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
	rs2::frameset frameset = pipe.wait_for_frames();
	    if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL))
    {
        rs2_vector accel_sample = accel_frame.get_motion_data();
                        // Print accelerometer data
                std::cout << std::fixed << std::setprecision(3)
                          << "Accel [m/s²]: X:" << accel_sample.x
                          << ", Y:" << accel_sample.y
                          << ", Z:" << accel_sample.z << std::endl;
    }

    if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO))
    {
        rs2_vector gyro_sample = gyro_frame.get_motion_data();
         // Print gyroscope data
                std::cout << std::fixed << std::setprecision(3)
                          << "Gyro [rad/s]: X:" << gyro_sample.x
                          << ", Y:" << gyro_sample.y
                          << ", Z:" << gyro_sample.z << std::endl;
    }
	//UPDATE
	auto current_time = std::chrono::high_resolution_clock::now();
        float delta_time = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;

        // Update rover pose
        //update_rover_pose(rover_pose, pose_data, delta_time);

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
