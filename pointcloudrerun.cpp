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
#include <cstdlib>
#include <rerun/demo_utils.hpp>


//chrono is for time
using namespace rerun;
using namespace std;
using namespace Eigen;
using namespace rs2;

namespace rerun {

template <>
struct AsComponents<float> {
    static std::vector<float> serialize(float value) {
        return {value};
    }
};
}

namespace rerun {

template <>
struct AsComponents<std::vector<float>> {
    static std::vector<ComponentBatch> serialize(const std::vector<float>& data) {
        std::vector<ComponentBatch> batches;
        for (float value : data) {
            ComponentBatch batch;
            //batch.data.push_back(value); // Add data to the batch
            batches.push_back(batch);   // Push batch to the vector
        }
        return batches;
    }
};

} // namespace rerun

 //because rerun doesnt automatically take float values


struct pair_hash {
	template <typename T1, typename T2>
	std::size_t operator()(const std::pair<T1, T2>& p) const {
    	auto h1 = std::hash<T1>{}(p.first);  // Hash the first element
    	auto h2 = std::hash<T2>{}(p.second); // Hash the second element
    	return h1 ^ (h2 << 1);           	// Combine the two hashes
	}
}; //READ ABOYT THIS- cant do pair hash in hash table usually so we use this

struct Gridmap {
	unordered_map<pair<int,int>,float,pair_hash>occupancy_grid;
	float min_x,min_y,max_x,max_y;
	Gridmap()
	{
   	//
	}
	Gridmap(unordered_map<pair<int, int>, float, pair_hash> grid, float min_x, float min_y, float max_x, float max_y)
    	: occupancy_grid(grid), min_x(min_x), min_y(min_y), max_x(max_x), max_y(max_y) {
    	//
	}
};


struct Pose
{
	Vector3f position;
	Vector3f velocity;
	Matrix3f orientation;
};

//GLOBALLY
Gridmap gridmap;
float grid_resolution = 0.1f;

/*struct state { double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; texture tex; };*/

void create_gridmap(Gridmap& gridmap,const vector<Vector3f>& points, const Pose& roverpose,float grid_resolution, float height=2.0,float proxfactor=0.5)
{
	float min_x=roverpose.position[0]-5.0f;
	float max_x=roverpose.position[0]+5.0f;
	float min_y=roverpose.position[1]-5.0f;
	float max_y=roverpose.position[1]+5.0f;
	

	if (roverpose.position[0] < min_x) {
    min_x = roverpose.position[0] - 5.0f;
    max_x = roverpose.position[0] + 5.0f;  // Adjust max_x if the rover moves left
}
if (roverpose.position[0] > max_x) {
    max_x = roverpose.position[0] + 5.0f;
    min_x = roverpose.position[0] - 5.0f;  // Adjust min_x if the rover moves right
}

if (roverpose.position[1] < min_y) {
    min_y = roverpose.position[1] - 5.0f;
    max_y = roverpose.position[1] + 5.0f;  // Adjust max_y if the rover moves down
}
if (roverpose.position[1] > max_y) {
    max_y = roverpose.position[1] + 5.0f;
    min_y = roverpose.position[1] - 5.0f;  // Adjust min_y if the rover moves up
}
	//to initialise;
	gridmap.min_x = min_x;
	gridmap.min_y = min_y;
	gridmap.max_x = max_x;
	gridmap.max_y = max_y;

   int xgridnum,ygridnum;
   xgridnum=static_cast<int>((max_x-min_x)/grid_resolution)+1; //adding one to ensure that all of the grids are counted
   ygridnum=static_cast<int>((max_y-min_y)/grid_resolution)+1;

   /*vector<vector<bool>>occupancy_grid(xgridnum, vector<bool>(ygridnum,false))*;/ //everything is initialised to be false at the start*/
   unordered_map<pair<int,int>,float,pair_hash>updated_occupancy_grid = gridmap.occupancy_grid;
   int proxradius=3;

   	int xpos=static_cast<int>((roverpose.position[0]-min_x)/grid_resolution);
   	int ypos=static_cast<int>((roverpose.position[1]-min_y)/grid_resolution); //to check which grid is it currently in
      	 
   	std::cout << "xpos: " << xpos << ", ypos: " << ypos << std::endl;
   	std::cout << "minx: " << min_x << ", maxx: " << max_x << std::endl;
   	std::cout << "miny: " << min_y << ", maxy: " << max_y << std::endl;
      	// to verify if it is valid

   	if(xpos>=0 && xpos<xgridnum && ypos>=0 && ypos<ygridnum)
   	{
       	float cost=0.0f;
       	if(roverpose.position[2]>height)
       	{
           	cost=10.0f; //very high= cant go cost is from range 0 to 10
       	}
       	else if(roverpose.position[2]>(height/2)&&roverpose.position[2]<=(height))
       	{
           	cost=5.0f;
       	}
       	else if(roverpose.position[2]>(height/4) &&roverpose.position[2]<=(height/2))
       	{
           	cost=1.0f;
       	}

       	//to add the obstacles to the list
       	if(cost>0.0f)
       	{
         	updated_occupancy_grid[{xpos,ypos}]+=cost;
         	//if we are not taking proximity then just do occupancy_grid{[xpos,ypos}]=cost
       	}
       	for(int dx=-proxradius;dx<=proxradius;++dx)
       	{
           	for(int dy=-proxradius;dy<=proxradius;++dy)
           	{
               	int nx=xpos+dx;
               	int ny=ypos+dy;
               	pair<int,int>neighbour={nx,ny};
               	float dist=sqrt(dx*dx+dy*dy)*grid_resolution;
               	if(dist<proxradius*grid_resolution)
               	{
                   	float proxcost=proxfactor*(1.0f/(dist+0.1f));//add 0.1 to avoid division by zero if ever it happens
                   	updated_occupancy_grid[neighbour]+=proxcost;
               	}
           	}
       	}
    
   	}

std::cout << "Updated occupancy grid size: " << updated_occupancy_grid.size() << std::endl;
for (const auto& [key, value] : updated_occupancy_grid) {
	std::cout << "Grid: (" << key.first << ", " << key.second << ") -> " << value << std::endl;
}

   /*Gridmap gridmap(occupancy_grid,min_x,min_y,max_x,max_y);
   return gridmap;*/
gridmap.occupancy_grid=updated_occupancy_grid;
std::cout << "After assignment: Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
}

// Function to determine the color based on cost 
components::Color get_color_for_cost(float cost) 
{
 if (cost >= 10.0f)
 {
return components::Color{
    static_cast<uint8_t>(0.0f * 255), 
    static_cast<uint8_t>(0.0f * 255), 
    static_cast<uint8_t>(0.0f * 255)
};

 }  // BLACK=FULLY OCCUPIED
 else if (cost >= 5.0f) 
 {
return components::Color{
    static_cast<uint8_t>(1.0f * 255), 
    static_cast<uint8_t>(0.0f * 255), 
    static_cast<uint8_t>(0.0f * 255)
};
}// RED=MILD }
 else if (cost >= 1.0f)
{
    return components::Color{
    static_cast<uint8_t>(1.0f * 255), 
    static_cast<uint8_t>(0.8f * 255), 
    static_cast<uint8_t>(0.4f * 255)
};

}	// ORANGE=CAN GO } 
else
{
    return components::Color{
    static_cast<uint8_t>(0.6f * 255), 
    static_cast<uint8_t>(1.0f * 255), 
    static_cast<uint8_t>(0.6f * 255)
};

}	// LIGHT GREEN } 
}

void draw_gridmap(const Gridmap& gridmap, const Pose& rover,float grid_resolution, rerun::RecordingStream& rec)
{
std::cout << "Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
if (gridmap.occupancy_grid.empty()) {
    std::cout << "Error: Occupancy grid is empty!" << std::endl;
} else {
    std::cout << "Occupancy grid has data!" << std::endl;
}

std::vector<rerun::Position3D> points;  // Declare outside the loop to accumulate points
std::vector<rerun::Color> colors;

 //const auto rec = rerun::RecordingStream("gridmap_log");
 //rec.log("rover/pose", components::Pose3D{ rover.position.x(), rover.position.y(), 0.0f, 0.0f, 0.0f, atan2(rover.orientation(1, 0), rover.orientation(0, 0)) });
for (const auto& entry : gridmap.occupancy_grid)
 { 
const auto& [coord, cost] = entry;
 float xpos = coord.first;
 float ypos = coord.second;

cout<<"Before normalising: ("<<xpos<<","<<ypos<<")"<<endl; 
// Normalize positions
 float x_pos = -10.0f + 20.0f * (xpos - gridmap.min_x) / (gridmap.max_x - gridmap.min_x);
 float y_pos = -10.0f + 20.0f * (ypos - gridmap.min_y) / (gridmap.max_y - gridmap.min_y);


//components::Color cell_color = get_color_for_cost(cost)


/*std::vector<rerun::Points3D> points = {
    rerun::Points3D{x_pos, y_pos, 0.0f}
};*/

///////////

cout<<"3d positions to be entered: "<<xpos<<","<<ypos<<endl;


//std::vector<rerun::Position3D> points = rerun::demo::grid3d<rerun::Position3D,float>(x_pos, y_pos, 0.0f) ;
//std::vector<rerun::Color> colors = rerun::demo::grid3d<rerun::Color, uint8_t>(0, 255, 10);
//std::vector<rerun::Position3D> points = {{xpos, ypos, 0.0f}};
//std::vector<rerun::Color> colors = {get_color_for_cost(cost)};

    rerun::Color color = get_color_for_cost(cost);

    // Add the point and its corresponding color to the vectors
    points.push_back({xpos, ypos, 0.0f});
    colors.push_back(color);

std::cout << "Logging points: ";
for (const auto& point : points) {
    std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ") ";
}

rec.log("gridcells", rerun::Points3D(points).with_colors(colors).with_radii({0.01f}));

if (points.empty()) {
    std::cerr << "Skipping logging due to empty points" << std::endl;
}
if (colors.empty()) {
	std::cerr<<"Skipping logging due to empty colors"<<std::endl;
}
/*try {
    points = rerun::demo::grid3d<rerun::Position3D, float>(x_pos, y_pos, 0.0f);
} catch (const std::exception& e) {
    std::cerr << "Error generating 3D grid points: " << e.what() << std::endl;
    // Handle the error (e.g., assign default points or rethrow)
    points = {}; // Default to an empty vector
} catch (...) {
    std::cerr << "An unknown error occurred while generating 3D grid points." << std::endl;
    points = {}; // Default to an empty vector
}

try {
    colors = rerun::demo::grid3d<rerun::Color, uint8_t>(0, 255, 10);
} catch (const std::exception& e) {
    std::cerr << "Error generating colors: " << e.what() << std::endl;
    // Handle the error (e.g., assign default colors or rethrow)
    colors = {}; // Default to an empty vector
} catch (...) {
    std::cerr << "An unknown error occurred while generating colors." << std::endl;
    colors = {}; // Default to an empty vector
}

try {
    rec.log("grid_cells", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));
} catch (const std::exception& e) {
    std::cerr << "Error logging grid cells: " << e.what() << std::endl;
    // Handle the error (e.g., log an alternative message or rethrow)
} catch (...) {
    std::cerr << "An unknown error occurred while logging grid cells." << std::endl;
}*/

 // Log the cost of the cell as a scalar property
// rec.log("grid_cells/cost",cost);
 // Optional: Log additional metadata such as coordinates
 // rec.log("grid_cells/coordinate", components::Text{std::to_string(xpos) + ", " + std::to_string(ypos)}); }

}
}
// Optional: Show grid boundaries 
/*rec.log("gridmap/edges", components::LineStrip3D{ 
{gridmap.min_x, gridmap.min_y, 0.0f},
 {gridmap.max_x, gridmap.min_y, 0.0f},
 {gridmap.max_x, gridmap.max_y, 0.0f},
 {gridmap.min_x, gridmap.max_y, 0.0f},
 {gridmap.min_x, gridmap.min_y, 0.0f}, });*/






Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec) {
	return Eigen::Vector3f(rs2_vec.x, rs2_vec.y, rs2_vec.z);
}//helper function to convert rs2 to eigen vector3f

void update_rover_pose(Pose& pose,Vector3f &accel_data,Vector3f &gyro_data, float delta_time) {

	pose.velocity+=accel_data*delta_time; //assume that the initial velocity can be non zero
	Vector3f del_position = (pose.velocity*delta_time)+(0.5f*accel_data*delta_time*delta_time); //(using the motion eq ut+1/2at^2)

	pose.position+=del_position; //for position
	cout<<"Position: "<<pose.position<<endl;                     	 
	Vector3f ang_velocity= gyro_data*delta_time;
	cout<<"Angular velocity: "<<ang_velocity<<endl;

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
        
	auto rec = rerun::RecordingStream("gridmap");
	rec.spawn().exit_on_failure();

	// Initialize the RealSense pipeline
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
	return -1;
}
	rs2::pointcloud pc;
	rs2::points points;

    	// Initialize rover pose
	Pose rover_pose;
	rover_pose.position = Eigen::Vector3f(0, 0, 0);
	rover_pose.orientation = Eigen::Matrix3f::Identity();
	rover_pose.velocity = Eigen::Vector3f(0, 0, 0);

	// Initialize timing
	auto last_time = std::chrono::high_resolution_clock::now();

	vector<Vector3f> point_vectors;

       while (true){
    	// Get the current time and compute the delta time
    	auto current_time = std::chrono::high_resolution_clock::now();
    	float delta_time = std::chrono::duration<float>(current_time - last_time).count();
    	last_time = current_time;

    	// Declare variables to hold sensor data
    	rs2_vector accel_data = {0.0f, 0.0f, 0.0f};
    	rs2_vector gyro_data = {0.0f, 0.0f, 0.0f};

    	// Get frames from the RealSense camera
    	rs2::frameset frameset;
    	try {
        	frameset = pipe.wait_for_frames();
    	} catch (const rs2::error& e) {
        	std::cerr << "RealSense error: " << e.what() << std::endl;
        	continue;  
    	}

    	// Retrieve accelerometer data
    	if (rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL)) {
        	accel_data = accel_frame.get_motion_data();
    	} else {
        	std::cerr << "Failed to retrieve accelerometer data" << std::endl;
    	}

    	// Retrieve gyroscope data
    	if (rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO)) {
        	gyro_data = gyro_frame.get_motion_data();
    	} else {
        	std::cerr << "Failed to retrieve gyroscope data" << std::endl;
    	}

    	// Convert accelerometer and gyroscope data to Eigen vectors
    	Eigen::Vector3f accel_eigen = convert_to_eigen_vector(accel_data);
    	Eigen::Vector3f gyro_eigen = convert_to_eigen_vector(gyro_data);

    	// Update the rover pose with accelerometer and gyroscope data
    	update_rover_pose(rover_pose, accel_eigen, gyro_eigen, delta_time);

    	// Process depth data to create point cloud
    	rs2::depth_frame depth_frame = frameset.get_depth_frame();
        	points = pc.calculate(depth_frame);

        	// Collect point cloud data
        	point_vectors.clear();
        	for (size_t i = 0; i < points.size(); ++i) {
            	auto point = points.get_vertices()[i];
            	if (point.z) {
                	Eigen::Vector3f transformed_point=rover_pose.orientation*Eigen::Vector3f(point.x, point.y, point.z)+rover_pose.position;
                	point_vectors.push_back(transformed_point);
            	}
       	}
        	create_gridmap(gridmap, point_vectors, rover_pose, grid_resolution);

        	cout<<"Generated PointCloud: "<< points.size()<< " points."<< std::endl;

     	draw_gridmap(gridmap, rover_pose, grid_resolution, rec);
        //std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	return 0;
}
