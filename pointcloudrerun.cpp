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
        for (float value:data) {
            ComponentBatch batch;
            batches.push_back(batch);  
        }
        return batches;
    }
};

}//because rerun doesnt automatically take float values

/*struct GridCell {
    float cost;     // Stores the cost value
    bool processed; // Marks whether the cell has been processed

    GridCell() : cost(0.0f), processed(false) {}

        friend std::ostream& operator<<(std::ostream& os, const GridCell& cell) {
        os << "Cost: " << cell.cost << ", Processed: " << cell.processed;
        return os;
    }
};*/

struct pair_hash {
	template <typename T1, typename T2>
	std::size_t operator()(const std::pair<T1, T2>& p) const {
    	auto h1 = std::hash<T1>{}(p.first);  
    	auto h2 = std::hash<T2>{}(p.second);
    	return h1 ^ (h2 << 1);//to combine the 2 hashes
	}
}; //READ ABOYT THIS- cant do pair hash in hash table usually so we use this

struct Gridmap {
	unordered_map<pair<int,int>,float,pair_hash>occupancy_grid;
	float min_x,min_y,max_x,max_y;
	Gridmap()
	{
   	//
	}
	Gridmap(unordered_map<pair<int, int>,float,pair_hash> grid, float min_x, float min_y, float max_x, float max_y)
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
float grid_resolution = 0.001f;

void create_gridmap(Gridmap& gridmap,const vector<Vector3f>& points, const Pose& roverpose,float grid_resolution, float height=0.2f,float proxfactor=0.5)
{
	float min_x=roverpose.position.x()-5.0f;
	float max_x=roverpose.position.x()+5.0f;
	float min_y=roverpose.position.x()-5.0f;
	float max_y=roverpose.position.x()+5.0f;

	//to initialise;
	gridmap.min_x = min_x;
	gridmap.min_y = min_y;
	gridmap.max_x = max_x;
	gridmap.max_y = max_y;

       int xgridnum,ygridnum;
       xgridnum=static_cast<int>((max_x-min_x)/grid_resolution)+1; //adding one to ensure that all of the grids are counted
       ygridnum=static_cast<int>((max_y-min_y)/grid_resolution)+1;

      /*vector<vector<bool>>occupancy_grid(xgridnum, vector<bool>(ygridnum,false))*;/ //everything is initialised to be false at the start*/
       unordered_map<pair<int,int>,float, pair_hash>updated_occupancy_grid = gridmap.occupancy_grid;
       int proxradius=3;

        //NORMALISED COORDINATES
   	int xpos=static_cast<int>((roverpose.position.x()-min_x)/grid_resolution);
   	int ypos=static_cast<int>((roverpose.position.y()-min_y)/grid_resolution); //to check which grid is it currently in
      	 
   	float rover_x=roverpose.position.x();
        float rover_y=roverpose.position.y();

      	// to verify if it is valid
        //OPTIONAL
	std::cout << "Rover position (real-world): (" << rover_x << ", " << rover_y << ")" << std::endl;

       // Optionally, round real-world coordinates to nearest grid cell for indexing
       float grid_x = round((rover_x - min_x) / grid_resolution) * grid_resolution + min_x;
       float grid_y = round((rover_y - min_y) / grid_resolution) * grid_resolution + min_y;

       std::cout << "Mapped grid position: (" << grid_x << ", " << grid_y << ")" << std::endl;
       
       cout<<"Height at ("<<rover_x<<" , "<<rover_y<<") ->"<<roverpose.position.z()<<"\n"<<endl;
       	float cost=0.0f;
       	if((-1)*roverpose.position.z()>height)
       	{
           	cost=10.0f; //very high= cant go cost is from range 0 to 10
       	}
       	else if((-1)*roverpose.position.z()>(height/2) && (-1)*roverpose.position.z()<=(height))
       	{
           	cost=5.0f;
       	}
       	else if((-1)*roverpose.position.z()>(height/4) && (-1)*roverpose.position.z()<=(height/2))
       	{
           	cost=1.0f;
       	}

	std::cout << "Mapped grid position: (" << grid_x << ", " << grid_y << "), COST ->" <<cost<<"\n"<< std::endl;

        
       	//to add the obstacles to the list
         //updated_occupancy_grid[{rover_x,rover_y}]=cost;
	
         //if we are not taking proximity then just do occupancy_grid{[xpos,ypos}]=cost
	 //
	 
       
// Check if the cell has been processed
      	 const float CLOSED = -1.0f;
        pair<int,int>current={rover_x,rover_y};
        if(updated_occupancy_grid[current]!=CLOSED)
        {
		updated_occupancy_grid[current]+=cost;
	}

       	

              	for(int dx=-proxradius;dx<=proxradius;++dx)
       	{
           	for(int dy=-proxradius;dy<=proxradius;++dy)
           	{
               	int nx=rover_x+dx;
               	int ny=rover_y+dy;
		int nx1=xpos+dx;
		int ny2=ypos+dy;
               	pair<int,int>neighbour={nx,ny};
               if (updated_occupancy_grid[neighbour] == CLOSED) {
                    continue;
                }
	       else
	       {
               	float dist=sqrt(dx*dx+dy*dy)*grid_resolution;
               	if(dist<proxradius*grid_resolution)
               	{
                   	float proxcost=proxfactor*(1.0f/(dist+0.1f));//add 0.1 to avoid division by zero if ever it happens
                   	if (updated_occupancy_grid.find(neighbour) != updated_occupancy_grid.end()) { // if the key exists, the iterator will not be equal to end
                                    updated_occupancy_grid[neighbour] = proxcost;
                          } //ensure it exists before updating before unintended entries
		}
               	}
           	}
       	}

    std::cout << "Updated occupancy grid size: " << updated_occupancy_grid.size() << std::endl;
    for (const auto& [key, value] : updated_occupancy_grid) 
    {
	std::cout << "Grid: (" << key.first << ", " << key.second << ") -> " << value << std::endl;
    }

   /*Gridmap gridmap(occupancy_grid,min_x,min_y,max_x,max_y);
   return gridmap;*/
   gridmap.occupancy_grid=updated_occupancy_grid;
   //gridmap.occupancy_grid=draw_occupancy_grid;
   std::cout << "After assignment: Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
   updated_occupancy_grid[current]=CLOSED; //after everything
}

//color based on cost
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
     
 	   /*cout<<"WORLD COORDINATES: ("<<xpos<<" , "<<ypos<<")"<<"\n"<<endl;

          //normalized in range[-100.0, 100.0] //CHECK THE BRACKETS PROPERLY OTHERWISE THERE COULD BE ERRORS
	  float oldrange_x = max_x-min_x;
	  float oldrange_y = max_y-min_y;
	  float newrange = 200.0f;
          //float normx = (((xpos-min_x)*newrange)/oldrange_x)-100.0;
          //float normy = (((ypos-min_y)*newrange)/oldrange_y)-100.0;
           
	  float normx = (xpos-min_x)/(oldrange_x);
	  float normy = (ypos-min_y)/(oldrange_y);

	  //int normx=static_cast<int>((xpos-min_x)/grid_resolution);
          //int normy=static_cast<int>((ypos-min_y)/grid_resolution);

           cout<<"Before normalising: ("<<xpos<<","<<ypos<<")"<<"\n"<<endl;
           cout<<"After normalising: ("<<normx<<","<<normy<<")"<<"\n"<<endl;

	   //SCALING
	   float gridx = (-100.0f + (200.0f*normx))*grid_resolution;
	   float gridy = (-100.0f + (200.0f*normy))*grid_resolution;

           cout<<"TO BE ENTERED: "<<gridx<<","<<gridy<<"\n"<<endl;           */      
     
 
 void draw_gridmap(const Gridmap& gridmap, const Pose& roverpose, float grid_resolution, rerun::RecordingStream& rec)
{        
    float min_x = (roverpose.position.x()-10.0f)/grid_resolution;
    float max_x = (roverpose.position.x()+10.0f)/grid_resolution;
    float min_y = (roverpose.position.y()-10.0f)/grid_resolution;
    float max_y = (roverpose.position.y()+10.0f)/grid_resolution;

    // Adjust the grid range based on rover's position
    /*if (roverpose.position.x() < min_x)
    {
    min_x = (roverpose.position.x()-10.0f)/grid_resolution;
    max_x = (roverpose.position.x()+10.0f)/grid_resolution;

    }
    else if (roverpose.position.x() > max_x)
    {
        max_x = roverpose.position.x() + 10.0f;
        min_x = roverpose.position.x() - 10.0f;
    }

    if (roverpose.position.y() < min_y)
    {
        min_y = roverpose.position.y() - 10.0f;
        max_y = roverpose.position.y() + 10.0f;
    }
    else if (roverpose.position.y() > max_y)
    {
        max_y = roverpose.position.y() + 10.0f;
        min_y = roverpose.position.y() - 10.0f;
    }*/

    // Optionally, scale the rover's position for the Rerun viewer
    // If the Rerun viewer requires scaling, adjust here.
    // For example, let's assume we want to scale by a factor of 100 for better visualization in Rerun.
    // Scaling factor should be chosen based on how large the grid should appear in the viewer
    float scale_factor = 1000.0f;  // Example scaling factor (adjust as necessary)
				   // I have put 1000 so that it is in mm

    // Apply scaling to the position to map it to the Rerun viewer grid
    /*min_x *= scale_factor;
    max_x *= scale_factor;
    min_y *= scale_factor;
    max_y *= scale_factor;*/

    std::cout << "Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
    if (gridmap.occupancy_grid.empty())
    {
        std::cout << "Error: Occupancy grid is empty!" << std::endl;
    }
    else
    {
        std::cout << "Occupancy grid has data!" << std::endl;
    }

    // Initialize empty vectors for points and colors
    std::vector<rerun::Position3D> points;
    std::vector<rerun::Color> colors;
    std::vector<rerun::Position3D> roverposition;

    for (const auto& entry : gridmap.occupancy_grid)
   {
    const auto& [coord, cost] = entry;
    float grid_x = coord.first;
    float grid_y = coord.second;
    

   // cout<<"TO LOG: ("<<grid_x<<","<<grid_y<<")"<<"\n"<<endl; 

    float scaled_rover_x = (roverpose.position.x()) * grid_resolution * scale_factor;
    float scaled_rover_y = (roverpose.position.y()) * grid_resolution * scale_factor;
    /*float scaled_x = (grid_x) * grid_resolution * scale_factor;
    float scaled_y = (grid_y) * grid_resolution * scale_factor;*/
    float scaled_x = grid_x; // Temporarily skip scaling
   float scaled_y = grid_y; // Temporarily skip scaling
	

   // cout<<"ROVER: ("<<scaled_rover_x<<","<<scaled_rover_y<<") & SCALED GRID: ("<<scaled_x<<","<<scaled_y<<")"<<"\n"<<endl;

   //float xpos = static_cast<float>((grid_x - min_x) / grid_resolution);
    //float ypos = static_cast<float>((grid_y - min_y) / grid_resolution);
    
    //float xpos = static_cast<float>(((grid_x*scale_factor) - min_x * grid_resolution) / grid_resolution);
    //float ypos = static_cast<float>(((grid_y*scale_factor) - min_y * grid_resolution) / grid_resolution);
    rerun::Color color = get_color_for_cost(cost);  

    // Add the point and its corresponding color to the vectors
    points.push_back({scaled_x, scaled_y, 0.0f});
    colors.push_back(color);
    roverposition.push_back({scaled_rover_x,scaled_rover_y,0.0f,});
}

std::vector<rerun::Color> colorrover = rerun::demo::grid3d<rerun::Color, uint8_t>(255, 255, 10);

// Log points if they are not empty
if (!points.empty() && !colors.empty()) {
    /*for(const auto &point:points)
    {
	    cout<<" Logging: ("<<point.x()<<","<<point.y()<<","<<point.z()<<endl;
    }*/
    rec.log("gridcells", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));
    rec.log("roverpos",rerun::Points3D(roverposition).with_colors(colorrover).with_radii({0.5f}));
} else {
    if (points.empty()) {
        std::cerr << "Skipping logging due to empty points" << std::endl;
    }
    if (colors.empty()) {
        std::cerr << "Skipping logging due to empty colors" << std::endl;
    }

  
}

// Clear points and colors after logging
points.clear();
colors.clear();

}



Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec) {
	return Eigen::Vector3f(rs2_vec.x, rs2_vec.y, rs2_vec.z);
}//helper function to convert rs2 to eigen vector3f

void update_rover_pose(Pose& pose,Vector3f &accel_data,Vector3f &gyro_data, float delta_time) {

	pose.velocity+=accel_data*delta_time; //assume that the initial velocity can be non zero
	Vector3f del_position = (pose.velocity*delta_time)+(0.5f*accel_data*delta_time*delta_time); //(using the motion eq ut+1/2at^2)

	pose.position+=del_position; //for position
				   
	pose.position /= 1000.0f; //because in intel realsense acceleration is in m/s2 and position is in mm so converting mm to m

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

