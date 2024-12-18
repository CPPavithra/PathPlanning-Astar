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

struct pair_hash {
	template <typename T1, typename T2>
	std::size_t operator()(const std::pair<T1, T2>& p) const {
    	auto h1 = std::hash<T1>{}(p.first);  
    	auto h2 = std::hash<T2>{}(p.second);
    	return h1 ^ (h2 << 1);//to combine the 2 hashes
	}
}; //READ ABOYT THIS- cant do pair hash in hash table usually so we use this
struct CellCost {
    float cost;  //cost of the cell
  float proxcost;  //cost based on proximity
    bool visited;  //flag indicating if the cell has been visited
    bool proxvisited;

    //edit this if needed
    CellCost(float c = 0.0f,float pc=0.0f,  bool v = false, bool p = false) : cost(c),proxcost(pc), visited(v),proxvisited(p) {}
};
struct Gridmap {
	unordered_map<pair<int,int>,CellCost,pair_hash>occupancy_grid;
	float min_x,min_y,max_x,max_y;
	Gridmap()
	{
   	//
	}
	Gridmap(unordered_map<pair<int, int>,CellCost,pair_hash> grid, float min_x, float min_y, float max_x, float max_y)
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
float grid_resolution = 0.001f; //because the distance is in mm and we have to convert it o metre

void create_gridmap(Gridmap& gridmap,const vector<Vector3f>& points, const Pose& roverpose,float grid_resolution, float height=0.5f,float proxfactor=0.5)
{
/*	float min_x=roverpose.position.x()-5.0f;
	float max_x=roverpose.position.x()+5.0f;
	float min_y=roverpose.position.x()-5.0f;
	float max_y=roverpose.position.x()+5.0f;

	//to initialise;
	gridmap.min_x=min_x;
	gridmap.min_y=min_y;
	gridmap.max_x=max_x;
	gridmap.max_y=max_y;*/
        float boundary_threshold = 0.01f;
	if (roverpose.position.x()<gridmap.min_x+boundary_threshold) 
	{
           gridmap.min_x-=(grid_resolution*3.0f);
        }
        if (roverpose.position.x()>gridmap.max_x-boundary_threshold) 
	{
           gridmap.max_x+=(grid_resolution*3.0f);
        }
        if (roverpose.position.y()<gridmap.min_y+boundary_threshold) 
	{
           gridmap.min_y-=(grid_resolution*3.0f);
        }
        if (roverpose.position.y()>gridmap.max_y-boundary_threshold)
       	{
           gridmap.max_y+=(grid_resolution*3.0f);
        }
	float min_x=gridmap.min_x;
	float max_x=gridmap.max_x;
	float min_y=gridmap.min_y;
	float max_y=gridmap.max_y;

	/*gridmap.min_x = std::min(gridmap.min_x, roverpose.position.x() - 5.0f);
          gridmap.max_x = std::max(gridmap.max_x, roverpose.position.x() + 5.0f);
          gridmap.min_y = std::min(gridmap.min_y, roverpose.position.y() - 5.0f);
          gridmap.max_y = std::max(gridmap.max_y, roverpose.position.y() + 5.0f);
        */

       int xgridnum,ygridnum;
       xgridnum=static_cast<int>((max_x-min_x)/grid_resolution)+1; //adding one to ensure that all of the grids are counted
       ygridnum=static_cast<int>((max_y-min_y)/grid_resolution)+1;

      /*vector<vector<bool>>occupancy_grid(xgridnum, vector<bool>(ygridnum,false))*;/ //everything is initialised to be false at the start*/
       unordered_map<pair<int,int>,CellCost, pair_hash>updated_occupancy_grid=gridmap.occupancy_grid;
       int proxradius=3;

        //NORMALISED COORDINATES
   	int xpos=static_cast<int>((roverpose.position.x()-min_x)/grid_resolution);
   	int ypos=static_cast<int>((roverpose.position.y()-min_y)/grid_resolution); //to check which grid is it currently in
      	 
   	float rover_x=roverpose.position.x();
        float rover_y=roverpose.position.y();

      	// to verify if it is valid
        //OPTIONAL
	cout<<"Rover position (real-world): ("<<rover_x<<", "<<rover_y<<")"<<endl;

       // Optionally, round real-world coordinates to nearest grid cell for indexing
       float grid_x=round((rover_x-min_x)/grid_resolution) * grid_resolution + min_x;
       float grid_y=round((rover_y-min_y)/grid_resolution) * grid_resolution + min_y;

       std::cout << "Mapped grid position: (" << grid_x << ", " << grid_y << ")" << std::endl;
       
       cout<<"Height at ("<<rover_x<<" , "<<rover_y<<") ->"<<roverpose.position.z()<<"\n"<<endl;
       	float cost=0.0f;
       	if(roverpose.position.z()>height)
       	{
           	cost=10.0f; //very high=cant go cost is from range 0 to 10
       	}
       	else if(roverpose.position.z()>(height/2) &&roverpose.position.z()<=(height))
       	{
           	cost=5.0f;
       	}
       	else if(roverpose.position.z()>(height/4) && roverpose.position.z()<=(height/2))
       	{
           	cost=1.0f;
       	}

	std::cout << "Mapped grid position: (" << grid_x << ", " << grid_y << "), COST ->" <<cost<<"\n"<< std::endl;
	 
    
	// USE BIT MASK ENCODING TO CHECK IF THE NODE IS VISITED OR NOT, I have used visited and proxvisited boolean visited and proxvisited with the cost proxcost.
        pair<int, int> current = {rover_x, rover_y};
		std::cout << "Before updating: (" << current.first << ", " << current.second << ") -> Cost: " << cost << std::endl;
		CellCost& cell=updated_occupancy_grid[current];

		float proxcostupdate=cell.proxcost;
		 bool proxvupdate=cell.proxvisited;
		 
   // if (updated_occupancy_grid.find(current)!=updated_occupancy_grid.end()) {
        if (!cell.visited) {
            //update the cost AND mark them as visited to avoid re-iteration of that cell
            cell.cost += cost;  //adding new cost to the existing cost (the existing cost might be proximity cost= proxcost)
            cell.visited = true; //mark that cell as visited to avoid reiteration
	    cell.proxvisited = proxvupdate;
        }
   /* } else {
        //if not found in the occupancy grid then visit that node and then update it
        updated_occupancy_grid[current] = CellCost(cost,proxcostupdate,true,proxvupdate); // CellCost(float c = 0.0f, pc=0.0f, bool v = false, bool p = false) : cost(c),proxcost,(pc), visited(v),proxvisited(p)

    }*/
    
    
        cout<< "After Updating: ("<< current.first<< ", "<< current.second<< ") -> Cost: "<< cost<<endl;

    int prox=5; //edit as needed
    for (int dx=-prox; dx<=prox; ++dx) {
        for (int dy=-prox; dy <=prox ; ++dy) {
            if (dx == 0 && dy == 0) continue;  // Skip the current cell

            int neighbor_x = rover_x + dx;
            int neighbor_y = rover_y + dy;
            pair<int, int> neighbor = {neighbor_x, neighbor_y};

           //add new neighbor if not already in the grid
           if (updated_occupancy_grid.find(neighbor)==updated_occupancy_grid.end()) {
               updated_occupancy_grid[neighbor]=CellCost{0.0f, 0.0f, false, false};  //set cost as default
               std::cout<< "Added new neighbor: ("<< neighbor.first<< ", "<< neighbor.second<< ")" << std::endl;
           }

        CellCost& neighbor_cell=updated_occupancy_grid[neighbor];

        //calculate proximity cost
        float dist = sqrt(dx *dx + dy*dy)  /*grid_resolution*/;//euclidean distance between them. 
        float proxcost = ((proxfactor*5.0f)/(0.1f+dist));

	/*if(updated_occupancy_grid.find(neighbor) != updated_occupancy_grid.end()) {
	float proxcost = (proxfactor * neighbor_cell.cost) / (0.1f + dist);
	} */                                                           	//exponential calculation

        //log calculations
        cout<< "Before updating: ("<< neighbor.first<< ", "<< neighbor.second<< ") -> Cost: "<< neighbor_cell.cost<<endl;

        //update proximity cost only if not already updated
        if (!neighbor_cell.proxvisited) {
            neighbor_cell.proxcost = proxcost;
            neighbor_cell.cost += neighbor_cell.proxcost; //add proximity cost
            neighbor_cell.proxvisited = true;            //mark it as visited for proximity
        }

        //logging it on the terminal to check errors
        std::cout << "After updating: (" << neighbor.first << ", " << neighbor.second << ") -> Cost: " << neighbor_cell.cost << std::endl;
    }
}
/*    std::cout << "Updated occupancy grid size: " << updated_occupancy_grid.size() << std::endl;
    for (const auto& [key, value] : updated_occupancy_grid) 
    {
	std::cout << "Grid: (" << key.first << ", " << key.second << ") -> " << value.cost << std::endl;
    }
   gridmap.occupancy_grid=updated_occupancy_grid;
   std::cout << "After assignment: Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
   //updated_occupancy_grid[current]=CLOSED; //after everything

}*/
/////////////////////////
///IF I HAVE TO CHECK AND NO OVERWRITING 
for (const auto& neighbor : updated_occupancy_grid) {
    auto key = neighbor.first;
    auto& value = neighbor.second;

    // Check if the key already exists in the updated map
    auto it = updated_occupancy_grid.find(key);
    if (it == updated_occupancy_grid.end()) {
        updated_occupancy_grid.insert({key,value});
    }
}

// Print the grid size after updating
std::cout << "Updated occupancy grid size: " << updated_occupancy_grid.size() << std::endl;
for (const auto& [key, value] : updated_occupancy_grid) {
    std::cout << "Grid: (" << key.first << ", " << key.second << ") -> " << value.cost << std::endl;
}

gridmap.occupancy_grid = updated_occupancy_grid;
std::cout << "After assignment: Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
//////////////////////////////////////////////////////////
///
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


void draw_gridmap(const Gridmap& gridmap,const vector<Vector3f>& point_vectors, const Pose& roverpose, float grid_resolution, rerun::RecordingStream& rec)
{
    float min_x=(roverpose.position.x()-5.0f)/grid_resolution;
    float max_x=(roverpose.position.x()+5.0f)/grid_resolution;
    float min_y=(roverpose.position.y()-5.0f)/grid_resolution;
    float max_y=(roverpose.position.y()+5.0)/grid_resolution;
    float scale_factor = 1000.0f;  // I have put 1000 so that it is in mm
    std::cout << "Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
    if (gridmap.occupancy_grid.empty())
    {
        std::cout << "Error: Occupancy grid is empty!" << std::endl;
    }
    else
    {
        std::cout << "Occupancy grid has data!" << std::endl;
    }
    std::vector<rerun::Color> colors;
    std::vector<rerun::Position3D> roverposition;
    std::ostringstream table_data;
    //table_data << "Position (x, y) | Color (r, g, b) | Cost\n";
    for (const auto& entry : gridmap.occupancy_grid)
   {
    const auto& [coord, value] = entry;
    float grid_x = coord.first;
    float grid_y = coord.second;

    float scaled_rover_x = (roverpose.position.x()) * grid_resolution * scale_factor;
    float scaled_rover_y = (roverpose.position.y()) * grid_resolution * scale_factor;
    /*float scaled_x = (grid_x) * grid_resolution * scale_factor;
    float scaled_y = (grid_y) * grid_resolution * scale_factor;*/
    float scaled_x = grid_x; //JUST FOR NOW
   float scaled_y = grid_y; //JUST FOR NOW

   //SIMPLY 
   //float xpos = static_cast<float>((grid_x - min_x) / grid_resolution);
    //float ypos = static_cast<float>((grid_y - min_y) / grid_resolution);
    //float xpos = static_cast<float>(((grid_x*scale_factor) - min_x * grid_resolution) / grid_resolution);
    //float ypos = static_cast<float>(((grid_y*scale_factor) - min_y * grid_resolution) / grid_resolution);

     rerun::Color color = get_color_for_cost(value.cost);
    
     std::vector<rerun::Position3D> points = {rerun::Position3D{grid_x, grid_y, 0.0f}};

           /*table_data << "(" << coord.first << ", " << coord.second << ") | "
                   << "(" << (int)color.r() << ", " << (int)color.g() << ", " << (int)color.b() << ") | "
                   << value.cost << "\n";*/
     colors.push_back(color);
     std::string tag = "gridcell_(" + std::to_string(grid_x) + "," + std::to_string(grid_y) + ")_"+ std::to_string(value.cost);
     rec.log(tag, rerun::Points3D(points).with_colors({color}).with_radii({0.5f}));
    // Add the point and its corresponding color to the vectors
    //points.push_back({scaled_x, scaled_y, 0.000000000f});
}
//std::vector<rerun::Color> colorrover = rerun::demo::grid3d<rerun::Color, uint8_t>(255, 255, 10);
    //rec.log("gridcells", rerun::Points3D(points).with_colors(colors).with_radii({0.5f}));
    //rec.log("roverpos",rerun::Points3D(roverposition).with_colors(colorrover).with_radii({0.5f}));
    //rec.log("grid_table", rerun::TextLog(table_data.str()));

colors.clear();
}
//////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec) {
	return Eigen::Vector3f(rs2_vec.x, rs2_vec.y, rs2_vec.z);
}//helper function to convert rs2 to eigen vector3f


//function to update the rover's pose using IMU data
void update_rover_pose(Pose& pose, const Vector3f& accel_data, const Vector3f& gyro_data, float delta_time) {
    //gravity vector in the world frame
    Vector3f gravity(0.0f, 0.0f, -9.81f);
    //convert acceleration from the sensor frame to the world frame
    Vector3f accel_world=pose.orientation*accel_data;

    //subtract gravity from the acceleration
    accel_world=accel_world-gravity;
    //v=u+at
    pose.velocity=pose.velocity+accel_world*delta_time;

    //s=ut+0.5at^2
    Vector3f delta_position=(pose.velocity*delta_time)+(0.5f*accel_world*delta_time*delta_time);
    pose.position=pose.position+delta_position;

    //convert position from millimeters to meters if required
    pose.position=pose.position/1000.0f;

    //compute angular velocity from gyroscope data
    Vector3f angular_velocity=gyro_data*delta_time;

    //update orientation using a small-angle quaternion
    Quaternionf delta_q=Quaternionf(AngleAxisf(angular_velocity.norm(),angular_velocity.normalized()));
    pose.orientation=delta_q*pose.orientation;
    pose.orientation.normalize(); //normalize quaternion to prevent drift

    cout<< "Position: "<<pose.position.transpose() <<endl;
    cout<< "Velocity: " <<pose.velocity.transpose() <<endl;
}

//////////////////////////////////////////////////////////////////
//convert the realsense points to pcl point
pcl::PointCloud<pcl::PointXYZ>::Ptr convert_to_pcl(const std::vector<Eigen::Vector3f>& point_vectors) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : point_vectors) {
        cloud->points.emplace_back(point.x(), point.y(), point.z());
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized cloud
    return cloud;
}
///////////////////////////////////////


int main() {
   
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
        	return -1;
      }
      rs2::pointcloud pc;
      rs2::points points;

    	//for rover pose
      Pose rover_pose;
      rover_pose.position = Eigen::Vector3f(0, 0, 0);
      rover_pose.orientation = Eigen::Matrix3f::Identity();
      rover_pose.velocity = Eigen::Vector3f(0, 0, 0);

	//for time
     auto last_time = std::chrono::high_resolution_clock::now();

     vector<Vector3f> point_vectors;

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

    //passthrough filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcl_cloud);
    passthrough.setFilterFieldName("z");  //this is based on depth
    passthrough.setFilterLimits(0.5, 5.0); //range in metres. 
    passthrough.filter(*passthrough_cloud);
    cout<<"After filtering AFTER PASSTHROUGH: "<<passthrough_cloud->size()<<" points."<<"\n"<<endl;

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

    cout<<"Generated PointCloud: "<< points.size()<< " points."<<"\n"<<endl;
    cout<<"After filtering FULLY: "<<filtered_cloud->size()<<" points."<<"\n"<<endl;

     	draw_gridmap(gridmap,point_vectors, rover_pose, grid_resolution, rec);
        //std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
	return 0;
}

