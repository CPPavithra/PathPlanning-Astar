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

using namespace std;
struct pair_hash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ (h2 << 1);//to combine the 2 hashes
        }
};

/*struct Node
{
	float x,y;
	double g_cost,h_cost,f_cost;
	Node *parent;
	//using an initialiser list instead of this keyword
	Node(float x,float y, double g_cost=0,double h_cost=0,Node *parent=NULL): x(x), y(y), g_cost(g_cost), h_cost(h_cost), f_cost(g_cost+h_cost), parent(parent) {}
        bool operator>(const Node& other) const
       	{ 
		return f_cost > other.f_cost;
       	}//overloading > to comapre between final cost of the nodes

       /* Node(float x, int y, double g_cost=0, double h_cost=0, Node* parent=NULL) {
            this->x = x;            
            this->y = y;           
            this->g_cost = g_cost;            
            this->h_cost = h_cost;            
            this->f_cost = g_cost + h_cost;        
            this->parent = parent;  
          }

        bool operator>(const Node& other) const {
            return f > other.f;
         }*/
//};

/*struct comparenode {
    bool operator()(Node*a, Node*b)
    {
            return *a>*b;
    }
};*/

double heuristic(int x1, int y1, int x2, int y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
vector<Node>astar(const unordered_map<pair<int, int>, double, pair_hash>& occupancyGrid, Node start, Node goal)
{
   priority_queue<Node*,vector<Node*>,comparenode>openlist;//priority list for 
   unordered_map<pair<int,int>,Node*, pair_hash>allNodes;
   //unordered_map<int, Node*>allNodes;//map for all the nodes
   start.h_cost = heuristic(start.x, start.y, goal.x, goal.y);//euclidean distance function h cost
   start.f_cost = start.g_cost + start.h_cost;
   openlist.push(&start);

   while(!openlist.empty())
   {
     //stack
     Node*current=openlist.top();
     openlist.pop();//read one by one
    //(check if the node has higher g cost than in all nodes, if yes it is outdated)
    pair<int,int>currentkey={current->x,current->y};
    if(allNodes.find(currentkey) != allNodes.end() && current->g_cost>allNodes[currentkey]->g_cost)
    {
	    continue;
    }
     if(current->x == goal.x && current->y == goal.y)
     {
	     vector <Node> path;
	     for(Node *n=current;n!=NULL;n=n->parent)
	     {
		     path.push_back(*n);//error
	     }
	     reverse(path.begin(), path.end()); //reverse path
	     return path;
     }
    //for variable square grid size
    int dx[] = {1,-1,0,0,1,-1,1,-1};
    int dy[] = {0,0,1,-1,1,-1,-1,1}; 
//
//ADD 2 ENTITIES-> MOVEMENT_COST AND OBSTACLE_COST TO THE G_COST.
//OBSTACLE_COST IS CALCULATED FROM
   int i; double newg_cost;
   double movement_cost;
   for(i=0; i<8; i++)
   {
	   int newx = current->x+dx[i];
	   int newy  = current->y+dy[i];
           if(newx<minx||newx>maxx || newy<miny||newy>maxy ||grid[newx][newy] != 0)
	   {
		   continue;
	   }
	     /*if(i<4)//frist 4 in the array is for straight movements
	     {
		     newg_cost=current->g_cost+1.0;
	     }
	     else
	     {
                    newg_cost=current->g_cost+1.414;//diagonal cost will be root2
	     }*/
	     if(i<4)
	     {
		     movement_cost=1.0;
	     }
	     else
	     {
		     movement_cost=1.414;
	     }
	     double obstacle_cost=1e6; // Default cost for free cells
		     auto it = occupancyGrid.find({newx,newy});
		     if(it!=occupancyGrid.end()) 
		     {
			     obstacle_cost=it->second; //variable cost assigned (in grid and within bounds)
		     }
		     else
		     {
			     obstacle_cost=0.0; //free space (if not in grid but within the bounds)
		     }
		     if(obstacle_cost>=1e6)
		     {
			     continue;
		     }
	     
	     //infinite cost for unexplored regions (out of bounds)
	     newg_cost=current->g_cost+movement_cost+obstacle_cost; 
	     Node* neighbour = new Node(newx, newy, newg_cost, heuristic(newx, newy, goal.x, goal.y), current);//create a new neighbour node
             //newx+newy shows collision
	     //int key=newx*grid[0].size()+newy;
	     pair<int, int>neighbor_key = {newx, newy};
	     if(allNodes.find(neighbour_key)==allNodes.end() || neighbour -> g_cost<allNodes.find(neighbour_key) -> second -> g_cost)//newx+newy is the unique key for map
	     {
               neighbour->f_cost=neighbour->g_cost+neighbour->h_cost;
	       openlist.push(neighbour);
	       //update all nodes
	       allNodes[neighbour_key]=neighbour;
	     }

     } 
   }
}
return{}; //return empty if not found in while loop
}

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

pcl::PointCloud<pcl::PointXYZ>::Ptr convert_to_pcl(const std::vector<Eigen::Vector3f>& point_vectors) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : point_vectors) {
        cloud->points.emplace_back(point.x(), point.y(), point.z());
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized cloud
    return cloud;
}

/*int main(){
//SAMPLE GRID
/*vector<vector<int>> grid = {
 (0,0) {0, 0, 0, 0, 0},
  {0, 1, 1, 1, 1},
  {1, 0, 0, 0, 0},
  {0, 1, 1, 1, 0},
  {0, 0, 0, 0, 0}(4,4)
};*/

/*cout << "Reading grid map from CSV file" << endl;

// Read the grid map from a CSV file
string filename = "grid_map.csv"; // Replace with your CSV file name
vector<vector<int>> grid;
try {
    grid = readGridFromCSV(filename);
} catch (const exception& e) {
    cerr << "Error: " << e.what() << endl;
    return 1;
}
cout << "Grid map loaded" << endl;

    
double min_x = 0;
double max_x = grid[0].size();
double min_y = 0;
double max_y = grid.size();

cout<<"Setting boundaries"<<endl;

Node start(0,0);
Node goal(5,6);

cout<<"Start and goal node set astar starts"<<endl;

vector<Node>path = astar(grid,start,goal);
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
}*/

int main()
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

   /* cout<<"Generated PointCloud: "<< points.size()<< " points."<<"\n"<<endl;
    cout<<"After filtering FULLY: "<<filtered_cloud->size()<<" points."<<"\n"<<endl;*/
    //to draw gridmap only after sometime

     if(gridmap.occupancy_grid.size()>=batch_threshold)
     {
        draw_gridmap(gridmap,point_vectors, rover_pose, grid_resolution, rec);
        batch_threshold=batch_threshold+gridmap.occupancy_grid.size();
     }  //std::this_thread::sleep_for(std::chrono::milliseconds(30));
}
        return 0;

}
