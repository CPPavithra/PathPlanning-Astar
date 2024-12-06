#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>

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

struct state { double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; texture tex; };

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
   
   for(const auto& points:points)
   {
	   int xpos=static_cast<int>((points(0)-min_x)/grid_resolution);
	   int ypos=static_cast<int>((points(1)-min_y)/grid_resolution); //to check which grid is it currently in
           
	   if(xpos>=0 && xpos<xgridnum && ypos>=0 && ypos<ygridnum)
	   {
		   if(pos(2)>height)
		   {
			   occupancy_grid[xpos][ypos]=true; //height threshhold to be two
		   }
	   }
   }	   
   return(occupancy_grid,min_x,min_y,max_x,max_y);
}

void draw_gridmap(const Gridmap& gridmap)
{
   float grid_resolution=(gridmap.max_x - gridmsp.min_x)/gridmap.occupancy_grid.size();

   for(size_t x=0;x<gridmap.occupancy_grid.size();++x)
   {
	   for(size_t y=0;y<gridmap.occupancy_grid.size();++y)
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

void roverupdate()
{

}
int main()
{

}
