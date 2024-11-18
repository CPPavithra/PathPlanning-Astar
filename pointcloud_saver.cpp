#include <librealsense2/rs.hpp>  
#include "/usr/include/pcl-1.14/pcl/point_cloud.h"     
#include <pcl/point_types.h>     
#include <pcl/io/ply_io.h>     
#include <iostream>
#include <cstdlib>              

using namespace std;
using namespace rs2;

int main() {
    try {
        cout << "Launching RealSense Viewer..." <<endl;
        system("realsense-viewer &");  

        int i;
        pipeline pipe;
        pipe.start();

        //pointcloud object
        pointcloud pc;
        //points object to hold it
        points points;

        cout << "Capturing point cloud data..." <<endl;

        // to stabilise
        for (i=0;i<30;++i) {
            pipe.wait_for_frames();
        }

        auto frames=pipe.wait_for_frames();

        //depth frame
        auto depth=frames.get_depth_frame();

        // Generate the pointcloud
        points=pc.calculate(depth);

        // Convert to PCL point cloud format
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl_cloud.width = points.size();
        pcl_cloud.height = 1;  // Unorganized point cloud
        pcl_cloud.is_dense = false;
        pcl_cloud.points.resize(points.size());

        auto vertices = points.get_vertices();

        for (size_t i = 0; i < points.size(); ++i) {
            pcl_cloud.points[i].x = vertices[i].x;
            pcl_cloud.points[i].y = vertices[i].y;
            pcl_cloud.points[i].z = vertices[i].z;
        }

        // Save point cloud to a .ply file
        string output_file = "pointcloud.ply";
        pcl::io::savePLYFileASCII(output_file, pcl_cloud);

        std::cout << "Point cloud saved to '" << output_file << "'" << std::endl;

    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}

