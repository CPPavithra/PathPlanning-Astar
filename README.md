# PathPlanning-Astar

##TASKS
- A star with user defined sample cost grid- DONE & WORKED
- with sample point cloud from interdomain through piping as it is a python file- DONE (didnt work due to latency)
- with sample point cloud using a csv file-  DONE & WORKED
- TO DO- inter real sense api
- TO DO- cpp code for grid map making
- TRY WITH INTEL REAL SENSE CAM

#Problems
-> While installing the realsense SDK, follow these steps https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
-> GO TO DEVELOPMENT BRANCH NOT THE MAIN BRANCH FOR UBUNTU 24.04 as main brach only supports till ubuntu 20
-> While patching to the kernel from the development branch, using the command ./scripts/patch-realsense-ubuntu-lts-hwe.sh an error for no ucv found will occur
-> This required either-
   1. Secure boot disabled
   2. Generating a key which can be put into MOK during boot
I tried with the key 6-7 times but it was never recognising it, so running the command with secure boot disabled is always better.
To disable secure boot, reboot and go to bios by pressing f10
-> Confirm if ucv can be accessed properly and then patch it to the kernel
-> Connect the realsense depth cam and then see if it is being recognised by the realsense-viewer

#makelist
-> The CMakeLists.txt is for glfw and librealsense code which is pointcloud.cpp which enables glfw window
-> To test it out I have another code pointcloud_saver.cpp which just opens the realsense-viewer and then saves the point cloud data to a ply file 
-> the gridmap.cpp file converts it to cpp file only. 

#TO CODE
-> gridmap.cpp should also save the occupancy grid as a png file
-> Test path planning on this
