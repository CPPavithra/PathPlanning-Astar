# PathPlanning-Astar

__________________________________________________________
#THREADS

We are implementing multithreading in the main function because the disparity in timing between data acquisition, grid map creation,
and user input handling can cause synchronization issues, leading to premature loop exits or improper execution.

Thread 1: Data Acquisition

    Capture data from the Intel RealSense Depth Camera.
    Convert depth data into a point cloud format.
    Pass the point cloud data to the grid map generation function.

Thread 2: Grid Map Creation

    Process the point cloud data and generate a grid map.
    Update the grid map in a shared resource safely.

Thread 3: A Pathfinding*

    Wait for valid start and goal inputs from the user.
    Use the latest grid map to calculate the path between the start and goal nodes.

Thread 4: User Input

    Accept and validate the user’s start and goal node inputs.
    Signal readiness to the A* thread when input is complete.

________________________________________________________
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

#to use with rerun
-> Usually rerun is on python/Rust
-> To run it in C++, in the documentation, a CMakeLists.txt is given with the following command
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)
-> With this we will have to clone into the repo and build everytime we test or run it
-> Instead of this build directly from the repo 
Commands for it
-------------

    git clone https://github.com/rerun-io/rerun.git
    cd rerun
    mkdir build
    cd build
    cmake ..
    make
    sudo make install

-----------

Now, for this we might need rust as it will show error while building

    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    source $HOME/.cargo/env

export the cargo path to $PATH variable to make it work

--------
#makelist
-> The CMakeLists.txt is for glfw and librealsense code which is pointcloud.cpp which enables glfw window
-> To test it out I have another code pointcloud_saver.cpp which just opens the realsense-viewer and then saves the point cloud data to a ply file 
-> the gridmap.cpp file converts it to cpp file only. 

#TO CODE
-> gridmap.cpp should also save the occupancy grid as a png file
-> Test path planning on this

_________________

We use BITMASK ENCODING technique, check in rerunpointcloud.cpp

---------------------
TO DO -
add multithreading and parallel processing for capturing and processing of frames
