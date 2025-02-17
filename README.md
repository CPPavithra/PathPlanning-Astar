 PathPlanning-Astar


##THIS REPO CONTAINS
- Path Planning Code- Astar
- Gridmap creation (occupancy grid map using sparse occupancy grid hash tables)
- Main code which integrates path planning and grridmap (real time updates)
- Using intel realsense depth cam d345i
- Header files facilitating these
- Integration with UKF

## Table of Contents
- [Overview](#overview)
- [Astar](#astar)
- [Gridmap creation](#gridmap-creation)
- [Integrating with UKF](#ukf-integration)
- [Intel Realsense Explanation](#intel-realsense)
- [Main Code Explanation](#main-code)
- [Future Improvements](future-improvements)

---

## Overview

This project involves path planning using the Astar algorithm which essentially follows the cost approach for the movement cost along with addition of user defined obstacle costs based on the height/vicinity of the obstacles defined through the gridmap. The occupancy grid map is a 3km x 3km grid with a grid resolution of 1m x 1m. This is made using the height threshold from the point cloud data (y axis) and variable cost for each grid based on this. Neighbour cells cost has also been implemented. The grid map is being updated real time per some amount of frames based on change in orientation and position of the rover. (position and orientation should be taken from UKF). The main code integrates both of this and it calls the astar function after the occupancy grid reaches a certain size. The grid map is visualised in rerun io for the user's ease in debugging. 


## Astar

I have implemented Astar algorithm based on the following-
-> Movement cost (using the traditional astar method)
-> Obstacle cost (takes the obstacle cost based on the occupancy grid made (the value of each key x,y of the hash table))
-> Total gcost=movement cost+obstacle cost
-> Path formed based on the cost and only for the boundaries. If the user queries and out of bounds node it will show an error


## Gridmap Creation


## Repo Guidance
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

    git clone https://github.com/rerun-io/rerun.git
    cd rerun
    mkdir build
    cd build
    cmake ..
    make
    sudo make install


Now, for this we might need rust as it will show error while building

    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    source $HOME/.cargo/env

export the cargo path to $PATH variable to make it work

- Main files-> the main files for real time testing
- Motor Car-> Test code for the motor car using L298N
- rrtstar1-> TEST code with arduino and serial connection (NOT THE FINAL CODE)
- rrtwithimu-> Test code for imu calibiration and kf (NOT THE FINAL CODE)
- printing output-> Code for visualisation
- sensor codes-> Sensor codes for imu-135
- FINAL CODE-> The code used for testing with the motor car+printing out the coordinates to test if it is being sent through serial connection or not.
- NOTE- the valid.py code is just a sample code to check which is the valid empty coordinates in the SLAM data set so that I can put the inital nodes and the goal nodes.

## Future Improvements

- Test the PID that has been included from motor control
- Make the obstacle detection using grid map better; right now I have used the logic of height threshhold (z axis) to help the rover navigate through and avoid the obstacles in sight.
The theshhold is set to 0.1, change it accordingly
- Remote Piloting
- Increase the threshold to reduce the error margin

