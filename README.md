
---

# 🤖 Efficient Path Planning Using Quadtree, A* Search, and Sparse Hash Maps

This project is my complete implementation of a **memory-efficient path planning system** for robots. It combines **quadtree spatial mapping**, **A* algorithm**, and a **sparse hash table** to compute optimal paths over large terrain maps with obstacles.

---

## FILES NEEDED
Files that are needed for execution and what they do
-> From src/
  1. main.cpp - the main code executable- intialise rerun also
  2. mainloop.cpp- helper functions and other func containing COMPLETE mapping and path planning logic and workflow. setup camera frames and log.
  3. ArucoDetect.cpp- to detect the aruco tags
  4. astarquadtree.cpp- to run dense A* on quadtree mapping
  5. astar.cpp- to run sparse A* on sparse hash maps occupancy grid
  6. hashgridmap.cpp- to create the sparse hash table grid map from point cloud and also to log them to rerun. (There is a function to convert pointcloud to pcl also here)
  7. quadtree.cpp- to create the quadtree map and also to log it. Here, we are not calling the log function in main as it will be messy.

-> From include/
  1. mainloop.h- helper for mainloop.cpp
  2. ArucoDetect.h- helper for ArucoDetect.cpp
  3. common.h- the common struct definitions used everywhere (esp mapping) . extern has not been used here to avoid build complexity. Like the pair hash and cellcost structs.
  4. imu.h- header for drive function to move the rover and also to serialise it before COBS encoding
  5. pathplanning.h- ENTIRE pathplanning (quadtree+sparsemap)
  6. quadtree.h, quadtreecommon.h, rerun.h- all of these are for mapping. This can be merged in the future. However, it might be a little tricky as extern global vars has been used.

-> From lib/
  1. cobs.c, cobs.h- for COBS 
  2. gps.cpp, imu.cpp- for the drive

---

## WHAT IS NOT NEEDED (DEPRECATED) BUT I HAVE KEPT IT IN THE REPO FOR REFERENCE
1. lib/slamcontrol.cpp- to test the drive with slam later on 
2. lib/tarzan.cpp- not currently used
3. src/simulation.cpp- this was to check how much memory will about 1M grid cells take if we run it to that extent with random int.
4. src/tune.cpp- this was used during initial test runs 
5. python_simulation- this is just a python simulation on how our rover will actually run. 
