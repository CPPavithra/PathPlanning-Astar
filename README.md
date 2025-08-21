
---

# Efficient Path Planning Using Quadtree, A* Search, and Sparse Hash Maps

A modular path planning system which has two main modules- mapping and path planning module depended on each other. The mapping module includes efficient and memory optimized Sparse Hash Maps for Global map and Multi-Level Quadtree Mapping for Local map. The path planning module also follows a modular two-level hierarchical approach wherein there exists; A* Sparse Grid Path Planning and A* Quad tree Local Path planning. This project focuses on efficient pathfinding and dynamic obstacle avoidance, leveraging heightmaps and sensor integration for real-world navigation scenarios.

---
# FEATURES

## Visualisation using Rerun

I have used Rerun for visualisation. It is a very good approach for debugging and for logging the camera frames and data in real time. It is pretty cool

## The Mapping module

Since this was made with the intention of deploying it in the University Rover Challenge (URC), we had to keep in mind the memory usage and make it optimal for a very large terrain (3km x 3km).
### Sparse Occupancy Grids with Hash Maps
#### How does it work?
  1. The point cloud data is collected and the orientation is converted to also transformed.
  2. 

---
 
