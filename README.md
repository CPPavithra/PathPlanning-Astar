
# Efficient Path Planning Using Quadtree, A* Search, and Sparse Hash Maps

A modular path planning system with two main modules: **Mapping** and **Path Planning**. The mapping module uses memory-optimized Sparse Hash Maps for a global map and Multi-Level Quadtrees for a local map. The path planning module employs a two-level hierarchical A* approach, combining global sparse grid planning with local quadtree planning for efficient, dynamic obstacle avoidance in real-world terrains.

---

## Features

### Visualization using Rerun
Rerun is used for real-time logging and debugging of camera frames, maps, and planning data.

### Mapping Module

#### Sparse Occupancy Grids (Global Map)
- Efficiently stores only occupied cells using `unordered_map<pair<int,int>, CellCost, pair_hash>`.  
- Cell costs are based on height thresholds.  
- Optimized for large terrains (3km × 3km).  
- Implementation: [hashgridmap.cpp](src/hashgridmap.cpp), [mapping.h](include/mapping.h)

#### Dense Multi-Level Quadtrees (Local Map)
- Multi-level quadtrees (`lowQuadTree`, `midQuadTree`, `highQuadTree`) for fine-grained local obstacle detection.  
- Supports accurate navigation between global waypoints.  
- Implementation: [quadtree.cpp](src/quadtree.cpp), [quadtree.h](include/quadtree.h)

### Path Planning Module

#### Global Sparse A*
- Computes high-level waypoints using the global sparse map.  
- Considers occupied cells and uses Euclidean distance heuristics.  
- Implementation: [astar.cpp](src/astar.cpp), [pathplanning.h](include/pathplanning.h)

#### Local Quadtree A*
- Plans fine-grained paths between global waypoints using quadtrees.  
- Handles dynamic obstacles by re-evaluating the local costmap in real time.  
- Implementation: [astarquadtree.cpp](src/astarquadtree.cpp), [pathplanning.h](include/pathplanning.h)

---

## Mission Approach

- **Mapping Module:** Continuously updates global sparse maps and local quadtrees.  
- **Path Planning Module:** Generates global and local paths incrementally.  
- **Motion Control:** Converts planned paths into smooth velocity commands for the rover.  

### Key Details
- Global A* ensures efficient long-distance planning.  
- Local Quadtree A* enables precise obstacle avoidance near the rover.  
- Handles dynamic and unknown obstacles with local replanning.  
- Velocity commands are smoothed for stable traversal.

### Testing & Performance
- Hierarchical planning works efficiently for large terrains (3km × 3km).  
- Local replanning occurs in real time for unexpected obstacles.  
- Modular design allows easy extension for new sensors or planners.

---

## Performance & Limitations

### Performance
- **Global Sparse A\***: Memory-efficient, fast path computation across open areas.  
- **Local Quadtree A\***: High-resolution local planning; handles dense point clouds.  
- **Motion Control**: Smooth path following; compatible with simulated and real rovers.  
- **Visualization**: Rerun integration for debugging global and local maps.

### Limitations
- **Sensor Dependency**: Accuracy depends on high-quality point cloud data.  
- **Sparse Global Map**: Extremely cluttered areas may require denser mapping.  
- **Computation**: Dense local A* planning can be intensive for very large maps.  
- **Dynamic Obstacles**: Very fast-moving obstacles may not always be avoided.  
- **Real-World Deployment**: Rough or unpredictable terrains may need further tuning.

---

**Notes:**
- `motionplanner.cpp` orchestrates the mapping, planning, and motion control, integrating global and local planning hierarchically.  
- Rerun visualization hooks are embedded in `hashgridmap.cpp` and `quadtree.cpp`.
