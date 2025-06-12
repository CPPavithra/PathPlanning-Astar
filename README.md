
---

# 🤖 Efficient Path Planning Using Quadtree, A* Search, and Sparse Hash Maps + VSLAM

This project is my complete implementation of a **memory-efficient path planning system** for robots. It combines **quadtree spatial mapping**, **A* algorithm**, and a **sparse hash table** to compute optimal paths over large terrain maps with obstacles.

---

## 🧭 What This Project Does

- Converts 3D point cloud data into a layered obstacle map using **quadtrees**
- Categorizes obstacles based on height (low/mid/high)
- Uses **sparse hash tables** instead of a full grid to reduce memory usage FOR GLOBAL MAPS
- Computes shortest paths with **A* search** considering obstacle cost
- Visualizes everything — obstacles, map, and path — using `rerun`
- Designed for real-time robotic navigation and mapping tasks

---

## 🧠 Core Concepts (Explained Simply)

### 🟩 1. Quadtree Grid Mapping

Rather than dividing the entire area into a rigid full grid (which uses lots of memory), I use **quadtrees**. These only subdivide where needed — for example, if an area has dense obstacles. Each node stores:
- Center position and size
- Obstacle density
- Cost value for path planning

We use **three separate quadtrees**:
- `lowQuadtree`: close-to-ground obstacles
- `midQuadtree`: medium height (e.g., humans, barriers)
- `highQuadtree`: taller structures like poles, trees

### 🧊 2. Point Cloud to Map Conversion

From the RealSense depth camera, I process point cloud data using PCL:
- Voxel filtering
- Pass-through slicing

Points are fed into the quadtrees and stored in a way that reflects the obstacle’s size and height.

### ⚡ 3. Sparse Hash Table

For planning, I **don’t create a full grid**. Instead, I use a **sparse hash map** to store only the grids with the obstacles, which is EXTREMELY MEMORY EFFICIENT:
- There is dynamic cost handling based on height thresholds, adding high cost for taller obstacles
- Very efficient for a 3km grid
- Will be used for global path planning


This means:

* Less memory wasted
* Faster access and updates
* Scales to large maps without lag

### 🌟 4. A\* Path Planning

Using the cost-aware map:

* A\* calculates the optimal path from a `start` to a `goal` node.
* Costs from all three quadtrees are combined dynamically.
* High-density zones = high cost = less likely to be chosen in path.

Full Map (Sparse Hash Table)
↓ Run global A*
→ Waypoints: A → B → C → D

For each (current_point, next_waypoint):
    ↓
    1. Extract local region in quadtrees around current_point
    2. Bias low-level A* toward direction(current_point → next_waypoint)
    3. Plan in that window only (5x5m or 10x10m)
    4. Output refined path segment
    5. Repeat from last point

### 👀 5. Visualization

I use **Rerun** to visualize:

* Obstacle points (color-coded per layer)
* Quadtree node borders
* Final gridmaps
* Final computed path (line segments from start to goal)

---

## 🧪 How To Run

### ✅ Build & Run

```bash
cd PathPlanning-Astar
mkdir build && cd build
cmake ..
make
./main
```

### 📦 Dependencies

Make sure the following libraries are installed:

* `librealsense2`
* `PCL`
* `Eigen`
* `Rerun`
* `Boost`
* `OpenCV` (for ArUco detection)
* `CMake`

---

## 📁 Project Structure

```bash
PathPlanning-Astar/
├── include/                 # All headers
│   ├── quadtree.h
│   ├── astarquadtree.h
│   ├── gridmap.h, imu.h, common.h, rerun.h, etc.
├── lib/                     # Core logic + hardware interfaces
│   ├── slamcontrol.cpp, imu.cpp, gps.cpp, tarzan.cpp
├── src/                     # Main logic
│   ├── main.cpp             # Entry point
│   ├── quadtree.cpp         # Quadtree logic
│   ├── astarquadtree.cpp    # A* pathfinding
│   ├── pointcloudrerun.cpp  # Point cloud visualisation
│   ├── astar.cpp            # Sparse A* algorithm
│   ├── ArucoDetect.cpp      # Marker detection logic
│   └── tune.cpp             # Param tuning
├── build/                   # Compiled outputs (after build)
├── rerun_rec/               # Saved rerun logs
├── *.bag                    # ROS bag recordings (RealSense data)
├── gridmap*                 # Executables
├── imu_data.log, new_plan.txt
├── CMakeLists.txt           # Build config
└── README.md                # You’re here!
```

---

## 🧠 Why This Approach?

Instead of blindly scanning an entire area, this project focuses only on what matters:

* Quadtrees allow for smart obstacle representation
* Sparse hash tables ensure we don’t waste memory
* A\* makes sure we get the **best** path, fast
* Real-world data (point cloud, IMU, GPS) makes it deployable on actual rovers

---
---

## 🙋‍♀️ Author

Built with a lot of love (and debugging) by me, Pavithra ❤️
I’m deeply passionate about robotics, efficient planning systems, and building things that *actually work* on hardware!

Feel free to connect with me:
🔗 [LinkedIn](https://linkedin.com/in/pavithra-cp)
✉️ [cppavithra05@gmail.com](mailto:cppavithra05@gmail.com)

---

## 📄 License

This project is under MIT License.
```

---

