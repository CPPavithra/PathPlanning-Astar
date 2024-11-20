cmake_minimum_required(VERSION 3.10)
project(Realsense_Pointcloud)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the RealSense SDK configuration file explicitly
include("/usr/local/lib/cmake/realsense2/realsense2Config.cmake")

# Set paths for dependencies
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/realsense2" CACHE STRING "Path to realsense2")

# Find PCL, OpenGL, and manually specify RealSense SDK location
find_package(PCL 1.8 REQUIRED)
find_package(OpenGL REQUIRED)

# Set GLFW include and library paths manually
set(GLFW_INCLUDE_DIR "/usr/include/GLFW")
set(GLFW_LIBRARY "/usr/lib/x86_64-linux-gnu/libglfw.so")  # Path to your shared GLFW library

# Include directories
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
    /usr/local/include  # Include RealSense headers explicitly
    ./include
    /usr/include/eigen3
)

# Link directories
link_directories(
    ${PCL_LIBRARY_DIRS}
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib  # Include RealSense libraries explicitly
    /home/pavithra/samplerrtstar/RRT-/Open3D/build/eigen/src/ext_eigen/bench/btl/libs/eigen3
)

# Suppress PCL warnings (optional)
add_definitions(${PCL_DEFINITIONS})

# Add the executable
add_executable(pointcloud pointcloud.cpp)

# Link libraries to the executable
target_link_libraries(pointcloud
    ${PCL_LIBRARIES}
    realsense2  # Link the RealSense2 library explicitly
    ${OPENGL_LIBRARIES}
    pthread
    ${GLFW_LIBRARY}  # Explicitly link the GLFW static library
)

