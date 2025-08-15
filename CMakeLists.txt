cmake_minimum_required(VERSION 3.10)
project(main LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Paths to dependencies (adjust if rerun_sdk is elsewhere)
set(CMAKE_PREFIX_PATH
    "/usr/lib/x86_64-linux-gnu/cmake/realsense2"
    "/usr/local/lib/cmake/rerun_sdk"
    CACHE STRING "Paths to dependencies"
)

enable_language(C)  # For MPI

# Dependencies
find_package(MPI REQUIRED)
find_package(PCL 1.14 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

# Include directories
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${MPI_INCLUDE_PATH}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/eigen3
    /usr/include/GL
    /usr/local/include/rerun
)

# Library directories
link_directories(
    ${PCL_LIBRARY_DIRS}
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
)

# Add executable
add_executable(main
    src/ArucoDetect.cpp
    src/main.cpp
    src/astar.cpp
    src/mainloop.cpp
    src/pointcloudrerun.cpp
    src/quadtree.cpp
    src/astarquadtree.cpp
    lib/imu.cpp
    lib/cobs.c
)

# Set C++ standard again at target level
set_property(TARGET main PROPERTY CXX_STANDARD 17)

# Extra include dirs for rerun components
target_include_directories(main
    PRIVATE "/usr/local/include/rerun/components"
)

# Link libraries
target_link_libraries(main PRIVATE
    rerun_sdk
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS}
    ${MPI_LIBRARIES}
    Boost::system
    realsense2
    pthread
)

