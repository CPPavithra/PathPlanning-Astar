cmake_minimum_required(VERSION 3.10)
project(main LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the RealSense SDK configuration file explicitly
include("/usr/local/lib/cmake/realsense2/realsense2Config.cmake")


# Set paths for dependencies
set(CMAKE_PREFIX_PATH "/usr/local/lib/cmake/realsense2;/usr/local/lib/cmake/rerun_sdk" CACHE STRING "Paths to dependencies")
#set(RERUN_PATH "/usr/local/lib/cmake/rerun_sdk/")

enable_language(C) #for mpi

#add_subdirectory("{RERUN_PATH}","${CMAKE_BINARY_DIR}/rerun_sdk")

# Find PCL (No need for OpenGL now)
find_package(MPI REQUIRED)
find_package(PCL 1.14 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${MPI_INCLUDE_PATH}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/boost
    /usr/local/include  # Include RealSense headers explicitly
    /usr/include/eigen3
    /usr/include/GL
    /usr/local/include/rerun/
    
)


# Link directories
link_directories(
    ${PCL_LIBRARY_DIRS}
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib  # Include RealSense libraries explicitly
)

# Suppress PCL warnings (optional)
add_definitions(${PCL_DEFINITIONS})

# Add the executable
add_executable(main
src/ArucoDetect.cpp
src/main.cpp
src/astar.cpp
src/pointcloudrerun.cpp
src/quadtree.cpp
src/astarquadtree.cpp
lib/imu.cpp
lib/cobs.c
)
# add_executable(tune
# src/tune.cpp
# lib/imu.cpp
# lib/cobs.c
# )

set_property(TARGET main PROPERTY CXX_STANDARD 17)
target_include_directories(main
PUBLIC include
PRIVATE "/usr/local/include/rerun/components")
# Link libraries to the executable
target_link_libraries(main PRIVATE rerun_sdk
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS}
    ${MPI_LIBRARIES}
    Boost::system
    realsense2  # Link the RealSense2 library explicitly
    pthread
)

