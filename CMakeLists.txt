cmake_minimum_required(VERSION 3.10)
project(pointcloudrun LANGUAGES CXX)

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
find_package(PCL 1.8 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(rerun_sdk REQUIRED)

# Include directories
include_directories(
    ${PCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${MPI_INCLUDE_PATH}
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
add_executable(pointcloudrerun pointcloudrerun.cpp)

set_property(TARGET pointcloudrerun PROPERTY CXX_STANDARD 17)
target_include_directories(pointcloudrerun PRIVATE "/usr/local/include/rerun/components")
# Link libraries to the executable
target_link_libraries(pointcloudrerun PRIVATE rerun_sdk
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${MPI_LIBRARIES}
    realsense2  # Link the RealSense2 library explicitly
    pthread
)

