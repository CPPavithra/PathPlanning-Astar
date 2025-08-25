cmake_minimum_required(VERSION 3.10)
project(motionplanner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_MAIN "Build the main executable" OFF)

# set(CMAKE_PREFIX_PATH
#     "/usr/lib/x86_64-linux-gnu/cmake/realsense2"
#     "/usr/local/lib/cmake/rerun_sdk"
#     CACHE STRING "Paths to dependencies"
# )

enable_language(C) # For MPI

find_package(PCL 1.14 REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(motionplanner
    src/astar.cpp
    src/motionplanner.cpp
    src/motionplanner_helper.cpp
    src/hashgridmap.cpp
    src/quadtree.cpp
    src/astarquadtree.cpp
    lib/imu.cpp
    lib/cobs.c
)

target_include_directories(motionplanner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${rerun_sdk_INCLUDE_DIRS}
)

target_link_libraries(motionplanner PRIVATE
    rerun_sdk
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS}
    Boost::system
    realsense2
    Eigen3::Eigen
)

if(BUILD_MAIN)
    add_executable(main
        src/ArucoDetect.cpp
        src/main.cpp
    )

    target_link_libraries(main PRIVATE motionplanner)

    target_include_directories(main
        PRIVATE "/usr/local/include/rerun/components"
    )
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ 
	DESTINATION include)

install(EXPORT ${PROJECT_NAME}
	FILE ${PROJECT_NAME}.cmake
	DESTINATION lib/cmake/${PROJECT_NAME})

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "motionplannerConfigVersion.cmake"
    VERSION ${motionplanner_VERSION}
    COMPATIBILITY AnyNewerVersion)

install(FILES "motionplannerConfig.cmake" "${CMAKE_CURRENT_BINARY_DIR}/motionplannerConfigVersion.cmake"
    DESTINATION lib/cmake/motionplanner)
