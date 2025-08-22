cmake_minimum_required(VERSION 3.10)
project(motionplanner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_MAIN "Build the main executable" OFF)

set(CMAKE_PREFIX_PATH
    "/usr/lib/x86_64-linux-gnu/cmake/realsense2"
    "/usr/local/lib/cmake/rerun_sdk"
    CACHE STRING "Paths to dependencies"
)

enable_language(C) # For MPI

find_package(MPI REQUIRED)
find_package(PCL 1.14 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

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
    ${PCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${MPI_INCLUDE_PATH}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/eigen3
    /usr/include/GL
    /usr/local/include/rerun
)

target_link_libraries(motionplanner PUBLIC
    rerun_sdk
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS}
    ${MPI_LIBRARIES}
    Boost::system
    realsense2
    pthread
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

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/motionplannerConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/motionplannerConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/motionplannerConfig.cmake"
    INSTALL_DESTINATION lib/cmake/motionplanner
)

install(TARGETS motionplanner
    EXPORT motionplannerTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_MAIN)
    install(TARGETS main
        RUNTIME DESTINATION bin
    )
endif()

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/motionplannerConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/motionplannerConfigVersion.cmake"
    DESTINATION lib/cmake/motionplanner
)

install(EXPORT motionplannerTargets
    FILE motionplannerTargets.cmake
    NAMESPACE motionplanner::
    DESTINATION lib/cmake/motionplanner
)

