cmake_minimum_required(VERSION 3.15)

project(
  MeshCartographyLib
  VERSION 1.0
  DESCRIPTION "Mesh Cartography Library"
  LANGUAGES CXX
)

# Use C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Make the project folder path available to the source code
add_definitions("-DMeshCartographyLib_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"")

# Use vcpkg integration
if (NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "Vcpkg toolchain file")
endif()

# Find required packages
find_package(pmp CONFIG REQUIRED)
find_package(boost_filesystem REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
find_package(glog CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres CONFIG REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

# Include directories
include_directories(.src ${OpenCV_INCLUDE_DIRS})

# Add the source files of your library
file(GLOB_RECURSE SIMULATION_SOURCES .src/*.cpp)

# Add your library
add_library(MeshCartographyLib STATIC ${SIMULATION_SOURCES})

# Link the library with the required packages
target_link_libraries(MeshCartographyLib PUBLIC
    pmp
    Boost::filesystem
    glog::glog
    Ceres::ceres
    ${OpenCV_LIBS}
)

add_executable(MeshCartography MeshCartography.cpp)
target_link_libraries(MeshCartography PUBLIC
    MeshCartographyLib
    Ceres::ceres
)

# Create the Python module using pybind11
pybind11_add_module(mesh_cartography_module bindings.cpp MeshCartography.cpp)

target_link_libraries(mesh_cartography_module PRIVATE
    MeshCartographyLib
    Ceres::ceres
    ${OpenCV_LIBS}
)

# Set the output directory for the Python module
set_target_properties(mesh_cartography_module PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/python
)

# Collect all test source files
file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)

# Create a test executable for all available tests
add_executable(MeshCartography_test ${TEST_SOURCES})

# Link your libraries to the test executable
target_link_libraries(MeshCartography_test PRIVATE
    MeshCartographyLib
    ${GTEST_BOTH_LIBRARIES}
    pthread
)
