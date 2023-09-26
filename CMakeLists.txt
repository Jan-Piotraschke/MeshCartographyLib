cmake_minimum_required(VERSION 3.22)

project(MeshCartographyLib VERSION 1.0)

# Make the project foulder path available to the source code
add_definitions("-DMeshCartographyLib_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"")

# Specify the required C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find required packages
find_package(PMP REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

include(CGAL_Eigen3_support)
include_directories(${GTEST_INCLUDE_DIRS})

# Add the source files of your library
file(GLOB_RECURSE SIMULATION_SOURCES src/*.cpp)
add_library(MeshCartographyLib STATIC ${SIMULATION_SOURCES})

# Link the library with the required packages
target_link_libraries(MeshCartographyLib PUBLIC
    CGAL::CGAL
    Boost::filesystem
    CGAL::Eigen3_support
    pmp
)

# Add include directories
target_include_directories(MeshCartographyLib PUBLIC src)

# Add C++ source files
add_executable(MeshCartography MeshCartography.cpp)
target_link_libraries(MeshCartography PUBLIC
        MeshCartographyLib
        pmp
)

# Collect all test source files
file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)

# Create a test executable for all available tests
add_executable(MeshCartography_test ${TEST_SOURCES})

# Link your libraries to the test executable
target_link_libraries(MeshCartography_test PRIVATE MeshCartographyLib ${GTEST_BOTH_LIBRARIES} pthread)
