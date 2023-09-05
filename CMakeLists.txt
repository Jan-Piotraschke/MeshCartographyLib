cmake_minimum_required(VERSION 3.22)

project(MeshCartographyLib VERSION 1.0)

# Specify the required C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find required packages
find_package(CGAL REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the source files of your library
file(GLOB SIMULATION_SOURCES src/*.cpp)
add_library(MeshCartographyLib STATIC ${SIMULATION_SOURCES})

# Link the library with the required packages
target_link_libraries(MeshCartographyLib PUBLIC
    CGAL::CGAL
    Boost::filesystem
    Eigen3::Eigen
)

# Add include directories
target_include_directories(MeshCartographyLib PUBLIC src)
