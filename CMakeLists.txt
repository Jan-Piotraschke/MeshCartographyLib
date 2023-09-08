cmake_minimum_required(VERSION 3.22)

project(MeshCartographyLib VERSION 1.0)

# Make the project foulder path available to the source code
add_definitions("-DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"")

# Specify the required C++ standard
set(CMAKE_CXX_STANDARD 20)

# Find required packages
find_package(CGAL REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Eigen3 REQUIRED)

include(CGAL_Eigen3_support)

# Add the source files of your library
file(GLOB SIMULATION_SOURCES src/*.cpp)
add_library(MeshCartographyLib STATIC ${SIMULATION_SOURCES})

# Link the library with the required packages
target_link_libraries(MeshCartographyLib PUBLIC
    CGAL::CGAL
    Boost::filesystem
    CGAL::Eigen3_support
)

# Add include directories
target_include_directories(MeshCartographyLib PUBLIC src)

# Add C++ source files
add_executable(main main.cpp)
target_link_libraries(main PUBLIC
        MeshCartographyLib
)
