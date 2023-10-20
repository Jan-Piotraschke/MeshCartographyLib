cmake_minimum_required(VERSION 3.22)
project(
  MeshCartographyLib
  VERSION 1.0
  DESCRIPTION "Mesh Cartography Library"
  LANGUAGES CXX
)
# cmake_policy(SET CMP0072 NEW)

# Make the project foulder path available to the source code
add_definitions("-DMeshCartographyLib_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/pmp-library/cmake)

# Specify the required C++ standard
set(CMAKE_CXX_STANDARD 17)

if(EMSCRIPTEN)
  add_compile_options(--no-heap-copy)
  add_compile_options(-fexceptions)
  add_link_options(-fexceptions)
  add_link_options(
    "SHELL:-sWASM=1 -sUSE_WEBGL2=1 -sUSE_GLFW=3 -sALLOW_MEMORY_GROWTH=1 -sALLOW_TABLE_GROWTH=1 -sSTACK_SIZE=5MB"
  )
  set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()

# which directories to process
if(NOT EMSCRIPTEN)
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
endif()

# Find required packages
set(EIGEN3_INCLUDE_DIR "${MeshCartographyLib_SOURCE_DIR}/pmp-library/external/eigen-3.4.0")

# Add the source files of your library
file(GLOB_RECURSE SIMULATION_SOURCES src/*.cpp)
add_library(MeshCartographyLib STATIC ${SIMULATION_SOURCES})

# Include the Eigen directory
target_include_directories(MeshCartographyLib PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

include_directories(${MeshCartographyLib_SOURCE_DIR}/pmp-library/src/)
add_subdirectory("${MeshCartographyLib_SOURCE_DIR}/pmp-library/src/pmp")

target_include_directories(MeshCartographyLib PUBLIC "${MeshCartographyLib_SOURCE_DIR}/pmp-library/src")

# Link the library with the required packages
target_link_libraries(MeshCartographyLib PUBLIC pmp)

# Add include directories
target_include_directories(MeshCartographyLib PUBLIC src)

# Add C++ source files
add_executable(MeshCartography MeshCartography.cpp)
target_link_libraries(MeshCartography PUBLIC
    MeshCartographyLib
    pmp
)

if (NOT EMSCRIPTEN)
  # Collect all test source files
  file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)

  # Create a test executable for all available tests
  add_executable(MeshCartography_test ${TEST_SOURCES})

  # Link your libraries to the test executable
  target_link_libraries(MeshCartography_test PRIVATE MeshCartographyLib ${GTEST_BOTH_LIBRARIES} pthread)
endif()
