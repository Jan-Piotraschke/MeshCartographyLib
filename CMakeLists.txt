cmake_minimum_required(VERSION 3.22)

project(
  MeshCartographyLib
  VERSION 1.0
  DESCRIPTION "Mesh Cartography Library"
  LANGUAGES CXX
)

# Include the modular CMake files
include(cmake/ProjectConfig.cmake)
include(cmake/Dependencies.cmake)
include(cmake/BuildTargets.cmake)
include(cmake/Tests.cmake)
