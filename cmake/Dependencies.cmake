# Set CMake module path for additional packages
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/pmp-library/cmake)

# Find Eigen (header-only library)
set(EIGEN3_INCLUDE_DIR "${MeshCartographyLib_SOURCE_DIR}/pmp-library/external/eigen-3.4.0")

add_library(Eigen3::Eigen INTERFACE IMPORTED)
set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MeshCartographyLib_SOURCE_DIR}/pmp-library/external/eigen-3.4.0"
)

# Add Ceres Solver
add_subdirectory("${MeshCartographyLib_SOURCE_DIR}/ceres-solver")

# Include the PMP library
include_directories(${MeshCartographyLib_SOURCE_DIR}/pmp-library/src/)
add_subdirectory("${MeshCartographyLib_SOURCE_DIR}/pmp-library/src/pmp")

# Find GoogleTest for testing
find_package(CGAL REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

include(CGAL_Eigen3_support)
include_directories(${GTEST_INCLUDE_DIRS})
