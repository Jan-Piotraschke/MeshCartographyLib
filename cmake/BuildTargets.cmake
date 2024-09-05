# Add the source files of your library
file(GLOB_RECURSE SIMULATION_SOURCES src/*.cpp)
if(NOT SIMULATION_SOURCES)
  message(FATAL_ERROR "No source files found in src/ directory.")
endif()

# Create the MeshCartographyLib library target
add_library(MeshCartographyLib STATIC ${SIMULATION_SOURCES})

# Include the Eigen directory for MeshCartographyLib
target_include_directories(MeshCartographyLib PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

# Add include directories for MeshCartographyLib
target_include_directories(MeshCartographyLib PUBLIC
    "${MeshCartographyLib_SOURCE_DIR}/pmp-library/src"
    src
)

# Link the library with PMP and Ceres
target_link_libraries(MeshCartographyLib PUBLIC pmp ceres)

# Add the MeshCartography executable
add_executable(MeshCartography MeshCartography.cpp)
target_link_libraries(MeshCartography PUBLIC
    MeshCartographyLib
    pmp
    ceres
)
