# Add the test executable
# Collect all test source files
file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)

# Create a test executable for all available tests
add_executable(MeshCartography_test ${TEST_SOURCES})

# Link the test executable with your library and GoogleTest
target_link_libraries(MeshCartography_test PRIVATE MeshCartographyLib ${GTEST_BOTH_LIBRARIES} pthread)
