# Specify the required C++ standard
set(CMAKE_CXX_STANDARD 17)

# Make the project folder path available to the source code
add_definitions("-DMeshCartographyLib_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"")
