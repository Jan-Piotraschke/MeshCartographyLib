# Specify the required C++ standard
set(CMAKE_CXX_STANDARD 17)

# Set up for WebAssembly (if needed)
if(EMSCRIPTEN)
  add_compile_options(--no-heap-copy)
  add_compile_options(-fexceptions)
  add_link_options(-fexceptions)
  add_link_options(
    "SHELL:-sWASM=1 -sUSE_WEBGL2=1 -sUSE_GLFW=3 -sALLOW_MEMORY_GROWTH=1 -sALLOW_TABLE_GROWTH=1 -sSTACK_SIZE=5MB"
  )
  set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()

# Make the project folder path available to the source code
add_definitions("-DMeshCartographyLib_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\"")
