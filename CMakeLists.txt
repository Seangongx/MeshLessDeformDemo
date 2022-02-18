cmake_minimum_required(VERSION 3.16)
project(MeshLessDeformationDemo)

# information
message(STATUS "CMake version: ${CMAKE_VERSION}")

# start from example project https://github.com/libigl/libigl-example-project
# Install the Libigl(updated to 2.4.0 with new configurations)
# https://libigl.github.io/changelog/
# CMakeLists.txt history https://github.com/libigl/libigl-example-project/commits/main/CMakeLists.txt
# libigl.cmake history https://github.com/libigl/libigl-example-project/commits/main/cmake
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
option(LIBIGL_OPENGL                 "Use OpenGL"                   ON)
option(LIBIGL_GLFW                   "Use GLFW"                     ON)
option(LIBIGL_IMGUI                  "Use ImGui"                    ON)
option(LIBIGL_PNG                       "Use PNG"                      ON)
# For compiling static library(may unstable so are commented)
#option(LIBIGL_USE_STATIC_LIBRARY     "Use libIGL as static librarie" ON)
#find_package(LIBIGL REQUIRED QUIET)

include(libigl)

# Add your project files
file(GLOB_RECURSE src_files ${CMAKE_CURRENT_LIST_DIR}/src/*.[ch]pp ${CMAKE_CURRENT_LIST_DIR}/src/*.[ch])
add_executable(${PROJECT_NAME} ${src_files})


if(MSVC)
   source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${src_files})  #Allow to explore source directories as a tree in Visual Studio
endif()


#target_include_directories(${PROJECT_NAME} PRIVATE src/)

# Some features need c++17
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

# Add imgui(Don't forget!)
target_link_libraries(${PROJECT_NAME} PUBLIC 
    igl::glfw
	igl::imgui
)
