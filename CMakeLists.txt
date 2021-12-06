cmake_minimum_required(VERSION 3.16)
project(demo)
############# Use message function to print information ############
message(STATUS "CMake version: ${CMAKE_VERSION}")

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui" ON)
# Before find_package
option(LIBIGL_USE_STATIC_LIBRARY     "Use libIGL as static librarie" ON)
#find_package(LIBIGL REQUIRED QUIET)
include(libigl)

# Add your project files
# set_property(GLOBAL PROPERTY USE_FOLDER ON)
file(GLOB HEADFILES src/*.h)
file(GLOB SRCFILES src/*.cpp)
add_executable(${PROJECT_NAME} ${HEADFILES} ${SRCFILES})

# Some features need c++17
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories(${PROJECT_NAME}
PRIVATE
	src/
)

# Add imgui(Don't forget!)
target_link_libraries(${PROJECT_NAME} PUBLIC 
	igl::core
	igl::opengl_glfw
	igl::opengl_glfw_imgui
)
