cmake_minimum_required(VERSION 3.5)
project(ros_qt5_gui_app VERSION 0.0.1)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake;${CMAKE_MODULE_PATH}")
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror=return-type")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-type")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_PREFIX ./install)

# 依赖
include(Getnlohmann)
include(GetDockWidget)
include(GetYamlCpp)
include(GTestHelper)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)

# options
option(BUILD_WITH_CHANNEL_AUTO ON)
option(BUILD_WITH_CHANNEL_ROS1 OFF)
option(BUILD_WITH_CHANNEL_ROS2 OFF)
option(BUILD_WITH_CHANNEL_ROSBRIDGE OFF)
option(BUILD_WITH_TEST OFF)

set(BUILD_WITH_CHANNEL_AUTO ON)
enable_testing()

# modules
add_subdirectory(src)
add_subdirectory(install)
