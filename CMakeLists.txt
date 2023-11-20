cmake_minimum_required(VERSION 3.5)
project(ros_qt5_gui_app VERSION 0.0.1)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake;${CMAKE_MODULE_PATH}")
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}" ABSOLUTE)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#options
option(BUILD_WITH_CHANNEL_ROS1 OFF)
option(BUILD_WITH_CHANNEL_ROS2 OFF)
option(BUILD_WITH_CHANNEL_PROTOBUF OFF)

#依赖
set(QT_VERSION_MAJOR 5)
set(VERSION_SHORT "0.0.1")
add_subdirectory(3rdpart/dockWidget)
add_subdirectory(3rdpart/JsonStruct)

#modules
add_subdirectory(common)
add_subdirectory(basic)
add_subdirectory(channel)
add_subdirectory(mainwindow)
add_subdirectory(core)


