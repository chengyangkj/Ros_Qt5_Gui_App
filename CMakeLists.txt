cmake_minimum_required(VERSION 3.5)
project(ros2_qt5_gui_app)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
##################################1,编译平台设置:设置编译平台是ROS1还是ROS2#######################
# set(COMPILE_PLATFORM "ROS1")
set(COMPILE_PLATFORM "ROS2")
##################################2,查找依赖包:根据ROS1还是ROS2#######################
# find dependencies
if(COMPILE_PLATFORM STREQUAL "ROS1")
    message("ros1")
    add_definitions(-DROS1_NODE=1) #设置程序宏定义
elseif(COMPILE_PLATFORM STREQUAL "ROS2")
    message("ros2")
    add_definitions(-DROS2_NODE=1) #设置程序宏定义
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_kdl REQUIRED)
    find_package(sensor_msgs REQUIRED)
else()

  message(FATAL_ERROR "ros1 or ros2 not set!!!!!!!!!")
endif()


find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
##################################3,设置程序源文件路径#######################
file(GLOB_RECURSE SOURCE_FILE RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  src/*.cpp)
file(GLOB_RECURSE UI_FILE RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
file(GLOB_RECURSE INCLUDE_FILE RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  include/*.h *.hpp)
file(GLOB_RECURSE QRC_FILE RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  resource/*.qrc)
#qt moc
qt5_wrap_ui(QT_UI_HPP ${UI_FILE})
qt5_wrap_cpp(QT_MOC_HPP ${INCLUDE_FILE})
qt5_add_resources(QT_RESOUCE_HPP ${QRC_FILE})

#################################4,不同平台，屏蔽不同源文件####################
####设置ros1相关原文将
file(GLOB_RECURSE ROS1_SOURCE_FILE RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  src/communication/ros1/*.cpp)
####设置ros2相关源文件
file(GLOB_RECURSE ROS2_SOURCE_FILE RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  src/communication/ros2_qt5_gui_app/*.cpp)
#根据不同平台 屏蔽源文件
if(COMPILE_PLATFORM STREQUAL "ROS2")
  list(REMOVE_ITEM SOURCE_FILE ${ROS1_SOURCE_FILE})
elseif(COMPILE_PLATFORM STREQUAL "ROS1")
    message("use ros1 node")
endif()

#项目包含的头文件路径
include_directories(
	include/
	${CMAKE_CURRENT_BINARY_DIR}
)
#向可执行程序添加源文件
add_executable(${PROJECT_NAME}
    ${SOURCE_FILE}
    ${INCLUDE_FILE}
    ${UI_FILE}
    ${QT_MOC_HPP}
    ${QT_RESOUCE_HPP}
  )
#链接公共库
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
#################################5,不同平台，链接不同库####################
# 判断使用ros1 还是ros2
if(COMPILE_PLATFORM STREQUAL "ROS2")
    ament_target_dependencies(${PROJECT_NAME} rclcpp  Boost nav_msgs tf2 tf2_geometry_msgs tf2_ros   sensor_msgs tf2_kdl)
    install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
    ament_package()
elseif(COMPILE_PLATFORM STREQUAL "ROS1")
    message("use ros1 node")
endif()

