set(TARGET_NAME channel_ros2)

# 包含topology_msgs获取脚本
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GetTopologyMsg.cmake)

file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} *.cpp *.cc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  *.h *.hpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

add_library(${TARGET_NAME} SHARED
    ${SOURCE_FILE} 
    ${INCLUDE_FILE}
)
target_include_directories(
${TARGET_NAME} 
PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(${TARGET_NAME} PUBLIC rclcpp rclcpp_action action_msgs Boost nav_msgs nav2_msgs tf2 tf2_geometry_msgs tf2_ros sensor_msgs tf2_kdl cv_bridge )
target_link_libraries(${TARGET_NAME} 
PUBLIC
basic
common
framework
topology_msgs__rosidl_generator_cpp #消息类型的ROS库
topology_msgs__rosidl_typesupport_cpp
topology_msgs__rosidl_typesupport_fastrtps_cpp
topology_msgs__rosidl_typesupport_introspection_cpp
)

set_target_properties(${TARGET_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
install(TARGETS ${TARGET_NAME} DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION bin/lib
    LIBRARY DESTINATION bin/lib
    ARCHIVE DESTINATION bin/lib
)
# ament_package()
# install(TARGETS ${TARGET_NAME} DESTINATION lib/${PROJECT_NAME})