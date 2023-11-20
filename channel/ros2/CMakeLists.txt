include(TargetNameHelper)
GetTargetName()

file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} *.cpp *.cc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  *.h *.hpp)
find_package(Qt5 COMPONENTS Core Gui REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
add_library(${TARGET_NAME} 
    ${SOURCE_FILE} 
    ${INCLUDE_FILE}
)
target_include_directories(
${TARGET_NAME} 
PUBLIC 
${rclcpp_INCLUDE_DIRS}
${tf2_ros_INCLUDE_DIRS}
${nav_msgs_INCLUDE_DIRS}
${tf2_geometry_msgs_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
${tf2_kdl_msgs_INCLUDE_DIRS}
${rclcpp_INCLUDE_DIRS}
)
# target_link_libraries(${TARGET_NAME} 
# PUBLIC 
# Qt5::Core 
# Qt5::Gui
# Boost
# ${rclcpp_LIBRARIES}
# ${tf2_ros_LIBRARIES}
# ${nav_msgs_LIBRARIES}
# ${tf2_geometry_msgs_LIBRARIES}
# ${sensor_msgs_msgs_LIBRARIES}
# ${tf2_kdl_msgs_LIBRARIES}
# )
ament_target_dependencies(${TARGET_NAME} PUBLIC rclcpp  Boost nav_msgs tf2 tf2_geometry_msgs  tf2_ros sensor_msgs tf2_kdl )
install(TARGETS ${TARGET_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
install(TARGETS ${TARGET_NAME} DESTINATION lib/${PROJECT_NAME})