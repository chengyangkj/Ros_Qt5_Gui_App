set(TARGET_NAME channel)

file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} *.cpp *.cc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  *.h *.hpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
add_library(${TARGET_NAME} 
    ${SOURCE_FILE} 
    ${INCLUDE_FILE}
)

ament_target_dependencies(${TARGET_NAME} rclcpp  Boost nav_msgs tf2 tf2_geometry_msgs tf2_ros   sensor_msgs tf2_kdl)
install(TARGETS ${TARGET_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()