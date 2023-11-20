include(TargetNameHelper)
GetTargetName()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_libraries(basic common)
if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
    message("USE ROS1 channel")
    add_subdirectory(ros1)
elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
    message("USE ROS2 channel")
    add_subdirectory(ros2)
else()
    message("Use protobuf channel")
    add_subdirectory(protobuf)
endif()

add_library(${TARGET_NAME} INTERFACE)

target_include_directories(${TARGET_NAME} INTERFACE 
${CMAKE_CURRENT_SOURCE_DIR}
base
)