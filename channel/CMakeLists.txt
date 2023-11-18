


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