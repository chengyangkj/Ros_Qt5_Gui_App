include(GTestHelper)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
if( BUILD_WITH_CHANNEL_AUTO STREQUAL "ON")
    message("BUILD_WITH_CHANNEL_AUTO")
    if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
        message("Auto USE ROS1 channel")
        add_subdirectory(ros1)
    elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
        message("Auto USE ROS2 channel")
        add_subdirectory(ros2)
    endif()
elseif(BUILD_WITH_CHANNEL_ROS2 STREQUAL "ON")
    message("Manual USE ROS2 channel")
    add_subdirectory(ros2)
elseif(BUILD_WITH_CHANNEL_ROS1 STREQUAL "ON")
    message("Manual USE ROS1 channel")
    add_subdirectory(ros1)
endif()

message("build ROSBridge channel")
add_subdirectory(rosbridge)

find_package(Boost REQUIRED COMPONENTS filesystem)

set(CHANNEL_MANAGER_SOURCES
  manager/channel_manager.cc
)

set(CHANNEL_MANAGER_HEADERS
  manager/channel_manager.h
)

add_library(channel STATIC
  ${CHANNEL_MANAGER_SOURCES}
  ${CHANNEL_MANAGER_HEADERS}
  virtual_channel_node.h
)

target_link_libraries(channel PUBLIC Boost::filesystem pthread dl common framework)

target_include_directories(channel PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}
manager
)

set_target_properties(channel PROPERTIES
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

#######test
find_library(DL_LIBRARY dl)
set(TEST_LINK_LIBRARIES channel ${DL_LIBRARY})
AddTest()