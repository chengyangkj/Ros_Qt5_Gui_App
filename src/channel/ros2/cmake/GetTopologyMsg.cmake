include(FetchContent)

message(STATUS "get topology_msgs ...")

set(topology_msgs_DOWNLOAD_URL
    "https://github.com/chengyangkj/topology_msgs/archive/refs/heads/main.zip"
    CACHE STRING "")

FetchContent_Declare(
  topology_msgs
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL "${topology_msgs_DOWNLOAD_URL}")

FetchContent_GetProperties(topology_msgs)
if(NOT topology_msgs_POPULATED)
  FetchContent_MakeAvailable(topology_msgs)
endif()

# import targets:
# topology_msgs::topology_msgs