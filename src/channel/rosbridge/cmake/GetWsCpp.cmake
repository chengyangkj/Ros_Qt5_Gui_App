include(FetchContent)

message(STATUS "get websocketpp ...")

set(websocketpp_GIT_TAG
    "0.8.2"
    CACHE STRING "websocketpp git tag")

FetchContent_Declare(
  websocketpp
  GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
  GIT_TAG ${websocketpp_GIT_TAG}
  GIT_SHALLOW TRUE)

FetchContent_GetProperties(websocketpp)
if(NOT websocketpp_POPULATED)
  FetchContent_MakeAvailable(websocketpp)
endif()

set(WEBSOCKETPP_INCLUDE_DIRS ${websocketpp_SOURCE_DIR} CACHE PATH "websocketpp include directory") 