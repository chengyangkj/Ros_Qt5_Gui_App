set(TARGET_NAME channel_rosbridge)

file(GLOB_RECURSE SOURCE_FILE CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc)
list(FILTER SOURCE_FILE EXCLUDE REGEX "(.*_test\\.cpp$)|(.*_test\\.cc$)|(.*client\\.cpp$)|(.*demo.*\\.cpp$)")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Threads REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GetWsCpp.cmake)

if(NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)

add_library(${TARGET_NAME} SHARED
    ${SOURCE_FILE}
)

target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
)

if(TARGET Boost::boost)
    target_link_libraries(${TARGET_NAME} PUBLIC Boost::boost)
endif()

if(WEBSOCKETPP_INCLUDE_DIRS)
    target_include_directories(${TARGET_NAME} PUBLIC ${WEBSOCKETPP_INCLUDE_DIRS})
endif()

target_link_libraries(${TARGET_NAME}
    PUBLIC
    basic
    common
    framework
    ${OpenCV_LIBS}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    Threads::Threads
)

if(WIN32)
    target_link_libraries(${TARGET_NAME} PUBLIC ws2_32)
endif()

set_target_properties(${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

install(TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION bin/lib
    LIBRARY DESTINATION bin/lib
    ARCHIVE DESTINATION bin/lib
)
