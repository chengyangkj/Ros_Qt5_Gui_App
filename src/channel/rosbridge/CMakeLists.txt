set(TARGET_NAME channel_rosbridge)

file(GLOB_RECURSE SOURCE_FILE CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc)
list(FILTER SOURCE_FILE EXCLUDE REGEX "(.*_test\\.cpp$)|(.*_test\\.cc$)|(.*client\\.cpp$)|(.*demo.*\\.cpp$)")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

if(PkgConfig_FOUND)
    pkg_search_module(LIBBSON libbson-1.0)
    if(NOT LIBBSON_FOUND)
        message(WARNING "libbson-1.0 not found via pkg-config, trying alternative methods")
        find_path(LIBBSON_INCLUDE_DIRS
            NAMES libbson/bson.h bson.h
            PATHS
                /usr/include
                /usr/local/include
                /opt/local/include
        )
        find_library(LIBBSON_LIBRARIES
            NAMES bson-1.0 libbson-1.0 bson-static-1.0
            PATHS
                /usr/lib
                /usr/local/lib
                /opt/local/lib
        )
        if(LIBBSON_INCLUDE_DIRS AND LIBBSON_LIBRARIES)
            set(LIBBSON_FOUND TRUE)
            set(LIBBSON_CFLAGS_OTHER "-I${LIBBSON_INCLUDE_DIRS}")
        endif()
    endif()
else()
    message(STATUS "PkgConfig not found, trying to find libbson manually")
    find_path(LIBBSON_INCLUDE_DIRS
        NAMES libbson/bson.h bson.h
        PATHS
            /usr/include
            /usr/local/include
            /opt/local/include
    )
    find_library(LIBBSON_LIBRARIES
        NAMES bson-1.0 libbson-1.0 bson-static-1.0
        PATHS
            /usr/lib
            /usr/local/lib
            /opt/local/lib
    )
    if(LIBBSON_INCLUDE_DIRS AND LIBBSON_LIBRARIES)
        set(LIBBSON_FOUND TRUE)
        set(LIBBSON_CFLAGS_OTHER "-I${LIBBSON_INCLUDE_DIRS}")
    endif()
endif()

if(NOT LIBBSON_FOUND)
    message(FATAL_ERROR "libbson-1.0 not found. Please install libbson-1.0-dev or libbson-static-1.0-dev")
endif()

find_path(WEBSOCKETPP_INCLUDE_DIRS
    NAMES websocketpp/config/asio_no_tls.hpp
    PATHS
        /usr/include
        /usr/local/include
        /opt/local/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(NOT WEBSOCKETPP_INCLUDE_DIRS)
    message(WARNING "websocketpp not found in system paths, assuming header-only library is included in source")
    set(WEBSOCKETPP_INCLUDE_DIRS "")
endif()

if(NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)
add_definitions(-DBSON_STATIC=1)

add_library(${TARGET_NAME} SHARED
    ${SOURCE_FILE}
)

target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBBSON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

if(WEBSOCKETPP_INCLUDE_DIRS)
    target_include_directories(${TARGET_NAME} PUBLIC ${WEBSOCKETPP_INCLUDE_DIRS})
endif()

target_compile_definitions(${TARGET_NAME} PRIVATE ${LIBBSON_CFLAGS_OTHER})

target_link_libraries(${TARGET_NAME}
    PUBLIC
    basic
    common
    framework
    ${OpenCV_LIBS}
    ${LIBBSON_LIBRARIES}
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
