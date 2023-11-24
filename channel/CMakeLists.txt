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
#manager
find_package(Qt5 COMPONENTS Core REQUIRED)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  manager/*.h manager/*.hpp)
file(GLOB_RECURSE SRC_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  manager/*.cpp manager/*.cc)
qt5_wrap_cpp(QT_MOC_HPP ${INCLUDE_FILE})
add_library(${TARGET_NAME} STATIC
${QT_MOC_HPP}
${SRC_FILE}
)
target_link_libraries(${TARGET_NAME} PUBLIC Qt5::Core )
target_include_directories(${TARGET_NAME} PUBLIC 
${CMAKE_CURRENT_SOURCE_DIR}
manager
)
set_target_properties(${TARGET_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)
if(BUILD_WITH_TEST)
add_subdirectory(test)
endif()