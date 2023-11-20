
include(TargetNameHelper)
GetTargetName()
file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} src/*.cpp src/*.cc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  *.h *.hpp)
find_package(Qt5 COMPONENTS Core REQUIRED)
add_library(${TARGET_NAME} STATIC
${SOURCE_FILE}
${INCLUDE_FILE}
)
target_link_libraries(${TARGET_NAME} Qt5::Core)
target_include_directories(${TARGET_NAME} PUBLIC include)