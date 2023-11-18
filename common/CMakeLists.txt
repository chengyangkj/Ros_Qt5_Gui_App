
include(TargetNameHelper)
GetTargetName()
file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} src/*.cpp src/*.cc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  include/*.h *.hpp)

add_library(${TARGET_NAME} STATIC
${SOURCE_FILE}
${INCLUDE_FILE}
json_struct
)