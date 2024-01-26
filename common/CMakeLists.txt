
include(TargetNameHelper)
include(GTestHelper)
GetTargetName()

file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} src/*.cpp src/*.cc)
list(FILTER SOURCE_FILE EXCLUDE REGEX "(.*_test\\.cpp$)|(.*_test\\.cc$)")
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  *.h *.hpp)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
add_library(${TARGET_NAME} SHARED
${SOURCE_FILE}
${INCLUDE_FILE}
)
target_link_libraries(${TARGET_NAME} ${Boost_LIBRARIES} Qt5::Core basic json_struct)
target_include_directories(${TARGET_NAME} PUBLIC include)

install(TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION bin/lib
    LIBRARY DESTINATION bin/lib
    ARCHIVE DESTINATION bin/lib
)

#######test
set(TEST_LINK_LIBRARIES Qt5::Core basic json_struct ${TARGET_NAME})
AddTest()