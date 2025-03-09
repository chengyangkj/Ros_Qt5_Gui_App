set(TARGET_NAME plugin)
include(GTestHelper)

file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} *.cpp *.cc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  *.h *.hpp)

add_library(${TARGET_NAME} STATIC
${SOURCE_FILE}
${INCLUDE_FILE}
)

target_link_libraries(${TARGET_NAME} ${Boost_LIBRARIES} Qt5::Core basic json_struct)
target_include_directories(${TARGET_NAME} PUBLIC include)

#######test
# set(TEST_LINK_LIBRARIES Qt5::Core basic json_struct ${TARGET_NAME})
# AddTest()