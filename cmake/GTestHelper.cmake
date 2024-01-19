macro(AddTest)
if(BUILD_WITH_TEST)

include(TargetNameHelper)
GetTargetName()
# 查找 GTest 库
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
file(GLOB_RECURSE SRC_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  *_test.cpp *_test.cc)
# 添加可执行文件
add_executable(${TARGET_NAME}_test ${SRC_FILE})

# 链接 GTest 库
target_link_libraries(${TARGET_NAME}_test ${GTEST_LIBRARIES} gtest_main pthread ${TEST_LINK_LIBRARIES} )

# 添加测试
add_test(NAME ${TARGET_NAME}_test COMMAND  ${TARGET_NAME}_test)
endif()
endmacro()