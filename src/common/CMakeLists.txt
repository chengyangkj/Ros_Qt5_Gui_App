set(TARGET_NAME common)

file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc)
list(FILTER SOURCE_FILE EXCLUDE REGEX "(.*_test\\.cpp$)|(.*_test\\.cc$)")
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)

add_library(${TARGET_NAME} SHARED
${SOURCE_FILE}
${INCLUDE_FILE}
)

target_link_libraries(${TARGET_NAME} ${Boost_LIBRARIES} Qt5::Core basic nlohmann_json::nlohmann_json yaml-cpp::yaml-cpp)
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

install(TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION bin/lib
    LIBRARY DESTINATION bin/lib
    ARCHIVE DESTINATION bin/lib
)

#######test
set(TEST_LINK_LIBRARIES Qt5::Core basic nlohmann_json ${TARGET_NAME})
AddTest()