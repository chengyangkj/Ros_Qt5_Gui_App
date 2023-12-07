include(TargetNameHelper)
gettargetname()
file(GLOB_RECURSE SOURCE_FILE CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} *.cpp *.cc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} *.h *.hpp)

find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs cv_bridge
                                        std_msgs image_transport)

add_library(${TARGET_NAME} SHARED ${SOURCE_FILE} ${INCLUDE_FILE})
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES} common basic)
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                                                 ${catkin_INCLUDE_DIRS})
set_target_properties(
  ${TARGET_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
                            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
install(TARGETS ${TARGET_NAME} DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${TARGET_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
