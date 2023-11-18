set(TARGET_NAME channel)

find_package(catkin REQUIRED COMPONENTS roscpp 
    tf
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    )
target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})

install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

