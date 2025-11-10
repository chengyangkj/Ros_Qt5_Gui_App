include(FetchContent)

message(STATUS "get topology_msgs ...")

set(topology_msgs_DOWNLOAD_URL
    "https://github.com/chengyangkj/topology_msgs/archive/refs/heads/main.zip"
    CACHE STRING "")

FetchContent_Declare(
  topology_msgs
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  URL "${topology_msgs_DOWNLOAD_URL}")

FetchContent_GetProperties(topology_msgs)
if(NOT topology_msgs_POPULATED)
  # 在 MakeAvailable 之前修复 topology_msgs 的 CMakeLists.txt
  # 读取并修复 CMakeLists.txt 中的不兼容属性设置
  FetchContent_Populate(topology_msgs)
  
  set(TOPOLOGY_MSGS_CMAKE_FILE "${topology_msgs_SOURCE_DIR}/CMakeLists.txt")
  if(EXISTS "${TOPOLOGY_MSGS_CMAKE_FILE}")
    file(READ "${TOPOLOGY_MSGS_CMAKE_FILE}" TOPOLOGY_MSGS_CMAKE_CONTENT)
    # 移除 INTERFACE_LIBRARY 的 LIBRARY_OUTPUT_DIRECTORY 和 RUNTIME_OUTPUT_DIRECTORY 设置
    string(REGEX REPLACE 
      "set_target_properties\\([^)]*INTERFACE_LIBRARY[^)]*\\)[^)]*LIBRARY_OUTPUT_DIRECTORY[^)]*\\)" 
      "" 
      TOPOLOGY_MSGS_CMAKE_CONTENT 
      "${TOPOLOGY_MSGS_CMAKE_CONTENT}")
    string(REGEX REPLACE 
      "set_target_properties\\([^)]*INTERFACE_LIBRARY[^)]*\\)[^)]*RUNTIME_OUTPUT_DIRECTORY[^)]*\\)" 
      "" 
      TOPOLOGY_MSGS_CMAKE_CONTENT 
      "${TOPOLOGY_MSGS_CMAKE_CONTENT}")
    file(WRITE "${TOPOLOGY_MSGS_CMAKE_FILE}" "${TOPOLOGY_MSGS_CMAKE_CONTENT}")
  endif()
  
  FetchContent_MakeAvailable(topology_msgs)
  
  set(TOPOLOGY_MSGS_TARGETS
    topology_msgs__rosidl_generator_cpp
    topology_msgs__rosidl_typesupport_cpp
    topology_msgs__rosidl_typesupport_fastrtps_cpp
    topology_msgs__rosidl_typesupport_introspection_cpp
  )
  
  foreach(TARGET_NAME ${TOPOLOGY_MSGS_TARGETS})
    if(TARGET ${TARGET_NAME})
      set_target_properties(${TARGET_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
      )
      
      install(TARGETS ${TARGET_NAME}
        RUNTIME DESTINATION bin/lib
        LIBRARY DESTINATION bin/lib
        ARCHIVE DESTINATION bin/lib
      )
    endif()
  endforeach()
endif()

# import targets:
# topology_msgs::topology_msgs 